#!/usr/bin/env node

// GQ Electronics GMC to MQTT
//
// This script connects to a GQ GMC Geiger Counter via serial port and publishes readings to MQTT.
// It does not use the heartbeat feature which returns CPS rather than CPM.
//
// https://www.gqelectronicsllc.com/download/GQ-RFC1801.txt
//

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

const { SerialPort } = require('serialport');
const mqtt = require('mqtt');
const fs = require('fs');

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

const READ_PERIOD = 30 * 1000;
let runningOkay = true;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

const configPath = 'gqgmctomqtt.cfg';

function configLoad(configPath) {
    try {
        const items = {};
        fs.readFileSync(configPath, 'utf8')
            .split('\n')
            .forEach((line) => {
                const [key, value] = line.split('=').map((s) => s.trim());
                if (key && value) items[key] = value;
            });
        return items;
    } catch (err) {
        console.warn(`config: cannot load '${configPath}', using defaults (which may not work correctly), error:`, err);
        return {};
    }
}
const conf = configLoad(configPath);
console.log(
    `Loaded 'config' using '${configPath}': ${Object.entries(conf)
        .map(([k, v]) => k.toLowerCase() + '=' + v)
        .join(', ')}`
);

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

let mqttClient = null;

function mqttBegin() {
    console.log(`mqtt: connecting to '${conf.MQTT_SERVER}'`);
    mqttClient = mqtt.connect(conf.MQTT_SERVER, {
        clientId: 'sensor-radiation-' + Math.random().toString(16).substring(2, 8),
    });
    mqttClient.on('connect', () => {
        console.log(`mqtt: connected`);
    });
    mqttClient.on('error', (err) => {
        console.error(`mqtt: error: ${err.message}`);
    });
    return mqttClient;
}
function mqttSend(topic, message) {
    if (mqttClient && mqttClient.connected) mqttClient.publish(topic, message.toString());
}
function mqttEnd() {
    if (mqttClient) {
        mqttClient.end();
        mqttClient = null;
    }
}

console.log(`Loaded 'mqtt' using '${conf.MQTT_SERVER}'`);

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

let port = null;

async function deviceSerialCheck() {
    try {
        await fs.promises.access(conf.SERIAL_PORT);
        return true;
    } catch {
        return false;
    }
}
async function deviceSerialConfig() {
    console.log(`device: configure ${conf.SERIAL_PORT} at ${conf.SERIAL_RATE} baud`);
    try {
        if (port) await new Promise((resolve) => port.close(resolve));
        port = new SerialPort({
            path: conf.SERIAL_PORT,
            baudRate: parseInt(conf.SERIAL_RATE),
            dataBits: 8,
            stopBits: 1,
            parity: 'none',
            autoOpen: false,
        });
        return new Promise((resolve, reject) => {
            port.open((err) => {
                if (err) {
                    console.error('device: error accessing serial port, check if the port exists and permissions are okay.');
                    reject(err);
                } else resolve(true);
            });
        });
    } catch (err) {
        console.error(`device: error configuring serial port: ${err.message}`);
        return false;
    }
}
async function deviceSerialEnd() {
    if (port && port.isOpen) port.close();
}
async function deviceSerialWait() {
    let counter = 0;
    while (runningOkay) {
        if (await deviceSerialCheck()) {
            if (!(await deviceSerialConfig())) return false;
            console.log(`device: connected`);
            return true;
        }
        if (counter++ % 6 == 0) console.log(`device: waiting for connection`);
        await new Promise((resolve) => setTimeout(resolve, 5000));
    }
    return false;
}
async function deviceSerialBegin() {
    return deviceSerialWait();
}
async function deviceSerialSend(cmd) {
    if (!port || !port.isOpen) return Promise.resolve();
    return port.write(Buffer.from(cmd));
}
function deviceSerialRead(count) {
    return new Promise((resolve) => {
        let data = Buffer.alloc(0);
        const dataHandler = (chunk) => {
            data = Buffer.concat([data, chunk]);
            if (data.length >= count) {
                cleanup();
                resolve(data.slice(0, count));
            }
        };
        const errorHandler = (err) => {
            console.error(`device: error reading serial port: ${err.message}`);
            cleanup();
            resolve(Buffer.alloc(count));
        };
        const cleanup = () => {
            port.removeListener('data', dataHandler);
            port.removeListener('error', errorHandler);
        };
        port.on('data', dataHandler);
        port.on('error', errorHandler);
        setTimeout(() => {
            cleanup();
            resolve(data);
        }, 1000);
    });
}
async function deviceSerialReconnect() {
    console.log(`device: reconnecting`);
    await deviceSerialWait();
    await deviceInfoDisplay();
    console.log(`device: reconnected, resuming`);
}

console.log(`Loaded 'device' using '${conf.SERIAL_PORT}'`);

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

async function deviceInfoDisplay() {
    if (!port || !port.isOpen) return;
    port.flush();
    await deviceSerialSend('<GETVER>>');
    const model = (await deviceSerialRead(14)).toString().replace(/[^\x20-\x7E]/g, '.');
    await deviceSerialSend('<GETSERIAL>>');
    const serial = Array.from(await deviceSerialRead(7))
        .map((byte) => byte.toString(16).padStart(2, '0').toUpperCase())
        .join(' ');
    await deviceSerialSend('<GETVOLT>>');
    const volts = await deviceSerialRead(5);
    await deviceSerialSend('<GETDATETIME>>');
    const dt = await deviceSerialRead(7);
    const pad = (num) => String(num).padStart(2, '0');
    const datetime = `20${pad(dt[0])}/${pad(dt[1])}/${pad(dt[2])} ${pad(dt[3])}:${pad(dt[4])}:${pad(dt[5])}`;
    console.log(`device: model='${model.substring(0, 8)}/${model.substring(8)}', serial='${serial}', datetime='${datetime}', volts='${volts}'`);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

function deviceCpmRequest() {
    return deviceSerialSend('<GETCPM>>');
}
async function deviceCpmRead() {
    try {
        const data = await deviceSerialRead(4);
        if (data.length < 4) return -1;
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    } catch (err) {
        console.error(`device: error reading CPM: ${err.message}`);
        return -1;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

let sleepTimeout = null;

async function readAndSendProcess() {
    console.log(`reader: reading CPM every ${READ_PERIOD / 1000} seconds, publishing to MQTT '${conf.MQTT_TOPIC}'`);
    while (runningOkay) {
        try {
            if (!(await deviceSerialCheck()) || !port || !port.isOpen) {
                console.log(`reader: device disconnected`);
                deviceSerialReconnect();
            }
            await deviceCpmRequest();
            const cpm = await deviceCpmRead();
            if (cpm !== -1 && !isNaN(cpm) && cpm < 100000) {
                const timestamp = new Date().toISOString().replace('T', ' ').substr(0, 19);
                console.log(`reader: CPM=${cpm} [${timestamp}]`);
                mqttSend(conf.MQTT_TOPIC, cpm.toString());
            } else {
                console.log(`reader: device short/faulty data, probably disconnected`);
                deviceSerialReconnect();
            }
            await new Promise((resolve) => {
                sleepTimeout = setTimeout(resolve, READ_PERIOD);
            });
        } catch (err) {
            console.error(`reader: error: ${err.message}`);
            await new Promise((resolve) => setTimeout(resolve, READ_PERIOD));
        }
    }
}
function readAndSendEnd() {
    if (sleepTimeout) clearTimeout(sleepTimeout);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

function cleanup() {
    runningOkay = false;
    readAndSendEnd();
    deviceSerialEnd();
    mqttEnd();
}
function terminate() {
    console.log(`terminating`);
    cleanup();
}

async function main() {
    console.log(`starting`);
    process.on('SIGINT', terminate);
    process.on('SIGTERM', terminate);
    if (await deviceSerialBegin()) {
        mqttBegin();
        await deviceInfoDisplay();
        await readAndSendProcess();
    }
}

main().catch((err) => {
    console.error(`exception: ${err.message}`);
    cleanup();
    process.exit(1);
});

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
