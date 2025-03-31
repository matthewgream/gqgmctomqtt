#!/usr/bin/env node

// GQ Electronics GMC to MQTT
//
// This script connects to a GQ GMC Geiger Counter via serial port and publishes readings to MQTT.
// It does not use the heartbeat feature which returns CPS ratrher than CPM.
//
// https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt
//

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

const { SerialPort } = require('serialport');
const mqtt = require('mqtt');
const fs = require('fs');

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

const READ_PERIOD = 15 * 1000;
let runningOkay = true;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

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
        console.warn(`sensor-radiation: config: could not load '${configPath}', using defaults (which may not work correctly)`);
        return {};
    }
}
const configPath = '/opt/sensors/radiation/secrets.txt';
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
    console.log(`sensor-radiation: mqtt: connecting to '${conf.MQTT}'`);
    mqttClient = mqtt.connect(conf.MQTT, {
        clientId: 'sensor-radiation-' + Math.random().toString(16).substring(2, 8),
    });
    mqttClient.on('connect', () => {
        console.log(`sensor-radiation: mqtt: connected`);
    });
    mqttClient.on('error', (err) => {
        console.error(`sensor-radiation: mqtt: error: ${err.message}`);
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

console.log(`Loaded 'mqtt' using '${conf.MQTT}'`);

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

let port = null;

async function deviceSerialCheck() {
    try {
        await fs.promises.access(conf.PORT);
        return true;
    } catch {
        return false;
    }
}
async function deviceSerialConfig() {
    console.log(`sensor-radiation: device: configure ${conf.PORT} at ${conf.RATE} baud`);
    try {
        if (port) await new Promise((resolve) => port.close(resolve));
        port = new SerialPort({
            path: conf.PORT,
            baudRate: parseInt(conf.RATE),
            dataBits: 8,
            stopBits: 1,
            parity: 'none',
            autoOpen: false,
        });
        return new Promise((resolve, reject) => {
            port.open((err) => {
                if (err) {
                    console.error('sensor-radiation: device: error accessing serial port, check if the port exists and permissions are okay.');
                    reject(err);
                } else resolve(true);
            });
        });
    } catch (err) {
        console.error(`sensor-radiation: device: error configuring serial port: ${err.message}`);
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
            console.log(`sensor-radiation: device: connected`);
            return true;
        }
        if (counter++ % 6 == 0) console.log(`sensor-radiation: device: waiting for connection`);
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
            console.error(`sensor-radiation: device: error reading serial port: ${err.message}`);
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
    console.log(`sensor-radiation: device: reconnecting`);
    await deviceSerialWait();
    await deviceInfoDisplay();
    console.log(`sensor-radiation: device: reconnected, resuming`);
}

console.log(`Loaded 'device' using '${conf.PORT}'`);

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
    console.log(`sensor-radiation: device: model='${model.substring(0, 8)}', revision='${model.substring(8)}', serial='${serial}'`);
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
        console.error(`sensor-radiation: device: error reading CPM: ${err.message}`);
        return -1;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

let sleepTimeout = null;

async function readAndSendProcess() {
    console.log(`sensor-radiation: reader: reading CPM every ${READ_PERIOD / 1000} seconds, publishing to MQTT '${conf.MQTT_TOPIC}'`);
    while (runningOkay) {
        try {
            if (!(await deviceSerialCheck()) || !port || !port.isOpen) {
                console.log(`sensor-radiation: device: disconnected`);
                deviceSerialReconnect();
            }
            await deviceCpmRequest();
            const cpm = await deviceCpmRead();
            if (cpm !== -1 && !isNaN(cpm) && cpm < 100000) {
                const timestamp = new Date().toISOString().replace('T', ' ').substr(0, 19);
                console.log(`sensor-radiation: reader: CPM=${cpm} [${timestamp}]`);
                mqttSend(conf.MQTT_TOPIC, cpm.toString());
            } else {
                console.log(`sensor-radiation: device: faulty data, probably disconnected`);
                deviceSerialReconnect();
            }
            await new Promise((resolve) => {
                sleepTimeout = setTimeout(resolve, READ_PERIOD);
            });
        } catch (err) {
            console.error(`sensor-radiation: reader: error: ${err.message}`);
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
    console.log(`sensor-radiation: terminating`);
    cleanup();
}

async function main() {
    console.log(`sensor-radiation: starting`);
    process.on('SIGINT', terminate);
    process.on('SIGTERM', terminate);
    if (await deviceSerialBegin()) {
        mqttBegin();
        await deviceInfoDisplay();
        await readAndSendProcess();
    }
}

main().catch((err) => {
    console.error(`sensor-radiation: exception: ${err.message}`);
    cleanup();
    process.exit(1);
});

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
