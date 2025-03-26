#!/bin/bash

# GQ Electronics GMC to MQTT
# This script connects to a GQ GMC Geiger Counter via serial port and publishes readings to MQTT.
# It does not use the heartbeat feature which returns CPS ratrher than CPM.

# https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt

SERIAL_PORT="/dev/ttyUSB0"
BAUD_RATE=115200
READ_PERIOD=5
MQTT_HOST="localhost"
MQTT_PORT=1883
MQTT_TOPIC="sensors/radiation/cpm"
DEVICE_MODEL=""
DEVICE_SERIAL=""

check_dependencies() {
    local missing_deps=()
    if ! command -v stty &>/dev/null; then
        missing_deps+=("coreutils")
    fi
    if ! command -v xxd &>/dev/null; then
        missing_deps+=("xxd")
    fi
    if ! command -v mosquitto_pub &>/dev/null; then
        missing_deps+=("mosquitto-clients")
    fi
    if [ ${#missing_deps[@]} -ne 0 ]; then
        echo "Error: Missing required packages: ${missing_deps[*]}, please install them: 'sudo apt-get install ${missing_deps[*]}'"
        exit 1
    fi
}

device_config_serial_port() {
    echo "Configuring serial port $SERIAL_PORT at $BAUD_RATE baud..."
    stty -F "$SERIAL_PORT" $BAUD_RATE cs8 -cstopb -parenb -echo raw
    if [ $? -ne 0 ]; then
        echo "Failed to config serial port. Check if the port exists and you have permission to access it."
        echo "You might need to run: 'sudo usermod -a -G dialout $USER', then log out and back in for the changes to take effect."
        exit 1
    fi
}

device_command_send() {
    local cmd="$1"
    echo -n "$cmd" >"$SERIAL_PORT"
    sleep 0.5 # Give the device time to respond
}

device_get_info() {
    echo "Getting device information..."
    device_command_send "<GETVER>>"
    local response=$(dd if="$SERIAL_PORT" bs=1 count=14 2>/dev/null)
    DEVICE_MODEL="$response"
    device_command_send "<GETSERIAL>>"
    local response=$(dd if="$SERIAL_PORT" bs=1 count=7 2>/dev/null)
    DEVICE_SERIAL="$response"
    echo "Connected to model=$DEVICE_MODEL, serial=$DEVICE_SERIAL"
}

device_get_cpm() {
    echo "Getting current CPM reading..."
    device_command_send "<GETCPM>>"
    echo "Command sent, waiting for response..."
    local data=$(dd if="$SERIAL_PORT" bs=1 count=4 2>/dev/null | xxd -p)
    local byte1=${data:0:2}
    local byte2=${data:2:2}
    local byte3=${data:4:2}
    local byte4=${data:6:2}
    byte1=$(printf "%d" 0x$byte1)
    byte2=$(printf "%d" 0x$byte2)
    byte3=$(printf "%d" 0x$byte3)
    byte4=$(printf "%d" 0x$byte4)
    local cpm=$(((byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4))
    cpm=$((cpm & 0x3FFF))
    echo "CPM: $cpm"
}

is_active=true
read_and_publish() {
    echo "Reading CPM data every $READ_PERIOD seconds and publishing to MQTT '$MQTT_TOPIC' ..."
    is_active=true
    while $is_active; do
        device_command_send "<GETCPM>>"
        local data=$(dd if="$SERIAL_PORT" bs=1 count=4 2>/dev/null | xxd -p)
        if [ -n "$data" ]; then
            local byte1=${data:0:2}
            local byte2=${data:2:2}
            local byte3=${data:4:2}
            local byte4=${data:6:2}
            byte1=$(printf "%d" 0x$byte1)
            byte2=$(printf "%d" 0x$byte2)
            byte3=$(printf "%d" 0x$byte3)
            byte4=$(printf "%d" 0x$byte4)
            local cpm=$(((byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4))
            cpm=$((cpm & 0x3FFF))
            local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
            echo "[$timestamp] CPM: $cpm"
            mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$MQTT_TOPIC" -m "$cpm" -q 1
            if [ $? -ne 0 ]; then
                echo "Warning: Failed to publish to MQTT broker."
            fi
        else
            echo "No data received. Check connection."
            sleep 1
        fi
        sleep $READ_PERIOD
    done
}

heartbeat_cancel() {
    is_active=false
}

trap heartbeat_cancel SIGINT SIGTERM

main() {
    echo "===== GQ Geiger Counter MQTT Publisher ====="
    check_dependencies
    device_config_serial_port
    device_get_info
    # device_get_cpm
    read_and_publish
}

main

exit 0
