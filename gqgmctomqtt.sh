#!/bin/bash

################################################################################

# GQ Electronics GMC to MQTT

# This script connects to a GQ GMC Geiger Counter via serial port and publishes readings to MQTT.
# It does not use the heartbeat feature which returns CPS ratrher than CPM.

# https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt

################################################################################

READ_PERIOD=5

SERIAL_PORT="/dev/ttyUSB0"
SERIAL_RATE=115200
SERIAL_CONF="cs8 -cstopb -parenb -echo"

MQTT_HOST="localhost"
MQTT_PORT=1883
MQTT_TOPIC="sensors/radiation/cpm"

DEVICE_MODEL=""
DEVICE_SERIAL=""

################################################################################

dependency_check() {
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

################################################################################

MQTT_FD=
mqtt_start() {
    MQTT_PIPE="/tmp/mqtt_pipe_$$"
    [ -e "$MQTT_PIPE" ] && rm -f "$MQTT_PIPE"
    mkfifo "$MQTT_PIPE"
    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$MQTT_TOPIC" -l -q 1 < "$MQTT_PIPE" &
    MQTT_PID=$!
    exec 8>"$MQTT_PIPE"
    MQTT_FD=8
    echo "MQTT client started with PID $MQTT_PID"
}
mqtt_publish() {
    local message="$1"
    echo "$message" >&$MQTT_FD
}
mqtt_stop() {
    if [ -n "$MQTT_PID" ]; then
        kill $MQTT_PID 2>/dev/null
        wait $MQTT_PID 2>/dev/null
        echo "MQTT client stopped"
    fi
    if [ -n "$MQTT_FD" ]; then
        exec 8>&-
    fi
    [ -e "$MQTT_PIPE" ] && rm -f "$MQTT_PIPE"
}

################################################################################

device_serial_check() {
    if [ ! -e "$SERIAL_PORT" ]; then
        return 1
    fi
    if ! stty -F "$SERIAL_PORT" $SERIAL_RATE $SERIAL_CONF raw 2>/dev/null; then
        return 1
    fi
    return 0
}
device_serial_config() {
    echo "Device configure $SERIAL_PORT at $SERIAL_RATE baud ..."
    stty -F "$SERIAL_PORT" $SERIAL_RATE $SERIAL_CONF raw
    if [ $? -ne 0 ]; then
        echo "Failed to config serial port. Check if the port exists and permissions are okay."
        return 1
    fi
    return 0
}
device_serial_wait() {
    while true; do
        if device_serial_check; then
            echo "Device connected $SERIAL_PORT"
            if device_serial_config; then
                return 0
            fi
        fi
        echo "Device waiting $SERIAL_PORT ..."
        sleep 10
    done
}
device_serial_send() {
    local cmd="$1"
    echo -n "$cmd" >"$SERIAL_PORT"
    sleep 0.5 # Give the device time to respond
}

################################################################################

device_info_display() {
    dd if="$SERIAL_PORT" iflag=nonblock count=100 2>/dev/null || true
    device_serial_send "<GETVER>>"
    local raw_response=$(dd if="$SERIAL_PORT" bs=1 count=14 2>/dev/null)
    DEVICE_MODEL=$(echo "$raw_response" | tr -c '[:graph:][:space:]' '.')
    device_serial_send "<GETSERIAL>>"
    local raw_response=$(dd if="$SERIAL_PORT" bs=1 count=7 2>/dev/null)
    DEVICE_SERIAL=$(echo "$raw_response" | tr -c '[:graph:][:space:]' '.')
    echo "Device connected to '$DEVICE_MODEL' with serial '$DEVICE_SERIAL'"
}

################################################################################

device_cpm_request() {
    device_serial_send "<GETCPM>>"
}
device_cpm_read() {
    local data=$(dd if="$SERIAL_PORT" bs=1 count=4 2>/dev/null | xxd -p)
    if [[ ! "$data" =~ ^[0-9a-f]+$ ]] || [ ${#data} -lt 8 ]; then
        echo "-1"
        return
    fi
    local byte1=${data:0:2}
    local byte2=${data:2:2}
    local byte3=${data:4:2}
    local byte4=${data:6:2}
    local val1=0 val2=0 val3=0 val4=0
    if [[ "$byte1" =~ ^[0-9a-f]{2}$ ]]; then
        val1=$(printf "%d" 0x$byte1 2>/dev/null || echo 0)
    fi
    if [[ "$byte2" =~ ^[0-9a-f]{2}$ ]]; then
        val2=$(printf "%d" 0x$byte2 2>/dev/null || echo 0)
    fi
    if [[ "$byte3" =~ ^[0-9a-f]{2}$ ]]; then
        val3=$(printf "%d" 0x$byte3 2>/dev/null || echo 0)
    fi
    if [[ "$byte4" =~ ^[0-9a-f]{2}$ ]]; then
        val4=$(printf "%d" 0x$byte4 2>/dev/null || echo 0)
    fi
    echo $(((val1 << 24) | (val2 << 16) | (val3 << 8) | val4))
}
device_cpm_display() {
    device_cpm_request
    cpm=$(device_cpm_read)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] CPM: $cpm"
}

################################################################################

read_and_send_okay=true

read_and_send_loop() {
    echo "Reading CPM data every $READ_PERIOD seconds and publishing to MQTT '$MQTT_TOPIC' ..."
    while $read_and_send_okay; do
        if ! device_serial_check; then
            echo "Device disconnected, waiting for reconnection ..."
            device_serial_wait
            device_info_display
            echo "Device reconnected, resuming data collection ..."
        fi
        device_cpm_request
        cpm=$(device_cpm_read)
        if [ "$cpm" != "-1" ] && [[ "$cpm" =~ ^[0-9]+$ ]] && [ "$cpm" -lt 100000 ]; then
            echo "[$(date +"%Y-%m-%d %H:%M:%S")] CPM: $cpm"
            mqtt_publish "$cpm"
        else
            echo "Warning: Invalid data received. Device may be disconnected. Will wait and restart."
            device_serial_wait
            device_info_display
        fi
        sleep $READ_PERIOD &
        SLEEP_PID=$!
        wait $SLEEP_PID
    done
}
read_and_send_stop() {
    echo "Received termination signal. Stopping ..."
    read_and_send_okay=false
    if [[ -n "$SLEEP_PID" ]]; then
        kill $SLEEP_PID 2>/dev/null
    fi
}

################################################################################

LOCK_FILE="/var/lock/gqgmctomqtt.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "Error: Another instance is already running. Exiting."
    exit 0
fi
echo $$ >&9

cleanup() {
    read_and_send_stop
    mqtt_stop
    flock -u 9
    rm -f "$LOCK_FILE"
}

trap read_and_send_stop SIGINT SIGTERM
trap cleanup EXIT

################################################################################

main() {
    echo "===== GQ Geiger Counter MQTT Publisher ====="
    dependency_check
    mqtt_start
    device_serial_wait
    device_info_display
    read_and_send_loop
}

main

exit 0

################################################################################

