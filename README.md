# gqgmctomqtt

Reads radiation measurements from a GQ Electronics GMC Geiger counter over USB serial and publishes them to MQTT
and/or [GMCMAP](https://www.gmcmap.com).

It is written in C, depends only on `libmosquitto`, and is meant to run unattended as a systemd service. It recovers
on its own when the counter is unplugged and plugged back in.

Tested on a GMC-500+ (firmware `Re 2.5`) running Debian.

## Functionality

### At startup

1. **Device information**: reads and logs the model and firmware version (`GETVER`), the serial number
   (`GETSERIAL`), the device date/time (`GETDATETIME`) and the battery voltage (`GETVOLT`).
2. **Clock check**: if the device clock is more than 5 minutes away from the host's local time, the device clock is
   set to the host's local time (`SETDATETIME`).
3. **Calibration**: reads the device configuration (`GETCFG`) and extracts the three CPM to µSv/h calibration points
   that are stored on the counter.

Steps 1 and 2 are repeated whenever the device reconnects. The calibration is read only once, so restart the program
after changing it on the counter.

### Main loop

Every `read-period` seconds (default 30):

1. Requests the current counts-per-minute from the device (`GETCPM`).
2. Checks that the value is plausible (0 to 5000 CPM). A failed read, a write error, an implausible value or a missing
   serial port is treated as a fault: the port is closed and the program polls every 5 seconds until the device comes
   back, then logs the device information again and carries on.
3. Calculates:
   - **CPM**: the instantaneous reading from the device.
   - **ACPM**: the average of all readings taken in the last hour.
   - **µSv/h**: the dose rate, found by linear interpolation between `(0 CPM, 0 µSv/h)` and the device's three
     calibration points. Readings above the highest calibration point are extrapolated from the last segment.
4. Logs the readings to stdout.
5. Publishes to MQTT, if configured.
6. Publishes to GMCMAP, if configured (at most once every 60 seconds).

The device's heartbeat mode is not used, because it reports counts per second rather than counts per minute.

### MQTT

The readings are published as a JSON object to the configured topic, with QoS 0 and without the retain flag:

```json
{"cpm":18,"acpm":17.42,"usvh":0.12}
```

The `mqtt-server` setting takes these forms:

| Form               | Port | Notes                                      |
| ------------------ | ---- | ------------------------------------------ |
| `mqtt://host`      | 1883 | plain TCP                                  |
| `mqtts://host`     | 8883 | TLS; the server certificate is not checked |
| `mqtt://host:port` | port | an explicit port works with either scheme  |

The MQTT client ID is the value of `mqtt-client` followed by a random 6-digit hexadecimal suffix, e.g.
`gqgmctomqtt-3FA2C1`. Broker authentication (username/password) is not supported.

The program connects to the broker at startup and exits if that fails. Setting `mqtt-topic` to an empty value stops
readings being published, but the program still needs a broker to connect to.

### GMCMAP

When both `gmcmap-user-id` and `gmcmap-counter-id` are set, the readings are uploaded to gmcmap.com with an HTTP GET
request to `http://www.gmcmap.com/log2.asp`, sending CPM, ACPM and µSv/h, at most once every 60 seconds. A failed
upload is logged as a warning and does not stop the program.

## Configuration

Settings are read first from a configuration file, then from the command line; command-line values take precedence.

### Configuration file

The file holds one `key=value` per line. Spaces around the key and the value are trimmed. Blank lines, lines whose
first non-blank character is `#` (comments) and lines without an `=` are ignored. A `#` later in a line is part of
the value, not the start of a comment. By default the program reads `gqgmctomqtt.cfg` from the current directory; another file can be given
with `--config <file>`. The installed service uses `/etc/default/gqgmctomqtt`.

An example, [`gqgmctomqtt.cfg`](gqgmctomqtt.cfg), is included:

```ini
serial-port=/dev/gqgmc500plus
serial-rate=115200
serial-bits=8N1
read-period=30
mqtt-server=mqtt://localhost
mqtt-client=gqgmctomqtt
mqtt-topic=sensors/radiation
gmcmap-user-id=12345
gmcmap-counter-id=12345678901
```

### Settings

| Key                 | Default             | Description                                                                 |
| ------------------- | ------------------- | --------------------------------------------------------------------------- |
| `serial-port`       | `/dev/ttyUSB0`      | Serial device of the counter (the udev rule below creates `/dev/gqgmc500plus`) |
| `serial-rate`       | `115200`            | Baud rate: 1200, 2400, 4800, 9600, 19200, 38400, 57600 or 115200             |
| `serial-bits`       | `8N1`               | Data bits, parity and stop bits; only `8N1` is supported                    |
| `read-period`       | `30`                | Seconds between readings                                                    |
| `mqtt-server`       | `mqtt://localhost`  | MQTT broker (see [MQTT](#mqtt))                                             |
| `mqtt-client`       | `gqgmctomqtt`       | Prefix of the MQTT client ID                                                |
| `mqtt-topic`        | `sensors/radiation` | Topic for readings; leave empty to stop publishing to MQTT                  |
| `gmcmap-user-id`    | *(empty)*           | GMCMAP account ID; GMCMAP upload is off unless both IDs are set             |
| `gmcmap-counter-id` | *(empty)*           | GMCMAP Geiger counter ID                                                    |

If an integer setting cannot be parsed, or `serial-bits` is not `8N1`, a warning is printed and the default is used.

### Command line

Every setting can also be given as a long option with the same name:

```sh
gqgmctomqtt --config /etc/default/gqgmctomqtt --serial-port /dev/ttyUSB1 --read-period 60
```

At startup the program prints the configuration it ended up with, which makes it easy to check which values
took effect.

## Building

The program needs `gcc`, `make` and the Mosquitto client library:

```sh
sudo make install-dev   # installs libmosquitto-dev
make
```

To cross-compile for 32-bit ARM (e.g. older Raspberry Pi models) on a Debian host:

```sh
sudo make install-dev-armhf
make armhf              # produces gqgmctomqtt.armhf
```

To run in the foreground against the local configuration:

```sh
make test
```

`make test` (and `make install`) use `gqgmctomqtt.<hostname>.cfg` if that file exists, and `gqgmctomqtt.cfg`
otherwise, so each machine can have its own configuration.

## Installation

```sh
sudo make install
```

This runs the following steps, which are also available as separate targets:

| Target            | Action                                                                                 |
| ----------------- | -------------------------------------------------------------------------------------- |
| `install_target`  | Installs the binary to `/usr/local/bin/gqgmctomqtt`                                     |
| `install_default` | Installs the configuration to `/etc/default/gqgmctomqtt` (overwrites any existing file) |
| `install_udev`    | Installs the udev rule to `/etc/udev/rules.d/90-gqgmctomqtt.rules` and reloads udev     |
| `install_service` | Installs, enables and starts the `gqgmctomqtt` systemd service                          |

After changing the configuration, restart the service with `sudo make restart`. Follow its logs with
`journalctl -u gqgmctomqtt -f`.

### udev rule

[`90-gqgmc500plus.rules`](90-gqgmc500plus.rules) matches the GMC-500+'s CH340 USB-serial adapter (USB ID
`1a86:7523`, `bcdDevice` `0264`). It creates a stable `/dev/gqgmc500plus` symlink owned by the `dialout` group.
Counters with a different USB adapter need the IDs in the rule changed; `lsusb` and
`udevadm info -a -n /dev/ttyUSB0` show the values to use.

### systemd service

[`gqgmctomqtt.service`](gqgmctomqtt.service) starts the program with `--config /etc/default/gqgmctomqtt` and restarts
it 5 seconds after a failure. It waits for the network, for `/dev/gqgmc500plus` and for a local `mosquitto.service`.
If the broker is on another host, remove the `mosquitto.service` lines from the unit file.

## Example output

```text
starting
config: file='/etc/default/gqgmctomqtt', serial-port='/dev/gqgmc500plus', serial-rate='115200', ...
config: serial: port=/dev/gqgmc500plus, rate=115200, bits=8N1
serial: connected
mqtt: connecting (host='localhost', port=1883, ssl=false, client='gqgmctomqtt')
mqtt: connected
device: model='GMC-500+/Re 2.5', serial='...', datetime='2026/11/09 10:15:02', volt='...'
device: config: cal1=(...), cal2=(...), cal3=(...)
reader: reading CPM every 30 seconds, publishing to MQTT:'sensors/radiation', GMCMAP:'12345/12345678901'
sample: period=3600, count=150
reader: CPM=18, ACPM=18.00, uSv/h=0.12 [2026/11/09-10:15:03]
```

## Protocol and other devices

The program implements the GQ serial protocol
[GQ-RFC1801](https://www.gqelectronicsllc.com/download/GQ-RFC1801.txt). Older counters that use
[GQ-RFC1201](https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt) need two changes in `gqgmctomqtt.c`:

- `GETVOLT` returns a single byte rather than a string.
- `GETCPM` returns 2 bytes rather than 4.
