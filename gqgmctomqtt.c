
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * GQ Electronics GMC to MQTT
 *
 * This program connects to a GQ GMC Geiger Counter via serial port and publishes readings to MQTT.
 *
 * Implemented for https://www.gqelectronicsllc.com/download/GQ-RFC1801.txt. To support devices that use
 * https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt, the GETVOLT command needs modification to read 1 byte
 * rather than a string, and the GETCPM command needs modification to read 2 bytes rather than 4 bytes.
 *
 * Tested on a 'GMC-500+/Re 2.5' on debian 6.1.
 *
 * Configuration file 'gqgmctomqtt.cfg' (or otherwise as provided as the first argument to the executable) looks like:
 *
 * SERIAL_PORT=/dev/ttyUSB0
 * SERIAL_RATE=115200
 * MQTT_SERVER=mqtt://localhost
 * MQTT_TOPIC=sensors/radiation/cpm
 * READ_PERIOD=5
 *
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <mosquitto.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define CONFIG_FILE_DEFAULT "gqgmctomqtt.cfg"

#define MQTT_SERVER_DEFAULT "mqtt://localhost"
#define MQTT_TOPIC_DEFAULT "sensors/radiation/cpm"
#define SERIAL_PORT_DEFAULT "/dev/ttyUSB0"
#define SERIAL_RATE_DEFAULT 115200
#define READ_PERIOD_DEFAULT 30

bool running = true;

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS 0
#define MQTT_PUBLISH_RETAIN false

#define SERIAL_READ_TIMEOUT 1000

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CONFIG_MAX_LINE 256
#define CONFIG_MAX_VALUE 128

const char *config_file = CONFIG_FILE_DEFAULT;
char config_mqtt_server[CONFIG_MAX_VALUE] = MQTT_SERVER_DEFAULT;
char config_mqtt_topic[CONFIG_MAX_VALUE] = MQTT_TOPIC_DEFAULT;
char config_serial_port[CONFIG_MAX_VALUE] = SERIAL_PORT_DEFAULT;
int config_serial_rate = SERIAL_RATE_DEFAULT;
int config_read_period = READ_PERIOD_DEFAULT;

bool config_load(int argc, const char **argv) {
    if (argc > 1)
        config_file = argv[1];
    FILE *file = fopen(config_file, "r");
    if (file == NULL) {
        fprintf(stderr, "config: could not load '%s', using defaults (which may not work correctly)\n", config_file);
        return false;
    }
    char line[CONFIG_MAX_LINE];
    while (fgets(line, sizeof(line), file)) {
        char *equals = strchr(line, '=');
        if (equals) {
            *equals = '\0';
            char *key = line;
            char *value = equals + 1;
            while (*key && isspace(*key))
                key++;
            char *end = key + strlen(key) - 1;
            while (end > key && isspace(*end))
                *end-- = '\0';
            while (*value && isspace(*value))
                value++;
            end = value + strlen(value) - 1;
            while (end > value && isspace(*end))
                *end-- = '\0';
            if (strcmp(key, "MQTT_SERVER") == 0)
                strncpy(config_mqtt_server, value, sizeof(config_mqtt_server) - 1);
            else if (strcmp(key, "MQTT_TOPIC") == 0)
                strncpy(config_mqtt_topic, value, sizeof(config_mqtt_topic) - 1);
            else if (strcmp(key, "SERIAL_PORT") == 0)
                strncpy(config_serial_port, value, sizeof(config_serial_port) - 1);
            else if (strcmp(key, "SERIAL_RATE") == 0)
                config_serial_rate = atoi(value);
            else if (strcmp(key, "READ_PERIOD") == 0)
                config_read_period = atoi(value);
        }
    }
    fclose(file);
    printf("config: '%s': mqtt=%s, topic=%s, port=%s, rate=%d, period=%d\n", config_file, config_mqtt_server,
           config_mqtt_topic, config_serial_port, config_serial_rate, config_read_period);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

int serial_fd = -1;

bool serial_check(void) { return (access(config_serial_port, F_OK) == 0); }

bool serial_configure(void) {
    printf("device: config %s at %d baud\n", config_serial_port, config_serial_rate);
    if (serial_fd >= 0)
        close(serial_fd);
    serial_fd = open(config_serial_port, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        fprintf(stderr, "device: error accessing serial port: %s\n", strerror(errno));
        return false;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        fprintf(stderr, "device: error getting serial port attributes: %s\n", strerror(errno));
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    speed_t baud;
    switch (config_serial_rate) {
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
    case 57600:
        baud = B57600;
        break;
    case 115200:
        baud = B115200;
        break;
    default:
        baud = B115200;
        break;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8-bit chars
    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;  // Read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "device: error setting serial port attributes: %s\n", strerror(errno));
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    return true;
}

#define SERIAL_CONNECT_CHECK_PERIOD 5
#define SERIAL_CONNECT_CHECK_PRINT 30

bool serial_connect(void) {
    int counter = 0;
    while (running) {
        if (serial_check()) {
            if (!serial_configure())
                return false;
            printf("device: connected\n");
            return true;
        }
        if (counter++ % (SERIAL_CONNECT_CHECK_PRINT / SERIAL_CONNECT_CHECK_PERIOD) == 0)
            printf("device: connection pending\n");
        sleep(SERIAL_CONNECT_CHECK_PERIOD);
    }
    return false;
}

bool serial_connected(void) { return serial_fd >= 0; }

void serial_disconnect(void) {
    if (serial_fd < 0)
        return;
    close(serial_fd);
    serial_fd = -1;
}

bool serial_reconnect(void) {
    printf("device: reconnecting\n");
    serial_disconnect();
    if (!serial_connect())
        return false;
    printf("device: reconnected\n");
    return true;
}

bool serial_write(const char *cmd, ssize_t len) {
    if (serial_fd < 0)
        return false;
    return write(serial_fd, cmd, len) == len;
}

int serial_read(unsigned char *buffer, size_t count, int timeout_ms) {
    if (serial_fd < 0)
        return -1;
    fd_set rdset;
    struct timeval tv;
    FD_ZERO(&rdset);
    FD_SET(serial_fd, &rdset);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int select_result = select(serial_fd + 1, &rdset, NULL, NULL, &tv);
    if (select_result <= 0)
        return select_result; // timeout or error
    return read(serial_fd, buffer, count);
}

void serial_flush() {
    if (serial_fd < 0)
        return;
    tcflush(serial_fd, TCIOFLUSH);
}

bool serial_begin(void) { return serial_connect(); }

void serial_end(void) { serial_disconnect(); }

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

struct mosquitto *mosq = NULL;

void mqtt_send(const char *topic, const char *message) {
    if (!mosq)
        return;
    int result = mosquitto_publish(mosq, NULL, topic, strlen(message), message, MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN);
    if (result != MOSQ_ERR_SUCCESS)
        fprintf(stderr, "mqtt: publish error: %s\n", mosquitto_strerror(result));
}

bool mqtt_parse(const char *string, char *host, int length, int *port, bool *ssl) {
    host[0] = '\0';
    *port = 1883;
    *ssl = false;
    if (strncmp(string, "mqtt://", 7) == 0) {
        strncpy(host, string + 7, length - 1);
    } else if (strncmp(string, "mqtts://", 8) == 0) {
        strncpy(host, string + 8, length - 1);
        *ssl = true;
        *port = 8883;
    } else {
        strcpy(host, string);
    }
    char *port_str = strchr(host, ':');
    if (port_str) {
        *port_str = '\0'; // Terminate host string at colon
        *port = atoi(port_str + 1);
    }
    return true;
}

void mqtt_connect_callback(struct mosquitto *m, void *o __attribute__((unused)), int r) {
    if (m != mosq)
        return;
    if (r != 0) {
        fprintf(stderr, "mqtt: connect failed: %s\n", mosquitto_connack_string(r));
        return;
    }
    printf("mqtt: connected\n");
}

bool mqtt_begin(void) {
    char host[CONFIG_MAX_VALUE];
    int port;
    bool ssl;
    if (!mqtt_parse(config_mqtt_server, host, sizeof(host), &port, &ssl)) {
        fprintf(stderr, "mqtt: error parsing details in '%s'\n", config_mqtt_server);
        return false;
    }
    printf("mqtt: connecting (host='%s', port=%d, ssl=%s)\n", host, port, ssl ? "true" : "false");
    char client_id[24];
    sprintf(client_id, "sensor-radiation-%06X", rand() & 0xFFFFFF);
    int result;
    mosquitto_lib_init();
    mosq = mosquitto_new(client_id, true, NULL);
    if (!mosq) {
        fprintf(stderr, "mqtt: error creating client instance\n");
        return false;
    }
    if (ssl)
        mosquitto_tls_insecure_set(mosq, true); // Skip certificate validation
    mosquitto_connect_callback_set(mosq, mqtt_connect_callback);
    if ((result = mosquitto_connect(mosq, host, port, MQTT_CONNECT_TIMEOUT)) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "mqtt: error connecting to broker: %s\n", mosquitto_strerror(result));
        mosquitto_destroy(mosq);
        mosq = NULL;
        return false;
    }
    if ((result = mosquitto_loop_start(mosq)) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "mqtt: error starting loop: %s\n", mosquitto_strerror(result));
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = NULL;
        return false;
    }
    return true;
}

void mqtt_end(void) {
    if (mosq) {
        mosquitto_loop_stop(mosq, true);
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = NULL;
    }
    mosquitto_lib_cleanup();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETMODEL_SIZE 15

bool device_getmodel_request(void) {
    const char *command = "<GETVER>>";
    return serial_write(command, strlen(command));
}

bool device_getmodel_read(char *model, int length) {
    unsigned char buffer[15];
    const int read_len = serial_read(buffer, sizeof(buffer), SERIAL_READ_TIMEOUT);
    if (read_len < (int)sizeof(buffer))
        return false;
    if (length < DEVICE_GETMODEL_SIZE)
        return false;
    for (int i = 0; i < DEVICE_GETMODEL_SIZE - 1; i++)
        model[i] = isprint(buffer[i]) ? buffer[i] : '.';
    model[DEVICE_GETMODEL_SIZE - 1] = '\0';
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETSERIAL_SIZE 21

bool device_getserial_request(void) {
    const char *command = "<GETSERIAL>>";
    return serial_write(command, strlen(command));
}

bool device_getserial_read(char *serial, int length) {
    unsigned char buffer[7];
    const int read_len = serial_read(buffer, sizeof(buffer), SERIAL_READ_TIMEOUT);
    if (read_len < (int)sizeof(buffer))
        return false;
    if (length < DEVICE_GETSERIAL_SIZE)
        return false;
    for (int i = 0; i < 7; i++)
        sprintf(serial + i * 3, "%02X ", buffer[i]);
    serial[DEVICE_GETSERIAL_SIZE - 1] = '\0';
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETDATETIME_SIZE 30

bool device_getdatetime_request(void) {
    const char *command = "<GETDATETIME>>";
    return serial_write(command, strlen(command));
}

bool device_getdatetime_read(char *datetime, int length) {
    unsigned char buffer[7];
    const int read_len = serial_read(buffer, sizeof(buffer), SERIAL_READ_TIMEOUT);
    if (read_len < (int)sizeof(buffer))
        return false;
    if (length < DEVICE_GETDATETIME_SIZE)
        return false;
    sprintf(datetime, "20%02d/%02d/%02d %02d:%02d:%02d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
            buffer[5]);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETVOLT_SIZE 5

bool device_getvolt_request(void) {
    const char *command = "<GETVOLT>>";
    return serial_write(command, strlen(command));
}

bool device_getvolt_read(char *volts, int length) {
    unsigned char buffer[5];
    const int read_len = serial_read(buffer, sizeof(buffer), SERIAL_READ_TIMEOUT);
    if (read_len < (int)sizeof(buffer))
        return false;
    if (length < DEVICE_GETVOLT_SIZE)
        return false;
    memcpy(volts, buffer, sizeof(buffer) - 1);
    volts[DEVICE_GETVOLT_SIZE - 1] = '\0';
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool device_getcpm_request(void) {
    const char *command = "<GETCPM>>";
    return serial_write(command, strlen(command));
}

bool device_getcpm_read(int *cpm) {
    unsigned char buffer[4];
    const int read_len = serial_read(buffer, sizeof(buffer), SERIAL_READ_TIMEOUT);
    if (read_len < 4)
        return false;
    *cpm = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define MAXIMUM_TIME_DRIFT 5 * 60.0

void device_check_time(const char *datetime) {
    struct tm tm = {0};
    if (sscanf(datetime, "%d/%d/%d %d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min,
               &tm.tm_sec) == 6) {
        tm.tm_year -= 1900;
        tm.tm_mon -= 1;
        double diff_seconds = difftime(time(NULL), mktime(&tm));
        if (fabs(diff_seconds) > MAXIMUM_TIME_DRIFT)
            printf("WARNING: Device time differs from system time by %.1f minutes!\n", fabs(diff_seconds) / 60.0);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool device_info_display(void) {
    serial_flush();
    char model[DEVICE_GETMODEL_SIZE], serial[DEVICE_GETSERIAL_SIZE], datetime[DEVICE_GETDATETIME_SIZE],
        volt[DEVICE_GETVOLT_SIZE];
    if (!device_getmodel_request() || !device_getmodel_read(model, sizeof(model)))
        return false;
    if (!device_getserial_request() || !device_getserial_read(serial, sizeof(serial)))
        return false;
    if (!device_getdatetime_request() || !device_getdatetime_read(datetime, sizeof(datetime)))
        return false;
    device_check_time(datetime);
    if (!device_getvolt_request() || !device_getvolt_read(volt, sizeof(volt)))
        return false;
    printf("device: model='%.8s/%.6s', serial='%s', datetime='%s', volt='%s'\n", model, model + 8, serial, datetime,
           volt);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool cpm_is_reasonable(int cpm) { return (cpm >= 0 && cpm <= 5000); }

void cpm_display(int cpm) {
    time_t now = time(NULL);
    struct tm *timeinfo = localtime(&now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
    printf("reader: CPM=%d [%s]\n", cpm, timestamp);
}

void cpm_publish(int cpm) {
    char cpm_str[16];
    sprintf(cpm_str, "%d", cpm);
    mqtt_send(config_mqtt_topic, cpm_str);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_fault(const char *type) {
    printf("reader: fault: %s\n", type);
    if (serial_reconnect())
        device_info_display();
}

void process_readings(void) {
    printf("reader: reading CPM every %d seconds, publishing to MQTT '%s'\n", config_read_period, config_mqtt_topic);
    int cpm = 0;
    while (running) {
        if (!serial_check() || !serial_connected())
            process_fault("device disconnected");
        else if (!device_getcpm_request())
            process_fault("device write error");
        else if (!device_getcpm_read(&cpm) || !cpm_is_reasonable(cpm))
            process_fault("short or faulty data, device probably disconnected");
        else {
            cpm_display(cpm);
            cpm_publish(cpm);
            sleep(config_read_period);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void cleanup(void) {
    running = false;
    serial_end();
    mqtt_end();
}

void signal_handler(int sig __attribute__((unused))) {
    printf("gqgmctomqtt: terminating\n");
    cleanup();
    exit(EXIT_SUCCESS);
}

int main(int argc, const char **argv) {
    setbuf(stdout, NULL);
    printf("gqgmctomqtt: starting\n");
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    if (!config_load(argc, argv)) {
        fprintf(stderr, "gqgmctomqtt: failed to load config\n");
        return EXIT_FAILURE;
    }
    if (serial_begin()) {
        if (mqtt_begin()) {
            device_info_display();
            process_readings();
        }
    }
    cleanup();
    return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
