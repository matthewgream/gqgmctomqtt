
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * GQ Electronics GMC to MQTT (C Version)
 *
 * This program connects to a GQ GMC Geiger Counter via serial port and
 * publishes readings to MQTT. It uses the same protocol as the Node.js version
 * but is implemented in C for lower memory usage.
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <mosquitto.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define READ_PERIOD 5

bool running = true;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CONFIG_FILE_DEFAULT "secrets.txt"
#define MAX_CONFIG_LINE 256
#define MAX_CONFIG_VALUE 128

char config_mqtt_broker[MAX_CONFIG_VALUE] = "";
char config_mqtt_topic[MAX_CONFIG_VALUE] = "";
char config_serial_port[MAX_CONFIG_VALUE] = "";
int config_serial_baud = 115200;

bool config_load(int argc, const char **argv) {
    const char *path = CONFIG_FILE_DEFAULT;
    if (argc > 1)
        path = argv[1];
    FILE *file = fopen(path, "r");
    if (file == NULL) {
        fprintf(stderr,
                "config: could not load '%s', using defaults (which may not work "
                "correctly)\n",
                path);
        return false;
    }
    char line[MAX_CONFIG_LINE];
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
            if (strcmp(key, "MQTT") == 0) {
                strncpy(config_mqtt_broker, value, sizeof(config_mqtt_broker) - 1);
            } else if (strcmp(key, "MQTT_TOPIC") == 0) {
                strncpy(config_mqtt_topic, value, sizeof(config_mqtt_topic) - 1);
            } else if (strcmp(key, "PORT") == 0) {
                strncpy(config_serial_port, value, sizeof(config_serial_port) - 1);
            } else if (strcmp(key, "RATE") == 0) {
                config_serial_baud = atoi(value);
            }
        }
    }
    fclose(file);
    printf("config: '%s': mqtt=%s, config_mqtt_topic=%s, port=%s, rate=%d\n", path, config_mqtt_broker,
           config_mqtt_topic, config_serial_port, config_serial_baud);
    return (config_mqtt_broker[0] != '\0' && config_mqtt_topic[0] != '\0' && config_serial_port[0] != '\0');
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

int serial_fd = -1;

bool serial_check(void) { return (access(config_serial_port, F_OK) == 0); }

bool serial_configure(void) {
    printf("device: configure %s at %d baud\n", config_serial_port, config_serial_baud);
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
    switch (config_serial_baud) {
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
        baud = B57600;
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

bool serial_connect(void) {
    int counter = 0;
    while (running) {
        if (serial_check()) {
            if (!serial_configure())
                return false;
            printf("device: connected\n");
            return true;
        }
        if (counter++ % 6 == 0)
            printf("device: connection pending\n");
        sleep(5);
    }
    return false;
}

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

bool serial_write(const char *cmd, size_t len) {
    if (serial_fd < 0)
        return false;
    ssize_t bytes_written = write(serial_fd, cmd, len);
    return (bytes_written == len);
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
    int result = mosquitto_publish(mosq, NULL, topic, strlen(message), message, 0, false);
    if (result != MOSQ_ERR_SUCCESS)
        fprintf(stderr, "mqtt: publish error: %s\n", mosquitto_strerror(result));
}

void mqtt_connect_callback(struct mosquitto *mosq, void *obj, int result) {
    if (result != 0) {
        fprintf(stderr, "mqtt: connect failed: %s\n", mosquitto_connack_string(result));
        return;
    }
    printf("mqtt: connected\n");
}

bool mqtt_begin(void) {
    char host[MAX_CONFIG_VALUE] = "";
    int port = 1883;
    bool use_ssl = false;
    if (strncmp(config_mqtt_broker, "mqtt://", 7) == 0) {
        strncpy(host, config_mqtt_broker + 7, sizeof(host) - 1);
    } else if (strncmp(config_mqtt_broker, "mqtts://", 8) == 0) {
        strncpy(host, config_mqtt_broker + 8, sizeof(host) - 1);
        use_ssl = true;
        port = 8883;
    } else {
        strncpy(host, config_mqtt_broker, sizeof(host) - 1);
    }
    char *port_str = strchr(host, ':');
    if (port_str) {
        *port_str = '\0'; // Terminate host string at colon
        port = atoi(port_str + 1);
    }
    printf("mqtt: connecting to '%s' (host='%s', port=%d, ssl=%s)\n", config_mqtt_broker, host, port,
           use_ssl ? "true" : "false");
    char client_id[24];
    sprintf(client_id, "sensor-radiation-%06X", rand() & 0xFFFFFF);
    mosquitto_lib_init();
    mosq = mosquitto_new(client_id, true, NULL);
    if (!mosq) {
        fprintf(stderr, "mqtt: error creating client instance\n");
        return false;
    }
    if (use_ssl)
        mosquitto_tls_insecure_set(mosq, true); // Skip certificate validation
    mosquitto_connect_callback_set(mosq, mqtt_connect_callback);
    if (mosquitto_connect(mosq, host, port, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "mqtt: error connecting to broker\n");
        mosquitto_destroy(mosq);
        mosq = NULL;
        return false;
    }
    if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "mqtt: error starting loop\n");
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = NULL;
        return false;
    }
    return true;
}

void mqtt_end(void) {
    if (!mosq)
        return;
    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosq = NULL;
    mosquitto_lib_cleanup();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void display_device_info(void) {
    if (serial_fd < 0)
        return;
    serial_flush();
    const char *cmd_ver = "<GETVER>>";
    if (!serial_write(cmd_ver, strlen(cmd_ver)))
        return;
    unsigned char model_buf[15];
    int read_len = serial_read(model_buf, sizeof(model_buf), 1000);
    if (read_len < 14)
        return;
    char model[15] = {0};
    for (int i = 0; i < 14; i++)
        model[i] = (model_buf[i] >= 0x20 && model_buf[i] <= 0x7E) ? model_buf[i] : '.';
    const char *cmd_serial = "<GETSERIAL>>";
    if (!serial_write(cmd_serial, strlen(cmd_serial)))
        return;
    unsigned char serial_buf[8];
    read_len = serial_read(serial_buf, sizeof(serial_buf), 1000);
    if (read_len < 7)
        return;
    char serial[22] = {0};
    for (int i = 0; i < 7; i++)
        sprintf(serial + i * 3, "%02X ", serial_buf[i]);
    serial[20] = '\0';
    printf("device: model='%.8s', revision='%.6s', serial='%s'\n", model, model + 8, serial);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool device_cpm_request(void) {
    const char *cmd = "<GETCPM>>";
    return serial_write(cmd, strlen(cmd));
}

int device_cpm_read(void) {
    unsigned char data[5];
    int read_len = serial_read(data, sizeof(data), 1000);
    if (read_len < 4)
        return -1;
    return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_readings(void) {
    printf("reader: reading CPM every %d seconds, publishing to MQTT '%s'\n", READ_PERIOD, config_mqtt_topic);

    while (running) {
        if (!serial_check() || serial_fd < 0) {
            printf("device: disconnected\n");
            if (!serial_reconnect()) {
                sleep(READ_PERIOD);
                continue;
            }
            display_device_info();
        }
        if (!device_cpm_request()) {
            printf("device: write error\n");
            serial_reconnect();
            display_device_info();
            sleep(READ_PERIOD);
            continue;
        }
        int cpm = device_cpm_read();
        if (cpm != -1 && cpm < 100000) {
            time_t now;
            time(&now);
            struct tm *timeinfo = localtime(&now);
            char timestamp[20];
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
            printf("reader: CPM=%d [%s]\n", cpm, timestamp);
            char cpm_str[16];
            sprintf(cpm_str, "%d", cpm);
            mqtt_send(config_mqtt_topic, cpm_str);
        } else {
            printf("device: faulty data, probably disconnected\n");
            serial_reconnect();
            display_device_info();
        }
        sleep(READ_PERIOD);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void cleanup(void) {
    running = false;
    serial_end();
    mqtt_end();
}

void signal_handler(int sig) {
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
            display_device_info();
            process_readings();
        }
    }
    cleanup();
    return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
