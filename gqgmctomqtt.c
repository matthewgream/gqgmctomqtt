
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
 * READ_PERIOD=5
 * MQTT_SERVER=mqtt://localhost
 * MQTT_TOPIC=sensors/radiation
 * GMCMAP_USER_ID=12345
 * GMCMAP_COUNTER_ID=12345678901
 *
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS 0
#define MQTT_PUBLISH_RETAIN false

#include "include/mqtt_linux.h"

#include "include/http_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CONFIG_FILE_DEFAULT "gqgmctomqtt.cfg"

#define MQTT_SERVER_DEFAULT "mqtt://localhost"
#define MQTT_TOPIC_DEFAULT "sensors/radiation"
#define SERIAL_PORT_DEFAULT "/dev/ttyUSB0"
#define SERIAL_RATE_DEFAULT 115200
#define READ_PERIOD_DEFAULT 30

volatile bool running = true;

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS 0
#define MQTT_PUBLISH_RETAIN false

#define GMCMAP_PUBLISH_PERIOD 60

#define SERIAL_READ_TIMEOUT 1000

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

unsigned short __unpack_h(const unsigned char *x) { return (x[0] << 8) | x[1]; }

float __unpack_f(const unsigned char *x) {
    union {
        uint32_t i;
        float f;
    } u;
    memcpy(&u.i, x, sizeof(uint32_t));
    u.i = ntohl(u.i);
    return u.f;
}

bool intervalable(const time_t period, time_t *previous) {
    const time_t current = time(NULL);
    if (current >= (*previous + period)) {
        *previous = current;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CONFIG_MAX_LINE 256
#define CONFIG_MAX_VALUE 128

const char *config_file = CONFIG_FILE_DEFAULT;
char config_mqtt_server[CONFIG_MAX_VALUE] = MQTT_SERVER_DEFAULT;
char config_mqtt_topic[CONFIG_MAX_VALUE] = MQTT_TOPIC_DEFAULT;
char config_gmcmap_user_id[CONFIG_MAX_VALUE] = "";
char config_gmcmap_counter_id[CONFIG_MAX_VALUE] = "";
char config_serial_port[CONFIG_MAX_VALUE] = SERIAL_PORT_DEFAULT;
int config_serial_rate = SERIAL_RATE_DEFAULT;
int config_read_period = READ_PERIOD_DEFAULT;

bool config_load(const int argc, const char **argv) {
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
            if (strcmp(key, "SERIAL_PORT") == 0)
                strncpy(config_serial_port, value, sizeof(config_serial_port) - 1);
            else if (strcmp(key, "SERIAL_RATE") == 0)
                config_serial_rate = atoi(value);
            else if (strcmp(key, "READ_PERIOD") == 0)
                config_read_period = atoi(value);
            else if (strcmp(key, "MQTT_SERVER") == 0)
                strncpy(config_mqtt_server, value, sizeof(config_mqtt_server) - 1);
            else if (strcmp(key, "MQTT_TOPIC") == 0)
                strncpy(config_mqtt_topic, value, sizeof(config_mqtt_topic) - 1);
            else if (strcmp(key, "GMCMAP_USER_ID") == 0)
                strncpy(config_gmcmap_user_id, value, sizeof(config_gmcmap_user_id) - 1);
            else if (strcmp(key, "GMCMAP_COUNTER_ID") == 0)
                strncpy(config_gmcmap_counter_id, value, sizeof(config_gmcmap_counter_id) - 1);
        }
    }
    fclose(file);
    printf("config: '%s': serial=%s+%d, period=%d, mqtt=%s+%s, gmcmap=%s+%s\n", config_file, config_serial_port,
           config_serial_rate, config_read_period, config_mqtt_server, config_mqtt_topic, config_gmcmap_user_id,
           config_gmcmap_counter_id);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

int serial_fd = -1;

bool serial_check(void) { return (access(config_serial_port, F_OK) == 0); }

bool serial_connect(void) {
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
bool serial_connect_wait(volatile bool *running) {
    int counter = 0;
    while (*running) {
        if (serial_check()) {
            if (!serial_connect())
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

bool serial_write(const char *cmd, ssize_t len) {
    if (serial_fd < 0)
        return false;
    return write(serial_fd, cmd, len) == len;
}

int serial_read(unsigned char *buffer, const int length, const int timeout_ms) {
    if (serial_fd < 0)
        return -1;
    usleep(50 * 1000); // yuck
    fd_set rdset;
    struct timeval tv;
    FD_ZERO(&rdset);
    FD_SET(serial_fd, &rdset);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int select_result = select(serial_fd + 1, &rdset, NULL, NULL, &tv);
    if (select_result <= 0)
        return select_result; // timeout or error
    int bytes_read = 0;
    unsigned char byte;
    bool buffer_complete = false;
    while (bytes_read < length) {
        FD_ZERO(&rdset);
        FD_SET(serial_fd, &rdset);
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        if (select(serial_fd + 1, &rdset, NULL, NULL, &tv) <= 0) {
            buffer_complete = true;
            break;
        }
        if (read(serial_fd, &byte, 1) != 1)
            break;
        buffer[bytes_read++] = byte;
    }
    if (!buffer_complete && bytes_read > length) {
        fprintf(stderr, "device: buffer_read: buffer too large (max %d bytes, read %d bytes)\n", length, bytes_read);
        return -1;
    }
    return bytes_read;
}

void serial_flush(void) {
    if (serial_fd < 0)
        return;
    tcflush(serial_fd, TCIOFLUSH);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETMODEL_SIZE 15

bool device_getmodel(char *model, const int length) {
    const char command[] = "<GETVER>>";
    if (!serial_write(command, sizeof(command) - 1))
        return false;
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

bool device_getserial(char *serial, const int length) {
    const char command[] = "<GETSERIAL>>";
    if (!serial_write(command, sizeof(command) - 1))
        return false;
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

#define DEVICE_GETCFG_SIZE 512

bool device_getcfg(unsigned char *cfg, const int length) {
    const char command[] = "<GETCFG>>";
    if (!serial_write(command, sizeof(command) - 1))
        return false;
    unsigned char buffer[512];
    const int read_len = serial_read(buffer, sizeof(buffer), SERIAL_READ_TIMEOUT);
    if (read_len < (int)sizeof(buffer))
        return false;
    if (length < DEVICE_GETCFG_SIZE)
        return false;
    memcpy(cfg, buffer, sizeof(buffer));
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETDATETIME_SIZE 30

bool device_getdatetime(char *datetime, const int length) {
    const char command[] = "<GETDATETIME>>";
    if (!serial_write(command, sizeof(command) - 1))
        return false;
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

bool device_setdatetime(const unsigned char year, const unsigned char month, const unsigned char day,
                        const unsigned char hour, const unsigned char minute, const unsigned char second) {
    char command[sizeof("<SETDATETIMEymdhms>>")] = "<SETDATETIMEymdhms>>";
    unsigned char *datetime = (unsigned char *)strchr(command, 'y');
    *datetime++ = year;
    *datetime++ = month;
    *datetime++ = day;
    *datetime++ = hour;
    *datetime++ = minute;
    *datetime++ = second;
    if (!serial_write(command, sizeof(command) - 1))
        return false;
    unsigned char confirmation = 0;
    const int read_len = serial_read(&confirmation, 1, SERIAL_READ_TIMEOUT);
    if (read_len < 1)
        return false;
    if (confirmation != 0xAA)
        return false;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETVOLT_SIZE 5

bool device_getvolt(char *volts, const int length) {
    const char command[] = "<GETVOLT>>";
    if (!serial_write(command, sizeof(command) - 1))
        return false;
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
    const char command[] = "<GETCPM>>";
    return serial_write(command, sizeof(command) - 1);
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

bool device_time_check(const char *datetime) {
    struct tm tm = {0};
    if (sscanf(datetime, "%d/%d/%d %d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min,
               &tm.tm_sec) == 6) {
        tm.tm_year -= 1900;
        tm.tm_mon -= 1;
        tm.tm_isdst = -1;
        const double diff_seconds = difftime(time(NULL), mktime(&tm));
        if (fabs(diff_seconds) > MAXIMUM_TIME_DRIFT) {
            printf("WARNING: Device time differs from system time by %.1f minutes!\n", fabs(diff_seconds) / 60.0);
            return false;
        }
    }
    return true;
}

bool device_time_write(void) {
    const time_t now = time(NULL);
    const struct tm *tm = localtime(&now);
    if (!device_setdatetime(tm->tm_year - 100, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec)) {
        printf("WARNING: Device time could not be written\n");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool device_info_display(void) {
    serial_flush();
    char model[DEVICE_GETMODEL_SIZE], serial[DEVICE_GETSERIAL_SIZE], datetime[DEVICE_GETDATETIME_SIZE],
        volt[DEVICE_GETVOLT_SIZE];
    if (!device_getmodel(model, sizeof(model)) || !device_getserial(serial, sizeof(serial)))
        return false;
    if (!device_getdatetime(datetime, sizeof(datetime)))
        return false;
    if (!device_getvolt(volt, sizeof(volt)))
        return false;
    printf("device: model='%.8s/%.6s', serial='%s', datetime='%s', volt='%s'\n", model, model + 8, serial, datetime,
           volt);
    if (!device_time_check(datetime))
        device_time_write();
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define ADDRESS_CALIBRATE1_CPM 0x08
#define ADDRESS_CALIBRATE1_SV 0x0a
#define ADDRESS_CALIBRATE2_CPM 0x0e
#define ADDRESS_CALIBRATE2_SV 0x10
#define ADDRESS_CALIBRATE3_CPM 0x14
#define ADDRESS_CALIBRATE3_SV 0x16

typedef struct {
    unsigned short cal1_cpm, cal2_cpm, cal3_cpm;
    float cal1_sv, cal2_sv, cal3_sv;
} device_cfg_t;

device_cfg_t device_cfg;

bool device_cfg_process_and_display(void) {
    serial_flush();
    unsigned char cfg[DEVICE_GETCFG_SIZE];
    if (!device_getcfg(cfg, sizeof(cfg)))
        return false;
    device_cfg.cal1_cpm = __unpack_h(&cfg[ADDRESS_CALIBRATE1_CPM]);
    device_cfg.cal2_cpm = __unpack_h(&cfg[ADDRESS_CALIBRATE2_CPM]);
    device_cfg.cal3_cpm = __unpack_h(&cfg[ADDRESS_CALIBRATE3_CPM]);
    device_cfg.cal1_sv = __unpack_f(&cfg[ADDRESS_CALIBRATE1_SV]);
    device_cfg.cal2_sv = __unpack_f(&cfg[ADDRESS_CALIBRATE2_SV]);
    device_cfg.cal3_sv = __unpack_f(&cfg[ADDRESS_CALIBRATE3_SV]);
    printf("device: config: cal1=(%u cpm: %.6f Sv/h), cal2=(%u cpm: %.6f Sv/h), cal3=(%u cpm: %.6f Sv/h)\n",
           device_cfg.cal1_cpm, device_cfg.cal1_sv, device_cfg.cal2_cpm, device_cfg.cal2_sv, device_cfg.cal3_cpm,
           device_cfg.cal3_sv);
    return true;
}

void device_get_conversion_factor(int *ref_cpm, double *ref_usv) {
    const double cal1_factor = device_cfg.cal1_sv * 1000.0 / (double)device_cfg.cal1_cpm;
    const double cal2_factor = device_cfg.cal2_sv * 1000.0 / (double)device_cfg.cal2_cpm;
    const double cal3_factor = device_cfg.cal3_sv * 1000.0 / (double)device_cfg.cal3_cpm;
    *ref_cpm = 1000;
    *ref_usv = (cal1_factor + cal2_factor + cal3_factor) / 3.0;
}

double device_convert_cpm_to_usievert(const int cpm) {
    int ref_cpm;
    double ref_usv;
    device_get_conversion_factor(&ref_cpm, &ref_usv);
    return (double)cpm * ref_usv / (double)ref_cpm;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    int cpm;
    double acpm;
    double usvh;
} readings_t;

typedef struct {
    time_t timestamp;
    int cpm;
} sample_t;

static struct {
    int sample_period, sample_count;
    int history_index, history_count;
    sample_t *history_store;
    time_t last_update_time;
} state;

void readings_begin(const int period, const int count) {
    state.sample_period = period;
    state.sample_count = count;
    state.history_index = state.history_count = 0;
    state.history_store = (sample_t *)malloc(sizeof(sample_t) * count);
    state.last_update_time = time(NULL);
    printf("sample: period=%d, count=%d\n", period, count);
}
void readings_end(void) { free(state.history_store); }

readings_t readings_update(const int cpm) {
    readings_t r;
    const time_t current_time = time(NULL);
    const sample_t sample = {.timestamp = current_time, .cpm = cpm};
    state.history_store[state.history_index] = sample;
    state.history_index = (state.history_index + 1) % state.sample_count;
    if (state.history_count < state.sample_count)
        state.history_count++;
    double total_weighted_count = 0.0, total_weight = 0.0;
    for (int i = 0; i < state.history_count; i++) {
        const sample_t *sample =
            &state.history_store[(state.history_index - 1 - i + state.sample_count) % state.sample_count];
        if (sample->timestamp >= (current_time - state.sample_period)) {
            double weight = 1.0;
            total_weighted_count += (double)sample->cpm * weight;
            total_weight += weight;
        }
    }
    r.cpm = cpm;
    r.acpm = (total_weight > 0) ? (total_weighted_count / total_weight) : cpm;
    r.usvh = device_convert_cpm_to_usievert(cpm);
    state.last_update_time = current_time;
    return r;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool cpm_is_reasonable(const int cpm) { return (cpm >= 0 && cpm <= 5000); }

void cpm_display(const readings_t *readings) {
    char timestamp[20];
    const time_t now = time(NULL);
    strftime(timestamp, sizeof(timestamp), "%Y/%m/%d-%H:%M:%S", localtime(&now));
    printf("reader: CPM=%d, ACPM=%.2f, uSv/h=%.2f [%s]\n", readings->cpm, readings->acpm, readings->usvh, timestamp);
}

void cpm_publish_mqtt(const readings_t *readings, const char *mqtt_topic) {
    char cpm_topic[CONFIG_MAX_VALUE], cpm_value[16];
    sprintf(cpm_topic, "%s/cpm", mqtt_topic);
    sprintf(cpm_value, "%d", readings->cpm);
    mqtt_send(cpm_topic, cpm_value, strlen(cpm_value));
}

bool __publish_gmcmap(const char *user, const char *device, const int cpm, const double acpm, const double usvh) {
    char url[256], buf[BUF_SIZE];
    sprintf(url, "/log2.asp?AID=%s&GID=%s&CPM=%d&ACPM=%.2f&uSV=%.2f", user, device, cpm, acpm, usvh);
    return http_get("www.gmcmap.com", "80", url, buf, sizeof(buf)) && strstr(buf, "OK.") != NULL;
}
void cpm_publish_gmcmap(const readings_t *readings, const char *gmcmap_user_id, const char *gmcmap_counter_id) {
    static time_t last_publish_time = 0;
    if (intervalable(GMCMAP_PUBLISH_PERIOD, &last_publish_time))
        if (!__publish_gmcmap(gmcmap_user_id, gmcmap_counter_id, readings->cpm, readings->acpm, readings->usvh))
            printf("WARNING: publish to gmcmap failed\n");
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_fault(const char *type) {
    printf("reader: fault: %s\n", type);
    printf("reader: device reconnect\n");
    serial_disconnect();
    if (serial_connect_wait(&running)) {
        device_info_display();
        printf("reader: device reconnected\n");
    }
}

#define PROCESS_READINGS_SAMPLE_PERIOD 60 * 60

void process_readings(void) {

    const bool publish_mqtt = (strlen(config_mqtt_topic) > 0),
               publish_gmcmap = (strlen(config_gmcmap_user_id) > 0 && strlen(config_gmcmap_counter_id) > 0);

    printf("reader: reading CPM every %d seconds", config_read_period);
    if (publish_mqtt)
        printf(", publishing to MQTT:'%s'", config_mqtt_topic);
    if (publish_gmcmap)
        printf("%s GMCMAP:'%s/%s'", publish_mqtt ? "," : ", publishig to", config_gmcmap_user_id,
               config_gmcmap_counter_id);
    printf("\n");

    readings_begin(PROCESS_READINGS_SAMPLE_PERIOD, (PROCESS_READINGS_SAMPLE_PERIOD / config_read_period) * 1.25);
    int cpm = 0;
    while (running) {
        if (!serial_check() || !serial_connected())
            process_fault("device disconnected");
        else if (!device_getcpm_request())
            process_fault("device write error");
        else if (!device_getcpm_read(&cpm) || !cpm_is_reasonable(cpm))
            process_fault("short or faulty data, device probably disconnected");
        else {
            const readings_t readings = readings_update(cpm);
            cpm_display(&readings);
            if (publish_mqtt)
                cpm_publish_mqtt(&readings, config_mqtt_topic);
            if (publish_gmcmap)
                cpm_publish_gmcmap(&readings, config_gmcmap_user_id, config_gmcmap_counter_id);
            sleep(config_read_period);
        }
    }
    readings_end();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void cleanup(void) {
    running = false;
    serial_disconnect();
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
    if (!serial_connect_wait(&running)) {
        fprintf(stderr, "gqgmctomqtt: failed to connect serial\n");
        return EXIT_FAILURE;
    }
    if (!mqtt_begin(config_mqtt_server, "sensor-radiation")) {
        fprintf(stderr, "gqgmctomqtt: failed to begin mqtt\n");
        serial_disconnect();
        return EXIT_FAILURE;
    }

    device_info_display();
    device_cfg_process_and_display();
    process_readings();

    cleanup();
    return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
