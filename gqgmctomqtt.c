
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * GQ Electronics GMC to MQTT/GMCMAP
 *
 * This program connects to a GQ GMC Geiger Counter via serial port and publishes readings to MQTT/GMCMAP.
 *
 * Implemented for https://www.gqelectronicsllc.com/download/GQ-RFC1801.txt. To support devices that use
 * https://www.gqelectronicsllc.com/download/GQ-RFC1201.txt, the GETVOLT command needs modification to read 1 byte
 * rather than a string, and the GETCPM command needs modification to read 2 bytes rather than 4 bytes.
 *
 * Tested on a 'GMC-500+/Re 2.5' on debian 6.1.
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <arpa/inet.h>
#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "include/util_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CONFIG_FILE_DEFAULT "gqgmctomqtt.cfg"

#define SERIAL_PORT_DEFAULT "/dev/ttyUSB0"
#define SERIAL_RATE_DEFAULT 115200
#define SERIAL_BITS_DEFAULT SERIAL_8N1

#define READ_PERIOD_DEFAULT 30

#define MQTT_SERVER_DEFAULT "mqtt://localhost"
#define MQTT_CLIENT_DEFAULT "gqgmctomqtt"
#define MQTT_TOPIC_DEFAULT "sensors/radiation"

#define GMCMAP_USER_ID_DEFAULT ""
#define GMCMAP_COUNTER_ID_DEFAULT ""

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define SERIAL_READ_TIMEOUT 1000

#include <stdarg.h>

void printf_stdout(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}
void printf_stderr(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
}

#define PRINTF_ERROR printf_stderr
#define PRINTF_INFO printf_stdout

#include "include/serial_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS 0
#define MQTT_PUBLISH_RETAIN false

#include "include/mqtt_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define GMCMAP_PUBLISH_PERIOD 60

#include "include/http_linux.h"

bool gmcmap_send(const char *user, const char *device, const int cpm, const double acpm, const double usvh) {
    char url[256], buf[BUF_SIZE];
    sprintf(url, "/log2.asp?AID=%s&GID=%s&CPM=%d&ACPM=%.2f&uSV=%.2f", user, device, cpm, acpm, usvh);
    return http_get("www.gmcmap.com", "80", url, buf, sizeof(buf)) && strstr(buf, "OK.") != NULL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "include/config_linux.h"

// clang-format off
const struct option config_options [] = {
    {"config",                required_argument, 0, 0},
    {"port",                  required_argument, 0, 0},
    {"rate",                  required_argument, 0, 0},
    {"bits",                  required_argument, 0, 0},
    {"read-period",           required_argument, 0, 0},
    {"mqtt-server",           required_argument, 0, 0},
    {"mqtt-client",           required_argument, 0, 0},
    {"mqtt-topic",            required_argument, 0, 0},
    {"gmcmap-user-id",        required_argument, 0, 0},
    {"gmcmap-counter-id",     required_argument, 0, 0},
    {0, 0, 0, 0}
};
// clang-format on

void config_populate_serial(serial_config_t *config) {
    config->port = config_get_string("port", SERIAL_PORT_DEFAULT);
    config->rate = config_get_integer("rate", SERIAL_RATE_DEFAULT);
    config->bits = config_get_bits("bits", SERIAL_BITS_DEFAULT);

    printf("config: serial: port=%s, rate=%d, bits=%s\n", config->port, config->rate, serial_bits_str(config->bits));
}

serial_config_t serial_config;
int read_period;
MqttConfig mqtt_config;
const char *mqtt_topic;
const char *gmcmap_user_id, *gmcmap_counter_id;

bool config(const int argc, const char *argv[]) {

    if (!config_load(CONFIG_FILE_DEFAULT, argc, argv, config_options))
        return false;

    config_populate_serial(&serial_config);

    read_period = config_get_integer("read-period", READ_PERIOD_DEFAULT);

    mqtt_config.server = config_get_string("mqtt-server", MQTT_SERVER_DEFAULT);
    mqtt_config.client = config_get_string("mqtt-client", MQTT_CLIENT_DEFAULT);
    mqtt_topic = config_get_string("mqtt-topic", MQTT_TOPIC_DEFAULT);

    gmcmap_user_id = config_get_string("gmcmap-user-id", GMCMAP_USER_ID_DEFAULT);
    gmcmap_counter_id = config_get_string("gmcmap-counter-id", GMCMAP_COUNTER_ID_DEFAULT);

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETMODEL_SIZE 15

bool device_getmodel(char *model, const int length) {
    const char command[] = "<GETVER>>";
    if (!serial_write_all((const unsigned char *)command, sizeof(command) - 1))
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

#define DEVICE_GETSERIAL_SIZE 21

bool device_getserial(char *serial, const int length) {
    const char command[] = "<GETSERIAL>>";
    if (!serial_write_all((const unsigned char *)command, sizeof(command) - 1))
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

#define CFG_OFFSERT_CALIBRATE1_CPM 0x08
#define CFG_OFFSERT_CALIBRATE1_SV 0x0a
#define CFG_OFFSERT_CALIBRATE2_CPM 0x0e
#define CFG_OFFSERT_CALIBRATE2_SV 0x10
#define CFG_OFFSERT_CALIBRATE3_CPM 0x14
#define CFG_OFFSERT_CALIBRATE3_SV 0x16

typedef struct {
    int cal1_cpm, cal2_cpm, cal3_cpm;
    float cal1_sv, cal2_sv, cal3_sv;
} device_conf_t;

#define DEVICE_GETCFG_SIZE 512

bool __device_getcfg(unsigned char *cfg, const int length) {
    const char command[] = "<GETCFG>>";
    if (!serial_write_all((const unsigned char *)command, sizeof(command) - 1))
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
bool device_getcfg(device_conf_t *config) {
    unsigned char cfg[DEVICE_GETCFG_SIZE];
    if (!__device_getcfg(cfg, sizeof(cfg)))
        return false;
    // hexdump(cfg, sizeof (cfg), "    ");
    config->cal1_cpm = (int)__unpack_h(&cfg[CFG_OFFSERT_CALIBRATE1_CPM]);
    config->cal2_cpm = (int)__unpack_h(&cfg[CFG_OFFSERT_CALIBRATE2_CPM]);
    config->cal3_cpm = (int)__unpack_h(&cfg[CFG_OFFSERT_CALIBRATE3_CPM]);
    config->cal1_sv = __unpack_f(&cfg[CFG_OFFSERT_CALIBRATE1_SV]);
    config->cal2_sv = __unpack_f(&cfg[CFG_OFFSERT_CALIBRATE2_SV]);
    config->cal3_sv = __unpack_f(&cfg[CFG_OFFSERT_CALIBRATE3_SV]);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

#define DEVICE_GETDATETIME_SIZE 30

bool device_getdatetime(char *datetime, const int length) {
    const char command[] = "<GETDATETIME>>";
    if (!serial_write_all((const unsigned char *)command, sizeof(command) - 1))
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

bool device_setdatetime(const unsigned char year, const unsigned char month, const unsigned char day,
                        const unsigned char hour, const unsigned char minute, const unsigned char second) {
    char command[sizeof("<SETDATETIMEymdhms>>")] = "<SETDATETIMEymdhms>>";
    unsigned char *datetime = (unsigned char *)strchr(command, 'y');
    if ((year < 25 || year > 75) || (month == 0 || month > 12) || (day == 0 || day > 31) || (hour > 23) ||
        (minute > 59) || (second > 59))
        return false;
    *datetime++ = year;
    *datetime++ = month;
    *datetime++ = day;
    *datetime++ = hour;
    *datetime++ = minute;
    *datetime++ = second;
    if (!serial_write_all((const unsigned char *)command, sizeof(command) - 1))
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

#define DEVICE_GETVOLT_SIZE 5

bool device_getvolt(char *volts, const int length) {
    const char command[] = "<GETVOLT>>";
    if (!serial_write_all((const unsigned char *)command, sizeof(command) - 1))
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

bool device_getcpm_request(void) {
    const char command[] = "<GETCPM>>";
    return serial_write_all((const unsigned char *)command, sizeof(command) - 1);
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

device_conf_t device_conf;
bool device_conf_load_and_display(void) {
    serial_flush();
    if (!device_getcfg(&device_conf))
        return false;
    printf("device: config: cal1=(%u cpm: %.6f Sv/h), cal2=(%u cpm: %.6f Sv/h), cal3=(%u cpm: %.6f Sv/h)\n",
           device_conf.cal1_cpm, device_conf.cal1_sv, device_conf.cal2_cpm, device_conf.cal2_sv, device_conf.cal3_cpm,
           device_conf.cal3_sv);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    int cpm_max;
    float slope, intercept;
} calibration_point_t;

typedef struct {
    calibration_point_t *points;
    size_t count;
} calibration_data_t;

void set_usvh_calibration(calibration_data_t *calib_data, const device_conf_t *cfg) {
    if (calib_data->points != NULL) {
        free(calib_data->points);
        calib_data->points = NULL;
        calib_data->count = 0;
    }
    struct {
        int cpm;
        float usv;
    } temp_points[4] = {
        {0, 0.0f}, {cfg->cal1_cpm, cfg->cal1_sv}, {cfg->cal2_cpm, cfg->cal2_sv}, {cfg->cal3_cpm, cfg->cal3_sv}};
    if ((calib_data->points = malloc(3 * sizeof(calibration_point_t))) != NULL) {
        calib_data->count = 0;
        for (int i = 0; i < 3; i++) {
            const int cpm_s = temp_points[i].cpm, cpm_e = temp_points[i + 1].cpm;
            const float usv_s = temp_points[i].usv, usv_e = temp_points[i + 1].usv;
            if (cpm_e > cpm_s) {
                const float m = (usv_e - usv_s) / ((float)(cpm_e - cpm_s));
                const float b = -(m * cpm_e) + usv_e;
                calib_data->points[calib_data->count].cpm_max = cpm_e;
                calib_data->points[calib_data->count].slope = m;
                calib_data->points[calib_data->count].intercept = b;
                calib_data->count++;
            }
        }
    }
}

float get_usvh_calibrated(const calibration_data_t *calib_data, const int cpm) {
    float usvh = -1.0f;
    if (calib_data->count > 0 && calib_data->points != NULL) {
        for (size_t i = 0; i < calib_data->count; i++)
            if (cpm <= calib_data->points[i].cpm_max) {
                usvh = cpm * calib_data->points[i].slope + calib_data->points[i].intercept;
                break;
            }
        if (usvh < 0 && calib_data->count > 0)
            usvh = cpm * calib_data->points[calib_data->count - 1].slope +
                   calib_data->points[calib_data->count - 1].intercept;
    }
    return usvh;
}

void calibration_begin(calibration_data_t *calib_data, const device_conf_t *device_conf) {
    calib_data->points = NULL;
    calib_data->count = 0;
    set_usvh_calibration(calib_data, device_conf);
}

void calibration_end(calibration_data_t *calib_data) {
    if (calib_data->points != NULL) {
        free(calib_data->points);
        calib_data->points = NULL;
    }
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

readings_t readings_update(const int cpm, const calibration_data_t *calib_data) {
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
    r.usvh = get_usvh_calibrated(calib_data, cpm);
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
    char json_payload[128];
    sprintf(json_payload, "{\"cpm\":%d,\"acpm\":%.2f,\"usvh\":%.2f}", readings->cpm, readings->acpm, readings->usvh);
    mqtt_send(mqtt_topic, json_payload, strlen(json_payload));
}

void cpm_publish_gmcmap(const readings_t *readings, const char *gmcmap_user_id, const char *gmcmap_counter_id) {
    static time_t last_publish_time = 0;
    if (intervalable(GMCMAP_PUBLISH_PERIOD, &last_publish_time))
        if (!gmcmap_send(gmcmap_user_id, gmcmap_counter_id, readings->cpm, readings->acpm, readings->usvh))
            printf("WARNING: publish to gmcmap failed\n");
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_fault(volatile bool *running, const char *type) {
    printf("reader: fault: %s\n", type);
    printf("reader: device reconnect\n");
    serial_disconnect();
    if (serial_connect_wait(running)) {
        device_info_display();
        printf("reader: device reconnected\n");
    }
}

void process_sleep(volatile bool *running) {
    for (int second = 0; second < read_period && *running; second++)
        sleep(1);
}

#define PROCESS_READINGS_SAMPLE_PERIOD 60 * 60

void process_readings(volatile bool *running) {

    const bool publish_mqtt = (strlen(mqtt_topic) > 0),
               publish_gmcmap = (strlen(gmcmap_user_id) > 0 && strlen(gmcmap_counter_id) > 0);

    printf("reader: reading CPM every %d seconds", read_period);
    if (publish_mqtt)
        printf(", publishing to MQTT:'%s'", mqtt_topic);
    if (publish_gmcmap)
        printf("%s GMCMAP:'%s/%s'", publish_mqtt ? "," : ", publishing to", gmcmap_user_id, gmcmap_counter_id);
    printf("\n");

    calibration_data_t calib_data;
    calibration_begin(&calib_data, &device_conf);
    readings_begin(PROCESS_READINGS_SAMPLE_PERIOD, (PROCESS_READINGS_SAMPLE_PERIOD / read_period) * 1.25);
    int cpm = 0;
    while (*running) {
        if (!serial_check() || !serial_connected())
            process_fault(running, "device disconnected");
        else if (!device_getcpm_request())
            process_fault(running, "device write error");
        else if (!device_getcpm_read(&cpm) || !cpm_is_reasonable(cpm))
            process_fault(running, "short or faulty data, device probably disconnected");
        else {
            const readings_t readings = readings_update(cpm, &calib_data);
            cpm_display(&readings);
            if (publish_mqtt)
                cpm_publish_mqtt(&readings, mqtt_topic);
            if (publish_gmcmap)
                cpm_publish_gmcmap(&readings, gmcmap_user_id, gmcmap_counter_id);
            process_sleep(running);
        }
    }
    readings_end();
    calibration_end(&calib_data);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

volatile bool running = true;

void signal_handler(int sig __attribute__((unused))) {
    if (running) {
        printf("stopping\n");
        running = false;
    }
}

int main(int argc, const char **argv) {
    setbuf(stdout, NULL);
    printf("starting\n");
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    if (!config(argc, argv)) {
        fprintf(stderr, "failed to load config\n");
        return EXIT_FAILURE;
    }
    if (!serial_begin(&serial_config) || !serial_connect_wait(&running)) {
        fprintf(stderr, "failed to begin/connect serial\n");
        serial_end();
        return EXIT_FAILURE;
    }
    if (!mqtt_begin(&mqtt_config)) {
        fprintf(stderr, "failed to begin mqtt\n");
        serial_end();
        return EXIT_FAILURE;
    }
    device_info_display();
    device_conf_load_and_display();
    process_readings(&running);
    mqtt_end();
    serial_end();
    return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
