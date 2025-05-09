
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <mosquitto.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    const char *server;
    const char *client;
    bool debug;
} MqttConfig;

typedef struct {
    void (*message_processor)(const char *);
} mqtt_callback_data;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef MQTT_CONNECT_TIMEOUT
#define MQTT_CONNECT_TIMEOUT 60
#endif
#ifndef MQTT_PUBLISH_QOS
#define MQTT_PUBLISH_QOS 0
#endif
#ifndef MQTT_PUBLISH_RETAIN
#define MQTT_PUBLISH_RETAIN false
#endif
#ifndef MQTT_SUBSCRIBE_QOS
#define MQTT_SUBSCRIBE_QOS 0
#endif

bool mosq_debug = false;
struct mosquitto *mosq = NULL;
mqtt_callback_data *mosq_callback_data = NULL;

bool mqtt_parse(const char *string, char *host, const int length, int *port, bool *ssl) {
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

bool mqtt_begin(const MqttConfig *config) {
    char host[128];
    int port;
    bool ssl;
    mosq_debug = config->debug;
    if (!mqtt_parse(config->server, host, sizeof(host), &port, &ssl)) {
        fprintf(stderr, "mqtt: error parsing details in '%s'\n", config->server);
        return false;
    }
    printf("mqtt: connecting (host='%s', port=%d, ssl=%s, client='%s')\n", host, port, ssl ? "true" : "false",
           config->client);
    char client_id[24];
    sprintf(client_id, "%s-%06X", config->client ? config->client : "mqtt-linux", rand() & 0xFFFFFF);
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
    if (mosq_callback_data) {
        free(mosq_callback_data);
        mosq_callback_data = NULL;
    }
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

void mqtt_send(const char *topic, const char *message, const int length) {
    if (!mosq)
        return;
    const int result = mosquitto_publish(mosq, NULL, topic, length, message, MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN);
    if (result != MOSQ_ERR_SUCCESS)
        fprintf(stderr, "mqtt: publish error: %s\n", mosquitto_strerror(result));
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void mqtt_message_callback(struct mosquitto *m, void *obj, const struct mosquitto_message *message) {
    if (m != mosq)
        return;
    const mqtt_callback_data *callback_data = (mqtt_callback_data *)obj;
    if (callback_data && callback_data->message_processor)
        callback_data->message_processor(message->topic);
}

void mqtt_subscribe_callback(struct mosquitto *m, void *obj __attribute__((unused)), int mid,
                             int qos_count __attribute__((unused)), const int *qos_granted __attribute__((unused))) {
    if (m != mosq)
        return;
    if (mosq_debug)
        printf("mqtt: subscribed (mid=%d)\n", mid);
}

bool mqtt_subscribe(const char *topic) {
    if (!mosq)
        return false;
    const int result = mosquitto_subscribe(mosq, NULL, topic, MQTT_SUBSCRIBE_QOS);
    if (result != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "mqtt: subscribe failed '%s': %s\n", topic, mosquitto_strerror(result));
        return false;
    }
    printf("mqtt: subscribed '%s' (QoS %d)\n", topic, MQTT_SUBSCRIBE_QOS);
    return true;
}

bool mqtt_unsubscribe(const char *topic) {
    if (!mosq)
        return false;
    mosquitto_unsubscribe(mosq, NULL, topic);
    return true;
}

bool mqtt_message_callback_register(void (*message_processor)(const char *)) {
    if (!mosq)
        return false;
    if (mosq_callback_data)
        free(mosq_callback_data);
    mosq_callback_data = malloc(sizeof(mqtt_callback_data));
    if (!mosq_callback_data) {
        fprintf(stderr, "mqtt: failed to allocate memory for callback data\n");
        return false;
    }
    mosq_callback_data->message_processor = message_processor;
    mosquitto_user_data_set(mosq, mosq_callback_data);
    mosquitto_message_callback_set(mosq, mqtt_message_callback);
    mosquitto_subscribe_callback_set(mosq, mqtt_subscribe_callback);
    return true;
}

void mqtt_message_callback_cancel(void) {
    if (!mosq)
        return;
    mosquitto_user_data_set(mosq, NULL);
    if (mosq_callback_data) {
        free(mosq_callback_data);
        mosq_callback_data = NULL;
    }
    mosquitto_message_callback_set(mosq, NULL);
    mosquitto_subscribe_callback_set(mosq, NULL);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
