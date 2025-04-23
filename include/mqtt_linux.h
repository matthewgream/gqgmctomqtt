
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <mosquitto.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

struct mosquitto *mosq = NULL;

void mqtt_send(const char *topic, const char *message, const int length) {
    if (!mosq)
        return;
    const int result = mosquitto_publish(mosq, NULL, topic, length, message, MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN);
    if (result != MOSQ_ERR_SUCCESS)
        fprintf(stderr, "mqtt: publish error: %s\n", mosquitto_strerror(result));
}

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

bool mqtt_begin(const char *server, const char *client) {
    char host[256];
    int port;
    bool ssl;
    if (!mqtt_parse(server, host, sizeof(host), &port, &ssl)) {
        fprintf(stderr, "mqtt: error parsing details in '%s'\n", server);
        return false;
    }
    printf("mqtt: connecting (host='%s', port=%d, ssl=%s, client='%s')\n", host, port, ssl ? "true" : "false", client);
    char client_id[24];
    sprintf(client_id, "%s-%06X", client ? client : "mqtt-linux", rand() & 0xFFFFFF);
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
