
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <ctype.h>
#include <getopt.h>

#define CONFIG_MAX_STRING 255
#ifndef CONFIG_MAX_ENTRIES
#define CONFIG_MAX_ENTRIES 32
#endif

typedef struct {
    char *key;
    char *value;
} config_entry_t;

config_entry_t config_entries[CONFIG_MAX_ENTRIES];
int config_entry_count = 0;

void __config_set_value(const char *key, const char *value) {
    for (int i = 0; i < config_entry_count; i++)
        if (strcmp(config_entries[i].key, key) == 0) {
            free(config_entries[i].value);
            config_entries[i].value = strdup(value);
            return;
        }
    if (config_entry_count < CONFIG_MAX_ENTRIES) {
        config_entries[config_entry_count].key = strdup(key);
        config_entries[config_entry_count].value = strdup(value);
        config_entry_count++;
    } else
        fprintf(stderr, "config: too many entries, ignoring %s=%s\n", key, value);
}

const char *config_get_string(const char *key, const char *default_value) {
    for (int i = 0; i < config_entry_count; i++)
        if (strcmp(config_entries[i].key, key) == 0)
            return config_entries[i].value;
    return default_value;
}

int config_get_integer(const char *key, const int default_value) {
    for (int i = 0; i < config_entry_count; i++)
        if (strcmp(config_entries[i].key, key) == 0) {
            char *endptr;
            const long val = strtol(config_entries[i].value, &endptr, 0);
            if (*endptr == '\0')
                return (int)val;
            else {
                fprintf(stderr, "config: invalid integer value '%s' for key '%s', using default\n",
                        config_entries[i].value, key);
                return default_value;
            }
        }
    return default_value;
}

bool config_get_bool(const char *key, const bool default_value) {
    for (int i = 0; i < config_entry_count; i++)
        if (strcmp(config_entries[i].key, key) == 0) {
            if (strcasecmp(config_entries[i].value, "true") == 0 || strcmp(config_entries[i].value, "1") == 0)
                return true;
            else if (strcasecmp(config_entries[i].value, "false") == 0 || strcmp(config_entries[i].value, "0") == 0)
                return false;
            fprintf(stderr, "config: invalid boolean value '%s' for key '%s', using default\n", config_entries[i].value,
                    key);
        }
    return default_value;
}

serial_bits_t config_get_bits(const char *key, const serial_bits_t default_value) {
    for (int i = 0; i < config_entry_count; i++)
        if (strcmp(config_entries[i].key, key) == 0) {
            if (strcmp(config_entries[i].value, "8N1") == 0)
                return SERIAL_8N1;
            fprintf(stderr, "config: invalid bits value '%s', using default\n", config_entries[i].value);
        }
    return default_value;
}

bool is_empty_or_comment(const char *line) {
    if (*line == '\0')
        return true;
    while ((*line == ' ' || *line == '\t') && *line != '\0')
        if (*line++ == '#')
            return true;
    return false;
}

void __config_load_file(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "config: could not load '%s'\n", filename);
        return;
    }
    char line[CONFIG_MAX_STRING];
    while (fgets(line, sizeof(line), file)) {
        if (is_empty_or_comment(line))
            continue;
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
            __config_set_value(key, value);
        }
    }
    fclose(file);
}

bool config_load(const char *config_file, const int argc, const char *argv[], const struct option *options_long) {
    int c;
    int option_index = 0;
    optind = 0;
    while ((c = getopt_long(argc, (char **)argv, "", options_long, &option_index)) != -1) {
        if (c == 0)
            if (strcmp(options_long[option_index].name, "config") == 0) {
                config_file = optarg;
                break;
            }
    }
    __config_load_file(config_file);
    optind = 0;
    while ((c = getopt_long(argc, (char **)argv, "", options_long, &option_index)) != -1) {
        if (c == 0)
            if (strcmp(options_long[option_index].name, "config") != 0)
                __config_set_value(options_long[option_index].name, optarg);
    }
    printf("config: loaded from '%s' and command line\n", config_file);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
