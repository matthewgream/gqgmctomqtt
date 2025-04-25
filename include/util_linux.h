
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <arpa/inet.h>
#include <ctype.h>
#include <stdint.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

uint32_t __unpack_h(const uint8_t *x) { return (x[0] << 8) | x[1]; }
float __unpack_f(const uint8_t *x) {
    union {
        uint32_t i;
        float f;
    } u;
    memcpy(&u.i, x, sizeof(uint32_t));
    u.i = ntohl(u.i);
    return u.f;
}

time_t intervalable(const time_t interval, time_t *last) {
    time_t now = time(NULL);
    if (*last == 0) {
        *last = now;
        return 0;
    }
    if ((now - *last) > interval) {
        const time_t diff = now - *last;
        *last = now;
        return diff;
    }
    return 0;
}

void hexdump(const unsigned char *data, const int size, const char *prefix) {
    static const int bytes_per_line = 16;
    for (int offset = 0; offset < size; offset += bytes_per_line) {
        printf("%s%04x: ", prefix, offset);
        for (int i = 0; i < bytes_per_line; i++) {
            if (i == bytes_per_line / 2)
                printf(" ");
            if (offset + i < size)
                printf("%02x ", data[offset + i]);
            else
                printf("   ");
        }
        printf(" ");
        for (int i = 0; i < bytes_per_line; i++) {
            if (i == bytes_per_line / 2)
                printf(" ");
            if (offset + i < size)
                printf("%c", isprint(data[offset + i]) ? data[offset + i] : '.');
            else
                printf(" ");
        }
        printf("\n");
    }
}

bool is_reasonable_json(const unsigned char *packet, const int length) {
    if (length < 2)
        return false;
    if (packet[0] != '{' || packet[length - 1] != '}')
        return false;
    for (int index = 0; index < length; index++)
        if (!isprint(packet[index]))
            return false;
    return true;
}

#define EMA_ALPHA 0.2f
void ema_update(unsigned char value, unsigned char *value_ema, unsigned long *value_cnt) {
    *value_ema =
        (*value_cnt)++ == 0 ? value : (unsigned char)((EMA_ALPHA * (float)value) + ((1.0f - EMA_ALPHA) * (*value_ema)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
