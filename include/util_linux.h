
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

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
