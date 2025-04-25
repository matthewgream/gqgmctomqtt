
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <termios.h>
#include <unistd.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define SERIAL_CONNECT_CHECK_PERIOD 5
#define SERIAL_CONNECT_CHECK_PRINT 30

typedef enum {
    SERIAL_8N1 = 0,
} serial_bits_t;

const char *serial_bits_str(const serial_bits_t bits) {
    switch (bits) {
    case SERIAL_8N1:
        return "8N1";
    default:
        return "unknown";
    }
}

typedef struct {
    const char *port;
    int rate;
    serial_bits_t bits;
} serial_config_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

serial_config_t serial_config;

int serial_fd = -1;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool serial_check(void) { return (access(serial_config.port, F_OK) == 0); }

bool serial_connect(void) {
    serial_fd = open(serial_config.port, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        PRINTF_ERROR("serial: error opening port: %s\n", strerror(errno));
        return false;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        PRINTF_ERROR("serial: error getting port attributes: %s\n", strerror(errno));
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    speed_t baud;
    switch (serial_config.rate) {
    case 1200:
        baud = B1200;
        break;
    case 2400:
        baud = B2400;
        break;
    case 4800:
        baud = B4800;
        break;
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
        PRINTF_ERROR("serial: unsupported baud rate: %d\n", serial_config.rate);
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    if (serial_config.bits != SERIAL_8N1) {
        PRINTF_ERROR("serial: unsupported bits: %s\n", serial_bits_str(serial_config.bits));
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8-bit characters
    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST; // Raw output
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        PRINTF_ERROR("serial: error setting port attributes: %s\n", strerror(errno));
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    tcflush(serial_fd, TCIOFLUSH);
    return true;
}

void serial_disconnect(void) {
    if (serial_fd < 0)
        return;
    close(serial_fd);
    serial_fd = -1;
}

bool serial_connected(void) { return serial_fd >= 0; }

bool serial_connect_wait(volatile bool *running) {
    int counter = 0;
    while (*running) {
        if (serial_check()) {
            if (!serial_connect())
                return false;
            PRINTF_INFO("serial: connected\n");
            return true;
        }
        if (counter++ % (SERIAL_CONNECT_CHECK_PRINT / SERIAL_CONNECT_CHECK_PERIOD) == 0)
            PRINTF_INFO("serial: connection pending\n");
        sleep(SERIAL_CONNECT_CHECK_PERIOD);
    }
    return false;
}

void serial_flush(void) {
    if (serial_fd < 0)
        return;
    tcflush(serial_fd, TCIOFLUSH);
}

int serial_write(const unsigned char *buffer, const int length) {
    if (serial_fd < 0)
        return -1;
    usleep(50 * 1000); // yuck
    return (int)write(serial_fd, buffer, length);
}

bool serial_write_all(const unsigned char *buffer, const int length) { return serial_write(buffer, length) == length; }

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
        PRINTF_ERROR("device: buffer_read: buffer too large (max %d bytes, read %d bytes)\n", length, bytes_read);
        return -1;
    }
    return bytes_read;
}

bool serial_begin(const serial_config_t *config) {
    memcpy((void *)&serial_config, config, sizeof(serial_config_t));
    return true;
}

void serial_end(void) { serial_disconnect(); }

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
