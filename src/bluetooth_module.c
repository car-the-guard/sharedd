#define _GNU_SOURCE
#include "bluetooth_module.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B9600;   // fallback
    }
}

int bt_open(const char *dev, int baud)
{
    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if (fd < 0) { perror("bt_open: open"); return -1; }

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) { perror("bt_open: tcgetattr"); close(fd); return -1; }

    cfmakeraw(&tio);
    speed_t sp = baud_to_speed(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    // block until 1 byte
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { perror("bt_open: tcsetattr"); close(fd); return -1; }

    return fd;
}

void bt_close(int fd)
{
    if (fd >= 0) close(fd);
}

bt_cmd_t bt_read_cmd_blocking(int fd, uint8_t *raw_out)
{
    for (;;) {
        unsigned char ch;
        ssize_t r = read(fd, &ch, 1);

        if (r < 0) {
            if (errno == EINTR) continue;
            perror("bt_read_cmd_blocking: read");
            return BT_CMD_NONE; // caller can decide to exit
        }
        if (r == 0) continue;

        if (raw_out) *raw_out = ch;

        ch = (unsigned char)tolower(ch);

        if (ch == 'd') return BT_CMD_D;
        if (ch == 's') return BT_CMD_S;
        if (ch == 'l') return BT_CMD_L;
        if (ch == 'r') return BT_CMD_R;

        // ignore junk: \r \n spaces etc
    }
}