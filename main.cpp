#include <iostream>
#include <fcntl.h>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <thread>
#include "cobs.h"
#include <iomanip>
#include <sys/epoll.h>

static constexpr size_t BUFFER_LEN = 16384;
static constexpr bool printData = false;

void print_hexdump(void *arr[], size_t len) {
    for (size_t i = 0; i < len; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << arr[i] << " ";
    }

    std::cout << std::dec << std::endl;
}

int ptyMasterOpen(std::string &slaveName) {
    int savedErrno, masterFd;
    char *p;

    masterFd = posix_openpt(O_RDWR | O_NOCTTY);
    if (masterFd == -1)
        return -1;

    if (grantpt(masterFd) == -1) {
        savedErrno = errno;
        close(masterFd);
        errno = savedErrno;
        return -1;
    }

    //Configure master pty to use raw mode
    struct termios opt{};
    int rc = tcgetattr(masterFd, &opt);
    if (rc == -1) {
        savedErrno = errno;
        close(masterFd);
        errno = savedErrno;
        return -1;
    }
    cfmakeraw(&opt);
    rc = tcsetattr(masterFd, TCSANOW, &opt);

    if (rc == -1) {
        savedErrno = errno;
        close(masterFd);
        errno = savedErrno;
        return -1;
    }

    p = ptsname(masterFd);
    if (p == nullptr) {
        savedErrno = errno;
        close(masterFd);
        errno = savedErrno;
        return -1;
    }
    slaveName = std::string(p);

    if (unlockpt(masterFd) == -1) {
        savedErrno = errno;
        close(masterFd);
        errno = savedErrno;
        return -1;
    }

    //Configure slave pty to use raw mode
    int slaveFd = open(slaveName.c_str(), O_RDWR);
    if (slaveFd == -1) {
        savedErrno = errno;
        close(masterFd);
        errno = savedErrno;
        return -1;
    }

    //Configure slave pty to use raw mode
    opt = termios();
    rc = tcgetattr(slaveFd, &opt);
    if (rc == -1) {
        savedErrno = errno;
        close(masterFd);
        close(slaveFd);
        errno = savedErrno;
        return -1;
    }
    cfmakeraw(&opt);

    rc = tcsetattr(slaveFd, TCSANOW, &opt);

    if (rc == -1) {
        savedErrno = errno;
        close(masterFd);
        close(slaveFd);
        errno = savedErrno;
        return -1;
    }

    close(slaveFd);

    return masterFd;
}

int openSerialPort(const std::string &serialPort, speed_t baudRate) {
    //int fd = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    int fd = open(serialPort.c_str(), O_RDWR);
    int savedErrno;

    if (fd == -1) {
        return fd;
    }

    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        savedErrno = errno;
        close(fd);
        errno = savedErrno;
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        savedErrno = errno;
        close(fd);
        errno = savedErrno;
        return -1;
    }

    return fd;
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

int main() {
    std::string slaveName;

    std::cout << "Opening PTY" << std::endl;
    auto masterFd = ptyMasterOpen(slaveName);
    std::cout << masterFd << std::endl;
    std::cout << slaveName << std::endl;

    if (masterFd == -1) {
        return errno;
    }

    std::cout << "Opening serial port" << std::endl;

    int baudRate = B2000000;
    std::string port = "/dev/teensy";

    int serialFd = openSerialPort(port, baudRate);
    while (serialFd == -1) {
        std::cout << "Error while opening serial port: " << errno << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        serialFd = openSerialPort(port, baudRate);
    }
    std::cout << "Serial port opened!" << std::endl;

    uint8_t ptyBuffer[BUFFER_LEN];
    memset(ptyBuffer, 0, sizeof(ptyBuffer));

    uint8_t serBuffer[BUFFER_LEN];
    memset(serBuffer, 0, sizeof(serBuffer));

    size_t serBuffer_index = 0;

    uint8_t encodeBuffer[BUFFER_LEN];
    memset(encodeBuffer, 0, sizeof(encodeBuffer));

    uint8_t decodeBuffer[BUFFER_LEN];
    memset(decodeBuffer, 0, sizeof(decodeBuffer));

    fcntl(masterFd, F_SETFL, FNDELAY);

    int epfd;
    struct epoll_event ev;
    struct epoll_event evlist[10];

    epfd = epoll_create(2);
    if (epfd == -1) {
        return -1;
    }

    ev.events = EPOLLIN;
    ev.data.fd = masterFd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, masterFd, &ev) == -1) {
        return -1;
    }

    ev.events = EPOLLIN;
    ev.data.fd = serialFd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, serialFd, &ev) == -1) {
        return -1;
    }

    while (true) {
        int ret = epoll_wait(epfd, evlist, 10, -1);
        if (ret == -1) {
            return errno;
        }

        for (int j = 0; j < ret; ++j) {
            if (evlist[j].data.fd == masterFd) {
                //Ready to read from pty
                ssize_t n = read(masterFd, ptyBuffer, sizeof(ptyBuffer));
                if (n > 0) {
                    //Encode ptyBuffer
                    auto encRes = cobs_encode(encodeBuffer, sizeof(encodeBuffer), ptyBuffer, n);
                    encodeBuffer[encRes.out_len++] = 0x00;

                    if (encRes.status != COBS_ENCODE_OK) {
                        return -encRes.status;
                    }

                    if (printData) {
                        std::cout << "Encoded Data: ";
                        print_hexdump(reinterpret_cast<void **>(encodeBuffer), encRes.out_len);
                        //std::cout << "Encoded Data: " << output << std::endl;
                    }

                    //Send encoded buffer to serial
                    ret = write(serialFd, encodeBuffer, encRes.out_len);
                    if (ret != encRes.out_len) {
                        std::cout << "Error writing to serial, only wrote: " << ret << std::endl;
                    }
                }
            } else if (evlist[j].data.fd == serialFd) {
                //Ready to read from serial
                ssize_t n = read(serialFd, serBuffer + serBuffer_index, sizeof(serBuffer) - serBuffer_index) +
                            serBuffer_index;
                if (n > 0) {
                    for (size_t i = 0; i < n; ++i) {
                        if (serBuffer[i] == 0) {
                            //Found packet end
                            //Decode serBuffer
                            auto decRes = cobs_decode(decodeBuffer, sizeof(decodeBuffer), serBuffer, i);

                            if (decRes.status != COBS_DECODE_OK) {
                                std::cout << "Error decoding serial data: " << decRes.status << std::endl;
                                std::cout << "Input size: " << n << std::endl;
                            } else {
                                if (printData) {
                                    std::cout << "Decoded data: " << std::endl;
                                    print_hexdump(reinterpret_cast<void **>(decodeBuffer), decRes.out_len);
                                }

                                //Send decoded buffer to pty
                                ret = write(masterFd, decodeBuffer, decRes.out_len);
                                if (ret != decRes.out_len) {
                                    std::cout << "Error writing to pty, only wrote: " << ret << std::endl;
                                }
                            }

                            memcpy(serBuffer, serBuffer + i + 1, (n + serBuffer_index) - 1 - i);
                            serBuffer_index = n - i - 1;
                            n = n - i - 1;
                            i = 0;
                        }
                    }
                }
            }


            //std::cout << "EXIT" << std::endl;
        }
    }

    return 0;
}

#pragma clang diagnostic pop
