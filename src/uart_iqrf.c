/*
 * Copyright 2020 MICRORISC s.r.o.
 * Copyright 2020 IQRF Tech s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * clibuart library serves as an programming interface for communication
 * between Linux system and TR modules using UART and IO.
 *
 * @file   uart_iqrf.c
 * @date   07.12.2018
 * @ver    1.0.2
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

//TODO use std::chrono::high_resolution_clock for timing
#ifndef WIN32
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifdef _SC_MONOTONIC_CLOCK
static uint64_t get_ms_ts()
{
    struct timespec ts;

    if (clock_gettime (CLOCK_MONOTONIC, &ts) == 0)
        return (uint64_t) (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    else
        return 0;
}
#else
static uint64_t get_ms_ts()
{
    struct timeval tv;

    if (gettimeofday (&tv, NULL) == 0)
        return (uint64_t) (tv.tv_sec * 1000 + tv.tv_usec / 1000);
    else
        return 0;
}
#endif

#else

#include <windows.h>
#include <winioctl.h>
#include <errno.h>
#include <fcntl.h>
//#include <unistd.h>

#define _IOC_SIZEBITS   13
#define _IOC_DIRBITS    3

int ioctl(int fd, int rw, int* mode)
{
    return 0;
}

int nanosleep(int time)
{
    return 0;
}

static uint64_t get_ms_ts()
{
    LARGE_INTEGER counter, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);

    return counter.QuadPart / (frequency.QuadPart / 1000);
}

#endif

#include <uart_iqrf.h>
#include "declspec.h"
#include <machines_def.h>
#include <sleepWrapper.h>

#ifdef WIN32
#define snprintf _snprintf
#endif

#include "iqrf_gpio.h"

/************************************/
/* Private constants                */
/************************************/

/** Designates, that file descriptor is not used. */
static const int NO_FILE_DESCRIPTOR = -1;

#define HDLC_FRM_FLAG_SEQUENCE    0x7E
#define HDLC_FRM_CONTROL_ESCAPE   0x7D
#define HDLC_FRM_ESCAPE_BIT       0x20

/************************************/
/* Private functions predeclaration */
/************************************/
static int check_data_len(unsigned int dataLen);
uint8_t dpa_do_CRC8(uint8_t byte, uint8_t seed);
uint8_t write_byte_to_buffer(T_UART_SOCKET_CONTROL *socket, uint8_t *dataBuffer, uint8_t dataByte);
int set_interface_attribs(int fd, int speed);
static int uart_iqrf_open(const T_UART_IQRF_CONFIG_STRUCT *uartConfig, T_UART_SOCKET_CONTROL *socket);
static int uart_iqrf_close(T_UART_SOCKET_CONTROL *socket);

/**
 * Checks length of the data to be written or read.
 *
 * @param	dataLen - Length (in bytes) of the data.
 *
 * @return	@c BASE_TYPES_OPER_ERROR if @dataLen is bellow or equal zero or is higher than @c UART_IQRF_MAX_DATA_LENGTH.
 */
static int check_data_len(unsigned int dataLen)
{
    if (dataLen <= 0) {
        return BASE_TYPES_OPER_ERROR;
    }

    if (dataLen > UART_IQRF_MAX_DATA_LENGTH) {
        return BASE_TYPES_OPER_ERROR;
    }

    return BASE_TYPES_OPER_OK;
}

/**
 * Compute the CRC8 value of a data set
 * @param byte One byte of data to compute CRC from
 * @param seed The starting value of the CRC
 * @return The CRC8 of inData with seed as initial value
 */
uint8_t dpa_do_CRC8(uint8_t byte, uint8_t seed)
{
    bool needsXor;
    for (uint8_t bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        needsXor = (seed ^ byte) & 1;
        seed >>= 1;
        if (needsXor) {
            seed ^= 0x8C;
        }
        byte >>= 1;
    }
    return seed;
}


/**
 * write data byte to to buffer for TR module + make HDLC byte stuffing and compute CRC
 * @param *dataBuffer Pointer to position in output buffer for TR module
 * @param dataByte Data byte for TR module
 *
 * @return @c number of bytes written to data buffer
 */
uint8_t write_byte_to_buffer(T_UART_SOCKET_CONTROL *socket, uint8_t *dataBuffer, uint8_t dataByte)
{
    socket->sendControl.CRC = dpa_do_CRC8(dataByte, socket->sendControl.CRC);

    if (dataByte == HDLC_FRM_FLAG_SEQUENCE || dataByte == HDLC_FRM_CONTROL_ESCAPE) {
        *dataBuffer = HDLC_FRM_CONTROL_ESCAPE;
        dataBuffer++;
        *dataBuffer = (dataByte ^ HDLC_FRM_ESCAPE_BIT);
        return (2);
    } else {
        *dataBuffer = dataByte;
        return (1);
    }
}

/**
* Initialization of the uart library for IQRF module
*
* @param	configStruct - configuration structure
*
* @return	@c BASE_TYPES_OPER_ERROR = initialization failed
* @return	@c BASE_TYPES_OPER_OK = initialization was correct
*/
int uart_iqrf_init(const T_UART_IQRF_CONFIG_STRUCT *uartConfig, T_UART_SOCKET_CONTROL *socket)
{
    if (uartConfig == NULL || socket == NULL) {
        return BASE_TYPES_OPER_ERROR;
    }

    if (socket->initialized == 1) {
        return BASE_TYPES_OPER_ERROR;
    }

    // allocate memory for received data
    socket->recvControl.receiveBuffer = malloc(SIZE_OF_UART_RX_BUFFER * sizeof(uint8_t));
    if (socket->recvControl.receiveBuffer == NULL) {
        return BASE_TYPES_OPER_ERROR;
    }

    // Initialize PGM SW pin, bus enable pin & power enable
    if (uartConfig->pgmSwitchGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_init_output(uartConfig->pgmSwitchGpioPin, false);
    }
    if (uartConfig->powerEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_init_output(uartConfig->powerEnableGpioPin, true);
    }
    if (uartConfig->busEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_init_output(uartConfig->busEnableGpioPin, false);
        SLEEP(1);
    } else {
        if (uartConfig->uartEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_init_output(uartConfig->uartEnableGpioPin, false);
        }
        if (uartConfig->spiEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_init_output(uartConfig->spiEnableGpioPin, false);
        }
        if (uartConfig->i2cEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_init_output(uartConfig->i2cEnableGpioPin, false);
        }
        SLEEP(1);
    }

    // Reset TR module
    if (uartConfig->trModuleReset == TR_MODULE_RESET_ENABLE) {
        if (uartConfig->powerEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            // Disable PWR for TR
            iqrf_gpio_set_value(uartConfig->powerEnableGpioPin, false);
            // Sleep for 300ms
            SLEEP(300);
            // Enable PWR for TR
            iqrf_gpio_set_value(uartConfig->powerEnableGpioPin, true);
            SLEEP(1);
        }
    }

    if (uartConfig->busEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_set_value(uartConfig->busEnableGpioPin, true);
    } else {
        if (uartConfig->uartEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_set_value(uartConfig->uartEnableGpioPin, true);
        }
        if (uartConfig->spiEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_set_value(uartConfig->spiEnableGpioPin, false);
        }
        if (uartConfig->i2cEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_set_value(uartConfig->i2cEnableGpioPin, true);
        }
    }

    // Sleep for 500ms (in this time TR module waits for sequence to switch to programming mode)
    SLEEP(500);

    if (uart_iqrf_open(uartConfig, socket) == BASE_TYPES_OPER_OK) {
        socket->initialized = 1;
        return BASE_TYPES_OPER_OK;
    } else {
        if (uartConfig->powerEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_unexport(uartConfig->powerEnableGpioPin);
        }

        if (uartConfig->busEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_unexport(uartConfig->busEnableGpioPin);
        } else {
            if (uartConfig->spiEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
                iqrf_gpio_unexport(uartConfig->spiEnableGpioPin);
            }
            if (uartConfig->uartEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
                iqrf_gpio_unexport(uartConfig->uartEnableGpioPin);
            }
            if (uartConfig->i2cEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
                iqrf_gpio_unexport(uartConfig->i2cEnableGpioPin);
            }
        }
        if (uartConfig->pgmSwitchGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_unexport(uartConfig->pgmSwitchGpioPin);
        }

        free(socket->recvControl.receiveBuffer);

        return BASE_TYPES_OPER_ERROR;
    }
}

/**
* Writes data to TR module
*
* @param	dataToWrite	- data to be written to TR module
* @param	dataLen		- length (in bytes) of the data
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during write operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = uart library is not initialized
* @return	@c BASE_TYPES_OPER_OK = data was successfully written
*/
int uart_iqrf_write(T_UART_SOCKET_CONTROL *socket, uint8_t *dataToWrite, unsigned int dataLen)
{
    uint8_t *dataToSend = NULL;
    int wlen;

    if (socket->initialized == 0) {
        return BASE_TYPES_LIB_NOT_INITIALIZED;
    }

    if (socket->fd < 0) {
        return BASE_TYPES_OPER_ERROR;
    }

    // checking input parameters
    if (dataToWrite == NULL) {
        return BASE_TYPES_OPER_ERROR;
    }

    if (check_data_len(dataLen)) {
        return BASE_TYPES_OPER_ERROR;
    }

    dataToSend = malloc(256 * sizeof(uint8_t));
    if (dataToSend == NULL) {
        return BASE_TYPES_OPER_ERROR;
    }

    // initialize CRC
    socket->sendControl.CRC = 0xFF;
    // start of packet character
    dataToSend[0] = HDLC_FRM_FLAG_SEQUENCE;
    // counter of sent bytes
    socket->sendControl.packetCnt = 1;

    // send user data
    while (dataLen) {
        socket->sendControl.packetCnt += write_byte_to_buffer(socket, &dataToSend[socket->sendControl.packetCnt], *(uint8_t *)dataToWrite);
        dataToWrite++;
        dataLen--;
    }

    // send CRC
    socket->sendControl.packetCnt += write_byte_to_buffer(socket, &dataToSend[socket->sendControl.packetCnt], socket->sendControl.CRC);
    // send stop of packet character
    dataToSend[socket->sendControl.packetCnt++] = HDLC_FRM_FLAG_SEQUENCE;

    // send data to module
    wlen = write(socket->fd, dataToSend, socket->sendControl.packetCnt);
    // delay for output
#ifndef WIN32
    tcdrain(socket->fd);
#endif

    free(dataToSend);

    if (wlen != socket->sendControl.packetCnt) {
        return BASE_TYPES_OPER_ERROR;
    }

    return BASE_TYPES_OPER_OK;
}

/**
* Reads data from TR module
*
* @param	readBuffer	- data are read to this buffer
* @param	dataLen		- length of received packet (in bytes)
* @param	timeout		- the time I wait for the packet to be received (in ms)
*
* @return	@c BASE_TYPES_OPER_ERROR = error occurs during read operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = UART library is not initialized
* @return	@c BASE_TYPES_OPER_OK = data were successfully read
* @return	@c UART_IQRF_ERROR_CRC = mismatched CRC
* @return	@c UART_IQRF_ERROR_TIMEOUT = receiver timeout (no data read)
*/
int uart_iqrf_read(T_UART_SOCKET_CONTROL *socket, uint8_t *readBuffer, uint8_t *dataLen, unsigned int timeout)
{
    uint64_t start;
    uint8_t inputChar;
    int rlen;

    // checking input parameter
    if (dataLen == NULL) {
        return BASE_TYPES_OPER_ERROR;
    } else {
        *dataLen = 0;
    }

    // checking input parameter
    if (readBuffer == NULL) {
        return BASE_TYPES_OPER_ERROR;
    }

    // checking if library is initialized
    if (socket->initialized == 0) {
        return BASE_TYPES_LIB_NOT_INITIALIZED;
    }

    socket->recvControl.decodeInProgress = 0;

    start = get_ms_ts();
    while (1) {

        rlen = read(socket->fd, socket->recvControl.receiveBuffer, SIZE_OF_UART_RX_BUFFER - 1);

        socket->recvControl.rBuffCnt = 0;
        while (rlen--) {
            inputChar = socket->recvControl.receiveBuffer[socket->recvControl.rBuffCnt++];

            if (socket->recvControl.decodeInProgress) {
                // end of packet or DPA structure is full
                if (inputChar == HDLC_FRM_FLAG_SEQUENCE || socket->recvControl.packetCnt >= 70) {
                    if (socket->recvControl.CRC == 0) {
                        *dataLen = socket->recvControl.packetCnt - 1;
                        return(BASE_TYPES_OPER_OK);
                    } else {
                        return (UART_IQRF_ERROR_CRC);
                    }
                }

                // discard received ESCAPE character
                if (inputChar == HDLC_FRM_CONTROL_ESCAPE) {
                    socket->recvControl.wasEscape = 1;
                    continue;
                }

                // previous character was ESCAPE
                if (socket->recvControl.wasEscape) {
                    socket->recvControl.wasEscape = 0;
                    inputChar ^= HDLC_FRM_ESCAPE_BIT;
                }

                // add Rx byte to CRC
                socket->recvControl.CRC = dpa_do_CRC8(inputChar, socket->recvControl.CRC);

                // write received char to buffer
                *((uint8_t *)readBuffer + socket->recvControl.packetCnt) = inputChar;
                socket->recvControl.packetCnt++;
            } else {
                if (inputChar == HDLC_FRM_FLAG_SEQUENCE) {
                    socket->recvControl.CRC = 0xFF;
                    socket->recvControl.packetCnt = 0;
                    socket->recvControl.wasEscape = 0;
                    socket->recvControl.delayRxTimeout = 0;
                    socket->recvControl.decodeInProgress = 1;
                }
            }
        }

        SLEEP(5);

        // check if receiver timeout has been elapsed
        if ((get_ms_ts() - start) > timeout) {
            if (socket->recvControl.decodeInProgress) {
                if (socket->recvControl.delayRxTimeout) {
                    break;
                } else {
                    socket->recvControl.delayRxTimeout = 1;
                    timeout += 500;
                }
            } else {
                break;
            }
        }
    }

    return (UART_IQRF_ERROR_TIMEOUT);
}

/**
* set parameters of uart port
*
* @param	fd	- port identificator
* @param	speed		- communication baud rate
*
* @return	@c BASE_TYPES_OPER_ERROR = error occurs during parameters set
* @return	@c BASE_TYPES_OPER_OK = parameters set OK
*/
int set_interface_attribs(int fd, int speed)
{
#ifndef WIN32
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flow control */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return BASE_TYPES_OPER_ERROR;
    }
#endif
    return BASE_TYPES_OPER_OK;
}

/**
* Open and setup UART channel.
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occurs during UART initialization
* @return	@c BASE_TYPES_OPER_OK = UART channel initialized OK
*/
int uart_iqrf_open(const T_UART_IQRF_CONFIG_STRUCT *uartConfig, T_UART_SOCKET_CONTROL *socket)
{
    if (socket->fd != NO_FILE_DESCRIPTOR) {
        return BASE_TYPES_OPER_ERROR;
    }

#ifndef WIN32
    socket->fd = open(uartConfig->uartDev, O_RDWR | O_NOCTTY | O_SYNC);
    if (socket->fd < 0) {
        socket->fd = NO_FILE_DESCRIPTOR;
        return BASE_TYPES_OPER_ERROR;
    }

    // configure port
    set_interface_attribs(socket->fd, uartConfig->baudRate);
#endif

    return BASE_TYPES_OPER_OK;
}

/**
* Close UART channel.
*
*
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = uart library was not initialized
* @return	@c BASE_TYPES_OPER_ERROR = error occurs during UART channel closing
* @return	@c BASE_TYPES_OPER_OK = UART channel closed OK
*/
int uart_iqrf_close(T_UART_SOCKET_CONTROL *socket)
{
    int closeRes;

    if (socket == NULL || socket->fd == NO_FILE_DESCRIPTOR) {
        return BASE_TYPES_LIB_NOT_INITIALIZED;
    }

    if (socket->fd < 0) {
        return BASE_TYPES_OPER_ERROR;
    }

    closeRes = close(socket->fd);
    socket->fd = NO_FILE_DESCRIPTOR;

    if (closeRes == -1) {
        return BASE_TYPES_OPER_ERROR;
    }

    return BASE_TYPES_OPER_OK;
}

/**
* Destroys IQRF UART library object and releases UART port
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occurs during UART destroy operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = UART library is not initialized
* @return	@c BASE_TYPES_OPER_OK = UART channel was successfully closed
*/
int uart_iqrf_destroy(const T_UART_IQRF_CONFIG_STRUCT *uartConfig ,T_UART_SOCKET_CONTROL *socket)
{
    if (socket == NULL || socket->initialized == 0) {
        return BASE_TYPES_LIB_NOT_INITIALIZED;
    }

    // after calling this method, the behavior of the library will be
    // like if the library was not initialized
    socket->initialized = 0;

    // destroy used GPIO pins
    if (uartConfig->powerEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_unexport(uartConfig->powerEnableGpioPin);
    }
    if (uartConfig->busEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_unexport(uartConfig->busEnableGpioPin);
    } else {
        if (uartConfig->spiEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_unexport(uartConfig->spiEnableGpioPin);
        }
        if (uartConfig->uartEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_unexport(uartConfig->uartEnableGpioPin);
        }
        if (uartConfig->i2cEnableGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
            iqrf_gpio_unexport(uartConfig->i2cEnableGpioPin);
        }
    }
    if (uartConfig->pgmSwitchGpioPin != IQRF_GPIO_PIN_UNKNOWN) {
        iqrf_gpio_unexport(uartConfig->pgmSwitchGpioPin);
    }

    // deallocate receive buffer
    free(socket->recvControl.receiveBuffer);

    return uart_iqrf_close(socket);
}
