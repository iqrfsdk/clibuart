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

/* constants */
#define GPIO_BASE_PATH "/sys/class/gpio"
#define GPIO_EXPORT_PATH GPIO_BASE_PATH"/export"
#define GPIO_UNEXPORT_PATH GPIO_BASE_PATH"/unexport"

#define GPIO_DIRECTION_STR "direction"
#define GPIO_VALUE_STR "value"

/* gpio clibuart_gpio_getDirection state */
#define GPIO_DIRECTION_IN_STR "in"
#define GPIO_DIRECTION_OUT_STR "out"

/************************************/
/* Private constants                */
/************************************/

/** Designates, that file descriptor is not used. */
static const int NO_FILE_DESCRIPTOR = -1;

#define HDLC_FRM_FLAG_SEQUENCE    0x7E
#define HDLC_FRM_CONTROL_ESCAPE   0x7D
#define HDLC_FRM_ESCAPE_BIT       0x20

/** Values that represent GPIO directions. */
typedef enum _clibuart_gpio_direction {
    ///< An enum constant representing not available GPIO direction
    GPIO_DIRECTION_NOT_AVAILABLE = -1,
    ///< An enum constant representing GPIO input
    GPIO_DIRECTION_IN = 0,
    ///< An enum constant representing GPIO output
    GPIO_DIRECTION_OUT
} clibuart_gpio_direction;

/************************************/
/* Private functions predeclaration */
/************************************/
static int check_data_len(unsigned int dataLen);
uint8_t dpa_do_CRC8(uint8_t inData, uint8_t seed);
uint8_t write_byte_to_buffer(T_UART_SOCKET_CONTROL *socket, uint8_t *dataBuffer, uint8_t dataByte);
int set_interface_attribs(int fd, int speed);
static int uart_iqrf_open(const T_UART_IQRF_CONFIG_STRUCT *uartConfig, T_UART_SOCKET_CONTROL *socket);
static int uart_iqrf_close(T_UART_SOCKET_CONTROL *socket);

int clibuart_gpio_export(uint32_t gpio);
int clibuart_gpio_unexport(uint32_t gpio);
int clibuart_gpio_setDirection(uint32_t gpio, clibuart_gpio_direction dir);
clibuart_gpio_direction clibuart_gpio_getDirection(uint32_t gpio);
int clibuart_gpio_setValue(uint32_t gpio, int val);
int clibuart_gpio_getValue(uint32_t gpio);
int clibuart_gpio_setup(uint32_t gpio, clibuart_gpio_direction dir, int val);
int clibuart_gpio_cleanup(uint32_t gpio);

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
 * @param inData One byte of data to compute CRC from
 * @param seed The starting value of the CRC
 * @return The CRC8 of inData with seed as initial value
 */
uint8_t dpa_do_CRC8(uint8_t inData, uint8_t seed)
{
    uint8_t bitsLeft;
    uint8_t seedTemp;

    for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        if (((seed ^ inData) & 0x01) == 0) {
            seed >>= 1;
	} else {
	    seedTemp = (seed >>= 1);
	    seedTemp ^= 0x8C;
            seed = seedTemp;
	}
        inData >>= 1;
    }
    return (seed);
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
    if (uartConfig->pgmSwitchGpioPin != -1)
        clibuart_gpio_setup((uint32_t)uartConfig->pgmSwitchGpioPin, GPIO_DIRECTION_OUT, 0);
    if (uartConfig->powerEnableGpioPin != -1)
        clibuart_gpio_setup((uint32_t)uartConfig->powerEnableGpioPin, GPIO_DIRECTION_OUT, 1);
    if (uartConfig->busEnableGpioPin != -1) {
        clibuart_gpio_setup((uint32_t)uartConfig->busEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        SLEEP(1);
    } else {
        if (uartConfig->uartEnableGpioPin != -1) {
            clibuart_gpio_setup((uint32_t)uartConfig->uartEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        }
        if (uartConfig->spiEnableGpioPin != -1) {
            clibuart_gpio_setup((uint32_t)uartConfig->spiEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        }
        if (uartConfig->i2cEnableGpioPin != -1) {
            clibuart_gpio_setup((uint32_t)uartConfig->i2cEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        }
        SLEEP(1);
    }

    // Reset TR module
    if (uartConfig->trModuleReset == TR_MODULE_RESET_ENABLE) {
        if (uartConfig->powerEnableGpioPin != -1) {
            // Disable PWR for TR
            clibuart_gpio_setValue((uint32_t)uartConfig->powerEnableGpioPin, 0);
            // Sleep for 300ms
            SLEEP(300);
            // Enable PWR for TR
            clibuart_gpio_setValue((uint32_t)uartConfig->powerEnableGpioPin, 1);
            SLEEP(1);
        }
    }

    if (uartConfig->busEnableGpioPin != -1) {
        clibuart_gpio_setValue((uint32_t)uartConfig->busEnableGpioPin, 1);
    } else {
        if (uartConfig->uartEnableGpioPin != -1) {
            clibuart_gpio_setValue((uint32_t)uartConfig->uartEnableGpioPin, 1);
        }
        if (uartConfig->spiEnableGpioPin != -1) {
            clibuart_gpio_setValue((uint32_t)uartConfig->spiEnableGpioPin, 0);
        }
        if (uartConfig->i2cEnableGpioPin != -1) {
            clibuart_gpio_setValue((uint32_t)uartConfig->i2cEnableGpioPin, 1);
        }
    }

    // Sleep for 500ms (in this time TR module waits for sequence to switch to programming mode)
    SLEEP(500);

    if (uart_iqrf_open(uartConfig, socket) == BASE_TYPES_OPER_OK) {
        socket->initialized = 1;
        return BASE_TYPES_OPER_OK;
    } else {
        if (uartConfig->powerEnableGpioPin != -1) {
            clibuart_gpio_cleanup((uint32_t)uartConfig->powerEnableGpioPin);
        }
        
        if (uartConfig->busEnableGpioPin != -1) {
            clibuart_gpio_cleanup((uint32_t)uartConfig->busEnableGpioPin);
        } else {
            if (uartConfig->uartEnableGpioPin != -1) {
                clibuart_gpio_cleanup((uint32_t)uartConfig->uartEnableGpioPin);
            }
            if (uartConfig->spiEnableGpioPin != -1) {
                clibuart_gpio_cleanup((uint32_t)uartConfig->spiEnableGpioPin);
            }
            if (uartConfig->i2cEnableGpioPin != -1) {
                clibuart_gpio_cleanup((uint32_t)uartConfig->i2cEnableGpioPin);
            }
        }
        if (uartConfig->pgmSwitchGpioPin != -1) {
            clibuart_gpio_cleanup((uint32_t)uartConfig->pgmSwitchGpioPin);
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
        socket = NO_FILE_DESCRIPTOR;
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
    if (uartConfig->powerEnableGpioPin != -1) {
        clibuart_gpio_cleanup((uint32_t)uartConfig->powerEnableGpioPin);
    }
    if (uartConfig->busEnableGpioPin != -1) {
        clibuart_gpio_cleanup((uint32_t)uartConfig->busEnableGpioPin);
    } else {
        if (uartConfig->spiEnableGpioPin != -1) {
            clibuart_gpio_cleanup((uint32_t)uartConfig->spiEnableGpioPin);
        }
        if (uartConfig->uartEnableGpioPin != -1) {
            clibuart_gpio_cleanup((uint32_t)uartConfig->uartEnableGpioPin);
        }
        if (uartConfig->i2cEnableGpioPin != -1) {
            clibuart_gpio_cleanup((uint32_t)uartConfig->i2cEnableGpioPin);
        }
    } 
    if (uartConfig->pgmSwitchGpioPin != -1) {
        clibuart_gpio_cleanup((uint32_t)uartConfig->pgmSwitchGpioPin);
    }

    // deallocate receive buffer
    free(socket->recvControl.receiveBuffer);

    return uart_iqrf_close(socket);
}

/* ========================================================================= */
/*                      GPIO manipulation functions                          */
/* ========================================================================= */

/**
* Setup GPIO path
*
* @param [in]	gpio		GPIO
* @param [in]	action		action
* @param [in]	target		target
* @param [in]	len			length
*
*/
static void clibuart_setup_gpio_path(const uint32_t gpio, const char *action, char *target, int len)
{
    snprintf(target, len, GPIO_BASE_PATH"/gpio%d/%s", gpio, action);
}

/**
* Writes data stored in @c fd file to buffer.
*
* @param [in]	fd		file to write.
* @param [in]	buf		buffer where file is written
*
* @return	-1 = error during write
* @return	0 = file was successfully written
*/
static int clibuart_write_data(FILE *fd, const char *buf)
{
    int ret = 0;

    ret = fwrite(buf, 1, strlen(buf), fd);
    if (ret != (int)strlen(buf)) {
        printf("Error during writing to file\n");
        ret = -1;
    } else {
        ret = 0;
    }

    return ret;
}

/**
* Exports the GPIO pin to the GPIO array
*
* @param num	GPIO number
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int clibuart_gpio_export(uint32_t num)
{
    FILE *fd = fopen(GPIO_EXPORT_PATH, "w");
    char buf[5];
    int ret;

    if (!fd) {
        printf("Error during opening file: %s\n", strerror(errno));
        return -1;
    }

    snprintf(buf, sizeof(buf), "%d", num);
    ret = clibuart_write_data(fd, buf);
    if (ret) {
        goto err;
    }

err:
    fclose(fd);
    return ret;
}

/**
* Unexports the GPIO pin from the GPIO array
*
* @param num	GPIO number
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int clibuart_gpio_unexport(uint32_t num)
{
    FILE *fd = fopen(GPIO_UNEXPORT_PATH, "w");
    char buf[5];
    int ret;

    if (!fd) {
        printf("Error during opening file: %s\n", strerror(errno));
        return -1;
    }

    snprintf(buf, sizeof(buf), "%d", num);
    ret = clibuart_write_data(fd, buf);
    if (ret) {
        goto err;
    }

err:
    fclose(fd);
    return ret;
}


/**
* Gets direction of the GPIO
*
* @param gpio	GPIO
* @param dir	GPIO direction  = GPIO_DIRECTION_IN or GPIO_DIRECTION_OUT
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int clibuart_gpio_setDirection(uint32_t gpio, clibuart_gpio_direction dir)
{
    char path[50];
    char buf[4];
    FILE *fd = NULL;
    int ret;

    clibuart_setup_gpio_path(gpio, GPIO_DIRECTION_STR, path, sizeof(path));

    fd = fopen(path, "w");

    if (!fd) {
        printf("Error during opening file (set direction): %s  %s\n", path, strerror(errno));
        return -1;
    }
    if (dir == GPIO_DIRECTION_IN) {
        strncpy(buf, GPIO_DIRECTION_IN_STR, sizeof(buf));
    } else if (dir == GPIO_DIRECTION_OUT) {
        strncpy(buf, GPIO_DIRECTION_OUT_STR, sizeof(buf));
    }

    ret = clibuart_write_data(fd, buf);
    if (ret) {
        goto err;
    }

err:
    fclose(fd);
    return ret;
}

/**
* Gets direction of the GPIO
*
* @param gpio	GPIO
*
* @return	GPIO_DIRECTION_NOT_AVAILABLE = GPIO direction information not available.
* @return	GPIO_DIRECTION_IN = GPIO is set to input.
* @return	GPIO_DIRECTION_OUT = GPIO is set to output.
*/
clibuart_gpio_direction clibuart_gpio_getDirection(uint32_t gpio)
{
    char path[50];
    char buf[4];
    FILE *fd = NULL;
    int ret;
    clibuart_gpio_direction dir;

    clibuart_setup_gpio_path(gpio, GPIO_DIRECTION_STR, path, sizeof(path));

    fd = fopen(path, "r");

    if (!fd) {
        printf("Error during opening file (get direction): %s\n", strerror(errno));
        return GPIO_DIRECTION_NOT_AVAILABLE;
    }

    dir = GPIO_DIRECTION_NOT_AVAILABLE;

    ret = fread(buf, 1, sizeof(buf), fd);
    if (!ret) {
        printf("Error during reading file\n");
        goto err;
    }

    if (!strcmp(buf, GPIO_DIRECTION_IN_STR)) {
        dir = GPIO_DIRECTION_IN;
    } else if (!strcmp(buf, GPIO_DIRECTION_OUT_STR)) {
        dir = GPIO_DIRECTION_OUT;
    }

err:
    fclose(fd);
    return dir;
}

/**
* Sets GPIO value
*
* @param gpio	GPIO
* @param val	value
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int clibuart_gpio_setValue(uint32_t gpio, int val)
{
    char path[50];
    char buf[2];
    FILE *fd = NULL;
    int ret;

    clibuart_setup_gpio_path(gpio, GPIO_VALUE_STR, path, sizeof(path));

    fd = fopen(path, "w");

    if (!fd) {
        printf("Error during opening file: %s\n", strerror(errno));
        return -1;
    }

    snprintf(buf, sizeof(buf), "%d", val);
    ret = clibuart_write_data(fd, buf);
    if (ret) {
        goto err;
    }

err:
    fclose(fd);
    return ret;
}

/**
* Gets GPIO value
*
* @param gpio	GPIO
*
* @return	GPIO value
*/
int clibuart_gpio_getValue(uint32_t gpio)
{
    char path[50];
    char buf[2];
    FILE *fd = NULL;
    int ret;

    clibuart_setup_gpio_path(gpio, GPIO_VALUE_STR, path, sizeof(path));

    fd = fopen(path, "r");

    if (!fd) {
        printf("Error during opening file: %s\n", strerror(errno));
        return -1;
    }

    ret = fread(buf, 1, sizeof(buf), fd);
    if (!ret) {
        printf("Error during reading file\n");
        ret = -1;
        goto err;
    }

    ret = strtol(buf, NULL, 10);

err:
    fclose(fd);
    return ret;
}

/**
* Setup GPIO
*
* @param gpio	GPIO
* @param dir	GPIO direction
* @param val	GPIO value
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int clibuart_gpio_setup(uint32_t gpio, clibuart_gpio_direction dir, int val)
{
    int ret;

    ret = clibuart_gpio_export(gpio);
    if (ret) {
        return ret;
    }

    int i;
    for (i = 1; i <= 10; i++) {
        ret = clibuart_gpio_setDirection(gpio, dir);
        if (!ret) {
            printf("clibuart_gpio_setup() setDir success: %d\n", i);
            break;
        } else {
            printf("clibuart_gpio_setup() setDir failed wait for 100 ms to next try: %d\n", i);
            SLEEP(100);
        }
    }

    // set gpio value when output clibuart_gpio_getDirection
    if (dir == GPIO_DIRECTION_OUT) {
        ret = clibuart_gpio_setValue(gpio, val);
        if (ret) {
            return ret;
        }
    }

    return ret;
}

/**
* Cleans and releases GPIO
*
* @param gpio	GPIO
*
* @return	0 = operation was performed successfully.
* @return	-1 = some error occurred.
*/
int clibuart_gpio_cleanup(uint32_t gpio)
{
    return (clibuart_gpio_unexport(gpio));
}
