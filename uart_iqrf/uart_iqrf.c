/*
 * Copyright 2015 MICRORISC s.r.o.
 * Copyright 2018 IQRF Tech s.r.o.
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

typedef struct {
    uint8_t packetCnt;
    uint8_t CRC;
} T_UART_SEND_CTRL;

T_UART_SEND_CTRL senderControl;

typedef struct {
    uint8_t *receiveBuffer;
    uint8_t packetCnt;
    uint8_t CRC;
    uint8_t decodeInProgress;
    uint8_t wasEscape;
    uint8_t delayRxTimeout;
    int     rBuffCnt;
    int     timeoutTimer;
} T_UART_RECEIVER_CONTROL;

T_UART_RECEIVER_CONTROL  receiverControl;

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
/* Private variables                */
/************************************/
/** Indicates, whether this library is initialized or not. 0 - not initialized 1 - initialized. */
static int libIsInitialized = 0;

/** File descriptor of the device special file. */
static int fd = -1;

/** pointer to actual uart iqrf configuration structure */
static T_UART_IQRF_CONFIG_STRUCT *uartIqrfConfig = NULL;

/************************************/
/* Private functions predeclaration */
/************************************/
static int check_data_len(unsigned int dataLen);
uint8_t dpa_do_CRC8(uint8_t inData, uint8_t seed);
uint8_t write_byte_to_buffer(uint8_t *dataBuffer, uint8_t dataByte);
int set_interface_attribs(int fd, int speed);
static int uart_iqrf_open(void);
static int uart_iqrf_close(void);

int clibuart_gpio_export(int gpio);
int clibuart_gpio_unexport(int gpio);
int clibuart_gpio_setDirection(int gpio, clibuart_gpio_direction dir);
clibuart_gpio_direction clibuart_gpio_getDirection(int gpio);
int clibuart_gpio_setValue(int gpio, int val);
int clibuart_gpio_getValue(int gpio);
int clibuart_gpio_setup(int gpio, clibuart_gpio_direction dir, int val);
int clibuart_gpio_cleanup(int gpio);

/**
 * Checks length of the data to be written or read.
 *
 * @param	dataLen - Length (in bytes) of the data.
 *
 * @return	@c BASE_TYPES_OPER_ERROR if @dataLen is bellow or equal zero or is higher than @c UART_IQRF_MAX_DATA_LENGTH.
 */
static int check_data_len(unsigned int dataLen)
{
    if (dataLen <= 0)
        return BASE_TYPES_OPER_ERROR;

    if (dataLen > UART_IQRF_MAX_DATA_LENGTH)
        return BASE_TYPES_OPER_ERROR;

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
uint8_t write_byte_to_buffer(uint8_t *dataBuffer, uint8_t dataByte)
{
    senderControl.CRC = dpa_do_CRC8(dataByte, senderControl.CRC);

    if (dataByte == HDLC_FRM_FLAG_SEQUENCE || dataByte == HDLC_FRM_CONTROL_ESCAPE) {
        *dataBuffer = HDLC_FRM_CONTROL_ESCAPE;
        dataBuffer++;
        *dataBuffer = (dataByte ^ HDLC_FRM_ESCAPE_BIT);
        return (2);
    }
    else {
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
int uart_iqrf_init(const T_UART_IQRF_CONFIG_STRUCT *configStruct)
{
    if (libIsInitialized == 1)
        return BASE_TYPES_OPER_ERROR;

    // allocate memory for received data
    receiverControl.receiveBuffer = malloc(SIZE_OF_UART_RX_BUFFER * sizeof(uint8_t));
    if (receiverControl.receiveBuffer == NULL)
        return BASE_TYPES_OPER_ERROR;

    uartIqrfConfig = (T_UART_IQRF_CONFIG_STRUCT *)configStruct;

    // Copy UART device name
    if (strlen(UART_IQRF_DEFAULT_DEVICE) > UART_DEV_CAPACITY)
        return BASE_TYPES_OPER_ERROR;
    else
        strcpy(uartIqrfConfig->uartDev, UART_IQRF_DEFAULT_DEVICE);

    uartIqrfConfig->powerEnableGpioPin = POWER_ENABLE_GPIO;
    uartIqrfConfig->busEnableGpioPin = BUS_ENABLE_GPIO;  

    if (uartIqrfConfig->busEnableGpioPin == -1) {
#ifdef SPI_ENABLE_GPIO 
        uartIqrfConfig->spiEnableGpioPin = SPI_ENABLE_GPIO;
#else
        uartIqrfConfig->spiEnableGpioPin = -1;
#endif
#ifdef UART_ENABLE_GPIO
        uartIqrfConfig->uartEnableGpioPin = UART_ENABLE_GPIO;
#else
        uartIqrfConfig->uartEnableGpioPin = -1;
#endif
#ifdef I2C_ENABLE_GPIO
        uartIqrfConfig->i2cEnableGpioPin = I2C_ENABLE_GPIO;
#else
        uartIqrfConfig->i2cEnableGpioPin = -1;
#endif
    } else {
        uartIqrfConfig->spiEnableGpioPin = -1;
        uartIqrfConfig->uartEnableGpioPin = -1;
        uartIqrfConfig->i2cEnableGpioPin = -1;
    }
    uartIqrfConfig->pgmSwitchGpioPin = PGM_SWITCH_GPIO;
    uartIqrfConfig->trModuleReset = TR_MODULE_RESET_ENABLE;

    // Initialize PGM SW pin, bus enable pin & power enable
    if (uartIqrfConfig->pgmSwitchGpioPin != -1)
        clibuart_gpio_setup(uartIqrfConfig->pgmSwitchGpioPin, GPIO_DIRECTION_OUT, 0);
    if (uartIqrfConfig->powerEnableGpioPin != -1)
        clibuart_gpio_setup(uartIqrfConfig->powerEnableGpioPin, GPIO_DIRECTION_OUT, 1);
    if (uartIqrfConfig->busEnableGpioPin != -1) {
        clibuart_gpio_setup(uartIqrfConfig->busEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        SLEEP(1);
    } else {
        clibuart_gpio_setup(uartIqrfConfig->uartEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        clibuart_gpio_setup(uartIqrfConfig->spiEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        clibuart_gpio_setup(uartIqrfConfig->i2cEnableGpioPin, GPIO_DIRECTION_OUT, 0);
        SLEEP(1);
    }

    // Reset TR module
    if (uartIqrfConfig->trModuleReset == TR_MODULE_RESET_ENABLE) {
        if (uartIqrfConfig->powerEnableGpioPin != -1) {
            // Disable PWR for TR
            clibuart_gpio_setValue(uartIqrfConfig->powerEnableGpioPin, 0);
            // Sleep for 300ms
            SLEEP(300);
            // Enable PWR for TR
            clibuart_gpio_setValue(uartIqrfConfig->powerEnableGpioPin, 1);
            SLEEP(1);
        }
    }

    if (uartIqrfConfig->busEnableGpioPin != -1) {
        clibuart_gpio_setValue(uartIqrfConfig->busEnableGpioPin, 1);
    } else {
        clibuart_gpio_setValue(uartIqrfConfig->uartEnableGpioPin, 1);
        clibuart_gpio_setValue(uartIqrfConfig->spiEnableGpioPin, 0);
        clibuart_gpio_setValue(uartIqrfConfig->i2cEnableGpioPin, 1);
    }

    // Sleep for 500ms (in this time TR module waits for sequence to switch to programming mode)
    SLEEP(500);

    if (uart_iqrf_open() == BASE_TYPES_OPER_OK) {
        libIsInitialized = 1;
        return BASE_TYPES_OPER_OK;
    } else {
        if (uartIqrfConfig->powerEnableGpioPin != -1)
            clibuart_gpio_cleanup(uartIqrfConfig->powerEnableGpioPin);
        if (uartIqrfConfig->busEnableGpioPin != -1) {
            clibuart_gpio_cleanup(uartIqrfConfig->busEnableGpioPin);
        } else {
            clibuart_gpio_cleanup(uartIqrfConfig->uartEnableGpioPin);
            clibuart_gpio_cleanup(uartIqrfConfig->spiEnableGpioPin);
            clibuart_gpio_cleanup(uartIqrfConfig->i2cEnableGpioPin);
        }
        if (uartIqrfConfig->pgmSwitchGpioPin != -1)
            clibuart_gpio_cleanup(uartIqrfConfig->pgmSwitchGpioPin);

        free(receiverControl.receiveBuffer);

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
int uart_iqrf_write(uint8_t *dataToWrite, unsigned int dataLen)
{
    uint8_t *dataToSend = NULL;
    int wlen;

    if (libIsInitialized == 0)
        return BASE_TYPES_LIB_NOT_INITIALIZED;

    if (fd < 0)
        return BASE_TYPES_OPER_ERROR;

    // checking input parameters
    if (dataToWrite == NULL)
        return BASE_TYPES_OPER_ERROR;

    if (check_data_len(dataLen))
        return BASE_TYPES_OPER_ERROR;

    dataToSend = malloc(256 * sizeof(uint8_t));
    if (dataToSend == NULL)
        return BASE_TYPES_OPER_ERROR;

    // initialize CRC
    senderControl.CRC = 0xFF;
    // start of packet character
    dataToSend[0] = HDLC_FRM_FLAG_SEQUENCE;
    // counter of sent bytes
    senderControl.packetCnt = 1;

    // send user data
    while (dataLen) {
        senderControl.packetCnt += write_byte_to_buffer(&dataToSend[senderControl.packetCnt], *(uint8_t *)dataToWrite);
        dataToWrite++;
        dataLen--;
    }

    // send CRC
    senderControl.packetCnt += write_byte_to_buffer(&dataToSend[senderControl.packetCnt], senderControl.CRC);
    // send stop of packet character
    dataToSend[senderControl.packetCnt++] = HDLC_FRM_FLAG_SEQUENCE;

    // send data to module
    wlen = write(fd, dataToSend, senderControl.packetCnt);
    // delay for output
#ifndef WIN32
    tcdrain(fd);
#endif

    free(dataToSend);

    if (wlen != senderControl.packetCnt)
        return BASE_TYPES_OPER_ERROR;

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
int uart_iqrf_read(uint8_t *readBuffer, uint8_t *dataLen, unsigned int timeout)
{
    uint64_t start;
    uint8_t inputChar;
    int rlen;

    // checking input parameter
    if (dataLen == NULL)
        return BASE_TYPES_OPER_ERROR;
    else
        *dataLen = 0;

    // checking input parameter
    if (readBuffer == NULL)
        return BASE_TYPES_OPER_ERROR;

    // checking if library is initialized
    if (libIsInitialized == 0)
        return BASE_TYPES_LIB_NOT_INITIALIZED;

    receiverControl.decodeInProgress = 0;

    start = get_ms_ts();
    while (1) {

        rlen = read(fd, receiverControl.receiveBuffer, SIZE_OF_UART_RX_BUFFER - 1);

        receiverControl.rBuffCnt = 0;
        while (rlen--) {
            inputChar = receiverControl.receiveBuffer[receiverControl.rBuffCnt++];

            if (receiverControl.decodeInProgress) {
                // end of packet or DPA structure is full
                if (inputChar == HDLC_FRM_FLAG_SEQUENCE || receiverControl.packetCnt >= 70) {
                    if (receiverControl.CRC == 0) {
                        *dataLen = receiverControl.packetCnt - 1;
                        return(BASE_TYPES_OPER_OK);
                    } else {
                        return (UART_IQRF_ERROR_CRC);
                    }
                }

                // discard received ESCAPE character
                if (inputChar == HDLC_FRM_CONTROL_ESCAPE) {
                    receiverControl.wasEscape = 1;
                    continue;
                }

                // previous character was ESCAPE
                if (receiverControl.wasEscape) {
                    receiverControl.wasEscape = 0;
                    inputChar ^= HDLC_FRM_ESCAPE_BIT;
                }

                // add Rx byte to CRC
                receiverControl.CRC = dpa_do_CRC8(inputChar, receiverControl.CRC);

                // write received char to buffer
                *((uint8_t *)readBuffer + receiverControl.packetCnt) = inputChar;
                receiverControl.packetCnt++;
            } else {
                if (inputChar == HDLC_FRM_FLAG_SEQUENCE) {
                    receiverControl.CRC = 0xFF;
                    receiverControl.packetCnt = 0;
                    receiverControl.wasEscape = 0;
                    receiverControl.delayRxTimeout = 0;
                    receiverControl.decodeInProgress = 1;
                }
            }
        }

        SLEEP(5);

        // check if receiver timeout has been elapsed
        if ((get_ms_ts() - start) > timeout) {
            if (receiverControl.decodeInProgress) {
                if (receiverControl.delayRxTimeout) {
                    break;
                } else {
                    receiverControl.delayRxTimeout = 1;
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
int uart_iqrf_open(void)
{
    if (fd != NO_FILE_DESCRIPTOR)
        return BASE_TYPES_OPER_ERROR;

#ifndef WIN32
    fd = open(uartIqrfConfig->uartDev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fd = NO_FILE_DESCRIPTOR;
        return BASE_TYPES_OPER_ERROR;
    }

    // configure port
    set_interface_attribs(fd, uartIqrfConfig->baudRate);
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
int uart_iqrf_close(void)
{
    int closeRes;

    if (fd == NO_FILE_DESCRIPTOR)
        return BASE_TYPES_LIB_NOT_INITIALIZED;

    if (fd < 0)
        return BASE_TYPES_OPER_ERROR;

    closeRes = close(fd);
    fd = NO_FILE_DESCRIPTOR;

    if (closeRes == -1)
        return BASE_TYPES_OPER_ERROR;

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
int uart_iqrf_destroy(void)
{
    if (libIsInitialized == 0)
        return BASE_TYPES_LIB_NOT_INITIALIZED;

    // after calling this method, the behavior of the library will be
    // like if the library was not initialized
    libIsInitialized = 0;

    // destroy used GPIO pins
    if (uartIqrfConfig->powerEnableGpioPin != -1)
        clibuart_gpio_cleanup(uartIqrfConfig->powerEnableGpioPin);
    if (uartIqrfConfig->busEnableGpioPin != -1)
        clibuart_gpio_cleanup(uartIqrfConfig->busEnableGpioPin);
    if (uartIqrfConfig->pgmSwitchGpioPin != -1)
        clibuart_gpio_cleanup(uartIqrfConfig->pgmSwitchGpioPin);

    // deallocate receive buffer
    free(receiverControl.receiveBuffer);

    return uart_iqrf_close();
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
static void clibuart_setup_gpio_path(const int gpio, const char *action, char *target, int len)
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
int clibuart_gpio_export(int num)
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
    if (ret)
        goto err;

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
int clibuart_gpio_unexport(int num)
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
    if (ret)
        goto err;

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
int clibuart_gpio_setDirection(int gpio, clibuart_gpio_direction dir)
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
    if (dir == GPIO_DIRECTION_IN)
        strncpy(buf, GPIO_DIRECTION_IN_STR, sizeof(buf));
    else if (dir == GPIO_DIRECTION_OUT)
        strncpy(buf, GPIO_DIRECTION_OUT_STR, sizeof(buf));

    ret = clibuart_write_data(fd, buf);
    if (ret)
        goto err;

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
clibuart_gpio_direction clibuart_gpio_getDirection(int gpio)
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

    if (!strcmp(buf, GPIO_DIRECTION_IN_STR))
        dir = GPIO_DIRECTION_IN;
    else if (!strcmp(buf, GPIO_DIRECTION_OUT_STR))
        dir = GPIO_DIRECTION_OUT;

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
int clibuart_gpio_setValue(int gpio, int val)
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
    if (ret)
        goto err;

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
int clibuart_gpio_getValue(int gpio)
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
int clibuart_gpio_setup(int gpio, clibuart_gpio_direction dir, int val)
{
    int ret;

    ret = clibuart_gpio_export(gpio);
    if (ret)
        return ret;

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
        if (ret)
            return ret;
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
int clibuart_gpio_cleanup(int gpio)
{
    return (clibuart_gpio_unexport(gpio));
}
