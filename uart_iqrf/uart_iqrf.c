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
#include <machines_def.h>
#include <sleepWrapper.h>

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
  uint8_t packetCnt;
  uint8_t CRC;
  uint8_t decodeInProgress;
  uint8_t wasEscape;
  uint8_t delayRxTimeout;
  int     rBuffCnt;
  int     timeoutTimer;
} T_UART_RECEIVER_CONTROL;

T_UART_RECEIVER_CONTROL  receiverControl;

/************************************/
/* Private variables                */
/************************************/
/** Indicates, whether this library is initialized or not. 0 - not initialized 1 - initialized. */
static int libIsInitialized = 0;

/** File descriptor of the device special file. */
static int fd = -1;

/** pointer to actual spi iqrf configuration structure */
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

	for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
    if (((seed ^ inData) & 0x01) == 0) {
      seed >>= 1;
    }
    else {
      seed = (seed >>= 1)^0x8C;
    }
    inData >>= 1;
  }
  return seed;
}


/**
 * write data byte to to buffer for TR module + make HDLC byte stuffing and comute CRC
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

  if (libIsInitialized == 1) {
    return BASE_TYPES_OPER_ERROR;
  }

  uartIqrfConfig = (T_UART_IQRF_CONFIG_STRUCT *)configStruct;

  if (uart_iqrf_open() == BASE_TYPES_OPER_OK){
    libIsInitialized = 1;
    return BASE_TYPES_OPER_OK;
  }
  else {
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

  if (libIsInitialized == 0) {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0) {
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
  tcdrain(fd);

  free(dataToSend);

  if (wlen != senderControl.packetCnt) {
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
* @return	@c BASE_TYPES_OPER_ERROR = error occures during read operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = SPI library is not initialized
* @return	@c BASE_TYPES_OPER_OK = data were successfully read
* @return	@c UART_IQRF_ERROR_CRC = mismatched CRC
* @return	@c UART_IQRF_ERROR_TIMEOUT = receiver timeout (no data read)
*/
int uart_iqrf_read(void *readBuffer, uint8_t *dataLen, unsigned int timeout)
{
  uint8_t *receiveBuffer = NULL;
  uint64_t start;
  uint8_t inputChar;
  int rlen;

  if (libIsInitialized == 0) {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0) {
    return BASE_TYPES_OPER_ERROR;
  }

  // checking input parameters
  if (readBuffer == NULL) {
    return BASE_TYPES_OPER_ERROR;
  }

  // allocate memory for received data
  receiveBuffer = malloc(SIZE_OF_UART_RX_BUFFER * sizeof(uint8_t));
  if (receiveBuffer == NULL) {
    return BASE_TYPES_OPER_ERROR;
  }

  receiverControl.decodeInProgress = 0;

  start = get_ms_ts();
  while (1) {

    rlen = read(fd, receiveBuffer, SIZE_OF_UART_RX_BUFFER - 1);

    receiverControl.rBuffCnt = 0;
    while (rlen--) {
      inputChar = receiveBuffer[receiverControl.rBuffCnt++];

      if (receiverControl.decodeInProgress) {
        // end of packet or DPA structure is full
        if (inputChar == HDLC_FRM_FLAG_SEQUENCE || receiverControl.packetCnt >= 70) {
          free(receiveBuffer);
          if (receiverControl.CRC == 0) {
            *dataLen = receiverControl.packetCnt - 1;
            return(BASE_TYPES_OPER_OK);
          }
          else {
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
      }
      else {
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
    if ((get_ms_ts() - start) > timeout){
      if (receiverControl.decodeInProgress){
        if (receiverControl.delayRxTimeout){
          break;
        }
        else{
          receiverControl.delayRxTimeout = 1;
          timeout += 500;
        }
      }
      else{
        break;
      }
    }
  }

  free(receiveBuffer);

  return (UART_IQRF_ERROR_TIMEOUT);
}

/**
* set parameters of uart port
*
* @param	fd	- port identificator
* @param	speed		- comunication baudrate
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during parameters set
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
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

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
* @return	@c BASE_TYPES_OPER_ERROR = error occures during UART initialization
* @return	@c BASE_TYPES_OPER_OK = UART channel initialized OK
*/
int uart_iqrf_open(void)
{
  if (fd != NO_FILE_DESCRIPTOR) {
    return BASE_TYPES_OPER_ERROR;
  }

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
* @return	@c BASE_TYPES_OPER_ERROR = error occures during UART channel closing
* @return	@c BASE_TYPES_OPER_OK = UART channel closed OK
*/
int uart_iqrf_close(void)
{
  int closeRes;

  if (fd == NO_FILE_DESCRIPTOR) {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  if (fd < 0) {
    return BASE_TYPES_OPER_ERROR;
  }

  closeRes = close(fd);
  fd = NO_FILE_DESCRIPTOR;

  if (closeRes == -1) {
    return BASE_TYPES_OPER_ERROR;
  }

  return BASE_TYPES_OPER_OK;
}

/**
* Destroys IQRF UART library object and releases UART port
*
*
* @return	@c BASE_TYPES_OPER_ERROR = error occures during SPI destroy operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = UART library is not initialized
* @return	@c BASE_TYPES_OPER_OK = UART channel was successfully closed
*/
int uart_iqrf_destroy(void)
{

  if (libIsInitialized == 0) {
    return BASE_TYPES_LIB_NOT_INITIALIZED;
  }

  // after calling this method, the behavior of the library will be
  // like if the library was not initialized
  libIsInitialized = 0;

  return uart_iqrf_close();
}
