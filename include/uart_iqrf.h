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
 * @file    uart_iqrf.h
 * @date    07.12.2018
 * @ver     1.0.2
 */

#ifndef __UART_IQRF_H
#define __UART_IQRF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "declspec.h"

#define UART_IQRF_DECLSPEC

// Constant is used to specify array. Therefore, they can not be defined as static const uint8_t.
#define UART_DEV_CAPACITY 128
#define UART_IQRF_MAX_DATA_LENGTH  64
#define SIZE_OF_UART_RX_BUFFER  (4 * UART_IQRF_MAX_DATA_LENGTH)


/** IQRF uart Error constants. */
typedef enum uart_iqrf_Errors {
    ///< An enum constant representing results without errors
    BASE_TYPES_OPER_OK = 0,
    ///< An enum constant representing results where some error occurs
    BASE_TYPES_OPER_ERROR = -1,
    ///< An enum constant representing operation on not initialized library
    BASE_TYPES_LIB_NOT_INITIALIZED = -2,
    ///< An enum constant representing the uart iqrf error CRC mismatch
    UART_IQRF_ERROR_CRC = -10,       /* CRC mismatch */
    ///< An enum constant representing the uart receiver timeout
    UART_IQRF_ERROR_TIMEOUT = -11    /* receiver timeout */
} uqrt_iqrf_Errors;

typedef enum _tr_module_reset
{
    TR_MODULE_RESET_ENABLE = 0,
    TR_MODULE_RESET_DISABLE
} tr_module_reset;

typedef struct {
    /** Device file name*/
    char uartDev[UART_DEV_CAPACITY+1];
    int baudRate;
    int64_t powerEnableGpioPin;      // GPIO to enable power supply to TR module
    int64_t busEnableGpioPin;        // GPIO to enable function of SPI bus
    int64_t pgmSwitchGpioPin;        // GPIO to switch TR module to PGM mode
    int64_t spiEnableGpioPin;        // GPIO to enable function of SPI bus
    int64_t uartEnableGpioPin;       // GPIO to enable function of UART bus
    int64_t i2cEnableGpioPin;        // GPIO to enable function of I2C bus
    tr_module_reset trModuleReset;  // enable / disable TR module reset during library initialization
} T_UART_IQRF_CONFIG_STRUCT;

typedef struct {
    uint8_t packetCnt;
    uint8_t CRC;
} T_UART_SEND_CONTROL;

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

typedef struct {
    int initialized;
    int fd;
    T_UART_SEND_CONTROL sendControl;
    T_UART_RECEIVER_CONTROL recvControl;
} T_UART_SOCKET_CONTROL;

/**
* Initialization of the UART for IQRF module with advanced setting
*
* @param	configStruct - configuration structure
*
* @return	@c BASE_TYPES_OPER_ERROR = initialization failed
* @return	@c BASE_TYPES_OPER_OK = initialization was correct
*/
UART_IQRF_DECLSPEC int uart_iqrf_init(const T_UART_IQRF_CONFIG_STRUCT *uartConfig, T_UART_SOCKET_CONTROL *socket);

/**
* Writes data to TR module
*
* @param	dataToWrite	- data to be written to TR module
* @param	dataLen		- length (in bytes) of the data
*
* @return	@c BASE_TYPES_OPER_ERROR = error occurs during write operation
* @return	@c BASE_TYPES_LIB_NOT_INITIALIZED = uart library is not initialized
* @return	@c BASE_TYPES_OPER_OK = data was successfully written
*/
UART_IQRF_DECLSPEC int uart_iqrf_write(T_UART_SOCKET_CONTROL *socket, uint8_t *dataToWrite, unsigned int dataLen);

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
UART_IQRF_DECLSPEC int uart_iqrf_read(T_UART_SOCKET_CONTROL *socket, uint8_t *readBuffer, uint8_t *dataLen, unsigned int timeout);


/**
 * Terminates the library and frees up used resources.
 *
 * @return	@c BASE_TYPES_OPER_OK if operation has performed successfully.
 * @return	@c BASE_TYPES_OPER_ERROR if an error has occurred during operation.
 * @return	@c BASE_TYPES_LIB_NOT_INITIALIZED if the library has not been initialized.
 */
UART_IQRF_DECLSPEC int uart_iqrf_destroy(const T_UART_IQRF_CONFIG_STRUCT *uartConfig ,T_UART_SOCKET_CONTROL *socket);

#ifdef __cplusplus
}
#endif

#endif // __UART_IQRF_H
