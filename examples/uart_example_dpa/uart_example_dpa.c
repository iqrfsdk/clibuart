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
#include <termios.h>
//#include <linux/types.h>
#include "machines_def.h"
#include "uart_iqrf.h"
#include "sleepWrapper.h"

#include "DPA.h"

/** Defines whole DPA message. */
typedef union
{
    uint8_t Buffer[64];

    struct
    {
        uint16_t NADR;
        uint8_t PNUM;
        uint8_t PCMD;
        uint16_t HWPID;
        uint8_t ResponseCode;
        uint8_t DpaValue;
        TDpaMessage DpaMessage;
    } Response;

    struct
    {
        uint16_t NADR;
        uint8_t PNUM;
        uint8_t PCMD;
        uint16_t HWPID;
        TDpaMessage DpaMessage;
    } Request;
} T_DPA_PACKET;

/** Defines an alias representing the LED colors. */
typedef enum _LedColor_t
{
    Red,
    Green
} LedColor_t;

/************************************/
/* Private constants                */
/************************************/

/** Time interval between DPA messagess controls LEDs. */
const unsigned long TIME_BETWEEN_LEDS_MS = 2000;

/************************************/
/* Private functions predeclaration */
/************************************/

int open_communication(void);

int close_communication(void);

void print_error_and_exit(const char *userMessage, int error, int retValue);

int execute_dpa_command(const uint8_t *dpaMessage, int dataLen);

void pulse_led(uint16_t address, LedColor_t color);

void print_data_in_hex(unsigned char *data, unsigned int length);

void empty_rx_buffer(void);

uint16_t dpa_get_estimated_timeout(void);

/************************************/
/* Private variables                */
/** State of the communication.		*/
int communicationOpen = -1;
/** DPA response. */
T_DPA_PACKET dpaResponsePacket;

/** UART IQRF configuration structure */
T_UART_IQRF_CONFIG_STRUCT myUartIqrfConfig;

/**
 * Main entry-point for this application.
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 */
int main(void)
{
    int i;
    int operResult;

    printf("DPA UART example application.\n\r");

    operResult = open_communication();
    if (operResult) {
        printf("Initialization failed: %d \n\r", operResult);
        return operResult;
    }

    printf("Communication port has been successfully open ....\n\r");

    empty_rx_buffer();

    for (i = 0; i < 3; ++i) {
        pulse_led(COORDINATOR_ADDRESS, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulse_led(COORDINATOR_ADDRESS, Red);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);

        pulse_led(BROADCAST_ADDRESS, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulse_led(BROADCAST_ADDRESS, Red);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);

        pulse_led(0x01, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulse_led(0x02, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
        pulse_led(0x03, Green);
        //nanosleep(&delayTime, 0);
        SLEEP(TIME_BETWEEN_LEDS_MS);
    }

    printf("End of DPA example\n\r");

    return 0;
}

/**
 * Prints specified user message and specified error description to standard output, cleans up
 * the Rpi_spi_iqrf library, and exits the program with specified return gpio_getValue.
 *
 * @param	userMessage	Message describing the error.
 * @param	error	   	The error identificator.
 * @param	retValue   	The returned value.
 */
void print_error_and_exit(const char *userMessage, int error, int retValue)
{
    printf("%s: %d", userMessage, error);
    close_communication();
    exit(retValue);
}

/**
 * Opens SPI communication.
 *
 * @return	0 for success.
 */
int open_communication(void)
{
    int operResult;

    strcpy (myUartIqrfConfig.uartDev, UART_IQRF_DEFAULT_DEVICE);
    myUartIqrfConfig.baudRate = UART_IQRF_DEFAULT_SPEED;
    myUartIqrfConfig.enableGpioPin = ENABLE_GPIO;
    myUartIqrfConfig.spiMasterEnGpioPin = SPI_MASTER_EN_GPIO;
    myUartIqrfConfig.spiPgmSwGpioPin = PGM_SW_GPIO;

    operResult = uart_iqrf_init(&myUartIqrfConfig);
    if (operResult < 0) {
        printf("Initialization failed: %d \n\r", operResult);
        return operResult;
    }

    printf("Communication port is open.\n\r");
    communicationOpen = 1;
    return 0;
}

/**
 * Closes the IQRF communication.
 *
 * @return	0 for success, -1 if communication is already closed.
 */
int close_communication(void)
{
    if (communicationOpen < 0) {
        printf("Communication port was not open.\n\r");
        return -1;
    }

    uart_iqrf_destroy();
    return 0;
}


/**
 * Pulse with LED selected by @c color on address defined by @c address.
 *
 * @param	address	The address of the destination module.
 * @param	color  	Color of the LED on module.
 */
void pulse_led(uint16_t address, LedColor_t color)
{
    T_DPA_PACKET message;
    message.Request.NADR = address;
    message.Request.HWPID = HWPID_DoNotCheck;
    message.Request.PCMD = CMD_LED_PULSE;
    if (color == Red) {
        message.Request.PNUM = PNUM_LEDR;
    }
    else {
        message.Request.PNUM = PNUM_LEDG;
    }

    execute_dpa_command((const uint8_t *)&message.Buffer, sizeof(TDpaIFaceHeader));
}

/**
 * Executes the DPA command operation.
 *
 * @param	dpaMessage	DPA message to be executed.
 * @param	dataLen   	Length of the message.
 *
 * @return	0 for success, in case of error, the app is closed.
 */
int execute_dpa_command(const uint8_t *dpaMessage, int dataLen)
{
    int operResult;
    int rxTimeout;
    uint8_t rxDataLen;
    uint8_t operationInProgress;

    typedef enum {
        PROCESS_CONFIMATION = 0,
        PROCESS_RESPONSE
    } T_CR_SM;

    T_CR_SM crSm;

    printf("\n\r");
    printf("Data ready to sent to UART device:\n\r");
    print_data_in_hex((unsigned char *)dpaMessage, dataLen);

    // sending some data to TR module
    operResult = uart_iqrf_write((uint8_t *)dpaMessage, dataLen);
    if (operResult) {
        print_error_and_exit("Error during data sending", 0, operResult);
    }
    printf("Data successfully sent to UART device.\n\r");

    operationInProgress = 1;
    rxTimeout = 500;

    if ((((T_DPA_PACKET *)dpaMessage)->Request.NADR == COORDINATOR_ADDRESS) || (((T_DPA_PACKET *)dpaMessage)->Request.NADR == LOCAL_ADDRESS)) {
        crSm = PROCESS_RESPONSE;
    }
    else {
        crSm = PROCESS_CONFIMATION;
    }

    while (operationInProgress) {

        printf("Waiting for data.\n\r");
        operResult = uart_iqrf_read((uint8_t *)&dpaResponsePacket, &rxDataLen, rxTimeout);

        switch (crSm) {
            case PROCESS_CONFIMATION: {
                if (operResult == BASE_TYPES_OPER_OK) {
                    printf("Received confirmation data:\n\r");
                    print_data_in_hex((unsigned char *)&dpaResponsePacket, rxDataLen);
                    if (dpaResponsePacket.Response.ResponseCode == STATUS_CONFIRMATION) {
                        printf("Confirmation OK.\n\r");
                        if (((T_DPA_PACKET *)dpaMessage)->Request.NADR == BROADCAST_ADDRESS) {
                            SLEEP(dpa_get_estimated_timeout());
                            operationInProgress = 0;
                        }
                        else {
                            rxTimeout = dpa_get_estimated_timeout();
                        }
                    }
                    else {
                        printf("Confirmation ERROR.\n\r");
                    }
                }
                else {
                    if (operResult == UART_IQRF_ERROR_TIMEOUT) {
                        printf("Confirmation TIMEOUT.\n\r");
                    }
                    else {
                        printf("Confirmation ERROR.\n\r");
                    }
                }

                crSm = PROCESS_RESPONSE;
            }
            break;

            case PROCESS_RESPONSE: {
                if (operResult == BASE_TYPES_OPER_OK) {
                    printf("Received response data:\n\r");
                    print_data_in_hex((unsigned char *)&dpaResponsePacket, rxDataLen);
                    if (dpaResponsePacket.Response.ResponseCode == STATUS_NO_ERROR) {
                        printf("Response OK.\n\r");
                    }
                    else {
                        printf("Response ERROR.\n\r");
                    }
                }
                else {
                    if (operResult == UART_IQRF_ERROR_TIMEOUT) {
                        printf("Response TIMEOUT.\n\r");
                    }
                    else {
                        printf("Response ERROR.\n\r");
                    }
                }

                operationInProgress = 0;
            }
            break;
        }
    }

    return 0;
}

/**
 * Prints specified data onto standard output in hex format.
 *
 * @param [in,out]	data	Pointer to data buffer.
 * @param	length			The length of the data.
 */
void print_data_in_hex(unsigned char *data, unsigned int length)
{
    int i = 0;

    for (i = 0; i < length; i++) {
        printf("0x%.2x", (int) *data);
        data++;
        if (i != (length - 1)) {
            printf(" ");
        }
    }
    printf("\n\r");
}

/**
 * Make system UART Rx buffer empty *
 */
void empty_rx_buffer(void)
{
    int operResult;
    int cnt;
    uint8_t rxDataLen;
    uint8_t rxDataBuffer[UART_IQRF_MAX_DATA_LENGTH];

    cnt = 5;
    while (cnt){
        operResult = uart_iqrf_read(rxDataBuffer, &rxDataLen, 100);
        if (operResult == UART_IQRF_ERROR_TIMEOUT) {
            cnt--;
        }
    }
}

/**
 * Returns estimated timeout for response packet in miliseconds
 * @return Estimated timeout for response packet in ms (computed from confirmation packet data)
 */
uint16_t dpa_get_estimated_timeout(void)
{
  uint16_t responseTimeSlotLength;
  uint16_t estimatedTimeout = (uint16_t) (dpaResponsePacket.Response.DpaMessage.IFaceConfirmation.Hops + 1) * (uint16_t) dpaResponsePacket.Response.DpaMessage.IFaceConfirmation.TimeSlotLength * 10;
  // DPA in diagnostic mode
  if (dpaResponsePacket.Response.DpaMessage.IFaceConfirmation.TimeSlotLength == 20) {
    responseTimeSlotLength = 200;
  }
  else {
    if (dpaResponsePacket.Response.DpaMessage.IFaceConfirmation.TimeSlotLength > 6) {
      // DPA in LP mode
      responseTimeSlotLength = 110;
    }
    else {
      // DPA in STD mode
      responseTimeSlotLength = 60;
    }
  }
  estimatedTimeout += ((uint16_t) (dpaResponsePacket.Response.DpaMessage.IFaceConfirmation.HopsResponse + 1) * responseTimeSlotLength + 40 + 100);
  return(estimatedTimeout);
}
