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

#ifndef __MACHINES_DEF_H
#define __MACHINES_DEF_H

// select used platform (uncomment one selection)
// #define RPI_1
#define RPI_3
// #define UP
// #define UP2
// #define OPIZ
// #define OPIZ2
// #define AXON

#if defined  (RPI_1) || defined (RPI_3)

/* !!! if GPIO pin is not implemented, define it as -1 */

/** PGM Switch GPIO. */
#define PGM_SWITCH_GPIO (22)
/** Bus enable GPIO. */
#define BUS_ENABLE_GPIO (7)
/** Power enable GPIO. */
#define POWER_ENABLE_GPIO (23)

#define UART_IQRF_DEFAULT_SPEED  B57600

#ifndef UART_IQRF_DEFAULT_DEVICE
    #ifdef RPI_1
        #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyAMA0"		// Raspberry PI 1
    #endif

    #ifdef RPI_3
        #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyS0"		// Raspberry PI 3
    #endif
#endif

#endif /* RPI_x */


#ifdef UP

/* !!! if GPIO pin is not implemented, define it as -1 */

/** PGM Switch GPIO. */
#define PGM_SWITCH_GPIO (22)
/** Bus enable GPIO. */
#define BUS_ENABLE_GPIO (7)
/** Power enable GPIO. */
#define POWER_ENABLE_GPIO (23)

#define UART_IQRF_DEFAULT_SPEED  B57600

#ifndef UART_IQRF_DEFAULT_DEVICE
    #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyS1"
#endif

#endif /* UP */


#ifdef UP2

/* !!! if GPIO pin is not implemented, define it as -1 */

/** PGM Switch GPIO. */
#define PGM_SWITCH_GPIO (22)
/** Bus enable GPIO. */
#define BUS_ENABLE_GPIO (7)
/** Power enable GPIO. */
#define POWER_ENABLE_GPIO (23)

#define UART_IQRF_DEFAULT_SPEED  B57600

#ifndef UART_IQRF_DEFAULT_DEVICE
    #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyS1"
#endif

#endif /* UP2 */


#ifdef OPIZ

/* !!! if GPIO pin is not implemented, define it as -1 */

/** PGM Switch GPIO. */
#define PGM_SWITCH_GPIO (3)
/** Bus enable GPIO. */
#define BUS_ENABLE_GPIO (10)
/** Power enable GPIO. */
#define POWER_ENABLE_GPIO (19)

#define UART_IQRF_DEFAULT_SPEED  B57600

#ifndef UART_IQRF_DEFAULT_DEVICE
    #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyS1"
#endif

#endif /* OPI */


#ifdef OPIZ2

/* !!! if GPIO pin is not implemented, define it as -1 */

/** PGM Switch GPIO. */
#define PGM_SWITCH_GPIO (3)
/** Bus enable GPIOs. */
#define BUS_ENABLE_GPIO (-1)
#define UART_ENABLE_GPIO (6)
#define SPI_ENABLE_GPIO (10)
#define I2C_ENABLE_GPIO (7)
/** Power enable GPIO. */
#define POWER_ENABLE_GPIO (19)

#define UART_IQRF_DEFAULT_SPEED  B57600

#ifndef UART_IQRF_DEFAULT_DEVICE
    #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyS1"
#endif

#endif /* OPI2 */


#ifdef AXON

/* !!! if GPIO pin is not implemented, define it as -1 */

/** PGM Switch GPIO. */
#define PGM_SWITCH_GPIO (-1)
/** Bus enable GPIO. */
#define BUS_ENABLE_GPIO (18)
/** Power enable GPIO. */
#define POWER_ENABLE_GPIO (19)

#define UART_IQRF_DEFAULT_SPEED  B57600

#ifndef UART_IQRF_DEFAULT_DEVICE
    #define UART_IQRF_DEFAULT_DEVICE "/dev/ttyS0"
#endif

#endif /* UNIPI AXON */

#endif /* __MACHINES_DEF_H */
