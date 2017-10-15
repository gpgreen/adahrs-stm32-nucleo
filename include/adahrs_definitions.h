/*
 * adahrs_definitions.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_DEFINITIONS_H_
#define ADAHRS_DEFINITIONS_H_

// Contains definitions for hard-coded values in ADAHRS application
// Also defines which hardware components are used if allowed by their
// drivers (see USART1_USED, etc)

// ----------------------------------------------------------------------------
// Timer frequencies
// ----------------------------------------------------------------------------
#define TIMER_FREQUENCY_HZ              (1000u)

// ----------------------------------------------------------------------------
// priorities
// ----------------------------------------------------------------------------
#define I2C1_IRQ_PRIORITY                2
#define SPI1_IRQ_PRIORITY                3
#define DMA1_IRQ_PRIORITY                4
#define ADXL_IRQ_PRIORITY                5
#define ITG_IRQ_PRIORITY                 5
#define USART_IRQ_PRIORITY               6
#define COMMAND_IRQ_PRIORITY             7
#define TIMER_IRQ_PRIORITY               0xf
#define WORKQUEUE_IRQ_MASKING            0x10 /* which irq priorities are masked 
                                                 in work queue critical section */
#define USART_IRQ_MASKING                0x50 /* which irq priorities are masked 
                                                 in work queue critical section */
#define TIMER_IRQ_MASKING                0x10 /* which irq priorities are masked 
                                                 in timer list critical sections */

// ----------------------------------------------------------------------------
// size of work queue buffer
// ----------------------------------------------------------------------------
#define WORK_QUEUE_LENGTH                16

// ----------------------------------------------------------------------------
// size of command tx/rx buffers
// ----------------------------------------------------------------------------

// Maximum number of data bytes that can be stored in a packet
#define MAX_PACKET_DATA                  72
#define COMMAND_BUFFER_SIZE              96 /* packet can be MAX_PACKET_DATA + 7 bytes long */
#define TX_PKT_RING_BUFFER_SIZE          10
#define RX_PKT_RING_BUFFER_SIZE          10

// ----------------------------------------------------------------------------
// size of USART tx/rx buffers
// ----------------------------------------------------------------------------
#define TX_BUFFER_SIZE                   96
#define RX_BUFFER_SIZE                   96

// ----------------------------------------------------------------------------
// which USART(s) in use
// ----------------------------------------------------------------------------
#define USART1_USED
//#define USART2_USED
//#define USART3_USED

// ----------------------------------------------------------------------------
// which DMA(s) in use
// ----------------------------------------------------------------------------
//#define DMA1_CHANNEL1_USED
#define DMA1_CHANNEL2_USED
#define DMA1_CHANNEL3_USED
#define DMA1_CHANNEL4_USED
#define DMA1_CHANNEL5_USED
#define DMA1_CHANNEL6_USED
#define DMA1_CHANNEL7_USED

#ifdef STM32F10X_HD_VL

//#define DMA2_CHANNEL1_USED
//#define DMA2_CHANNEL2_USED
//#define DMA2_CHANNEL3_USED
//#define DMA2_CHANNEL4_USED
//#define DMA2_CHANNEL5_USED

#endif // STM32F10X_HD_VL

// ----------------------------------------------------------------------------
// which SPI(s) in use
// ----------------------------------------------------------------------------
#define SPI1_USED
//#define SPI2_USED

#ifdef STM32F10X_HD_VL

//#define SPI3_USED

#endif // STM32F10X_HD_VL

// ----------------------------------------------------------------------------
// which I2C(s) in use
// ----------------------------------------------------------------------------
#define I2C1_USED
//#define I2C2_USED

// ----------------------------------------------------------------------------
// Debugging, Tracing
// ----------------------------------------------------------------------------
#define USE_DELAYTIMER_TRACE
#define USE_WORKQUEUE_TRACE

// ----------------------------------------------------------------------------
#endif /* ADAHRS_DEFINITIONS_H_ */
