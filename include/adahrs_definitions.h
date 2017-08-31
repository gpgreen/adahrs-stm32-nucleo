/*
 * adahrs_definitions.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_DEFINITIONS_H_
#define ADAHRS_DEFINITIONS_H_


// ----------------------------------------------------------------------------
// size of work queue buffer
// ----------------------------------------------------------------------------
#define WORK_QUEUE_LENGTH                8
#define WORKQUEUE_IRQ_PRIORITY           0xF
#define WORKQUEUE_IRQ_MASKING            0x10

// ----------------------------------------------------------------------------
// size of UART tx/rx buffers
// ----------------------------------------------------------------------------
#define TX_BUFFER_SIZE                   64
#define RX_BUFFER_SIZE                   64

// ----------------------------------------------------------------------------
// which DMA's are we using
// ----------------------------------------------------------------------------
//#define DMA1_CHANNEL1_USED
//#define DMA1_CHANNEL2_USED
//#define DMA1_CHANNEL3_USED
//#define DMA1_CHANNEL4_USED
//#define DMA1_CHANNEL5_USED
#define DMA1_CHANNEL6_USED
#define DMA1_CHANNEL7_USED

#ifdef STM32F10X_HD_VL

#define DMA2_CHANNEL1_USED
#define DMA2_CHANNEL2_USED
#define DMA2_CHANNEL3_USED
#define DMA2_CHANNEL4_USED
#define DMA2_CHANNEL5_USED

#endif // STM32F10X_HD_VL

// ----------------------------------------------------------------------------
#endif /* ADAHRS_DEFINITIONS_H_ */
