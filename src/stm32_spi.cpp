/*
 * stm32_spi.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: ggreen
 *
 */

// ----------------------------------------------------------------------------

#include <string.h>
#include "stm32_spi.h"
#include "stm32_dma.h"
#include "work_queue.h"

SPI::SPI(int device_no)
    : _devno(device_no), _tx_dma(nullptr), _rx_dma(nullptr),
      _tx_buffer(nullptr), _tx_busy(false), _rx_busy(false),
      _buffer_len(0)
{
    if (device_no == 1)
    {
#ifdef DMA1_CHANNEL3_USED
        _tx_dma = &DMA1Channel3;
#endif
#ifdef DMA1_CHANNEL2_USED
        _rx_dma = &DMA1Channel2;
#endif
        _spi = SPI1;
        _irqno = SPI1_IRQn;
    }
    else if (device_no == 2)
    {
#ifdef DMA1_CHANNEL5_USED
        _tx_dma = &DMA1Channel5;
#endif
#ifdef DMA1_CHANNEL4_USED
        _rx_dma = &DMA1Channel4;
#endif
        _spi = SPI2;
        _irqno = SPI2_IRQn;
    }
#ifdef STM32F10X_HD_VL

#ifdef SPI3_USED
    else if (device_no == 3)
    {
#ifdef DMA2_CHANNEL5_USED
        _tx_dma = &DMA2Channel2;
#endif
#ifdef DMA2_CHANNEL4_USED
        _rx_dma = &DMA2Channel1;
#endif
        _spi = SPI3;
        _irqno = SPI3_IRQn;
    }
#endif
    
#endif  // STM32F10X_HD_VL
    else
    {
        while (1)
            ;
    }
}

// ----------------------------------------------------------------------------

void SPI::begin(uint8_t priority, uint8_t subpriority)
{
    // enable the SPI pins and port numbers
    uint16_t miso_pin = 0;
    uint16_t mosi_pin = 0;
    uint16_t sck_pin = 0;
    uint16_t ss_pin = 0;
    GPIO_TypeDef* pinport = nullptr;

    if (_devno == 1)
    {
        ss_pin = GPIO_Pin_4;
        sck_pin = GPIO_Pin_5;
        miso_pin = GPIO_Pin_6;
        mosi_pin = GPIO_Pin_7;
        pinport = GPIOA;
    }
    else if (_devno == 2)
    {
        ss_pin = GPIO_Pin_12;
        sck_pin = GPIO_Pin_13;
        miso_pin = GPIO_Pin_14;
        mosi_pin = GPIO_Pin_15;
        pinport = GPIOB;
    }

    // configure the pins
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure SPI MISO as input, floating
    GPIO_InitStructure.GPIO_Pin = miso_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(pinport, &GPIO_InitStructure);

    // Configure SPI SCK as output, push-pull
    GPIO_InitStructure.GPIO_Pin = sck_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(pinport, &GPIO_InitStructure);

    // Configure SPI MOSI as output, push-pull
    GPIO_InitStructure.GPIO_Pin = mosi_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(pinport, &GPIO_InitStructure);

    // Configure SPI SS as output, push-pull
    GPIO_InitStructure.GPIO_Pin = ss_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(pinport, &GPIO_InitStructure);

    /* SPI is configured as follows:
       - Both lines used for data
       - Master model
       - Data size 8 bits
       - Receive and transmit enabled
       - CPOL low, CPHA start
       - prescalar = 4; 72/4 = 18 MHz
       - MSB bit first
       - hardware controls slave select
    */
    SPI_InitTypeDef SPI_InitStructure;

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 0U;

    // configure SPI
    SPI_I2S_DeInit(_spi);
    SPI_Init(_spi, &SPI_InitStructure);

    // configure interrupt vector
    configure_nvic(priority, subpriority);
}

void SPI::configure_nvic(uint8_t priority, uint8_t subpriority)
{
    // enable the IRQ
    NVIC_InitTypeDef NVIC_InitStructure;

    // enable the DMA Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = _irqno;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

bool SPI::send(uint8_t* txdata, int buflen)
{
    if (_tx_busy)
        return false;
    
    _tx_buffer = txdata;
    _buffer_len = buflen;

    tx_start(false);

    return true;
}

void SPI::tx_start_irq(void* data)
{
    SPI* spi = reinterpret_cast<SPI*>(data);
    spi->tx_start(true);
}

void SPI::tx_start(bool in_irq)
{
    if (_tx_busy || _buffer_len <= 0)
        return;

    _tx_busy = true;

    // Configure the DMA controller to make the transfer
    DMA_InitTypeDef DMA_txInit;
    DMA_InitTypeDef DMA_rxInit;

    // Configure the tx DMA controller to make the transfer
    if (_devno == 1)
        DMA_txInit.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
    else if (_devno == 2)
        DMA_txInit.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR);
    else if (_devno == 3)
        DMA_txInit.DMA_PeripheralBaseAddr = (uint32_t) &(SPI3->DR);
    DMA_txInit.DMA_MemoryBaseAddr = (uint32_t) _tx_buffer;
    DMA_txInit.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_txInit.DMA_BufferSize = _buffer_len;
    DMA_txInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_txInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_txInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_txInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_txInit.DMA_Mode = DMA_Mode_Normal;
    DMA_txInit.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_txInit.DMA_M2M = DMA_M2M_Disable;

    // copy the tx DMA init to rx
    memcpy(&DMA_rxInit, &DMA_txInit, sizeof(DMA_InitTypeDef));
    DMA_rxInit.DMA_DIR = DMA_DIR_PeripheralSRC;
    
    // check that BSY flag is cleared, before starting the new transmission
    if (SPI_I2S_GetFlagStatus(_spi, SPI_I2S_FLAG_BSY) != RESET) {
        _tx_busy = false;
        if (in_irq)
            g_work_queue.add_work_irq(SPI::tx_start_irq, this);
        else
            g_work_queue.add_work(SPI::tx_start_irq, this);
        return;
    }

    if (!_tx_dma->start(&DMA_txInit, SPI::tx_dma_complete, this))
    {
        _tx_busy = false;
        if (in_irq)
            g_work_queue.add_work_irq(SPI::tx_start_irq, this);
        else
            g_work_queue.add_work(SPI::tx_start_irq, this);
        return;
    }
    if (!_rx_dma->start(&DMA_rxInit, SPI::rx_dma_complete, this))
    {
        _tx_dma->cancel();
        _tx_busy = false;
        if (in_irq)
            g_work_queue.add_work_irq(SPI::tx_start_irq, this);
        else
            g_work_queue.add_work(SPI::tx_start_irq, this);
        return;
    }
    // spi dma tx&rx request enabled
    SPI_I2S_DMACmd(_spi, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

    // enable SPI
    SPI_Cmd(_spi, ENABLE);
}

void SPI::tx_dma_complete(void* data)
{
    SPI* spi = reinterpret_cast<SPI*>(data);
    spi->_tx_busy = false;
}

void SPI::rx_dma_complete(void* data)
{
    SPI* spi = reinterpret_cast<SPI*>(data);
    spi->_rx_busy = false;
    // disable SPI
    SPI_Cmd(spi->_spi, DISABLE);
}

void SPI::priv_rx_complete()
{
    _rx_busy = false;
    // disable SPI
    SPI_Cmd(_spi, DISABLE);
}

#ifdef SPI1_USED

SPI spi1(1);

void SPI1_IRQHandler(void)
{
    if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != RESET) {
        spi1.priv_rx_complete();
        // clear the RXNE bit in the SR register (not needed if data read)
        SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    }
}

#endif

#ifdef SPI2_USED

SPI spi2(2);

void SPI2_IRQHandler(void)
{
    if (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) != RESET) {
        spi2.priv_rx_complete();
        // clear the RXNE bit in the SR register (not needed if data read)
        SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
    }
}

#endif

#ifdef STM32F10X_HD_VL

#ifdef SPI3_USED
SPI spi3(3);

void SPI3_IRQHandler(void)
{
    if (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != RESET) {
        spi3.priv_rx_complete();
        // clear the RXNE bit in the SR register (not needed if data read)
        SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_RXNE);
    }
}

#endif

#endif // STM32F10X_HD_VL
// ----------------------------------------------------------------------------
