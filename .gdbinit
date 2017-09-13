define pi2c1
  printf "CR1: "
  x 0x40005400
  printf "CR2: "
  x 0x40005404
  printf "SR1: "
  x 0x40005414
  printf "SR2: "
  x 0x40005418
end

define pdma1
  printf "DMA1_ISR: "
  x 0x40020000
end

define pdma1-6
  printf "DMA1_6 CCR: "
  x 0x4002006c
end

target extended-remote :4242
load
b ITG3200::bus_callback
b I2C::dma_complete
b I2C::priv_rx_complete
b I2C::priv_tx_complete
