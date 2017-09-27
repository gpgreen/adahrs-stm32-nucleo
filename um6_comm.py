#!/usr/bin/env python3

import serial
import sys

# definitions of states for USART state machine
USART_STATE_WAIT = 1
USART_STATE_TYPE = 2
USART_STATE_ADDRESS = 3
USART_STATE_DATA = 4
USART_STATE_CHECKSUM = 5

# flags for type of packet address
ADDRESS_TYPE_CONFIG = 0
ADDRESS_TYPE_DATA = 1
ADDRESS_TYPE_COMMAND = 2

# flags for interpreting packet type byte
PACKET_HAS_DATA = (1 << 7)
PACKET_IS_BATCH = (1 << 6)
PACKET_BATCH_LENGTH_MASK = 0xf
PACKET_BATCH_LENGTH_OFFSET = 2

g_rx_buffer = bytearray();
g_rx_offset = 0
g_rx_counter = 0
g_state = USART_STATE_WAIT

class USARTPacket:
    def __init__(self):
        self.PT = 0
        self.address = 0
        self.address_type = 0
        self.data_length = 0
        self.packet_data = bytearray()
        self.checksum = 0

    def compute_checksum(self):
        pass

    def print(self):
        pass
        
g_packet = USARTPacket()

def process_next_char(data):
    global g_rx_buffer, g_rx_offset, g_rx_counter, g_packet

    g_rx_buffer.append(data);
    g_rx_offset += 1

    last_byte = g_rx_buffer[g_rx_offset]
    
    if g_state == USART_STATE_TYPE:
        if g_rx_counter == 0 and last_byte == 's':
            g_rx_counter += 1
        elif g_rx_counter == 1 and last_byte == 'n':
            g_rx_counter += 1
        elif g_rx_counter == 2 and last_byte == 'p':
            g_rx_counter = 0
            g_state = USART_STATE_ADDRESS
        else:
            g_rx_counter = 0
            
    elif g_state == USART_STATE_TYPE:
        g_packet.PT = last_byte
        g_state = USART_STATE_ADDRESS

    elif g_state == USART_STATE_ADDRESS:
        g_packet.address = last_byte

        if g_packet.address < 12:
            g_packet.address_type = ADDRESS_TYPE_CONFIG
        elif g_packet.address >= 12 and g_packet.address < 100:
            g_packet.address_type = ADDRESS_TYPE_DATA
        else:
            g_packet.address_type = ADDRESS_TYPE_COMMAND

        if (g_packet.PT & PACKET_HAS_DATA) == 0:
            g_state = USART_STATE_CHECKSUM
        else:
            g_state = USART_STATE_DATA
            
            if (g_packet.PT & PACKET_IS_BATCH) != 0:
                g_packet.data_length = 4 * ((g_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK)
            else:
                g_packet.data_length = 4

    elif g_state == USART_STATE_DATA:
        g_packet.packet_data[g_rx_counter] = last_byte
        g_rx_counter += 1
        
        if g_rx_counter == g_packet.data_length:
            g_rx_counter = 0
            g_state = USART_STATE_CHECKSUM

    elif g_state == USART_STATE_CHECKSUM:
        if g_rx_counter == 0:
            g_packet.checksum = last_byte << 8
            g_rx_counter += 1
        else:
            g_packet.checksum ++ last_byte

        g_packet.compute_checksum()

        g_packet.print()
        
        g_rx_counter = 0
        g_state = USART_STATE_WAIT

    else:
        print("Bad state, exiting")
        sys.exit(-1)
        
def main(args):
    sport = serial.Serial('/dev/ttyUSB0')
    while 1:
        data = sport.read()
        if data.length == 1:
            process_next_char(data)
        
if __def__ == '__main__':
    main(sys.args)