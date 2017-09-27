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

    def dump(self):
        print("Packet Dump")
        print("PT:%x" % self.PT)
        print("address:%x" % self.address)
        print("data_length:%d" % self.data_length)
        for i in range(self.data_length):
            print("\tbyte:%x" % self.packet_data[i])
        print("chksum:", hex(self.checksum))
        
g_packet = USARTPacket()

def process_buffer(data):
    for b in data:
        print("byte:", hex(b), "ch:'%c'" % b)
        process_next_char(b)
        
def process_next_char(ch):
    global g_rx_counter, g_packet, g_state

    print("state:", g_state)
    if g_state == USART_STATE_WAIT:
        if g_rx_counter == 0 and ch == 0x73:
            g_rx_counter += 1
        elif g_rx_counter == 1 and ch == 0x6e:
            g_rx_counter += 1
        elif g_rx_counter == 2 and ch == 0x70:
            g_rx_counter = 0
            g_state = USART_STATE_TYPE
        else:
            g_rx_counter = 0
            
    elif g_state == USART_STATE_TYPE:
        g_packet.PT = ch
        g_state = USART_STATE_ADDRESS
        #print("PT:", hex(g_packet.PT))
        
    elif g_state == USART_STATE_ADDRESS:
        g_packet.address = ch
        #print("address:", g_packet.address)
        
        if g_packet.address < 12:
            g_packet.address_type = ADDRESS_TYPE_CONFIG
        elif g_packet.address >= 12 and g_packet.address < 100:
            g_packet.address_type = ADDRESS_TYPE_DATA
        else:
            g_packet.address_type = ADDRESS_TYPE_COMMAND

        if (g_packet.PT & PACKET_HAS_DATA) == 0:
            g_state = USART_STATE_CHECKSUM
            #print("no data packet")
        else:
            g_state = USART_STATE_DATA
            
            if (g_packet.PT & PACKET_IS_BATCH) != 0:
                g_packet.data_length = 4 * ((g_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK)
            else:
                g_packet.data_length = 4
            g_rx_counter = 0
            #print("length:", g_packet.data_length)
            
    elif g_state == USART_STATE_DATA:
        g_packet.packet_data.append(ch)
        g_rx_counter += 1
        
        if g_rx_counter == g_packet.data_length:
            g_rx_counter = 0
            g_state = USART_STATE_CHECKSUM

    elif g_state == USART_STATE_CHECKSUM:
        if g_rx_counter == 0:
            g_packet.checksum = ch << 8
            g_rx_counter += 1
            #print("chksum1:", hex(g_packet.checksum))
        else:
            g_packet.checksum += ch
            #print("chksum2:", hex(g_packet.checksum))

            g_packet.compute_checksum()

            g_packet.dump()
        
            g_rx_counter = 0
            g_state = USART_STATE_WAIT

    else:
        print("Bad state, exiting")
        sys.exit(-1)
        
def main(args):
    sport = serial.Serial('/dev/ttyUSB0', 115200)
    while 1:
        process_buffer(sport.read(2))
        
if __name__ == '__main__':
    main(sys.argv)
