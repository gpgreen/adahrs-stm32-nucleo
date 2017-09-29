#!/usr/bin/env python3

##############################################################################
# imports

import serial
import sys
from optparse import OptionParser
from threading import Thread
from queue import Queue

##############################################################################

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

# definitions of address start indexes
CONFIG_REG_START_ADDRESS = 0
DATA_REG_START_ADDRESS = 85
COMMAND_START_ADDRESS = 170

# length of addresses
CONFIG_ARRAY_SIZE = 64
DATA_ARRAY_SIZE = 48
COMMAND_COUNT = 11

##############################################################################
# addresses of config registers
UM6_COMMUNICATION                       = 0
UM6_MISC_CONFIG                         = 1
UM6_MAG_REF_X                           = 2
UM6_MAG_REF_Y                           = 3
UM6_MAG_REF_Z                           = 4
UM6_ACCEL_REF_X                         = 5
UM6_ACCEL_REF_Y                         = 6
UM6_ACCEL_REF_Z                         = 7
UM6_EKF_MAG_VARIANCE                    = 8
UM6_EKF_ACCEL_VARIANCE                  = 9
UM6_EKF_PROCESS_VARIANCE                = 10
UM6_GYRO_BIAS_XY                        = 11
UM6_GYRO_BIAS_Z                         = 12
UM6_ACCEL_BIAS_XY                       = 13
UM6_ACCEL_BIAS_Z                        = 14
UM6_MAG_BIAS_XY                         = 15
UM6_MAG_BIAS_Z                          = 16
UM6_ACCEL_CAL_00                        = 17
UM6_ACCEL_CAL_01                        = 18
UM6_ACCEL_CAL_02                        = 19
UM6_ACCEL_CAL_10                        = 20
UM6_ACCEL_CAL_11                        = 21
UM6_ACCEL_CAL_12                        = 22
UM6_ACCEL_CAL_20                        = 23
UM6_ACCEL_CAL_21                        = 24
UM6_ACCEL_CAL_22                        = 25
UM6_GYRO_CAL_00                         = 26
UM6_GYRO_CAL_01                         = 27
UM6_GYRO_CAL_02                         = 28
UM6_GYRO_CAL_10                         = 29
UM6_GYRO_CAL_11                         = 30
UM6_GYRO_CAL_12                         = 31
UM6_GYRO_CAL_20                         = 32
UM6_GYRO_CAL_21                         = 33
UM6_GYRO_CAL_22                         = 34
UM6_MAG_CAL_00                          = 35
UM6_MAG_CAL_01                          = 36
UM6_MAG_CAL_02                          = 37
UM6_MAG_CAL_10                          = 38
UM6_MAG_CAL_11                          = 39
UM6_MAG_CAL_12                          = 40
UM6_MAG_CAL_20                          = 41
UM6_MAG_CAL_21                          = 42
UM6_MAG_CAL_22                          = 43
UM6_GYROX_BIAS_0                        = 44
UM6_GYROX_BIAS_1                        = 45
UM6_GYROX_BIAS_2                        = 46
UM6_GYROX_BIAS_3                        = 47
UM6_GYROY_BIAS_0                        = 48
UM6_GYROY_BIAS_1                        = 49
UM6_GYROY_BIAS_2                        = 50
UM6_GYROY_BIAS_3                        = 51
UM6_GYROZ_BIAS_0                        = 52
UM6_GYROZ_BIAS_1                        = 53
UM6_GYROZ_BIAS_2                        = 54
UM6_GYROZ_BIAS_3                        = 55
UM6_GPS_HOME_LAT                        = 56
UM6_GPS_HOME_LONG                       = 57
UM6_GPS_HOME_ALTITUDE                   = 58
UM6_EXT_MAG_X                           = 59
UM6_EXT_MAG_Y                           = 60
UM6_EXT_MAG_Z                           = 61
UM6_NODE_ID                             = 62
UM6_EQUIPMENT                           = 63

##############################################################################
# addresses of data registers
UM6_STATUS                              = 85
UM6_GYRO_RAW_XY                         = 86
UM6_GYRO_RAW_Z                          = 87
UM6_ACCEL_RAW_XY                        = 88
UM6_ACCEL_RAW_Z                         = 89
UM6_MAG_RAW_XY                          = 90
UM6_MAG_RAW_Z                           = 91
UM6_GYRO_PROC_XY                        = 92
UM6_GYRO_PROC_Z                         = 93
UM6_ACCEL_PROC_XY                       = 94
UM6_ACCEL_PROC_Z                        = 95
UM6_MAG_PROC_XY                         = 96
UM6_MAG_PROC_Z                          = 97
UM6_EULER_PHI_THETA                     = 98
UM6_EULER_PSI                           = 99
UM6_QUAT_AB                             = 100
UM6_QUAT_CD                             = 101
UM6_ERROR_COV_00                        = 102
UM6_ERROR_COV_01                        = 103
UM6_ERROR_COV_02                        = 104
UM6_ERROR_COV_03                        = 105
UM6_ERROR_COV_10                        = 106
UM6_ERROR_COV_11                        = 107
UM6_ERROR_COV_12                        = 108
UM6_ERROR_COV_13                        = 109
UM6_ERROR_COV_20                        = 110
UM6_ERROR_COV_21                        = 111
UM6_ERROR_COV_22                        = 112
UM6_ERROR_COV_23                        = 113
UM6_ERROR_COV_30                        = 114
UM6_ERROR_COV_31                        = 115
UM6_ERROR_COV_32                        = 116
UM6_ERROR_COV_33                        = 117
UM6_TEMPERATURE                         = 118
UM6_GPS_LONGITUDE                       = 119
UM6_GPS_LATITUDE                        = 120
UM6_GPS_ALTITUDE                        = 121
UM6_GPS_POSITION_N                      = 122
UM6_GPS_POSITION_E                      = 123
UM6_GPS_POSITION_H                      = 124
UM6_GPS_COURSE_SPEED                    = 125
UM6_GPS_SAT_SUMMARY                     = 126
UM6_GPS_SAT_1_2                         = 127
UM6_GPS_SAT_3_4                         = 128
UM6_GPS_SAT_5_6                         = 129
UM6_GPS_SAT_7_8                         = 130
UM6_GPS_SAT_9_10                        = 131
UM6_GPS_SAT_11_12                       = 132

##############################################################################
# addresses of commands
UM6_GET_FW_VERSION                      = 170
UM6_FLASH_COMMIT                        = 171
UM6_ZERO_GYROS                          = 172
UM6_RESET_EKF                           = 173
UM6_GET_DATA                            = 174
UM6_SET_ACCEL_REF                       = 175
UM6_SET_MAG_REF                         = 176
UM6_RESET_TO_FACTORY                    = 177
UM6_SAVE_FACTORY                        = 178
UM6_SET_HOME_POSITION                   = 179
UM6_USE_EXT_MAG                         = 180

##############################################################################
# Packet class
# data sent to/from the ADAHRS via Serial port
class USARTPacket (object):

    def __init__(self, pt):
        super()
        self._pt = pt
        self.set_address(0)
        self._packet_data = bytearray()
        self.set_checksum(0)

    @staticmethod
    def make_packet(address, data_array):
        pkt = USARTPacket(0)
        pkt.set_address(address)
        pkt.set_data_length(len(data_array))
        pkt._packet_data = data_array
        pkt.set_checksum(pkt.compute_checksum())
        return pkt
    
    def pt(self):
        return self._pt

    def address(self):
        return self._address

    def set_address(self, addr):
        if (addr >= CONFIG_REG_START_ADDRESS \
            and addr < (CONFIG_REG_START_ADDRESS + CONFIG_ARRAY_SIZE)) \
            or (addr >= DATA_REG_START_ADDRESS \
                and addr < (DATA_REG_START_ADDRESS + DATA_ARRAY_SIZE)) \
            or (addr >= COMMAND_START_ADDRESS \
                and addr < (COMMAND_START_ADDRESS + COMMAND_COUNT)):
            self._address = addr
        else:
            raise IndexError("bad address")

    def address_type(self):
        if self._address < DATA_REG_START_ADDRESS:
            return ADDRESS_TYPE_CONFIG
        elif self._address < COMMAND_START_ADDRESS:
            return ADDRESS_TYPE_DATA
        else:
            return ADDRESS_TYPE_COMMAND
        
    def has_data(self):
        return (self._pt & PACKET_HAS_DATA) != 0

    def set_has_data(self, have):
        if have:
            self._pt |= PACKET_HAS_DATA
        else:
            self._pt &= ~PACKET_HAS_DATA
            
    def is_batch(self):
        return (self._pt & PACKET_IS_BATCH) != 0

    def set_is_batch(self, batch):
        if batch:
            self._pt |= PACKET_IS_BATCH
        else:
            self._pt &= ~PACKET_IS_BATCH
            
    def data_length(self):
        if not self.has_data():
            return 0
        if self.is_batch():
            return 4 *((self._pt >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK)
        else:
            return 4

    def set_data_length(self, dl):
        if dl % 4:
            raise ValueError("data length must be divisible by 4")
        if dl < 0:
            raise ValueError("data length must be >= 0")
        self._pt = 0
        if dl > 4:
            self._pt = (dl << PACKET_BATCH_LENGTH_OFFSET)
            self.set_is_batch(True)
            self.set_has_data(True)
        elif dl == 4:
            self.set_has_data(True)
        else:
            self.set_is_batch(False)
            self.set_has_data(False)

    def data(self, index):
        if index < 0 or index > self.data_length() - 1:
            raise IndexError("index out of range")
        return self._packet_data[index]
    
    def set_data(self, index, val):
        if index < 0 or index > self.data_length() - 1:
            raise IndexError("index out of range")
        # if data array isn't long enough extend it with 0's
        if len(self._packet_data) < (index + 1):
            if len(self._packet_data) < index:
                while len(self._packet_data) < index:
                    self._packet_data.append(0)
            self._packet_data.append(val)
        else:
            self._packet_data[index] = val
            
    def checksum(self):
        return self._checksum

    def set_checksum(self, cksum):
        self._checksum = cksum
        
    def compute_checksum(self):
        total = sum(self.to_bytearray()[:-2])
        # Fold 32-bits into 16-bits
        total = (total >> 16) + (total & 0xffff)
        total += total >> 16
        return (total + 0x10000 & 0xffff)

    def to_bytearray(self):
        l = len(self._packet_data)
        buf = bytearray(7 + l)
        buf[0] = 0x73
        buf[1] = 0x6e
        buf[2] = 0x70
        buf[3] = self._pt
        buf[4] = self._address
        for i, ch in enumerate(self._packet_data):
            buf[i+5] = ch
        buf[5+l] = ((self._checksum >> 8) & 0xff)
        buf[6+l] = (self._checksum & 0xff)
        return buf

    def dump_csv(self):
        s = ""
        for ch in self.to_bytearray():
            s += "%x," % ch
        print(s[:-1])
            
    def dump(self):
        print("Packet Dump\n-----------")
        print("PT:", hex(self._pt), "dlen:", self.data_length(), "data:",
                  self.has_data(), "batch:", self.is_batch())
        print("address:", self._address, "chksum:", hex(self.checksum()))
        for b in self._packet_data:
            print("\tbyte:", hex(b))
        
##############################################################################
# Parser - takes stream of data from serial port and turns into packets
class Parser (object):

    def __init__(self, rxqueue):
        super()
        self._packet = None
        self._rx_counter = 0
        self._state = USART_STATE_WAIT
        self._rx_queue = rxqueue
        
    def process_buffer(self, data):
        for b in data:
            #print("byte:", hex(b), "ch:'%c'" % b)
            self.process_next_char(b)
        
    def process_next_char(self, ch):
        #print("state:", self._state)
        if self._state == USART_STATE_WAIT:
            if self._rx_counter == 0 and ch == 0x73:
                self._rx_counter += 1
            elif self._rx_counter == 1 and ch == 0x6e:
                self._rx_counter += 1
            elif self._rx_counter == 2 and ch == 0x70:
                self._rx_counter = 0
                self._state = USART_STATE_TYPE
            else:
                self._rx_counter = 0
            
        elif self._state == USART_STATE_TYPE:
            self._packet = USARTPacket(ch)
            self._state = USART_STATE_ADDRESS
            #print("PT:", hex(self._packet.pt()))
        
        elif self._state == USART_STATE_ADDRESS:
            self._packet.set_address(ch)
            #print("address:", self._packet.address())
        
            if not self._packet.has_data():
                self._state = USART_STATE_CHECKSUM
                #print("no data packet")
            else:
                self._state = USART_STATE_DATA
                self._rx_counter = 0
                #print("length:", self._packet.data_length())
            
        elif self._state == USART_STATE_DATA:
            self._packet.set_data(self._rx_counter, ch)
            self._rx_counter += 1
        
            if self._rx_counter == self._packet.data_length():
                self._rx_counter = 0
                self._state = USART_STATE_CHECKSUM

        elif self._state == USART_STATE_CHECKSUM:
            if self._rx_counter == 0:
                self._packet.set_checksum(ch << 8)
                self._rx_counter += 1
                #print("chksum1:", hex(self._packet.checksum()))
            else:
                self._packet.set_checksum(self._packet.checksum() + ch)
                #print("chksum2:", hex(self._packet.checksum()))

                chksum = self._packet.compute_checksum()
                if chksum != self._packet.checksum():
                    raise RuntimeError("packet has bad checksum")

                # add packet to queue, reset to starting state
                self._rx_queue.put(self._packet)
                self._packet = None
                self._rx_counter = 0
                self._state = USART_STATE_WAIT

        else:
            print("Bad state, exiting")
            sys.exit(-1)

##############################################################################
# Printer - pulls packet off the queue and prints them
class Printer (Thread):

    def __init__(self, rxqueue):
        super().__init__()
        self._rx_queue = rxqueue

    def run(self):
        while 1:
            pkt = self._rx_queue.get()
            pkt.dump()
        
##############################################################################
# Receiver - receives data from the serial port and dispatches it
# contains parser and has a received packet queue
class Receiver (Thread):

    def __init__(self, serialport):
        super().__init__()
        self._port = serialport
        self._rx_queue = Queue()
        self._parser = Parser(self._rx_queue)

    def get_rx_queue(self):
        return self._rx_queue
    
    def run(self):
        while 1:
            self._parser.process_buffer(self._port.read(1))
            
##############################################################################

def main(args):

    # command line options
    usage = "usage: %prog [options]"
    parser = OptionParser(usage=usage)
    parser.set_defaults(port="/dev/ttyUSB0",
                            bitrate="115200")
    parser.add_option("-p", "--port", action="store", dest="port",
                          help="Serial port file name")
    parser.add_option("-b", "--bitrate", action="store", dest="bitrate",
                          help="Serial port bitrate")
    
    (options, args) = parser.parse_args()

    print("Opening serial port '%s' with rate:%d" % (options.port, int(options.bitrate)))

    serport = serial.Serial(options.port, int(options.bitrate))
    receiver = Receiver(serport)
    printer = Printer(receiver.get_rx_queue())

    try:
        # start threads
        receiver.start()
        printer.start()
    except KeyboardInterrupt:
        # join threads
        printer.join()
        receiver.join()
    
if __name__ == '__main__':
    main(sys.argv)
