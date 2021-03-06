#!/usr/bin/env python3

##############################################################################
# imports

import serial
import sys
import time
from argparse import ArgumentParser
from threading import Thread
import queue
import struct
import tkinter
import crcmod
import crcmod.predefined

##############################################################################

# definitions of states for USART state machine
USART_STATE_WAIT                        = 1
USART_STATE_TYPE                        = 2
USART_STATE_ADDRESS                     = 3
USART_STATE_DATA                        = 4
USART_STATE_CHECKSUM                    = 5

# flags for type of packet address
ADDRESS_TYPE_CONFIG                     = 0
ADDRESS_TYPE_DATA                       = 1
ADDRESS_TYPE_COMMAND                    = 2

# flags for interpreting packet type byte
PACKET_HAS_DATA                         = (1 << 7)
PACKET_IS_BATCH                         = (1 << 6)
PACKET_BATCH_LENGTH_MASK                = 0xf
PACKET_BATCH_LENGTH_OFFSET              = 2

# definitions of address start indexes
CONFIG_REG_START_ADDRESS                = 0
DATA_REG_START_ADDRESS                  = 85
COMMAND_START_ADDRESS                   = 170

# length of addresses
CONFIG_ARRAY_SIZE                       = 64
DATA_ARRAY_SIZE                         = 48
COMMAND_COUNT                           = 11

##############################################################################
# addresses of errors

# Sent if the UM6 receives a packet with a bad checksum
UM6_BAD_CHECKSUM		                = 253
# Sent if the UM6 receives a packet with an unknown address
UM6_UNKNOWN_ADDRESS                     = 254
# Sent if a requested batch read or write operation would go beyond the bounds
# of the config or data array
UM6_INVALID_BATCH_SIZE                  = 255

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
# flags for UM6_COMMUNICATION
# Enable serial data transmission
UM6_BROADCAST_ENABLED		            = (1 << 30)
# Enable transmission of raw gyro data
UM6_GYROS_RAW_ENABLED                   = (1 << 29)
# Enable transmission of raw accelerometer data
UM6_ACCELS_RAW_ENABLED                  = (1 << 28)
# Enable transmission of raw magnetometer data
UM6_MAG_RAW_ENABLED                     = (1 << 27)
# Enable transmission of processed gyro data (biases removed, scale factor applied, rotation correction applied)
UM6_GYROS_PROC_ENABLED                  = (1 << 26)
# Enable transmission of processed accel data (biases removed, scale factor applied, rotation correction applied)
UM6_ACCELS_PROC_ENABLED                 = (1 << 25)
# Enable transmission of processed mag data (biases removed, scale factor applied, rotation correction applied)
UM6_MAG_PROC_ENABLED                    = (1 << 24)
# Enable transmission of quaternion data
UM6_QUAT_ENABLED                        = (1 << 23)
# Enable transmission of euler angle data
UM6_EULER_ENABLED                       = (1 << 22)
# Enable transmission of state covariance data
UM6_COV_ENABLED                         = (1 << 21)
# Enable transmission of gyro temperature readings     
UM6_TEMPERATURE_ENABLED                 = (1 << 20)
# Enable transmission of latitude and longitude data
UM6_GPS_POSITION_ENABLED                = (1 << 19)
# Enable transmission of computed North and East position (with respect to home position)
UM6_GPS_REL_POSITION_ENABLED            = (1 << 18)
# Enable transmission of computed GPS course and speed
UM6_GPS_COURSE_SPEED_ENABLED            = (1 << 17)
# Enable transmission of satellite summary data (count, HDOP, VDP, mode)
UM6_GPS_SAT_SUMMARY_ENABLED             = (1 << 16)
# Enable transmission of satellite data (ID and SNR of each satellite)
UM6_GPS_SAT_DATA_ENABLED                = (1 << 15)

# Mask specifying the number of bits used to set the GPS serial baud rate
UM6_GPS_BAUD_RATE_MASK                  = (0x07)
# Specifies the start location of the GPS baud rate bits
UM6_GPS_BAUD_START_BIT                  = 11

# Mask specifying the number of bits used to set the serial baud rate
UM6_BAUD_RATE_MASK                      = (0x07)
# Specifies the start location of the serial baud rate bits
UM6_BAUD_START_BIT                      = 8

# Mask specifying which bits in this register are used to indicate the broadcast frequency
# rate is from 0 to 255, where 0 is 20 Hz and 255 is 300 Hz
UM6_SERIAL_RATE_MASK                    = (0x000FF)

##############################################################################
# flags for MISC Configuration register
# Enable magnetometer-based updates in the EKF
UM6_MAG_UPDATE_ENABLED		            = (1 << 31)
# Enable accelerometer-based updates in the EKF
UM6_ACCEL_UPDATE_ENABLED                = (1 << 30)
# Enable automatic gyro calibration on startup
UM6_GYRO_STARTUP_CAL                    = (1 << 29)
# Enable quaternion-based state estimation
UM6_QUAT_ESTIMATE_ENABLED               = (1 << 28)
# Enable external magnetic sensor updates in the EKF, internal sensor will be disabled if this is set
UM6_EXT_MAG_UPDATE_ENABLED              = (1 << 27)

##############################################################################
# data conversion functions

def int_to_bytearray(val):
    return struct.pack(">L", val)

def float_to_bytearray(val):
    return struct.pack(">f", val)

##############################################################################
# Exceptions
class BadChecksum (RuntimeError):
    def __init__(self, msg):
        super().__init__(msg)

class UnknownAddress (ValueError):
    def __init__(self, msg):
        super().__init__(msg)

class InvalidBatchSize (OverflowError):
    def __init__(self, msg):
        super().__init__(msg)

##############################################################################
# Packet class
# data sent to/from the ADAHRS via Serial port
class USARTPacket (object):

    crc16 = crcmod.predefined.mkCrcFun('crc-aug-ccitt')
    
    def __init__(self, pt, t):
        super()
        self._t = t
        self._pt = pt
        self.set_address(0)
        self._packet_data = bytearray()
        self.set_checksum(0)

    @staticmethod
    def make_reg_write_packet(address, data_array):
        pkt = USARTPacket(0, time.time())
        pkt.set_address(address)
        pkt.set_num_registers(len(data_array) >> 2)
        pkt._packet_data = data_array
        pkt.set_checksum(pkt.compute_checksum())
        return pkt
    
    @staticmethod
    def make_reg_read_packet(address, num_regs):
        pkt = USARTPacket(0, time.time())
        pkt.set_address(address)
        pkt.set_num_registers(num_regs)
        pkt.set_has_data(False)
        pkt.set_checksum(pkt.compute_checksum())
        return pkt
    
    @staticmethod
    def make_cmd_packet(cmd_address):
        pkt = USARTPacket(0, time.time())
        pkt.set_address(cmd_address)
        pkt.set_num_registers(0)
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
        if self.is_batch():
            return 4 *((self._pt >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK)
        elif self.has_data():
            return 4
        else:
            return 0

    def set_num_registers(self, dl):
        if dl < 0:
            raise ValueError("data length must be >= 0")
        self._pt = 0
        if dl > 8:
            raise ValueError("Maximum packet data length is 8")
        elif dl > 1:
            self._pt = (dl << PACKET_BATCH_LENGTH_OFFSET)
            self.set_is_batch(True)
            self.set_has_data(True)
        elif dl == 1:
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
        return USARTPacket.crc16(self.to_bytearray()[:-2])

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
        s = "%f," % self._t
        for ch in self.to_bytearray():
            s += "%02x," % ch
        print(s[:-1])
            
    def dump(self):
        print("Packet Dump\n-----------")
        print("t:", self._t)
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
        try:
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
                self._packet = USARTPacket(ch, time.time())
                self._state = USART_STATE_ADDRESS
                #print("PT:", hex(self._packet.pt()))

            elif self._state == USART_STATE_ADDRESS:
                try:
                    self._packet.set_address(ch)
                    #print("address:", self._packet.address())
                except IndexError:
                    if ch >= UM6_BAD_CHECKSUM:
                        self._packet._address = ch
                    else:
                        raise
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
                        raise IndexError("packet has bad checksum")
                    # add packet to queue, reset to starting state
                    self._rx_queue.put(self._packet)
                    self._packet = None
                    self._rx_counter = 0
                    self._state = USART_STATE_WAIT

            else:
                print("Bad state, exiting")
                sys.exit(-1)
        except IndexError:
            print("Index error during packet parsing:")
            print("ch:", hex(ch), "state:", self._state, "rx_counter:", self._rx_counter)
            if self._state == USART_STATE_CHECKSUM:
                print("computed checksum:", hex(self._packet.compute_checksum()))
            self._packet.dump()
            self._state = USART_STATE_WAIT
            self._rx_counter = 0
            
##############################################################################
# Printer - pulls packet off the queue and prints them
class Printer (Thread):

    def __init__(self, rxqueue, format):
        super().__init__()
        self._rx_queue = rxqueue
        self._stopme = False
        self._format = format

    def stop_me(self):
        self._stopme = True
        
    def run(self):
        while not self._stopme:
            try:
                mode, pkt = self._rx_queue.get(timeout=0.5)
                if self._format == 0:
                    if mode == 0:
                        print("RX")
                    else:
                        print("TX")
                    pkt.dump()
                else:
                    if mode == 0:
                        print("RX,", end='')
                    else:
                        print("TX,", end='')
                    pkt.dump_csv()
            except queue.Empty:
                pass
        
##############################################################################
# Receiver - receives data from the serial port and dispatches it
# contains parser and has a received packet queue
class Receiver (Thread):

    def __init__(self, serialport):
        super().__init__()
        self._port = serialport
        self._rx_queue = queue.Queue()
        self._parser = Parser(self._rx_queue)
        self._stopme = False

    def get_rx_queue(self):
        return self._rx_queue
    
    def stop_me(self):
        self._stopme = True
        
    def run(self):
        while not self._stopme:
            try:
                chars = self._port.read(1)
                if len(chars):
                    self._parser.process_buffer(chars)
            except serial.serialutil.SerialException:
                return

##############################################################################
# Register - contains data at an address received from UM6
class Register (object):

    def __init__(self):
        self._data = bytearray(4)
        self._uninit = True

    def is_initialized(self):
        return not self._uninit
    
    def set_bytes(self, barray):
        if len(barray) != 4:
            raise ValueError("Register contains exactly 4 bytes only")
        self._data = barray
        self._uninit = False
        
    def get_float(self):
        return struct.unpack(">f", self._data)[0]

    def get_int(self):
        return struct.unpack(">L", self._data)[0]

    def __str__(self):
        return "init:%s %02x %02x %02x %02x" % (self.is_initialized(),
                                                    self._data[0], self._data[1],
                                                    self._data[2], self._data[3])
        
##############################################################################
# Registers - contains data received from UM6
class Registers (object):

    def __init__(self):
        self._regs = {}
        for i in range(CONFIG_ARRAY_SIZE):
            self._regs[i] = Register()
        for i in range(DATA_ARRAY_SIZE):
            self._regs[i+DATA_REG_START_ADDRESS] = Register()

    def get_register(self, addr):
        if addr >= 0 and addr < CONFIG_ARRAY_SIZE:
            return self._regs[addr]
        if addr >= DATA_REG_START_ADDRESS and addr < DATA_REG_START_ADDRESS + DATA_ARRAY_SIZE:
            return self._regs[addr]
        else:
            raise IndexError("bad register address: %d(%x)" % (addr, addr))
        
##############################################################################
# Command - sends commands/register settings to UM6
class Command (Thread):

    def __init__(self, serialport, rxqueue):
        super().__init__()
        self._port = serialport
        self._rx_queue = rxqueue
        self._stopme = False
        self._print_queue = queue.Queue()
        self._pkt_count = 0
        self._wait_for_reply_pkt = -1
        self._regs = Registers()
        self._firmware_version = ""
        
    def stop_me(self):
        self._stopme = True

    def get_print_queue(self):
        return self._print_queue

    def waiting_for_reply(self):
        return self._wait_for_reply_pkt != -1

    def registers(self):
        return self._regs
    
    def run(self):
        # endless loop, extracting packets from rx queue
        while not self._stopme:
            try:
                # get a packet with timeout, raises queue.Empty if timed out
                pkt = self._rx_queue.get(timeout=0.5)
                self._pkt_count += 1
                print("pkts received:", self._pkt_count)
                # if we are waiting for a reply, check for it
                if self._wait_for_reply_pkt != -1:
                    self.check_response(pkt)
                # extract the data, if possible
                if pkt.has_data():
                    addr = pkt.address()
                    if addr >= COMMAND_START_ADDRESS:
                        if addr == UM6_GET_FW_VERSION:
                            barray = bytearray(4)
                            for j in range(4):
                                barray[j] = pkt.data(j)
                            self._firmware_version = barray.decode('utf-8')
                            print("UM6 Firmware Version:", self._firmware_version)
                    else:
                        num_regs = pkt.data_length() >> 2
                        for i in range(num_regs):
                            reg = self._regs.get_register(addr)
                            barray = bytearray(4)
                            for j in range(4):
                                barray[j] = pkt.data(i*4 + j)
                            reg.set_bytes(barray)
                            #print("reg:", addr, "data:", str(reg))
                            addr += 1
                # print the packet contents
                if self._print_queue:
                    # put the received packet on print queue
                    self._print_queue.put((0, pkt))
            except queue.Empty:
                pass

    def check_response(self, pkt):
        if pkt.address() == self._wait_for_reply_pkt:
            print("reply received for:", pkt.address())
            self._wait_for_reply_pkt = -1
        elif pkt.address() == UM6_BAD_CHECKSUM:
            raise BadChecksum("reply to '%d' is bad checksum" % self._wait_for_reply_pkt)
        elif pkt.address() == UM6_UNKNOWN_ADDRESS:
            raise UnknownAddress("reply to '%d' is unknown address" % self._wait_for_reply_pkt)
        elif pkt.address() == UM6_INVALID_BATCH_SIZE:
            raise InvalidBatchSize("reply to '%d' is invalid batch size" % self._wait_for_reply_pkt)
        else:
            print("waiting for reply:", self._wait_for_reply_pkt)
                
    def get_registers(self, addr, numregs):
        self.send_packet(USARTPacket.make_reg_read_packet(addr, numregs))

    def send_command(self, cmd):
        pkt = USARTPacket.make_cmd_packet(cmd)
        self.send_packet(pkt)
                             
    def broadcast(self, on):
        if on:
            comm = UM6_BROADCAST_ENABLED | UM6_GYROS_PROC_ENABLED | UM6_ACCELS_PROC_ENABLED | UM6_QUAT_ENABLED | UM6_TEMPERATURE_ENABLED
            comm |= (5 << UM6_BAUD_START_BIT)
        else:
            comm = (5 << UM6_BAUD_START_BIT)
        pkt = USARTPacket.make_reg_write_packet(UM6_COMMUNICATION, int_to_bytearray(comm))
        self.send_packet(pkt)
        
    def send_packet(self, pkt):
        buf = pkt.to_bytearray()
        num = self._port.write(buf)
        if num != len(buf):
            raise RuntimeError("didn't send all bytes")
        print("pkt sent")
        # record that we are waiting for a response
        self._wait_for_reply_pkt = pkt.address()
        # put the sent packet on the print queue
        self._print_queue.put((1, pkt))
        
##############################################################################
# GUI - 
class GUI:

    def __init__(self, master, command_thd):
        self._root = master
        self._cmd_thd = command_thd
        master.title("um6_comm")
        self._menu = tkinter.Menu(self._root)
        self._root.config(menu = self._menu)
        filemenu = tkinter.Menu(self._menu, tearoff=0)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self._root.quit)
        self._menu.add_cascade(label="File", menu=filemenu)

        # create the broadcast button
        self._broadcast_btn = tkinter.Button(self._root,
                                                 text="Broadcast",
                                                 command=self.broadcast,
                                                 relief="sunken",
                                                 state=tkinter.DISABLED)
        self._broadcast_btn.place(x=0, y=0)

        self._state = 0
        
        # get the firmware version
        self._cmd_thd.send_command(UM6_GET_FW_VERSION)
        self._root.after(100, self.get_reply)
        
    def broadcast(self):
        if self._broadcast_btn.config("relief")[-1] == "sunken":
            self._broadcast_btn.config(relief="raised")
            self._cmd_thd.broadcast(False)
        else:
            self._broadcast_btn.config(relief="sunken")
            self._cmd_thd.broadcast(True)

    def read_communication_register(self):
        comm_reg = self._cmd_thd.registers().get_register(UM6_COMMUNICATION).get_int()
        print("UM6_COMMUNICATION:0x%x" % comm_reg)
        if (comm_reg & UM6_BROADCAST_ENABLED) == UM6_BROADCAST_ENABLED:
            self._broadcast_btn.config(relief="sunken", state=tkinter.NORMAL)
        else:
            self._broadcast_btn.config(relief="raised", state=tkinter.NORMAL)
        
    def get_reply(self):
        if self._state == 0:
            if not self._cmd_thd.waiting_for_reply():
                # get the first set of registers
                self._cmd_thd.get_registers(UM6_COMMUNICATION, 8)
                self._state = 1
        elif self._state == 1:
            if not self._cmd_thd.waiting_for_reply():
                # we have the first set of registers
                self.read_communication_register()
                # regs 8-15
                self._cmd_thd.get_registers(UM6_EKF_MAG_VARIANCE, 8)
                self._state = 2
        elif self._state == 2:
            if not self._cmd_thd.waiting_for_reply():
                # regs 16-23
                self._cmd_thd.get_registers(UM6_MAG_BIAS_Z, 8)
                self._state = 3
        elif self._state == 3:
            if not self._cmd_thd.waiting_for_reply():
                # regs 24-31
                self._cmd_thd.get_registers(UM6_ACCEL_CAL_21, 8)
                self._state = 4
        elif self._state == 4:
            if not self._cmd_thd.waiting_for_reply():
                # regs 32-39
                self._cmd_thd.get_registers(UM6_GYRO_CAL_20, 8)
                self._state = 5
        elif self._state == 5:
            if not self._cmd_thd.waiting_for_reply():
                # regs 40-47
                self._cmd_thd.get_registers(UM6_MAG_CAL_12, 8)
                self._state = 6
        elif self._state == 6:
            if not self._cmd_thd.waiting_for_reply():
                # regs 48-55
                self._cmd_thd.get_registers(UM6_GYROY_BIAS_0, 8)
                self._state = 7
        elif self._state == 7:
            if not self._cmd_thd.waiting_for_reply():
                # regs 56-63
                self._cmd_thd.get_registers(UM6_GPS_HOME_LAT, 8)
                self._state = 8
        elif self._state == 8:
            return
        self._root.after(100, self.get_reply)
            
    def donothing(self):
        filewin = tkinter.Toplevel(self._root)
        button = tkinter.Button(filewin, text="Do nothing button")
        button.pack()
        
##############################################################################

def main(args):

    # command line options
    desc = "Serial Port communicator with UM6 ADAHRS"
    parser = ArgumentParser(description=desc)
    parser.add_argument("-p", "--port", action="store", dest="port",
                            default="/dev/ttyUSB0", help="Serial port file name")
    parser.add_argument("-b", "--bitrate", action="store", dest="bitrate", type=int,
                            default=115200, help="Serial port bitrate")
    
    args = parser.parse_args()

    print("Opening serial port '%s' with rate:%d" % (args.port, args.bitrate))

    serport = serial.Serial(args.port, args.bitrate, timeout=0.5)
    receiver = Receiver(serport)
    command = Command(serport, receiver.get_rx_queue())
    printer = Printer(command.get_print_queue(), 1)
    
    top = tkinter.Tk()
    top.geometry("640x480")
    gui = GUI(top, command)
    
    try:
        # start threads
        printer.start()
        command.start()
        receiver.start()
        top.mainloop()
    except KeyboardInterrupt:
        pass
    receiver.stop_me()
    command.stop_me()
    printer.stop_me()
    # join threads
    receiver.join()
    printer.join()
    command.join()
    
if __name__ == '__main__':
    main(sys.argv)
