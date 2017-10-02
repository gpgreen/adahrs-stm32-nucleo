/*
 * ADAHRSCommand.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: ggreen
 */

#include <math.h>

#include "adahrs_command.h"
#include "adahrs_config_def.h"
#include "work_queue.h"
#include "isr_def.h"

// Definitions of states for USART receiver state machine (for receiving packets)
#define	USART_STATE_WAIT	        1
#define	USART_STATE_TYPE		2
#define	USART_STATE_ADDRESS		3
#define	USART_STATE_DATA		4
#define	USART_STATE_CHECKSUM		5

// Flags for interpreting the packet type byte in communication packets
#define	PACKET_HAS_DATA			(1 << 7)
#define	PACKET_IS_BATCH			(1 << 6)
#define	PACKET_BATCH_LENGTH_MASK	(0x0F)

#define	PACKET_BATCH_LENGTH_OFFSET	2

#define	BATCH_SIZE_2			2
#define	BATCH_SIZE_3			3

#define	PACKET_NO_DATA			0
#define	PACKET_COMMAND_FAILED	        (1 << 0)

// Define flags for identifying the type of packet address received
#define	ADDRESS_TYPE_CONFIG		0
#define	ADDRESS_TYPE_DATA		1
#define	ADDRESS_TYPE_COMMAND		2

// Define states used for receiving GPS data
#define STATE_GPS_IDLE			0
#define STATE_GPS_PACKET		1
#define STATE_GPS_CHECKSUM		2

// Define maximum GPS packet length in characters
#define MAX_GPS_PACKET_SIZE		86

// ----------------------------------------------------------------------------

USARTPacket::USARTPacket()
: PT(0), address(0), checksum(0), data_length(0), address_type(0)
{
    // does nothing else
}

USARTPacket::USARTPacket(const USARTPacket& pkt)
{
    if (&pkt == this)
        return;
    PT = pkt.PT;
    address = pkt.address;
    checksum = pkt.checksum;
    data_length = pkt.data_length;
    address_type = pkt.address_type;
    for (int i=0; i<data_length; ++i)
        packet_data[i] = pkt.packet_data[i];
}

USARTPacket& USARTPacket::operator=(const USARTPacket& pkt)
{
    PT = pkt.PT;
    address = pkt.address;
    checksum = pkt.checksum;
    data_length = pkt.data_length;
    address_type = pkt.address_type;
    for (int i=0; i<data_length; ++i)
        packet_data[i] = pkt.packet_data[i];
    return *this;
}

// http://srecord.sourceforge.net/crc16-ccitt.html

#define poly 0x1021

static uint16_t crc_ccitt_update(uint16_t crc, uint8_t data)
{
    // align test bit with leftmost bit of the message byte
    uint8_t v = 0x80;

    for (int i=0; i<8; ++i)
    {
        uint8_t xor_flag = 0;
        
        if (crc & 0x8000)
        {
            xor_flag = 1;
        }
        crc = static_cast<uint16_t>(crc << 1);

        if (data & v)
        {
            /*
              append next bit of message to end of CRC if it is not zero
              the zero bit placed there by the shift above need not be
              changed if the next bit of the message is zero
            */
            crc = static_cast<uint16_t>(crc + 1);
        }

        if (xor_flag)
        {
            crc ^= poly;
        }

        // align test bit with next bit of the message byte
        v = static_cast<uint8_t>(v >> 1);
    }
    return crc;
}

static uint16_t crc_ccitt_augment_message(uint16_t crc)
{
    for (int i=0; i<16; i++)
    {
        uint8_t xor_flag = 0;
        if (crc & 0x8000)
        {
            xor_flag = 1;
        }
        crc = static_cast<uint16_t>(crc << 1);

        if (xor_flag)
        {
            crc ^= poly;
        }
    }
    return crc;
}

uint16_t USARTPacket::compute_checksum() const
{
    uint16_t chksum = 0xffff;
    chksum = crc_ccitt_update(chksum, 0x73);
    chksum = crc_ccitt_update(chksum, 0x6e);
    chksum = crc_ccitt_update(chksum, 0x70);
    chksum = crc_ccitt_update(chksum, PT);
    chksum = crc_ccitt_update(chksum, address);
    
    for (int index=0; index<data_length; ++index)
    {
        chksum = crc_ccitt_update(chksum, packet_data[index]);
    }
    
    return crc_ccitt_augment_message(chksum);
}

// ----------------------------------------------------------------------------
// static variable to flag broadcast ready
// ----------------------------------------------------------------------------
static volatile int gSendStateData = 0;

ADAHRSCommand::ADAHRSCommand()
    : _uart(nullptr), _config(nullptr), _states(nullptr), _ekf(nullptr),
      _state(USART_STATE_WAIT), _data_counter(0), _new_packet_received(0),
      _rx_offset(0), _tx_offset(0)
{
    // does nothing else
}

void ADAHRSCommand::begin(USART* uart, ADAHRSConfig* config, ADAHRSSensorData* states,
                          EKF* ekf, uint8_t priority, uint8_t subpriority)
{
    _uart = uart;
    _config = config;
    _states = states;
    _ekf = ekf;
    _state = USART_STATE_WAIT;
    _data_counter = 0;
    _rx_offset = 0;
    _tx_offset = 0;
    _new_packet_received = 0;
    gSendStateData = 0;
    
    // Enable TIM2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // setup timer2 for broadcast loop
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_DeInit(TIM2);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // timer is not started until broadcast interval is setup

    configure_nvic(TIM2_IRQn, priority, subpriority);

    // set serial baud rate
    update_serial_baud();

    // if broadcasting, start it
    if (_config->get_register(UM6_COMMUNICATION) & UM6_BROADCAST_ENABLED)
    {
        enable_broadcast_mode(_config->get_register(UM6_COMMUNICATION) & UM6_SERIAL_RATE_MASK);
    }
}

bool ADAHRSCommand::broadcast_ready() const
{
    return (_config->get_register(UM6_COMMUNICATION) & UM6_BROADCAST_ENABLED)
        && gSendStateData;
}

/*
Takes the character in the RX buffer pointed to by rx_offset and processes it.  This
function should only be called if the buffer pointer points to a character that has not
yet been handled.

The RX handler automatically parses input characters and formats strings of
characters into packets.  The behavior of the RX handler is based on the current
"state" of the packet handler.  There are 5 states:

USART_STATE_WAIT
	 In this state, the RX handler is waiting to receive the packet start sequence
	 's' 'n' 'p'.  When the sequence is received, the state transitions to 
	 USART_STATE_TYPE

USART_STATE_TYPE
	 In this state, the RX handler waits to receive the packet type byte, which indicates
	 whether a read or write operation is being performed, and whether it is a batch
	 operation.  After this byte has been received, the state transitions to 
	 USART_STATE_ADDRESS

USART_STATE_ADDRESS
	 In this state, the RX handler waits to receive the address of the register being
	 modified or, in the case of a command packet, the ID of the command being sent.
	 
	 If it is a command packet, then the state transitions to USART_STATE_CHECKSUM.  
	 If the packet is reading from a register address, the state transitions to 
	 USART_STATE_CHECKSUM.
	 If the packet is writing to a register, the state transitions to USART_STATE_DATA.

USART_STATE_DATA
	 In this state, the RX handler expects to receive four bytes for a single register read,
	 or 4*batch_size bytes for a batch write operation.  Once all the data bytes have
	 been received, the state transitions to the USART_STATE_CHECKSUM state.

USART_STATE_CHECKSUM
	 In this state, the RX handler expects to receive two bytes containing the sum
	 of all other bytes transmitted in the packet.  After the two bytes have been
	 received, the checksum is evaluated.  If it is valid, then the entire packet is
	 copied into the RX packet buffer.  The state then transitions back to the 
	 USART_STATE_WAIT state.  The packet will be handled later from within the
	 main program loop. 
*/
void ADAHRSCommand::process_next_character()
{
    // if no new data, return
    if (!_uart->has_received_data())
    {
        return;
    }
    // copy the next character to buffer
    if (_uart->get_received_data(&_rx_buffer[_rx_offset], 1) != 1)
    {
        return;
    }
    uint8_t ch = _rx_buffer[_rx_offset++];
    if (_rx_offset == COMMAND_BUFFER_SIZE)
    {
        _data_counter = 0;
        _rx_offset = 0;
        return;
    }
    
    // copy the received data into our buffer
    // The next action should depend on the USART state.
    switch (_state)
    {
        // USART in the WAIT state.  In this state, the USART is waiting to see the sequence of bytes
        // that signals a new incoming packet.
    case USART_STATE_WAIT:
        if (_data_counter == 0)		// Waiting on 's' character
        {
            if (ch == 's')
            {
                ++_data_counter;
            }
            else
            {
                _data_counter = 0;
            }
        }
        else if (_data_counter == 1)		// Waiting on 'n' character
        {
            if (ch == 'n')
            {
                ++_data_counter;
            }
            else
            {
                _data_counter = 0;
            }
        }
        else if (_data_counter == 2)		// Waiting on 'p' character
        {
            if (ch == 'p')
            {
                // The full 'snp' sequence was received.  Reset
                // _data_counter (it will be used again later) and
                // transition to the next state.
                _data_counter = 0;
                _state = USART_STATE_TYPE;
            }
            else
            {
                _data_counter = 0;
            }
        }
        break;
		  
        // USART in the TYPE state.  In this state, the USART has just
        // received the sequence of bytes that indicates a new packet
        // is about to arrive.  Now, the USART expects to see the
        // packet type.
    case USART_STATE_TYPE:
        _new_packet.PT = ch;
        _state = USART_STATE_ADDRESS;
        break;
	
        // USART in the ADDRESS state.  In this state, the USART
        // expects to receive a single byte indicating the address
        // that the packet applies to
    case USART_STATE_ADDRESS:
        _new_packet.address = ch;
				
        // For convenience, identify the type of packet this is and copy to the packet structure
        // (this will be used by the packet handler later)
        // also adjust the address based on which type
        if (_new_packet.address < DATA_REG_START_ADDRESS)
        {
            _new_packet.address_type = ADDRESS_TYPE_CONFIG;
        }
        else if ((_new_packet.address >= DATA_REG_START_ADDRESS)
                 && (_new_packet.address < COMMAND_START_ADDRESS) )
        {
            _new_packet.address_type = ADDRESS_TYPE_DATA;
        }
        else
        {
            _new_packet.address_type = ADDRESS_TYPE_COMMAND;
        }
	
        // Identify the type of communication this is (whether reading
        // or writing to a data or configuration register, or sending
        // a command) If this is a read operation, jump directly to
        // the USART_STATE_CHECKSUM state - there is no more data in
        // the packet
        if ((_new_packet.PT & PACKET_HAS_DATA) == 0)
        {
            _state = USART_STATE_CHECKSUM;
            _new_packet.data_length = 0;
        }
        // If this is a write operation, go to the USART_STATE_DATA state to read in the relevant data
        else
        {
            _state = USART_STATE_DATA;
					 
            // Determine the expected number of bytes in this data
            // packet based on the packet type.  A write operation
            // consists of 4 bytes unless it is a batch operation, in
            // which case the number of bytes equals 4*batch_size,
            // where the batch size is in the packet type byte.
            if (_new_packet.PT & PACKET_IS_BATCH)
            {
                _new_packet.data_length = static_cast<uint8_t>(
                    4 * ((_new_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK));
            }
            else
            {
                _new_packet.data_length = 4;
            }
        }
	
        break;
	
        // USART in the DATA state.  In this state, the USART expects
        // to receive new_packet.length bytes of data.
    case USART_STATE_DATA:
        _new_packet.packet_data[_data_counter++] =  ch;
	
        // If the expected number of bytes has been received, transition to the CHECKSUM state.
        if (_data_counter == _new_packet.data_length)
        {
            // Reset _data_counter, since it will be used in the CHECKSUM state.
            _data_counter = 0;
            
            _state = USART_STATE_CHECKSUM;
        }
	
        break;
	
        // USART in CHECKSUM state.  In this state, the entire packet
        // has been received, with the exception of the 16-bit
        // checksum.
    case USART_STATE_CHECKSUM:
				
        // Get the highest-order byte
        if (_data_counter == 0)
        {
            _new_packet.checksum = static_cast<uint16_t>(ch << 8);
					 
            ++_data_counter;
        }
        else // ( _data_counter == 1 )
        {
            // Get lower-order byte
            _new_packet.checksum = static_cast<uint16_t>(_new_packet.checksum | ch);
					 
            // Both checksum bytes have been received.  Make sure that the checksum is valid.
            uint16_t checksum = _new_packet.compute_checksum();
					 
            // If checksum does not match, send a BAD_CHECKSUM packet
            if (checksum != _new_packet.checksum)
            {
                // Send bad checksum packet
                _new_packet.PT = PACKET_NO_DATA;
                _new_packet.address = UM6_BAD_CHECKSUM;
                _new_packet.data_length = 0;	// No data bytes
                _new_packet.checksum = _new_packet.compute_checksum();
						  
                add_tx_packet(_new_packet);
            }
            else
            {
                // Packet was received correctly.  Add the packet to the RX packet buffer and
                // set a flag indicating that a new packet has been received.  
                add_rx_packet(_new_packet);
            }
					 
            // A full packet has been received.
            // Put the USART back into the WAIT state and reset 
            // the _data_counter variable so that it can be used to receive the next packet.
            _data_counter = 0;
						  
            _state = USART_STATE_WAIT;					  
        }
				
        break;
    }
}

void ADAHRSCommand::add_tx_packet(const USARTPacket& pkt)
{
    _tx_packets.push(pkt);
}

void ADAHRSCommand::add_rx_packet(const USARTPacket& pkt)
{
    _rx_packets.push(pkt);
    _new_packet_received = 1;
}

/*
Handles packets received over the UART.  First, the packet is checked to make
sure it is understood.  If not, a packet is transmitted over the USART to that
effect.  If it is, then the following takes place:

1. If the packet is a write operation, then the data contained in data section
of the packet is copied to the relevant registers.
2. If the packet is a read operation, the relevant registers are copied into
a new packet.  The new packet is then transmitted.
3. Finally, the packet is sent to the dispatch_packet(.) function.  The DispatchPacket
function is used to handle packet-specific actions that must be performed when the
packet is received.  For example, a packet that alters the broadcast rate of the
device must change the broadcast timer configuration to reflect the change.  This
is handled by the dispatch_packet function.
*/
void ADAHRSCommand::process_rx_packet()
{
    bool address_valid = true;
    bool batch_good = true;

    if (_rx_packets.size() == 0)
    {
        return;
    }
    USARTPacket new_packet = _rx_packets.top();
    _rx_packets.pop();
    
    // Check to make sure that the packet address is recognizable.
    // For example, if it involves a read or a write to the
    // configuration array, then the address must be less than
    // CONFIG_ARRAY_SIZE, which is defined in UM6_config.h.
    // Similarly, a read or write to the data array must be to an
    // address lower than DATA_ARRAY_SIZE, and a command packet must
    // have an address lower than COMMAND_COUNT (also defined in
    // UM6_config.h)
	 
    // Start with a check for configuration register reads/writes.
    // Note that valid configuration registers have addresses starting
    // at CONFIG_REG_START_ADDRESS and ending at
    // DATA_REG_START_ADDRESS - 1 (this is based on how the
    // communication protocol is defined)
    if (new_packet.address_type == ADDRESS_TYPE_CONFIG)
    {
        if ((new_packet.address) >= _config->config_size())
        {
            address_valid = false;
        }
		  
        // Check if this is a batch operation and, if so, whether it
        // is valid (cannot allow a batch operation to go beyond array
        // bounds)
        if (new_packet.PT & PACKET_IS_BATCH)
        {
            if ((new_packet.address + ((new_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK) - 1)
                >= _config->config_size())
            {
                batch_good = false;
            }
        }
    }
    // Check for invalid data register addresses now...
    else if (new_packet.address_type == ADDRESS_TYPE_DATA)
    {
        if ((new_packet.address) >= DATA_REG_START_ADDRESS + _config->data_size())
        {
            address_valid = false;
        }
		  
        // Check if this is a batch operation and, if so, whether it
        // is valid (cannot allow a batch operation to go beyond array
        // bounds)
        if (new_packet.PT & PACKET_IS_BATCH)
        {
            if ((new_packet.address + ((new_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK) - 1)
                >= DATA_REG_START_ADDRESS + _config->data_size())
            {
                batch_good = false;
            }
        }
    }
    // Now check for invalid commands
    else if (new_packet.address_type == ADDRESS_TYPE_COMMAND)
    {
        if (new_packet.address >= COMMAND_START_ADDRESS + COMMAND_COUNT)
        {
            address_valid = false;
        }
    }
	 
    // If the address check failed, send a packet informing about the problem
    if (!address_valid)
    {
        USARTPacket response_packet;
		  
        // Send "unknown address" packet
        response_packet.PT = PACKET_NO_DATA;
        response_packet.address = UM6_UNKNOWN_ADDRESS;
        response_packet.data_length = 0;	// No data bytes
        response_packet.checksum = response_packet.compute_checksum();
		  
        add_tx_packet(response_packet);
    }

    // If the the batch size was too large to be valid, send a packet informing about the problem
    else if (!batch_good)
    {
        USARTPacket response_packet;
		  
        // Send "invalid batch size" packet
        response_packet.PT = PACKET_NO_DATA;
        response_packet.address = UM6_INVALID_BATCH_SIZE;
        response_packet.data_length = 0;	// No data bytes
        response_packet.checksum = response_packet.compute_checksum();
		  
        add_tx_packet(response_packet);
    }
	 
    // Now process the packet.  Only read and write packets will be
    // processed in this function.  Commands will be handled later, in
    // the call to DispatchPacket(.)
    else if (new_packet.address_type != ADDRESS_TYPE_COMMAND)
    {
        // If the packet is performing a write operation, copy the
        // data to the specified address(es) If it is a batch write
        // operation, then multiple registers will be written.
        if (new_packet.PT & PACKET_HAS_DATA)
        {
            int address_offset = 0;
				
            // Received packets will always contain data in multiples of 4 bytes
            for (int i=0; i<new_packet.data_length; i+=4)
            {
                int address = new_packet.address + address_offset;
                _config->set_register(address, new_packet.packet_data[i+3],
                		new_packet.packet_data[i+2],
						new_packet.packet_data[i+1],
						new_packet.packet_data[i]);
                address_offset++;
            }
        }
        // If the packet is performing a read operation, construct a
        // new packet containing the data to be transmitted
        else
        {
            send_global_data(new_packet.address, new_packet.address_type,
                             (new_packet.PT & PACKET_IS_BATCH),
                             (new_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK);
        }
    }
	 
    // Now dispatch the packet for additional processing
    dispatch_packet(new_packet);
}

/*******************************************************************************

If the TX packet buffer is not empty, and if the USART transmitter is not already 
operating, this function copies the next packet in the TX packet buffer to 
the uart.

Note the distinction between the TX Packet Buffer, and the TX Buffer.  The 
TX Packet Buffer is an array of structures containing data that needs to be
transmitted over the USART.  The TX Buffer is an array of bytes, representing
data that is currently being transmitted.  Packets are first copied into the TX Packet
Buffer, where they "wait" until the USART1 transmitter isn't busy.  Then, when
the transmitter becomes available, the packet is copied into the TX buffer.

*******************************************************************************/
void ADAHRSCommand::send_next_packet()
{
    // If there are no packets in the buffer that need to be transmitted, return
    if (_tx_packets.size() == 0)
    {
        return;
    }
	 
    // If there is already data in the TX buffer, return
    if (_tx_offset != 0)
    {
        return;
    }

    USARTPacket& pkt = _tx_packets.top();
    
    _tx_buffer[_tx_offset++] = 's';
    _tx_buffer[_tx_offset++] = 'n';
    _tx_buffer[_tx_offset++] = 'p';
    _tx_buffer[_tx_offset++] = pkt.PT;
    _tx_buffer[_tx_offset++] = pkt.address;
	 
    for (int i=0; i<pkt.data_length; ++i)
    {
        _tx_buffer[_tx_offset++] = pkt.packet_data[i];
    }

    _tx_buffer[_tx_offset++] = static_cast<uint8_t>((pkt.checksum >> 8) & 0xFF);
    _tx_buffer[_tx_offset++] = static_cast<uint8_t>((pkt.checksum) & 0xFF);

    _tx_packets.pop();

    // Start the transmission
    if (!_uart->transmit(reinterpret_cast<char*>(_tx_buffer), _tx_offset))
    {
        g_work_queue.add_work_irq(ADAHRSCommand::retransmit, this);
    }
    else
    {
        _tx_offset = 0;
    }
}

// if the uart was busy when told to transmit a packet, this callback will be added
// to the work queue. It will try to send the packet again, or put it on the work
// queue again if the uart is still busy
void ADAHRSCommand::retransmit(void* data)
{
    ADAHRSCommand* cmd = reinterpret_cast<ADAHRSCommand*>(data);
    if (!cmd->_uart->transmit(reinterpret_cast<char*>(cmd->_tx_buffer), cmd->_tx_offset))
    {
        g_work_queue.add_work_irq(ADAHRSCommand::retransmit, cmd);
    }
    else
    {
        cmd->_tx_offset = 0;
    }
}

// Constructs a packet containing the specified data and sends it over the USART
// TODO: remove address_type
void ADAHRSCommand::send_global_data(uint8_t address, uint8_t /*address_type*/,
                                     uint8_t packet_is_batch,
                                     uint8_t batch_size)
{
    USARTPacket response_packet;
	 
    response_packet.PT = PACKET_HAS_DATA;
    response_packet.address = address;
	 
    // If this is a batch read, then define the packet accordingly
    if (packet_is_batch)
    {
        response_packet.PT |= PACKET_IS_BATCH;
        response_packet.PT |= static_cast<uint8_t>(batch_size << 2);
        response_packet.data_length = static_cast<uint8_t>(4*batch_size);
		  
        for (int i=0; i<batch_size; ++i)
        {
            response_packet.packet_data[4*i]
                = static_cast<uint8_t>((_config->get_register(address + i) >> 24) & 0x0FF);
            response_packet.packet_data[4*i+1]
                = static_cast<uint8_t>((_config->get_register(address + i) >> 16) & 0x0FF);
            response_packet.packet_data[4*i+2]
                = static_cast<uint8_t>((_config->get_register(address + i) >> 8) & 0x0FF);
            response_packet.packet_data[4*i+3]
                = static_cast<uint8_t>(_config->get_register(address + i) & 0x0FF);
        }
    }
    // If this is not a batch read, just transmit the data found in the specified address
    else
    {
        response_packet.data_length = 4;
        response_packet.packet_data[0]
            = static_cast<uint8_t>((_config->get_register(address) >> 24) & 0x0FF);
        response_packet.packet_data[1]
            = static_cast<uint8_t>((_config->get_register(address) >> 16) & 0x0FF);
        response_packet.packet_data[2]
            = static_cast<uint8_t>((_config->get_register(address) >> 8) & 0x0FF);
        response_packet.packet_data[3]
            = static_cast<uint8_t>(_config->get_register(address) & 0x0FF);
    }
	 
    // The response packet should now be filled with data.  Compute the Checksum and transmit the packet
    response_packet.checksum = response_packet.compute_checksum();

    add_tx_packet(response_packet);
	 
}

// Handles processing specific to individual packets.   This function is called
// by ProcessPacket after all requisite read and write operation are performed.
void ADAHRSCommand::dispatch_packet(const USARTPacket& pkt)
{
    USARTPacket response_packet;
    int returnval;
    
    // If this packet wrote new data to the device, copy that data to
    // global structures for convenience
    if (pkt.PT & PACKET_HAS_DATA)
    {
        // If this packet wrote to the MISC_CONFIG register, it is
        // possible that the estimation mode changed (from quaternions
        // to Euler Angles or vice versa) Check to see if there was a
        // change.  If so, reset the EKF.
        if (((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
            && (_states->mode() == ADAHRSSensorData::MODE_QUATERNION))
        {
            _states->set_mode(ADAHRSSensorData::MODE_EULER);
            _ekf->begin(_config, _states);				
        }
        else if (((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) != 0)
                 && (_states->mode() == ADAHRSSensorData::MODE_EULER) )
        {
            _states->set_mode(ADAHRSSensorData::MODE_QUATERNION);
            _ekf->begin(_config, _states);
        }
		  
        _states->copy_config_to_states(_config);		  
    }
	 
    switch(pkt.address)
    {
    case UM6_GET_FW_VERSION:
        response_packet.PT = PACKET_HAS_DATA;
        response_packet.address = UM6_GET_FW_VERSION;
        response_packet.data_length = 4;
        response_packet.packet_data[0] = (UM6_FIRMWARE_REVISION >> 24) & 0x0FF;
        response_packet.packet_data[1] = (UM6_FIRMWARE_REVISION >> 16) & 0x0FF;
        response_packet.packet_data[2] = (UM6_FIRMWARE_REVISION >> 8) & 0x0FF;
        response_packet.packet_data[3] = UM6_FIRMWARE_REVISION & 0x0FF;
		  
        response_packet.checksum = response_packet.compute_checksum();
		  
        add_tx_packet(response_packet);
        break;
		  
    case UM6_COMMUNICATION:
        if (pkt.PT & PACKET_HAS_DATA)
        {
            // Update broadcast frequency
            if (_config->get_register(UM6_COMMUNICATION) & UM6_BROADCAST_ENABLED)
            {
                enable_broadcast_mode(static_cast<uint8_t>(
                                          _config->get_register(UM6_COMMUNICATION)
                                          & UM6_SERIAL_RATE_MASK));
            }
            else
            {
                disable_broadcast_mode();
            }
				
            // Update serial baud rate
            update_serial_baud();
					 
            send_command_success_packet(UM6_COMMUNICATION);
        }
        break;
		  
    case UM6_ZERO_GYROS:
        send_command_success_packet(pkt.address);					
        start_gyro_calibration();
        break;
		  
    case UM6_RESET_EKF:
        _ekf->begin(_config, _states);
		  
        send_command_success_packet(pkt.address);				
        break;
		  
    case UM6_GET_DATA:
        send_data_packets();
        break;
		  
    case UM6_SET_ACCEL_REF:				
        _states->state_data.accel_ref_x = _states->state_data.accel_x;  
        _states->state_data.accel_ref_y = _states->state_data.accel_y;
        _states->state_data.accel_ref_z = _states->state_data.accel_z;

        _config->set_register(UM6_ACCEL_REF_X, _states->state_data.accel_ref_x);
        _config->set_register(UM6_ACCEL_REF_Y, _states->state_data.accel_ref_y);
        _config->set_register(UM6_ACCEL_REF_Z, _states->state_data.accel_ref_z);
		  
        send_command_success_packet(pkt.address);
        break;
		  
    case UM6_SET_MAG_REF:
        _states->state_data.mag_ref_x = _states->state_data.mag_x;  
        _states->state_data.mag_ref_y = _states->state_data.mag_y;
        _states->state_data.mag_ref_z = _states->state_data.mag_z;
		  
        _config->set_register(UM6_MAG_REF_X, _states->state_data.mag_ref_x);
        _config->set_register(UM6_MAG_REF_Y, _states->state_data.mag_ref_y);
        _config->set_register(UM6_MAG_REF_Z, _states->state_data.mag_ref_z);
		  
        send_command_success_packet(pkt.address);
        break;
		  
    case UM6_FLASH_COMMIT:
						  
        returnval = _config->write_config_to_flash(UM6_USE_CONFIG_ADDRESS);
		  
        if (returnval != FLASH_COMPLETE)
        {
            send_command_failed_packet(pkt.address);
        }
        else
        {
            send_command_success_packet(pkt.address);                
        }
        break;
				
    case UM6_SAVE_FACTORY:
						  
        returnval = _config->write_config_to_flash(UM6_USE_FACTORY_ADDRESS);
		  
        if (returnval != FLASH_COMPLETE)
        {
            send_command_failed_packet(pkt.address);
        }
        else
        {
            send_command_success_packet(pkt.address);
        }
				
        break;
				
    case UM6_RESET_TO_FACTORY:
				
        send_command_success_packet(pkt.address);
		  
        _config->reset_to_factory();
		  
        break;
		  
    case UM6_SET_HOME_POSITION:
			  
        _config->set_register(UM6_GPS_HOME_LAT, _config->get_register(UM6_GPS_LATITUDE));
        _config->set_register(UM6_GPS_HOME_LONG, _config->get_register(UM6_GPS_LONGITUDE));
        _config->set_register(UM6_GPS_HOME_ALTITUDE, _config->get_register(UM6_GPS_ALTITUDE));
		  
        _states->copy_config_to_states(_config);
		  
        send_command_success_packet(pkt.address);
		  
        break;
				
    default:
        // If this was a write command, send a "success" packet to
        // signal that the write was successful A read packet does not
        // need this because the returned data packet will fill the
        // role.  Similarly, a command packet does not need it because
        // commands are handled above
        if (pkt.PT & PACKET_HAS_DATA)
        {
            send_command_success_packet(pkt.address);
        }
        break;
		  
    }
	 
}

void ADAHRSCommand::send_command_success_packet(uint8_t address)
{
    USARTPacket packet;
    
    packet.PT = PACKET_NO_DATA;
    packet.address = address;
    packet.data_length = 0;
    packet.checksum = packet.compute_checksum();
    
    add_tx_packet(packet);	 
}

void ADAHRSCommand::send_command_failed_packet(uint8_t address)
{
    USARTPacket packet;
	 
    packet.PT = PACKET_NO_DATA | PACKET_COMMAND_FAILED;
    packet.address = address;
    packet.data_length = 0;
    packet.checksum = packet.compute_checksum();
    
    add_tx_packet(packet);
}

// 
void ADAHRSCommand::update_broadcast_rate(uint8_t new_rate)
{
    // Calculate new period.  The desired broadcast frequency is given by
    // ft = (280/255)*new_rate + 20
    // which yields rates ranging from 20 Hz to ~ 300 Hz.
    // With a prescaler value of 100, a system clock of 72Mhz, and no clock
    // division, the timer period should be:
    // new_period = 720000/ft
    int ft = new_rate * 280 / 255 + 20;
    uint16_t new_period = static_cast<uint16_t>(roundf(720000.0f / static_cast<float>(ft)));
    
    // Update TIM2
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = new_period;
    TIM_TimeBaseStructure.TIM_Prescaler = 100;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_DeInit(TIM2);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
}

// 
void ADAHRSCommand::enable_broadcast_mode(uint8_t new_rate)
{
    // disable timer2
    TIM_Cmd(TIM2, DISABLE);
    
    // Set broadcast rate
    update_broadcast_rate(new_rate);

    // Enable Timer 2
    TIM_Cmd(TIM2, ENABLE);

    // Clear pending interrupt bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    // TIM IT enable
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void ADAHRSCommand::disable_broadcast_mode()
{
    // Disable Timer 2
    TIM_Cmd(TIM2, DISABLE);
}

void ADAHRSCommand::update_serial_baud()
{
    int baud_rate = 115200;

    uint8_t baud_selection = (_config->get_register(UM6_COMMUNICATION) >> UM6_BAUD_START_BIT)
        & UM6_BAUD_RATE_MASK;

    switch(baud_selection)
    {
    case 0:
        baud_rate = 9600;
        break;

    case 1:
        baud_rate = 14400;
        break;

    case 2:
        baud_rate = 19200;
        break;

    case 3:
        baud_rate = 38400;
        break;

    case 4:
        baud_rate = 57600;
        break;

    case 5:
        baud_rate = 115200;
        break;

    default:
        baud_rate = 115200;
        break;
    }

    _uart->change_baud_rate(baud_rate);

    // TODO: no gps functionality yet
#if 0
    // Now configure the GPS baud rate
    baud_selection = (_config->get_register(UM6_COMMUNICATION) >> UM6_GPS_BAUD_START_BIT)
        & UM6_GPS_BAUD_RATE_MASK;

    switch(baud_selection)
    {
    case 0:
        baud_rate = 9600;
        break;

    case 1:
        baud_rate = 14400;
        break;

    case 2:
        baud_rate = 19200;
        break;

    case 3:
        baud_rate = 38400;
        break;

    case 4:
        baud_rate = 57600;
        break;

    case 5:
        baud_rate = 115200;
        break;

    default:
        baud_rate = 115200;
        break;
    }

    USART2_Configuration( baud_rate );
#endif
}

// TODO:
void ADAHRSCommand::start_gyro_calibration()
{
#if 0
    gZeroGyroSampleCount = 0;
    gZeroGyroAverages[0] = 0;
    gZeroGyroAverages[1] = 0;
    gZeroGyroAverages[2] = 0;
    
    gZeroGyroEnable = 1;
#endif
}

void ADAHRSCommand::broadcast()
{
    if (gSendStateData == 0)
        return;
    send_data_packets();
    gSendStateData = 0;
}

void ADAHRSCommand::send_data_packets()
{
    // Raw accelerometer data
    if (_config->get_register(UM6_COMMUNICATION) & UM6_ACCELS_RAW_ENABLED)
    {
        send_global_data(UM6_ACCEL_RAW_XY, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Raw rate gyro data
    if (_config->get_register(UM6_COMMUNICATION) & UM6_GYROS_RAW_ENABLED)
    {
        send_global_data(UM6_GYRO_RAW_XY, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Raw magnetometer data
    if (_config->get_register(UM6_COMMUNICATION) & UM6_MAG_RAW_ENABLED)
    {
        send_global_data(UM6_MAG_RAW_XY, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Processed gyro data
    if (_config->get_register(UM6_COMMUNICATION) & UM6_GYROS_PROC_ENABLED)
    {
        send_global_data(UM6_GYRO_PROC_XY, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Processed accelerometer data
    if (_config->get_register(UM6_COMMUNICATION) & UM6_ACCELS_PROC_ENABLED)
    {
        send_global_data(UM6_ACCEL_PROC_XY, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Processed magnetometer data
    if (_config->get_register(UM6_COMMUNICATION) & UM6_MAG_PROC_ENABLED)
    {
        send_global_data(UM6_MAG_PROC_XY, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Euler angle outputs
    if (_config->get_register(UM6_COMMUNICATION) & UM6_EULER_ENABLED)
    {
        send_global_data(UM6_EULER_PHI_THETA, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Quaternion outputs
    if (_config->get_register(UM6_COMMUNICATION) & UM6_QUAT_ENABLED)
    {
        send_global_data(UM6_QUAT_AB, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 2);
    }
    
    // Covariance output
    if (_config->get_register(UM6_COMMUNICATION) & UM6_COV_ENABLED)
    {
        send_global_data(UM6_ERROR_COV_00, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 4);
        send_global_data(UM6_ERROR_COV_10, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 4);
        send_global_data(UM6_ERROR_COV_20, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 4);
        send_global_data(UM6_ERROR_COV_30, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 4);
    }
    
    // Temperature output
    if (_config->get_register(UM6_COMMUNICATION) & UM6_TEMPERATURE_ENABLED)
    {
        send_global_data(UM6_TEMPERATURE, ADDRESS_TYPE_DATA, 0, 0);
    }
    
	// Latitude, longitude, altitude
    if ((_config->get_register(UM6_COMMUNICATION) & UM6_GPS_POSITION_ENABLED)
        && _states->raw_data.new_GPS_position == 1)
    {
        send_global_data(UM6_GPS_LONGITUDE, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 3);
	
        if ((_config->get_register(UM6_COMMUNICATION) & UM6_GPS_REL_POSITION_ENABLED) == 0)
        {
            _states->raw_data.new_GPS_position = 0;
        }
    }
    
    // Relative position
    if ((_config->get_register(UM6_COMMUNICATION) & UM6_GPS_REL_POSITION_ENABLED)
        && _states->raw_data.new_GPS_position == 1)
    {
        send_global_data(UM6_GPS_POSITION_N, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 3);
	
        _states->raw_data.new_GPS_position = 0;
    }
    
    // Satellite summary
    if ((_config->get_register(UM6_COMMUNICATION) & UM6_GPS_SAT_SUMMARY_ENABLED)
        && _states->raw_data.new_GPS_satellite_summary)
    {
        send_global_data(UM6_GPS_SAT_SUMMARY, ADDRESS_TYPE_DATA, 0, 0);
        _states->raw_data.new_GPS_satellite_summary = 0;
    }
    
    // Course and speed
    if ((_config->get_register(UM6_COMMUNICATION) & UM6_GPS_COURSE_SPEED_ENABLED)
        && _states->raw_data.new_GPS_course_speed)
    {
        send_global_data(UM6_GPS_COURSE_SPEED, ADDRESS_TYPE_DATA, 0, 0);
        _states->raw_data.new_GPS_course_speed = 0;
    }
    
    // GPS satellite data
    if ((_config->get_register(UM6_COMMUNICATION) & UM6_GPS_SAT_DATA_ENABLED)
        && _states->raw_data.new_GPS_satellite_data)
    {
        send_global_data(UM6_GPS_SAT_1_2, ADDRESS_TYPE_DATA, PACKET_IS_BATCH, 6);
        _states->raw_data.new_GPS_satellite_data = 0;
    }
}

// interrupt handler for Timer2
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // send broadcast packet
        gSendStateData = 1;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// ----------------------------------------------------------------------------

