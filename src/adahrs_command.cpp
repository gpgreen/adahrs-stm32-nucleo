/*
 * ADAHRSCommand.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: ggreen
 */

#include "adahrs_command.h"
#include "stm32_usart.h"

// Size of TX and RX buffers. These buffers hold raw transmitted and received data from the USART.
// These buffers should never have to be accessed directly by user code.
#define	RX_BUF_SIZE					50
#define TX_BUF_SIZE     			50

// Maximum number of data bytes that can be stored in a packet
#define	MAX_PACKET_DATA			70 

// Sizes of buffers for storing RX and TX packet data.  The TX Packet Buffer is used
// to store packets that are "waiting" to be transmitted.  This is necessary since 
// multiple resources may need access to the USART hardware simultaneously.  The TX buffer
// acts as a moderator, storing packets and transmitting them when the hardware becomes available.
// The RX Packet Buffer stores packets received over the USART receiver.  Since multiple packets
// may arrive before a packet is processed, the RX Packet Buffer is neccessary to ensure that no data
// is lost.  Both the TX and RX Packet Buffers are circular.
#define	TX_PACKET_BUFFER_SIZE	15
#define	RX_PACKET_BUFFER_SIZE	10

// Definitions of states for USART receiver state machine (for receiving packets)
#define	USART_STATE_WAIT			1
#define	USART_STATE_TYPE			2
#define	USART_STATE_ADDRESS			3
#define	USART_STATE_DATA			4
#define	USART_STATE_CHECKSUM		5

// Flags for interpreting the packet type byte in communication packets
#define	PACKET_HAS_DATA			(1 << 7)
#define	PACKET_IS_BATCH			(1 << 6)
#define	PACKET_BATCH_LENGTH_MASK	(0x0F)

#define	PACKET_BATCH_LENGTH_OFFSET	2

#define	BATCH_SIZE_2				2
#define	BATCH_SIZE_3				3

#define	PACKET_NO_DATA				0
#define	PACKET_COMMAND_FAILED	(1 << 0)

// Define flags for identifying the type of packet address received
#define	ADDRESS_TYPE_CONFIG		0
#define	ADDRESS_TYPE_DATA			1
#define	ADDRESS_TYPE_COMMAND		2

// Define states used for receiving GPS data
#define STATE_GPS_IDLE			0
#define STATE_GPS_PACKET		1
#define STATE_GPS_CHECKSUM		2

// Define maximum GPS packet length in characters
#define MAX_GPS_PACKET_SIZE		86

// Buffer, buffer index, and TX status flag for USART transmit
extern volatile char gTXBuf[TX_BUF_SIZE];
extern volatile int32_t gTXBufPtr;
extern volatile char gTXBusy;
extern uint8_t gUSART_State;

// USART RX buffer and associated index and flags
extern volatile char gRXBuf[RX_BUF_SIZE];
extern volatile int32_t gRXBufPtr;
extern volatile char gRXPacketReceived;
extern volatile char gRXBufOverrun;

extern volatile char gGPSBuf[RX_BUF_SIZE];
extern volatile int32_t gGPSBufPtr;
extern volatile char gGPSPacketReceived;
	 
// Structure for storing TX and RX packet data
typedef struct __USARTPacket
{
    uint8_t PT;				// Packet type
    uint8_t address;			// Packet address
    uint16_t checksum;		// Checksum
    
    // Data included for convenience, but that isn't stored in the packet itself
    uint8_t data_length; 	// Number of bytes in data section 
    uint8_t address_type;	// Specified the address type (DATA, CONFIG, OR COMMAND)
    
    uint8_t packet_data[MAX_PACKET_DATA];
} USARTPacket;

// ----------------------------------------------------------------------------

ADAHRSCommand::ADAHRSCommand()
{
    // does nothing else
}

void ADAHRSCommand::begin(ADAHRSConfig* config, ADAHRSSensorData* state)
{
}

/*
Takes the character in the RX buffer pointed to by gRXBufPtr and processes it.  This
function should only be called if the buffer pointer points to a character that has not
yet been handled.  This function is designed to be called by the function 
HandleUSART1Reception()

This function does NOT increment gRXBufPtr after the character has been handled.  This
should be done by the calling function.

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
void ADAHRSCommand::process_next_char()
{
    static uint8_t data_counter = 0;
    static USARTPacket new_packet;
	 
    // The next action should depend on the USART state.
    switch( gUSART_State )
    {
        // USART in the WAIT state.  In this state, the USART is waiting to see the sequence of bytes
        // that signals a new incoming packet.
    case USART_STATE_WAIT:
        if( data_counter == 0 )		// Waiting on 's' character
        {
            if( gRXBuf[gRXBufPtr] == 's' )
            {
                data_counter++;
            }
            else
            {
                data_counter = 0;
            }
        }
        else if( data_counter == 1 )		// Waiting on 'n' character
        {
            if( gRXBuf[gRXBufPtr] == 'n' )
            {
                data_counter++;
            }
            else
            {
                data_counter = 0;
            }
        }
        else if( data_counter == 2 )		// Waiting on 'p' character
        {
            if( gRXBuf[gRXBufPtr] == 'p' )
            {
                // The full 'snp' sequence was received.  Reset data_counter (it will be used again
                // later) and transition to the next state.
                data_counter = 0;
                gUSART_State = USART_STATE_TYPE;
            }
            else
            {
                data_counter = 0;
            }
        }
        break;
		  
        // USART in the TYPE state.  In this state, the USART has just received the sequence of bytes that
        // indicates a new packet is about to arrive.  Now, the USART expects to see the packet type.
    case USART_STATE_TYPE:
        new_packet.PT = gRXBuf[gRXBufPtr];
        gUSART_State = USART_STATE_ADDRESS;
        break;
		  
        // USART in the ADDRESS state.  In this state, the USART expects to receive a single byte indicating
        // the address that the packet applies to
    case USART_STATE_ADDRESS:
        new_packet.address = gRXBuf[gRXBufPtr];
				
        // For convenience, identify the type of packet this is and copy to the packet structure
        // (this will be used by the packet handler later)
        if( (new_packet.address >= CONFIG_REG_START_ADDRESS) && (new_packet.address < DATA_REG_START_ADDRESS) )
        {
            new_packet.address_type = ADDRESS_TYPE_CONFIG;
        }
        else if( (new_packet.address >= DATA_REG_START_ADDRESS) && (new_packet.address < COMMAND_START_ADDRESS) )
        {
            new_packet.address_type = ADDRESS_TYPE_DATA;
        }
        else
        {
            new_packet.address_type = ADDRESS_TYPE_COMMAND;
        }
		  
        // Identify the type of communication this is (whether reading or writing to a data or configuration register, or sending a command)
        // If this is a read operation, jump directly to the USART_STATE_CHECKSUM state - there is no more data in the packet
        if( (new_packet.PT & PACKET_HAS_DATA) == 0 )
        {
            gUSART_State = USART_STATE_CHECKSUM;
        }
        // If this is a write operation, go to the USART_STATE_DATA state to read in the relevant data
        else
        {
            gUSART_State = USART_STATE_DATA;
					 
            // Determine the expected number of bytes in this data packet based on the packet type.  A write operation
            // consists of 4 bytes unless it is a batch operation, in which case the number of bytes equals 4*batch_size,
            // where the batch size is also given in the packet type byte.
            if( new_packet.PT & PACKET_IS_BATCH )
            {
                new_packet.data_length = 4*((new_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK);
            }
            else
            {
                new_packet.data_length = 4;
            }
        }
				
        break;
		  
        // USART in the DATA state.  In this state, the USART expects to receive new_packet.length bytes of
        // data.
    case USART_STATE_DATA:
        new_packet.packet_data[data_counter] =  gRXBuf[gRXBufPtr];
        data_counter++;				
		  
        // If the expected number of bytes has been received, transition to the CHECKSUM state.
        if( data_counter == new_packet.data_length )
        {
            // Reset data_counter, since it will be used in the CHECKSUM state.
            data_counter = 0;
					 
            gUSART_State = USART_STATE_CHECKSUM;
        }
				
        break;
		  
        // USART in CHECKSUM state.  In this state, the entire packet has been received, with the exception
        // of the 16-bit checksum.
    case USART_STATE_CHECKSUM:
				
        // Get the highest-order byte
        if( data_counter == 0 )
        {
            new_packet.checksum = ((uint16_t)gRXBuf[gRXBufPtr] << 8);
					 
            data_counter++;
        }
        else // ( data_counter == 1 )
        {
            // Get lower-order byte
            new_packet.checksum = new_packet.checksum | ((uint16_t)gRXBuf[gRXBufPtr] & 0x0FF);
					 
            // Both checksum bytes have been received.  Make sure that the checksum is valid.
            uint16_t checksum = ComputeChecksum( &new_packet );
					 
            // If checksum does not match, send a BAD_CHECKSUM packet
            if( checksum != new_packet.checksum )
            {
                // Send bad checksum packet
                new_packet.PT = PACKET_NO_DATA;
                new_packet.address = UM6_BAD_CHECKSUM;
                new_packet.data_length = 0;	// No data bytes
                new_packet.checksum = ComputeChecksum( &new_packet );
						  
                SendTXPacketSafe( &new_packet );
            }
            else
            {
                // Packet was received correctly.  Add the packet to the RX packet buffer and
                // set a flag indicating that a new packet has been received.  
                AddRXPacket( &new_packet );
                gRXPacketReceived = 1;
            }
					 
            // A full packet has been received.
            // Put the USART back into the WAIT state and reset 
            // the data_counter variable so that it can be used to receive the next packet.
            data_counter = 0;
						  
            gUSART_State = USART_STATE_WAIT;					  
        }
				
        break;
    }
		  
}

/*******************************************************************************
* Function Name  : SendNextPacket
* Input          : None
* Output         : None
* Return         : None
* Description    : 

If the TX packet buffer is not empty, and if the USART transmitter is not already 
operating, this function copies the next packet in the TX packet buffer into 
the TX buffer.  

Note the distinction between the TX Packet Buffer, and the TX Buffer.  The 
TX Packet Buffer is an array of structures containing data that needs to be
transmitted over the USART.  The TX Buffer is an array of bytes, representing
data that is currently being transmitted.  Packets are first copied into the TX Packet
Buffer, where they "wait" until the USART1 transmitter isn't busy.  Then, when
the transmitter becomes available, the packet is copied into the TX buffer.

*******************************************************************************/
void ADAHRSCommand::send_next_packet()
{
    uint8_t PT;
    uint8_t address;
    uint8_t data_length;
    uint16_t checksum;
    int32_t i;
	 
    // If there are no packets in the buffer that need to be transmitted, return
    if( gTXPacketBufferStart == gTXPacketBufferEnd )
    {
        return;
    }
	 
    // If there is already data in the TX buffer, return
    if( gTXBusy )
    {
        return;
    }
	 
    PT = gTXPacketBuffer[gTXPacketBufferStart].PT;
    address = gTXPacketBuffer[gTXPacketBufferStart].address;
    data_length = gTXPacketBuffer[gTXPacketBufferStart].data_length;
    checksum = gTXPacketBuffer[gTXPacketBufferStart].checksum;
	 
    TXBufPush( 's' );
    TXBufPush( 'n' );
    TXBufPush( 'p' );
    TXBufPush( PT );
    TXBufPush( address );
	 
    for( i = 0; i < data_length; i++ )
    {
        TXBufPush( gTXPacketBuffer[gTXPacketBufferStart].packet_data[i] );
    }
	 
    TXBufPush( (char)((checksum >> 8) & 0x0FF) );
    TXBufPush( (char)((checksum) & 0x0FF) );
	 
    // Increment packet buffer start pointer.
    gTXPacketBufferStart++;
    if( gTXPacketBufferStart >= TX_PACKET_BUFFER_SIZE )
    {
        gTXPacketBufferStart = 0;
    }
	 
    // Start the transmission
    USART1_TX_start();
}

// ----------------------------------------------------------------------------

