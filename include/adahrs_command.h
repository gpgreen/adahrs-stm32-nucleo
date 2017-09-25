/*
 * adahrs_command.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_COMMAND_H_
#define ADAHRS_COMMAND_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "adahrs_config.h"
#include "adahrs_states.h"
#include "adahrs_ekf.h"
#include "stm32_usart.h"
#include "ring_buffer.h"

// ----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

// Structure for storing TX and RX packet data
struct USARTPacket
{
public:
    uint8_t PT;				// Packet type
    uint8_t address;			// Packet address
    uint16_t checksum;		        // Checksum
    
    // Data included for convenience, but that isn't stored in the packet itself
    uint8_t data_length; 	        // Number of bytes in data section 
    uint8_t address_type;	        // Specified the address type (DATA, CONFIG, OR COMMAND)
    
    uint8_t packet_data[MAX_PACKET_DATA];

    USARTPacket();
    USARTPacket(const USARTPacket& pkt);
    USARTPacket& operator=(const USARTPacket& pkt);
};

// ----------------------------------------------------------------------------

class ADAHRSCommand
{
public:
    explicit ADAHRSCommand();

    void begin(USART* uart, ADAHRSConfig* config, ADAHRSSensorData* state,
               EKF* ekf);

    // main loop, check for new char, process it
    void process_next_character();

    // add a new packet to outgoing queue
    void add_tx_packet(const USARTPacket& pkt);

    // have we received a new packet?
    bool have_new_packet() const
    {
        return _new_packet_received;
    }

    // process a new received packet
    void process_rx_packet();

    // main loop, check for and send an outgoing packet from queue
    void send_next_packet();

    // create a data packet, and add packet to outgoing queue
    void send_global_data(uint8_t address, uint8_t address_type, uint8_t packet_is_batch,
                          uint8_t batch_size);

    // after process_rx_packet, execute the command it entails
    void dispatch_packet(const USARTPacket& pkt);

    // command methods
    void update_broadcast_rate(uint8_t new_rate);
    void enable_broadcast_mode(uint8_t new_rate);
    void disable_broadcast_mode();
    void update_serial_baud();
    void start_gyro_calibration();
    void send_data_packet();

    // checksum on a packet
    static uint16_t compute_checksum(USARTPacket* new_packet);

private:

    // try to send outgoing tx packet
    static void retransmit(void* data);

    // add a received packet to incoming queue
    void add_rx_packet(const USARTPacket& pkt);

    // do what the name says
    void send_command_success_packet(uint8_t address);
    void send_command_failed_packet(uint8_t address);

    // private data for class
    
    USART* _uart;
    ADAHRSConfig* _config;
    ADAHRSSensorData* _states;
    EKF* _ekf;
    uint8_t _state;
    uint8_t _data_counter;
    uint8_t _new_packet_received;
    USARTPacket _new_packet;

    uint32_t _rx_offset;
    uint32_t _tx_offset;
    uint8_t _rx_buffer[COMMAND_BUFFER_SIZE];
    uint8_t _tx_buffer[COMMAND_BUFFER_SIZE];

    RingBuffer<USARTPacket> _rx_packets;
    RingBuffer<USARTPacket> _tx_packets;
    
    // define away copy constructor and assignment operator
    ADAHRSCommand(const ADAHRSCommand&);
    const ADAHRSCommand& operator=(const ADAHRSCommand&);
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
#endif
