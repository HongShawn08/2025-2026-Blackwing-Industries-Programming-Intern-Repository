#pragma once
#include <cstring>
#include "SPI.h"
#include "dw3000.h"
#include "states.h"
#include "config.h"

#define MEASUREMENT_INVALID -10000
#define BROADCAST_INVALID 254
#define RESPONSE_INVALID 0
#define RESPONSE_TERMINATE 1
#define RESPONSE_VALID 2
#define STATUS_LISTENING (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)
#define STATUS_TRANSMITTING (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)

enum NodeType: char {
    ANCHOR = 'a',
    TAG = 'T'
};

struct NodeID {
    NodeType type;
    uint8_t id;
};

class UWBNode {
private:
    enum class Mode : char {
        BROADCASTING = 'b',
        SYNCING = 's',
        COLLECTING = 'c',
        POLLING = 'p',
        RESPONDING = 'r',
        IDLE = 'i'
    };

    uint64_t next_tx_time{0};
    uint32_t measurements_in_session{0};
    //      NAME        | SIZE | INDEX
    // ————————————————————————————————
    // BLACKBIRD PREFIX |   2  |   0
    // HEADER           |   2  |   2
    // TRANSMITTER ID   |   2  |   4
    // RECEIVER ID      |   2  |   6
    // SEQUENCE NUMBER  |   1  |   8
    // PAYLOAD          |   8  |   9
    uint8_t tx_msg[MAX_MSG_LEN]{};      // next message to send
    uint8_t rx_msg[MAX_MSG_LEN]{};      // expected response template
    uint8_t rx_buffer[RX_BUFFER_LEN]{}; // last message received
    
    NodeID id;
    Mode mode {Mode::IDLE};

    uint8_t tx_sequence_number{0};          // Used to avoid duplicate messages
    uint8_t rx_sequence_number{0};          // Used to avoid duplicate messages

    // Prepare a message
    void write_tx_broadcast(uint8_t code);
    void write_tx_report(double distance);
    
    // Write common fields into tx_msg (sets receiver info and header)
    inline void write_tx_header(BroadcastHeader header) {
        memcpy(tx_msg + HEADER_IDX, &header, HEADER_LEN);
    }

    // Write common fields into rx_msg (sets expected transmitter info and header for matching)
    inline void write_rx_header(BroadcastHeader header) {
        memcpy(rx_msg + HEADER_IDX, &header, HEADER_LEN);
    }

    // Broadcast the queued message
    int transmit(uint8_t mode);
    
    // Load any messages received
    bool check_messages(uint32_t hang_timeout_ms, unsigned long status_mask);
public:
    UWBNode(NodeType type, uint8_t id);
    UWBNode(NodeID id);

    // Set the messaging target
    inline void set_target(NodeType target_type, uint8_t target_id) {
        tx_msg[RX_TYPE_IDX] = target_type;
        tx_msg[RX_ID_IDX] = target_id;
        rx_msg[TX_TYPE_IDX] = target_type;
        rx_msg[TX_ID_IDX] = target_id;
    }

    inline void set_target(NodeID id) {
        memcpy(&this->tx_msg[RX_TYPE_IDX], &id, sizeof(NodeID));
        memcpy(&this->rx_msg[TX_TYPE_IDX], &id, sizeof(NodeID));
    }
    
    // Poll measurement
    double poll_measurement(uint32_t ping_interval_ms);

    // Respond to a measurment poll
    uint8_t resp_measurement();

    // Synchronize and report
    uint8_t sync(uint32_t ping_interval_ms, uint8_t round, double distance = 0);

    // Broadcast a message
    void broadcast(uint8_t code, uint32_t ping_interval_ms);
    
    // Check for ready anchors
    void check_ready(NodeID& tx_id_out, NodeID& rx_id_out, double& dist_out, uint8_t& sn, bool is_broadcasting=false);

    // Get the number of measurements in the session (resets every time the mode is changed)
    inline uint32_t get_measurements_in_session() const { return measurements_in_session; }

    // Reset mode to IDLE (forces reinitialization on next poll/resp call)
    inline void reset_mode() { mode = Mode::IDLE; }

    inline NodeID get_id() const { return id; }
    inline uint8_t get_slot() const { return id.id; }
    inline NodeType get_type() const { return id.type; }
};