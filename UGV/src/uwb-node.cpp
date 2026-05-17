#include <limits>
#include <cstring>
#include "uwb-node.h"

UWBNode::UWBNode(NodeType type, uint8_t id): UWBNode({.type=type, .id=id}) {}

UWBNode::UWBNode(NodeID id): id(id) {
    this->tx_msg[0] = BB0;
    this->tx_msg[1] = BB1;
    this->rx_msg[0] = BB0;
    this->rx_msg[1] = BB1;
    memcpy(&this->tx_msg[TX_TYPE_IDX], &id, sizeof(NodeID));
    memcpy(&this->rx_msg[RX_TYPE_IDX], &id, sizeof(NodeID));
}

// Prepare a tx message for broadcast
void UWBNode::write_tx_broadcast(uint8_t code) {
    set_target(NodeType::TAG, 0);
    write_tx_header(BroadcastHeader::BROADCAST);
    memcpy(&this->tx_msg[PAYLOAD_IDX], &code, sizeof(uint8_t));
}

// Prepare a tx message for reporting a measured distance
void UWBNode::write_tx_report(double distance) {
    write_tx_header(BroadcastHeader::FINISH);
    memcpy(this->tx_msg + PAYLOAD_IDX, &distance, sizeof(double));
}

// Broadcast the queued message
int UWBNode::transmit(uint8_t mode) {
    this->tx_msg[SN_IDX] = this->tx_sequence_number;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(this->tx_msg), this->tx_msg, 0);
    dwt_writetxfctrl(sizeof(this->tx_msg), 0, 1);
    return dwt_starttx(mode);
}

// Check for matching messages
bool UWBNode::check_messages(uint32_t hang_timeout_ms, unsigned long status_mask) {
    uint32_t start_wait = millis();
    static uint32_t status_reg = 0;

    while (((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & status_mask) == 0) {
        // Safety timeout in case hardware hangs
        // yield();
        if (millis() - start_wait >= hang_timeout_ms) { 
            Serial.println("Timed out");
            return false;
        }
    };

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        // Get message length
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            // Load the message in
            dwt_readrxdata(rx_buffer, frame_len, 0);
            return true;
        }
    } else {
        // Clean up error bits
        dwt_write32bitreg(SYS_STATUS_ID, status_mask );
        Serial.print("E");
    }

    return false;
}

double UWBNode::poll_measurement(uint32_t ping_interval_ms) {
    auto time_ms = millis();
    if (this->mode != Mode::POLLING) {
        write_tx_header(BroadcastHeader::MEASURE_POLL);
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        this->measurements_in_session = 0;
        this->mode = Mode::POLLING;
        this->next_tx_time = time_ms;
        this->rx_sequence_number = 255;
        this->tx_sequence_number = 0;
    }

    // Wait to transmit
    if (time_ms < next_tx_time) 
        delay(next_tx_time - time_ms);
    this->next_tx_time = millis() + ping_interval_ms;
    this->transmit(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    this->write_rx_header(BroadcastHeader::MEASURE_RESP);

    // Await response
    if (check_messages(CHECK_TIMEOUT_MS, STATUS_TRANSMITTING)) {
        if (memcmp(rx_buffer, rx_msg, MSG_COMMON_LEN) == 0) {
            uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32_t rtd_init, rtd_resp;
            float clockOffsetRatio;
            double tof;

            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();
            clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

            resp_msg_get_ts(&this->rx_buffer[PAYLOAD_IDX], &poll_rx_ts);
            resp_msg_get_ts(&this->rx_buffer[PAYLOAD_IDX + 4], &resp_tx_ts);

            // DEBUG: Hex dump of payload bytes
            // Serial.print("RAW: ");
            // for (int i = PAYLOAD_IDX; i < PAYLOAD_IDX + 8; i++) {
            //     if (this->rx_buffer[i] < 16) Serial.print("0");
            //     Serial.print(this->rx_buffer[i], HEX);
            //     Serial.print(" ");
            // }
            // Serial.println();

            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            if (this->rx_buffer[SN_IDX] != this->rx_sequence_number) {
                this->rx_sequence_number = this->rx_buffer[SN_IDX];
                ++this->measurements_in_session;
            }
            ++this->tx_sequence_number;

            tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
            return tof * SPEED_OF_LIGHT * SLOPE + INTERCEPT;
        } else {
            // Message didn't match expected format
        }
    }
    return MEASUREMENT_INVALID;
}

uint8_t UWBNode::resp_measurement() {
    static uint64_t poll_rx_ts;
    static uint64_t resp_tx_ts;
    
    if (this->mode != Mode::RESPONDING) {
        write_tx_header(BroadcastHeader::MEASURE_RESP);
        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0);
        this->measurements_in_session = 0;
        this->mode = Mode::RESPONDING;
        this->rx_sequence_number = 255;
        this->tx_sequence_number = 0;
    }

    // Listen for polls
    this->tx_msg[SN_IDX] = this->tx_sequence_number;
    this->write_rx_header(BroadcastHeader::MEASURE_POLL);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    if (check_messages(CHECK_TIMEOUT_MS, STATUS_LISTENING)) {
        if (memcmp(rx_buffer, rx_msg, MSG_COMMON_LEN) == 0) {
            uint32_t resp_tx_time;
            int ret;

            poll_rx_ts = get_rx_timestamp_u64();
            
            uint64_t delay_ticks = (uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * (uint64_t)UUS_TO_DWT_TIME;
            uint64_t resp_tx_time_u64 = (poll_rx_ts + delay_ticks) >> 8;
            
            resp_tx_time = (uint32_t)resp_tx_time_u64;
            
            dwt_setdelayedtrxtime(resp_tx_time);

            resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            
            resp_msg_set_ts(&tx_msg[PAYLOAD_IDX], (uint32_t)poll_rx_ts);
            resp_msg_set_ts(&tx_msg[PAYLOAD_IDX + 4], (uint32_t)resp_tx_ts);
            
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            dwt_writetxdata(sizeof(this->tx_msg), this->tx_msg, 0);
            dwt_writetxfctrl(sizeof(this->tx_msg), 0, 1);
            ret = dwt_starttx(DWT_START_TX_DELAYED);
            
            bool is_new = this->rx_buffer[SN_IDX] != this->rx_sequence_number;
            this->rx_sequence_number = this->rx_buffer[SN_IDX];
            ++this->tx_sequence_number;

            if (ret == DWT_SUCCESS) {
                // Wait for the transmission to complete
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { yield(); };
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                if (is_new) ++this->measurements_in_session;
                return RESPONSE_VALID;
            } else {
                Serial.print("Failed to respond in time.");
                // Abort any pending TX to prevent stale data from being sent
                // dwt_forcetrxoff();
            }
        } else {
            write_rx_header(BroadcastHeader::FINISH);
            if (memcmp(rx_buffer, rx_msg, MSG_COMMON_LEN) == 0) {
                return RESPONSE_TERMINATE;
            }
        }
        
    }
    return RESPONSE_INVALID;
}

uint8_t UWBNode::sync(uint32_t ping_interval, uint8_t round, double distance) {
    auto time_ms = millis();
    if (this->mode != Mode::SYNCING) {
        write_tx_report(distance);
        dwt_setrxaftertxdelay(240);
        dwt_setrxtimeout(ping_interval * 1000);
        this->measurements_in_session = 0;
        this->mode = Mode::SYNCING;
        this->next_tx_time = time_ms;
    }
    this->tx_sequence_number = round;

    // Send ping
    auto ret = transmit(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // Check for broadcasts
    this->write_rx_header(BroadcastHeader::BROADCAST);
    // dwt_rxenable(DWT_START_RX_IMMEDIATE);

    if (check_messages(CHECK_TIMEOUT_MS, STATUS_TRANSMITTING) && memcmp(this->rx_buffer, this->rx_msg, BROADCAST_COMMON_LEN) == 0) {
        // Load the broadcast code
        uint8_t code;
        memcpy(&code, &this->rx_buffer[PAYLOAD_IDX], sizeof(code));
        return code;
    }
    return BROADCAST_INVALID;
}

void UWBNode::check_ready(NodeID& tx_id_out, NodeID& rx_id_out, double& dist_out, uint8_t& sn, bool is_broadcasting) {
    if (this->mode != Mode::COLLECTING && this->mode != Mode::BROADCASTING) {
        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0);
        this->measurements_in_session = 0;
        this->mode = Mode::COLLECTING;
    }
    // Check for FINISH messages
    this->write_rx_header(BroadcastHeader::FINISH);

    unsigned long status_mask = STATUS_TRANSMITTING;
    if (!is_broadcasting) {
        status_mask = STATUS_LISTENING;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    if (check_messages(CHECK_TIMEOUT_MS, status_mask) && memcmp(this->rx_buffer, this->rx_msg, BROADCAST_COMMON_LEN) == 0) {
        // Get the sender ID
        memcpy(&tx_id_out, &this->rx_buffer[TX_TYPE_IDX], sizeof(tx_id_out));
        memcpy(&rx_id_out, &this->rx_buffer[RX_TYPE_IDX], sizeof(rx_id_out));
        sn = this->rx_buffer[SN_IDX];
        
        // Check if this was sent to an anchor (i.e., it was a termination message)
        if (rx_id_out.type == ANCHOR && tx_id_out.id < rx_id_out.id) {
            // Get distance if this one has the lower ID in the pair
            memcpy(&dist_out, &this->rx_buffer[PAYLOAD_IDX], sizeof(double));
        } else {
            dist_out = MEASUREMENT_INVALID;
        }
        return;
    }

    // Nothing was received
    tx_id_out = {TAG, 0};
    rx_id_out = {TAG, 0};
    dist_out = MEASUREMENT_INVALID;
}

void UWBNode::broadcast(uint8_t code, uint32_t ping_interval_ms) {
    auto time_ms = millis();
    if (this->mode != Mode::BROADCASTING) {
        write_tx_broadcast(code);
        dwt_setrxaftertxdelay(240);
        dwt_setrxtimeout(ping_interval_ms * 1000);
        this->measurements_in_session = 0;
        this->mode = Mode::BROADCASTING;
        this->next_tx_time = time_ms;
    }

    // Wait to transmit
    if (time_ms < next_tx_time) 
        return;
    this->next_tx_time = time_ms + ping_interval_ms;
    transmit(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}
