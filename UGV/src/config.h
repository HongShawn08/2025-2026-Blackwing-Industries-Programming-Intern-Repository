#pragma once

// Ranging delays
#define RNG_DELAY_MS 3     // Delay between ranging polls
#define PAIR_DELAY_MS 250  // Delay between pairing polls

#define POLL_RX_TO_RESP_TX_DLY_UUS 900  // Delay from when the RX receives the poll to when it broadcasts a response
#define POLL_TX_TO_RESP_RX_DLY_UUS 300  // Delay from when the TX transmits a poll to when it checks for responses
#define RESP_RX_TIMEOUT_UUS 2700        // Duration for which the TX checks for responses before moving on

#define CHECK_TIMEOUT_MS 1000           // Duration for which we check for messages before timing out

#define NUM_ANCHORS 3
#define NUM_ROUNDS ((NUM_ANCHORS % 2 == 0) ? (NUM_ANCHORS - 1) : NUM_ANCHORS)
#define NUM_SAMPLES 1000

#define PEER_NONE 255

// ToF coefficients
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define SLOPE 0.954
#define INTERCEPT 0.112

#ifndef UUS_TO_DWT_TIME
#define UUS_TO_DWT_TIME 63890
#endif

// UWB message lengths
#define MSG_COMMON_LEN 8
#define BROADCAST_COMMON_LEN 4
#define MAX_MSG_LEN 20
#define RX_BUFFER_LEN 24

// Header start for all the blackbird industry UWB messages
#define BB0 '\xBB'
#define BB1 '\x26'

// UWB message parsing
#define BB_LEN 2
#define HEADER_LEN 2
#define PAYLOAD_LEN 8
#define HEADER_IDX 2
#define TX_TYPE_IDX 4
#define TX_ID_IDX 5
#define RX_TYPE_IDX 6
#define RX_ID_IDX 7
#define SN_IDX 8
#define PAYLOAD_IDX 9
