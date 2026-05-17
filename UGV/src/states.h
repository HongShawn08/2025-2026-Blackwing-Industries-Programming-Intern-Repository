#pragma once
#include "prng.h"


enum BroadcastHeader: uint16_t {
    BROADCAST     = rand16(0),  // Broadcast to all other nodes. TX must match. Payload (uint16_t step)
    FINISH        = rand16(1),  // End communication with a specific node. RX and TX must match for anchors. Payload (optional): (double distance)
    MEASURE_POLL  = rand16(2),  // Measure distance between two nodes. RX and TX must match. Payload: (uint32 t1, uint32 t2)
    MEASURE_RESP  = rand16(3),  // Measure distance between two nodes. RX and TX must match. Payload: (uint32 t1, uint32 t2)
};

enum State: char {
    // ANCHOR STATES
    AWAITING_START,   // Send sync messages and await start message
    RANGING_INIT,     // Initializes the ranging process with the partner
    RANGING_RESP,     // Listens for its partner's ranging pings
    RANGING_IDLE,     // Awaits the next ranging round
    REPORTING,        // Reporting distances
    CALIBRATED,       // Calibration complete. Stand by for localization

    // TAG STATES
    AWAITING_READY,
    STARTING_ROUND, 
    LOCALIZING
};