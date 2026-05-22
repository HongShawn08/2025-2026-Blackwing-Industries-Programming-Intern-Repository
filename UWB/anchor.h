#pragma once
#include "uwb-node.h"
#include "scheduler.h"

class Anchor {
private:
    double mean_distance{0};
    UWBNode node;
    State state;
    uint8_t ranging_round{255};
    uint8_t partner;
    uint16_t sync_delay;

    uint32_t next_print_time;

    // AWAIT_START,      // Send sync messages and await start message
    // RANGING_INIT,     // Initializes the ranging process with the partner
    // RANGING_RESP,     // Listens for its partner's ranging pings
    // RANGING_IDLE,     // Awaits the next ranging round
    // REPORTING,        // Reporting distances
    // CALIBRATED,       // Calibration complete. Stand by for localization

    void await_start();
    void range_init();
    void range_resp();
    void range_idle();
    void report();
    void localize_resp();
public:
    Anchor(uint8_t id);
    void update();
};