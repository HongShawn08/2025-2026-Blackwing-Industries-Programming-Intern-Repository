#include "anchor.h"


Anchor::Anchor(uint8_t id): node(ANCHOR, id), state(AWAITING_START) {
    // Similar-ish numbers that are linearly independent
    static const uint16_t delays[] = {
        3<<5, 5<<4, 7<<3, 11<<3, 13<<2, 17<<2, 19<<2, 23<<2, 29<<1, 31<<1, 37<<1, 41<<1, 43<<1, 47<<1, 53, 59, 61, 67, 71
    };
    if (id < 20) this->sync_delay = delays[id] * 2;
    else this->sync_delay = id * 12;  // Fallback for IDs > 20
    this->node.set_target(TAG, 0);
}

void Anchor::await_start() {
    auto now = millis();
    if (now >= next_print_time) {
        next_print_time = now + 1000;
        Serial.print(".");
    }
    // Sync with the tag to tell it that it's ready.
    // Also report back the mean distance from the round (this will be ignored if it's not a valid distance)
    auto code = this->node.sync(this->sync_delay, this->ranging_round, this->mean_distance);
    // Check if the calibration is complete
    constexpr uint8_t num_rounds = NUM_ROUNDS;
    if (code >= num_rounds && code != BROADCAST_INVALID) {
        this->node.set_target(TAG, 0);
        this->node.reset_mode();  // Reset mode for localization
        this->state = CALIBRATED;
        Serial.print("RECEIVED CODE ");
        Serial.print(int(code));
        Serial.println(". CALIBRATED!");
    } else if (code != BROADCAST_INVALID && code != this->ranging_round) {
        // Start next round
        this->ranging_round = code;
        this->partner = get_peer_for_round(this->node.get_slot(), NUM_ANCHORS, this->ranging_round);
        this->node.set_target(ANCHOR, this->partner);
        this->mean_distance = 0;
        Serial.print("Received broadcast to start round ");
        Serial.println(this->ranging_round);
        Serial.print("Peer for round: ");
        Serial.println(this->partner);

        if (this->partner == PEER_NONE)
            this->state = RANGING_IDLE;
        else if (this->node.get_slot() < partner)
            this->state = RANGING_INIT;
        else
            this->state = RANGING_RESP;
    }
}

void Anchor::range_init() {
    auto distance = this->node.poll_measurement(RNG_DELAY_MS);
    if (distance != MEASUREMENT_INVALID) {
        auto n = this->node.get_measurements_in_session();
        this->mean_distance += (distance - this->mean_distance) / double(n);
        Serial.print("Received ");
        Serial.print(n);
        Serial.print("/");
        Serial.print(NUM_SAMPLES);
        Serial.print(" measurements to ");
        Serial.print(this->partner);
        Serial.print(". Newest = ");
        Serial.println(distance);
        if (n >= NUM_SAMPLES) {
            Serial.print("ROUND ");
            Serial.print(this->ranging_round);
            Serial.print(" COMPLETE. REPORTING DISTANCE ");
            Serial.println(this->mean_distance);
            this->state = REPORTING;
        }
    } else {
        Serial.println("NaN");
    }
}

void Anchor::range_resp() {
    auto response = this->node.resp_measurement();

    if (response == RESPONSE_TERMINATE) {
        // Check if we're done
        Serial.print("ROUND ");
        Serial.print(this->ranging_round);
        Serial.println(" COMPLETE. AWAITING START...");
        this->state = AWAITING_START;
        if (this->node.get_measurements_in_session() < NUM_SAMPLES) {
            Serial.println("ERROR: BAD TERMINATION!");
        }
    } else if (response == RESPONSE_VALID) {
        auto n = this->node.get_measurements_in_session();
        Serial.print("Responded ");
        Serial.print(n);
        Serial.print("/");
        Serial.print(NUM_SAMPLES);
        Serial.print(" measurements to ");
        Serial.println(this->partner);
    } else {
        auto now = millis();
        if (now >= next_print_time) {
            next_print_time = now + 1000;
            Serial.println("FAILED TO RESPOND");
        }
    }
}

void Anchor::range_idle() {
    this->node.set_target(TAG, 0);
    Serial.print("IDLE FOR ROUND ");
    Serial.println(this->ranging_round);
    this->state = AWAITING_START;
} 

void Anchor::report() {
    await_start();
}

void Anchor::localize_resp() {
    // Ensure we're configured to respond to the Tag, not another anchor
    this->node.set_target(TAG, 0);
    auto response = this->node.resp_measurement();
    if (response == RESPONSE_VALID) {
        Serial.print("L");  // Localization response sent
    } else if (response == RESPONSE_INVALID) {
        Serial.print("E");  // Error
    }
}

void Anchor::update() {
    switch (this->state) {
        case AWAITING_START: 
            await_start(); break;
        case RANGING_INIT:
            range_init(); break;
        case RANGING_RESP:
            range_resp(); break;
        case RANGING_IDLE:
            range_idle(); break;
        case REPORTING:
            report(); break;
        case CALIBRATED:
            localize_resp(); break;
        default:
            // Hopefully we never reach this branch
            Serial.println("HOW THE FUCK DID WE GET HERE???");
            break;
    }
}