#include "tag.h"
#include <math.h>
#include <common/mavlink.h>

Tag::Tag(uint8_t id): state(AWAITING_READY), node(TAG, id), ranging_round(255), matrix(new double[NUM_ANCHORS * (NUM_ANCHORS - 1) / 2]) {
    this->node.set_target(TAG, 0);
}

double Tag::matrix_get(int r, int c) {
    if (r == c) return 0;
    if (r < c) std::swap(r, c);
    return this->matrix[r * (r - 1) / 2 + c];
}

void Tag::matrix_set(int r, int c, double dist) {
    if (r < c) std::swap(r, c);
    this->matrix[r * (r - 1) / 2 + c] = dist;
}

void Tag::free_matrix() {
    if (this->matrix != nullptr) {
        delete[] this->matrix;
        this->matrix = nullptr;
    }
}

void Tag::free_distances() {
    if (this->distances != nullptr) {
        delete[] this->distances;
        this->distances = nullptr;
    }
}

void Tag::reset() {
    this->ranging_round = 255;
    this->node.set_target(TAG, 0);
    this->state = AWAITING_READY;
    this->free_matrix();
    this->free_distances();
    this->matrix = new double [NUM_ANCHORS * (NUM_ANCHORS - 1) / 2];
}

void Tag::reset_anchors() {
    memset(this->anchors_found, 0, NUM_ANCHORS);
    this->num_anchors_found = 0;
}

void Tag::await_ready() {
    auto now = millis();
    if (now >= next_print_time) {
        next_print_time = now + 3000;
        Serial.print("Awaiting connections... ");
        Serial.print(int(num_anchors_found));
        Serial.print("/");
        Serial.println(NUM_ANCHORS);
    }
    NodeID tx_id, rx_id;
    double dist_found;
    uint8_t round;
    this->node.check_ready(tx_id, rx_id, dist_found, round, this->state == STARTING_ROUND);

    if (dist_found != MEASUREMENT_INVALID) {
        int row = rx_id.id, col = tx_id.id;
        this->matrix_set(row, col, dist_found);
        Serial.print("Distance ");
        Serial.print(row);
        Serial.print("-");
        Serial.print(col);
        Serial.print(": ");
        Serial.println(dist_found);
    }

    if (tx_id.type != TAG && (round == this->ranging_round || this->ranging_round == 255)) {
        bool isNew = !anchors_found[tx_id.id];
        if (isNew) {
            Serial.print("Found anchor ");
            Serial.println(int(tx_id.id));
        }
        this->num_anchors_found += isNew;
        this->anchors_found[tx_id.id] = true;
        if (this->num_anchors_found >= NUM_ANCHORS) {
            ++this->ranging_round;
            Serial.print("ROUND ");
            Serial.println(int(this->ranging_round));
            reset_anchors();
            const uint8_t num_rounds = NUM_ROUNDS;
            if (this->ranging_round < num_rounds) {
                Serial.print("Advancing to round ");
                Serial.println(int(this->ranging_round));
                this->state = STARTING_ROUND;
            } else {
                Serial.println("Calibration complete!");
                
                // --- GEOMETRY CALCULATION ---
                double d01 = matrix_get(0, 1);
                double d02 = matrix_get(0, 2);
                double d12 = matrix_get(1, 2);

                if (d01 > 0 && d02 > 0 && d12 > 0) {
                    this->anchor1_x = d01;
                    this->anchor2_x = (pow(d01, 2) + pow(d02, 2) - pow(d12, 2)) / (2 * d01);
                    this->anchor2_y = sqrt(fabs(pow(d02, 2) - pow(this->anchor2_x, 2)));
                    
                    Serial.println("--- GEOMETRY CALCULATED ---");
                    Serial.print("A0: (0.00, 0.00)\n");
                    Serial.print("A1: ("); Serial.print(this->anchor1_x); Serial.println(", 0.00)");
                    Serial.print("A2: ("); Serial.print(this->anchor2_x); Serial.print(", "); 
                    Serial.print(this->anchor2_y); Serial.println(")");
                } else {
                    // Fallback to defaults
                    this->anchor1_x = 2.0;
                    this->anchor2_x = 1.0; 
                    this->anchor2_y = 1.73; 
                    Serial.println("WARN: Calibration missing data. Using defaults.");
                }

                this->distances = new double[NUM_ANCHORS]{0};
                
                // Broadcast to start localization mode
                for (auto i = 0; i < 20; ++i) {
                    this->node.broadcast(++this->ranging_round, PAIR_DELAY_MS);
                }
                this->state = LOCALIZING;
            }
        }
    }
}

void Tag::next_round() {
    auto now = millis();
    if (now >= next_print_time) {
        next_print_time = now + 3000;
        Serial.print("Awaiting reports for round ");
        Serial.println(int(this->ranging_round));
    }
    this->node.broadcast(this->ranging_round, 1000);
    this->await_ready();
    yield();
}

void Tag::localize() {
    // 1. Measure all anchors
    for (uint8_t i = 0; i < NUM_ANCHORS; ++i) {
        node.set_target(ANCHOR, i);
        
        // Timeout 10ms (matches your uwb-node.h signature)
        double d = this->node.poll_measurement(10); 
        
        if(d > 0.0 && d < 100.0) { 
            distances[i] = d;
        }
    }

    // 2. Trilateration
    double r0 = distances[0];
    double r1 = distances[1];
    double r2 = distances[2];

    if (this->anchor1_x > 0 && r0 > 0 && r1 > 0 && r2 > 0) {
        double x = (pow(r0, 2) - pow(r1, 2) + pow(this->anchor1_x, 2)) / (2 * this->anchor1_x);
        double y = (pow(r0, 2) - pow(r2, 2) + pow(this->anchor2_x, 2) + pow(this->anchor2_y, 2) - 2 * this->anchor2_x * x) / (2 * this->anchor2_y);

        Serial.print("POS: ");
        Serial.print(x);
        Serial.print(", ");
        Serial.println(y);

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        // Get current time in microseconds (required by MAVLink)
        uint64_t usec_time = micros();

        // Pack the variables into the VISION_POSITION_ESTIMATE message
        // System ID = 1, Component ID = 1 (Identifies the ESP32)
        // 1. Create an empty array of 21 floats right above the MAVLink function
        float covariance[21] = {0};
        
        // 2. Pass that array into the function, and change Component ID to 158
        mavlink_msg_vision_position_estimate_pack(
            1, 158, &msg,         // 158 is the official MAVLink ID for a Companion Computer
            usec_time,            // Timestamp
            x,                    // X position in meters (North)
            y,                    // Y position in meters (East)
            0.0,                  // Z position 
            0.0, 0.0, 0.0,        // Roll, Pitch, Yaw 
            covariance,           // <-- Pass the empty array instead of NULL
            0                     // Reset counter
        );

        // Convert the message into a binary byte array
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        // Send it to the Pixhawk! 
        // (Assuming you initialized Serial2 in your main .ino setup to talk to the Pixhawk)
        Serial2.write(buf, len);
    }
    
    auto now = millis();
    if (now >= next_print_time) {
        next_print_time = now + 1000;
        print_distances();
    }
}
    
void Tag::update() {
    switch (this->state) {
        case AWAITING_READY:
            await_ready(); break;
        case STARTING_ROUND:
            next_round(); break;
        case LOCALIZING:
            localize(); break;
        default:
            Serial.println("Error: Unknown State");
            break;
    }
}

void Tag::print_matrix() {
    Serial.println("Distance Matrix: [");
    for (int i = 0; i < NUM_ANCHORS; ++i) {
        Serial.print("[ ");
        for (int j = 0; j < NUM_ANCHORS; ++j) {
            Serial.print(this->matrix_get(i, j), 3);
            Serial.print(",\t");
        }
        Serial.println(" ],");
    }
    Serial.println("]");
}

void Tag::print_distances() {
    Serial.print("Distances: [");
    for (int i = 0; i < NUM_ANCHORS; ++i) {
        Serial.print(this->distances[i], 3);
        Serial.print(", ");
    }
    Serial.println("]");
}
