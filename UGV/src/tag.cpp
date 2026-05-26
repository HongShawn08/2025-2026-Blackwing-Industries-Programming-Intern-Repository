#include "tag.h"
#include <math.h>
#include <common/mavlink.h>

// ============================================
// ANCHOR POSITIONS — FILL IN DAY-OF
// Get these from the main team after their calibration runs.
// Index = anchor ID. Set all 7 even if you only expect some to respond.
// ============================================
static const double ANCHOR_X[NUM_ANCHORS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static const double ANCHOR_Y[NUM_ANCHORS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

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
                Serial.println("Calibration complete — starting localization.");
                this->distances = new double[NUM_ANCHORS]{0};
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
    // 1. Poll all anchors
    for (uint8_t i = 0; i < NUM_ANCHORS; ++i) {
        node.set_target(ANCHOR, i);
        double d = this->node.poll_measurement(10);
        if (d > 0.0 && d < 100.0) {
            distances[i] = d;
        }
    }

    // 2. Collect valid anchor readings
    double vx[NUM_ANCHORS], vy[NUM_ANCHORS], vr[NUM_ANCHORS];
    int n = 0;
    for (uint8_t i = 0; i < NUM_ANCHORS; ++i) {
        if (distances[i] > 0.0 && distances[i] < 100.0) {
            vx[n] = ANCHOR_X[i];
            vy[n] = ANCHOR_Y[i];
            vr[n] = distances[i];
            n++;
        }
    }
    if (n < 3) return;

    // 3. Linearized least squares using anchor 0 as reference
    // Subtracting anchor 0's equation from each other gives a linear system A*[x,y]^T = b
    double x0 = vx[0], y0 = vy[0], r0 = vr[0];
    double ATA00 = 0, ATA01 = 0, ATA11 = 0;
    double ATb0  = 0, ATb1  = 0;

    for (int i = 1; i < n; ++i) {
        double ai0 = 2.0 * (vx[i] - x0);
        double ai1 = 2.0 * (vy[i] - y0);
        double bi  = vx[i]*vx[i] - x0*x0 + vy[i]*vy[i] - y0*y0 - vr[i]*vr[i] + r0*r0;
        ATA00 += ai0 * ai0;
        ATA01 += ai0 * ai1;
        ATA11 += ai1 * ai1;
        ATb0  += ai0 * bi;
        ATb1  += ai1 * bi;
    }

    double det = ATA00 * ATA11 - ATA01 * ATA01;
    if (fabs(det) < 1e-10) return;

    double x = (ATA11 * ATb0 - ATA01 * ATb1) / det;
    double y = (ATA00 * ATb1 - ATA01 * ATb0) / det;

    this->current_x = x;
    this->current_y = y;

    Serial.print("POS: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);

    // Send position to Pixhawk via MAVLink
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    float covariance[21] = {0};
    mavlink_msg_vision_position_estimate_pack(
        1, 158, &msg,
        micros(),
        x, y, 0.0,
        0.0, 0.0, 0.0,
        covariance, 0
    );
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    auto now = millis();
    if (now >= next_print_time) {
        next_print_time = now + 1000;
        print_distances();
    }
}

void Tag::update() {
#if SKIP_CALIBRATION
    if (this->state == AWAITING_READY) {
        this->free_matrix();
        this->distances = new double[NUM_ANCHORS]{0};
        this->state = LOCALIZING;
        Serial.println("Skipping calibration — starting localization.");
    }
#endif
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
