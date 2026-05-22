#pragma once
#include "uwb-node.h"
#include "config.h"
#include "states.h"

class Tag {
private:
    UWBNode node;
    State state;
    uint8_t num_broadcasts{0};
    uint8_t ranging_round{255};
    uint8_t target=0;
    uint8_t num_anchors_found{0};
    double* matrix;
    double* distances = nullptr;
    bool anchors_found[NUM_ANCHORS];

    double anchor1_x = 0.0;
    double anchor2_x = 0.0;
    double anchor2_y = 0.0;

    uint32_t next_print_time;

    void matrix_set(int r, int c, double dist);
    double matrix_get(int r, int c);
    void reset_anchors();
    
    void await_ready();
    void next_round();
    void localize();
public:
    Tag(uint8_t id);
    void reset();
    void update();
    void print_matrix();
    void print_distances();
    void free_matrix();
    void free_distances();
};
