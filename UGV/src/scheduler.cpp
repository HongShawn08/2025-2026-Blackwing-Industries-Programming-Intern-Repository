#include "scheduler.h"

uint8_t get_peer_for_round(uint8_t my_id, uint8_t n, uint8_t round) {
    // Convert to 0-indexed
    uint8_t my_idx = my_id;
    
    // Effective number of nodes (round up to even for ghost node)
    uint8_t eff_n = (n % 2 == 0) ? n : (n + 1);
    uint8_t num_rotate = eff_n - 1;  // Number of rotating positions (excludes fixed node)
    uint8_t fixed_idx = eff_n - 1;   // The fixed/pivot node index (ghost if N is odd)
    
    uint8_t round_mod = round % num_rotate;
    
    uint8_t peer_idx;
    if (my_idx == fixed_idx) {
        // I am the fixed node; pair with the node at position 'roundMod'
        peer_idx = round_mod;
    } else if (my_idx == round_mod) {
        // I'm the node at position 'roundMod'; I pair with the fixed node
        peer_idx = fixed_idx;
    } else {
        // Standard symmetric pairing
        // Partner position = (2 * roundMod - myIdx) mod numRotate
        // Using safe modulo for potentially negative intermediate values
        int16_t diff = 2 * (int16_t)round_mod - (int16_t)my_idx;
        peer_idx = (uint8_t)(((diff % num_rotate) + num_rotate) % num_rotate);
    }
    
    // Check if partner is the ghost node (only possible if N is odd)
    if (n % 2 == 1 && peer_idx >= n) {
        return PEER_NONE;
    }
    
    return peer_idx;
}
