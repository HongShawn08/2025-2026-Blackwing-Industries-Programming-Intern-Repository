#pragma once
#include <cstdint>
#include "config.h"
// =============================================================================
// 1-FACTORIZATION SCHEDULING (CIRCLE METHOD)
// =============================================================================

/**
 * Get the peer ID for a given anchor in a given round using 1-factorization.
 * 
 * The circle method works as follows:
 * - For N nodes, if N is odd, add a "ghost" node to make it even (effN)
 * - The last node (effN-1, 0-indexed) is the fixed "pivot"
 * - In each round r, the pivot pairs with node r
 * - Other nodes pair symmetrically: node i pairs with (2r - i) mod (effN-1)
 * 
 * @param my_id   My anchor ID (0-indexed, 0..N-1)
 * @param n       Total number of anchors
 * @param round   Current round index (0-indexed, 0..numRounds-1)
 * @return        Peer ID or PEER_NONE if idle this round
 */
uint8_t get_peer_for_round(uint8_t my_id, uint8_t n, uint8_t round);