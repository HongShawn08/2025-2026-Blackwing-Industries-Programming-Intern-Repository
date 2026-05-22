#pragma once
#include <cstdint>

/**
 * @brief Simple pseudorandom 8-bit generator
 * @param x: 8-bit seed
 * @returns: 8-bit pseudorandom number
 */
constexpr uint8_t rand8(uint8_t x) {
    x ^= x >> 3;      // invertible
    x *= 197u;        // odd → invertible mod 256
    x ^= x >> 4;      // invertible
    x += 101u;        // invertible
    return x;
}

/**
 * @brief Inverse of rand8
 * @param x: 8-bit pseudorandom number
 * @returns: 8-bit seed
 */
constexpr uint8_t inv_rand8(uint8_t x) {
    x += 155u;        // equivalent to x -= 101 (mod 256)
    x ^= x >> 4;      // inverse of: x ^= x >> 4
    x *= 13u;         // inverse of *197 mod 256, since 197*13 ≡ 1 (mod 256)
    x ^= x >> 3;      // inverse of: x ^= x >> 3  (part 1)
    x ^= x >> 6;      // inverse of: x ^= x >> 3  (part 2)
    return x;
}

/**
 * @brief 16-bit collision-free "random-looking" permutation.
 *        (bijection over 0..65535)
 * @param x: 16-bit seed
 * @returns: 16-bit pseudorandom value
 */
constexpr uint16_t rand16(uint16_t x) {
    x ^= (uint16_t)(x >> 7);      // invertible (needs 2 steps to undo)
    x *= (uint16_t)43691u;        // 0xAAAB, odd => invertible mod 2^16
    x ^= (uint16_t)(x >> 9);      // invertible (one step to undo for 16-bit)
    x = (uint16_t)(x + 0xBEEFu);  // additive constant
    return x;
}

/**
 * @brief Inverse of rand16
 * @param x: 16-bit pseudorandom value
 * @returns: original 16-bit seed
 */
constexpr uint16_t inv_rand16(uint16_t x) {
    x = (uint16_t)(x - 0xBEEFu);  // undo +0xBEEF
    x ^= (uint16_t)(x >> 9);      // undo x ^= x >> 9 (one step is enough in 16-bit)
    x *= (uint16_t)3u;            // inverse of 43691 mod 65536 is 3
    // undo x ^= x >> 7  (needs cascading shifts: 7, 14)
    x ^= (uint16_t)(x >> 7);
    x ^= (uint16_t)(x >> 14);
    return x;
}

/**
 * @brief 16-bit collision-free "random-looking" permutation combining two 8-bit integers.
 * @param x: first 8-bit seed
 * @param y: second 8-bit seed
 * @returns: 16-bit pseudorandom value
 */
constexpr uint16_t rand8x2(uint8_t x, uint8_t y) {
    auto combined = (uint16_t)x << 8 | (uint16_t)y;
    return rand16(combined);
}

/**
 * @brief Inverse of rand8x2
 * @param r: 16-bit pseudorandom value
 * @param x: first 16-bit seed output
 * @param y: second 16-bit seed output
 */
constexpr void inv_rand8x2(uint16_t r, uint8_t& x, uint8_t& y) {
    auto combined = inv_rand16(r);
    x = static_cast<uint8_t>(combined >> 8);
    y = static_cast<uint8_t>(combined);
}