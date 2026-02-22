/*
 * helper.c
 *
 *  Created on: Feb 21, 2026
 *      Author: gg
 */

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
