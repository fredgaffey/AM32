/*
 * hakea_profile.h
 *
 * Bench test mode for the Hakea swashplateless rotor: adds a sine to the
 * throttle inside the 20 kHz loop, stepping through a fixed programme of
 * frequencies and amplitudes. The flight controller still sets the base
 * throttle and arms/disarms as normal.
 *
 * Built only when HAKEA_PROFILE is defined (target FOXEER_F421_PROFILE).
 */

#ifndef INC_HAKEA_PROFILE_H_
#define INC_HAKEA_PROFILE_H_

#include <stdint.h>

#ifdef HAKEA_PROFILE

// call once per control loop tick; active = motor running under FC throttle
void hakea_profile_tick(uint8_t active);

// base duty (0-2000, after the ramp limiter) in, modulated duty out
uint16_t hakea_profile_apply(uint16_t base_duty, uint16_t max_duty);

#endif

#endif /* INC_HAKEA_PROFILE_H_ */
