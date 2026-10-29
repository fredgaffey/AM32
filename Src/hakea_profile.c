/*
 * hakea_profile.c
 *
 * Bench test mode for the Hakea swashplateless rotor. See hakea_profile.h.
 *
 * While the motor runs under flight-controller throttle, a sine is added to
 * the duty cycle after AM32's ramp limiter, so the ramp limit only applies to
 * the base throttle and not to the modulation. The programme below steps
 * through frequencies and amplitudes and restarts every time the FC re-arms.
 */

#include "hakea_profile.h"
#include "targets.h"

#ifdef HAKEA_PROFILE

// never let the modulated duty go below this: below 150 AM32 falls back to its
// start-up ramp rate while running, which collapses large swings
#define PROFILE_DUTY_MIN 180

// one programme step: sine frequency, amplitude (duty units, 2000 = 100%), length
typedef struct {
    uint16_t freq_hz;
    uint16_t amp;
    uint16_t time_ms;
} profile_step_t;

// amp 0 = plain base throttle (settle / rest between tests)
#ifdef SIM_STUDY_PROGRAMME
// STUDY COPY ONLY (not in Fred's firmware): the simulator may replace the
// programme at start-up through hakea_profile_study_set()
#define PROGRAMME_MAX 32
static profile_step_t programme[PROGRAMME_MAX] = {
#else
static const profile_step_t programme[] = {
#endif
    {   0,   0, 3000 },  // settle at base throttle
    { 250,  40, 3000 },  // +/-2%
    {   0,   0, 1000 },
    { 250, 100, 3000 },  // +/-5%
    {   0,   0, 1000 },
    { 400, 100, 3000 },
    {   0,   0, 1000 },
    { 600, 100, 3000 },
    {   0,   0, 1000 },
    { 600, 200, 3000 },  // +/-10%
    {   0,   0, 1000 },
    { 800, 100, 3000 },
    {   0,   0, 1000 },
    {1000, 100, 3000 },
    {   0,   0, 1000 },
};
#ifdef SIM_STUDY_PROGRAMME
static uint8_t programme_steps = 15;   // Fred's v1 programme length
#define PROGRAMME_STEPS programme_steps
void hakea_profile_study_set(const uint16_t* f_amp_ms, int n)
{
    if (n > PROGRAMME_MAX) {
        n = PROGRAMME_MAX;
    }
    for (int i = 0; i < n; i++) {
        programme[i].freq_hz = f_amp_ms[3 * i];
        programme[i].amp = f_amp_ms[3 * i + 1];
        programme[i].time_ms = f_amp_ms[3 * i + 2];
    }
    programme_steps = (uint8_t)n;
}
#else
#define PROGRAMME_STEPS (sizeof(programme) / sizeof(programme[0]))
#endif

// first quarter of a sine wave, 0..32767, 65 points
static const int16_t quarter_sine[65] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
    6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767,
};

static uint8_t step;            // current programme step, PROGRAMME_STEPS = finished
static uint32_t step_ticks;     // ticks spent in this step
static uint32_t step_length;    // ticks this step lasts
static uint32_t phase;          // sine phase, full circle = 2^32
static uint32_t phase_inc;      // phase added per tick
static int16_t modulation;      // current modulation in duty units
#ifdef SIM_STUDY_PROGRAMME
volatile int16_t sim_profile_out;  // STUDY COPY ONLY: exported for the SITL log
#endif

static void start_step(uint8_t s)
{
    step = s;
    step_ticks = 0;
    phase = 0;
    if (s >= PROGRAMME_STEPS) {
        step_length = 0;
        phase_inc = 0;
        return;
    }
    step_length = (uint32_t)programme[s].time_ms * LOOP_FREQUENCY_HZ / 1000;
    phase_inc = (uint32_t)(((uint64_t)programme[s].freq_hz << 32) / LOOP_FREQUENCY_HZ);
}

// sine of the top 8 bits of phase, -32767..32767
static int32_t sine(uint32_t p)
{
    const uint8_t index = p >> 24;
    const uint8_t pos = index & 63;
    switch (index >> 6) {
    case 0: return quarter_sine[pos];
    case 1: return quarter_sine[64 - pos];
    case 2: return -quarter_sine[pos];
    default: return -quarter_sine[64 - pos];
    }
}

void hakea_profile_tick(uint8_t active)
{
    if (!active) {
        // FC disarmed or motor stopped: restart the programme next time
        start_step(0);
        modulation = 0;
        return;
    }
    if (step >= PROGRAMME_STEPS) {
        modulation = 0;              // programme finished: plain base throttle
        return;
    }
    if (++step_ticks >= step_length) {
        start_step(step + 1);
        modulation = 0;
        return;
    }
    phase += phase_inc;
    modulation = (int16_t)(((int32_t)programme[step].amp * sine(phase)) >> 15);
#ifdef SIM_STUDY_PROGRAMME
    sim_profile_out = modulation;
#endif
}

uint16_t hakea_profile_apply(uint16_t base_duty, uint16_t max_duty)
{
    if (modulation == 0) {
        return base_duty;            // not pulsing: leave AM32's duty untouched
    }
    int32_t duty = (int32_t)base_duty + modulation;
    if (max_duty > 2000) {
        max_duty = 2000;
    }
    if (duty > max_duty) {
        duty = max_duty;
    }
    if (duty < PROFILE_DUTY_MIN) {
        duty = PROFILE_DUTY_MIN;
    }
    return (uint16_t)duty;
}

#endif // HAKEA_PROFILE
