/*
 * user_code.c - All User Code should be applied here unless specified otherwise.
 *
 */

/* File Includes */
#include "user_code.h"
#include "backend_functions.h"
#include "main.h"
#include "snprintf.h"
#include <string.h>
#include "stm32g4xx_hal.h"
#include <stdio.h>

/* End File Includes */

/* Variable Declarations */
uint32_t serialnumber;
double testval = 0.0f;

/* ---------------------------------------------------------------------------
 * Config / constants
 * -------------------------------------------------------------------------*/

// IDs
#define ID_STEERING_CLUTCH          0xBF   // steering wheel clutch message (ID 0x0BF)
#define ID_BLINK_KEYPAD_IO          0x195  // Blink keypad I/O
#define ID_BLINK_KEYPAD_LED_STATIC  0x215  // Blink keypad LED static (ON/OFF)
#define ID_BLINK_KEYPAD_LED_BLINK   0x315  // Blink keypad LED blink command
#define ID_BLINK_HEARTBEAT          0x715  // Blink keypad heartbeat
#define ID_THROTTLE_DEMAND          0x6DE  // DBWC throttle demand

// Blink keypad button codes (Data0 bitmask)
#define BTN_ALL_RELEASED        0x00   // all keys unpressed
#define BTN_CLUTCH_MODE         0x01   // key #1
#define BTN_THROTTLE_MODE       0x02   // key #2
#define BTN_3_MASK              0x04   // key #3
#define BTN_4_MASK              0x08   // key #4

// Throttle demand encoding (byte 0 = 0–100 %)
#define THROTTLE_MIN_PCT        0.0f
#define THROTTLE_MAX_PCT        100.0f

/*
 * 0xBF layout from DBC:
 *
 *   B0_Validation        : 7|8@0+   (byte 0)
 *   B1_Counter05to15     : 15|8@0+  (byte 1)
 *   B2_alway00           : 23|8@0+  (byte 2)
 *   b24L1_0or1           : 24|1@0+  (bit 24)
 *   b25_L7_always30      : 31|7@0+  (bits 25..31)
 *   b32L4                : 35|4@0+
 *   b36L2_always_2       : 37|2@0+
 *   b38L2                : 39|2@0+
 *   b44L2_unknown        : 45|2@0+
 *   b46L2_Always_0       : 47|2@0+
 *   b50L10_Clutch_A      : 43|10@0+  (Motorola / big-endian)
 *   b60L10_Clutch_B      : 49|10@0+  (Motorola / big-endian)
 *
 * Clutch_A and Clutch_B are two redundant sensors on the same clutch paddle.
 * We decode both, check them against each other, and derive a single clutch%
 * for throttle mapping. 0% values:
 *   - Clutch_A ≈ 246
 *   - Clutch_B ≈ 744
 * 100% values from log:
 *   - Clutch_A ≈ 753
 *   - Clutch_B ≈ 277
 */

// Clutch_A / Clutch_B DBC start bits and lengths
#define CLUTCH_A_START_BIT        43u
#define CLUTCH_A_LENGTH_BITS      10u
#define CLUTCH_B_START_BIT        49u
#define CLUTCH_B_LENGTH_BITS      10u

// Per-channel 0% / 100% calibration
#define CLUTCH_A_RAW_0PCT         246u
#define CLUTCH_A_RAW_100PCT       753u
#define CLUTCH_B_RAW_0PCT         744u
#define CLUTCH_B_RAW_100PCT       277u

// Clutch plausibility config
#define CLUTCH_PCT_MIN_VALID     -20.0f   // allow some overshoot for noise
#define CLUTCH_PCT_MAX_VALID     120.0f
#define CLUTCH_AB_MAX_DIFF_PCT    10.0f   // max allowed A/B disagreement
#define CLUTCH_MAX_STEP_PCT       25.0f   // max change per 0xBF frame

// "Static" clutch value when in throttle mode, sent to the car on 0xBF B6/B7.
// This is the OEM 0% clutch pattern measured from logs (16-bit composite).
#define CLUTCH_RAW_STATIC_0PCT    0xDAE8u

// Throttle filtering / slew limiting
// 0xBF is ~200 Hz from the log, so use that as the filter rate.
#define BF_MESSAGE_RATE_HZ          200.0f
#define THROTTLE_FILTER_TIME_CONST  0.05f   // 50 ms time constant for throttle filter
#define THROTTLE_MAX_STEP_PER_FRAME 10u     // max +-10% change per 0xBF frame

// Timeouts
#define BF_TIMEOUT_TICKS_100HZ       50     // 50 ticks @100Hz ≈ 0.5s
#define BLINK_HB_TIMEOUT_TICKS_5HZ   10     // 10 ticks @5Hz  ≈ 2.0s

/* ---------------------------------------------------------------------------
 * Mode state / failsafe
 * -------------------------------------------------------------------------*/

typedef enum
{
    MODE_CLUTCH = 0,
    MODE_THROTTLE = 1
} gateway_mode_t;

static gateway_mode_t g_mode        = MODE_CLUTCH;
static bool           g_failsafe    = false;   // when true, always pure pass-through

// Clutch plausibility (on derived percent)
static float   g_last_clutch_pct   = 0.0f;
static bool    g_have_last_clutch  = false;

// Throttle filtering / tracking
static uint8_t g_last_throttle_byte = 0;
static bool    g_have_last_throttle = false;
static float   g_throttle_filtered  = 0.0f;

// 0xBF timeout
static uint16_t g_bf_timeout_counter = 0;
static bool     g_bf_seen_ever       = false;

// Blink keypad heartbeat timeout
static uint16_t g_blink_hb_miss_counter = 0;
static bool     g_blink_hb_seen         = false;

// CAN error monitoring (previous snapshot)
static CAN_ErrorCounts g_prev_err_can1 = {0};
static CAN_ErrorCounts g_prev_err_can2 = {0};
static CAN_ErrorCounts g_prev_err_can3 = {0};
static bool            g_prev_err_valid = false;

// Debug snapshot for 500 ms status print
static uint16_t g_dbg_rawA        = 0;
static uint16_t g_dbg_rawB        = 0;
static float    g_dbg_clutch_pct  = 0.0f;
static uint8_t  g_dbg_thr_cmd     = 0;
static bool     g_dbg_have_bf     = false;

/* ---------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * Extract bits from a CAN payload according to DBC Intel (=little-endian) semantics:
 *  - start_bit: bit index (0 = LSB of byte 0)
 *  - length:    number of bits
 * Bits are returned with bit 0 = value of start_bit, bit 1 = next, etc.
 */
static uint16_t extract_bits_le(const uint8_t *data, uint8_t start_bit, uint8_t length)
{
    uint32_t value = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t bit_index   = (uint8_t)(start_bit + i);
        uint8_t byte_index  = (uint8_t)(bit_index / 8u);
        uint8_t bit_in_byte = (uint8_t)(bit_index % 8u);

        uint8_t bit = (data[byte_index] >> bit_in_byte) & 0x01u;
        value |= ((uint32_t)bit << i);
    }

    return (uint16_t)value;
}

/**
 * Extract bits using DBC Motorola (big-endian) semantics.
 *
 *  - start_bit: DBC start bit index (0..63)
 *  - length:    number of bits
 * Bit numbering is vector style:
 *   byte_index = start_bit / 8, bit_in_byte = start_bit % 8 (0 = LSB, 7 = MSB)
 * We walk backwards through bits within a byte; on a boundary we jump
 * to MSB of the previous byte.
 *
 * Result is assembled MSB-first.
 */
static uint16_t extract_bits_motorola(const uint8_t *data,
                                      uint8_t start_bit,
                                      uint8_t length)
{
    uint8_t  b      = start_bit;
    uint16_t result = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t byte_index  = (uint8_t)(b / 8u);
        uint8_t bit_in_byte = (uint8_t)(b % 8u);  // 0 = LSB, 7 = MSB

        uint8_t bit = (data[byte_index] >> bit_in_byte) & 0x01u;

        // Shift MSB-first into result
        result = (uint16_t)((result << 1) | bit);

        // Next bit index per Motorola rules
        if ((b % 8u) != 0u)
        {
            // Move to next more-significant bit in same byte
            b--;
        }
        else
        {
            // Cross to MSB of previous byte
            b += 15u;  // 8 - 1 + 8
        }
    }

    return result;
}

/**
 * Helper to send throttle demand on CAN3 and keep internal state in sync.
 */
static void send_throttle_command(uint8_t pct)
{
    if (pct > 100u)
        pct = 100u;

    uint8_t thr_msg[8] = {0};
    thr_msg[0] = pct;

    send_message(CAN_3, false, ID_THROTTLE_DEMAND, 8, thr_msg);

    g_last_throttle_byte = pct;
    g_have_last_throttle = true;
    g_dbg_thr_cmd        = pct;
}

/* ---------------------------------------------------------------------------
 * Clutch A/B conversion & plausibility
 * -------------------------------------------------------------------------*/

/**
 * Convert raw Clutch_A reading to percent using calibrated 0% and 100% values.
 */
static float clutchA_raw_to_percent(uint16_t rawA)
{
    float span = (float)(CLUTCH_A_RAW_100PCT - CLUTCH_A_RAW_0PCT);
    if (span <= 0.0f)
        return 0.0f; // shouldn't happen

    float pct = ((float)rawA - (float)CLUTCH_A_RAW_0PCT) * 100.0f / span;
    return pct;
}

/**
 * Convert raw Clutch_B reading to percent using calibrated 0% and 100% values.
 * Note Clutch_B is inverted (0% high, 100% low).
 */
static float clutchB_raw_to_percent(uint16_t rawB)
{
    float span = (float)(CLUTCH_B_RAW_0PCT - CLUTCH_B_RAW_100PCT);
    if (span <= 0.0f)
        return 0.0f; // shouldn't happen

    float pct = ((float)CLUTCH_B_RAW_0PCT - (float)rawB) * 100.0f / span;
    return pct;
}

/**
 * Plausibility of individual clutch% value (range + rate-of-change vs last good).
 */
static bool clutch_pct_plausible(float pct)
{
    // Hard range check
    if (pct < CLUTCH_PCT_MIN_VALID || pct > CLUTCH_PCT_MAX_VALID)
    {
        return false;
    }

    if (g_have_last_clutch)
    {
        float diff = (pct > g_last_clutch_pct)
                         ? (pct - g_last_clutch_pct)
                         : (g_last_clutch_pct - pct);

        if (diff > CLUTCH_MAX_STEP_PCT)
        {
            return false;
        }
    }

    g_last_clutch_pct  = pct;
    g_have_last_clutch = true;
    return true;
}

/**
 * Combine Clutch_A and Clutch_B into a single plausible clutch%.
 *
 *  - Convert each to percent using calibrated 0% and 100% raw values.
 *  - Check each channel individually is in a sane percent range.
 *  - Check their difference is within CLUTCH_AB_MAX_DIFF_PCT.
 *
 * On success:
 *   *pct_out = average of A and B (after clamping to [0..100]),
 *   returns true.
 *
 * On failure:
 *   returns false (caller should trigger failsafe).
 */
static bool clutch_pair_to_percent(uint16_t rawA, uint16_t rawB, float *pct_out)
{
    float pctA = clutchA_raw_to_percent(rawA);
    float pctB = clutchB_raw_to_percent(rawB);

    bool validA = (pctA >= CLUTCH_PCT_MIN_VALID && pctA <= CLUTCH_PCT_MAX_VALID);
    bool validB = (pctB >= CLUTCH_PCT_MIN_VALID && pctB <= CLUTCH_PCT_MAX_VALID);

    if (!validA && !validB)
    {
        // Both channels out of sane range -> fail
        return false;
    }

    if (validA && validB)
    {
        float diff = (pctA > pctB) ? (pctA - pctB) : (pctB - pctA);
        if (diff > CLUTCH_AB_MAX_DIFF_PCT)
        {
            // Channels disagree too much -> fail
            return false;
        }

        float avg = 0.5f * (pctA + pctB);
        *pct_out = clampf(avg, THROTTLE_MIN_PCT, THROTTLE_MAX_PCT);
        return true;
    }

    // One channel is clearly bad, the other is usable. Trust the usable one.
    if (validA)
    {
        *pct_out = clampf(pctA, THROTTLE_MIN_PCT, THROTTLE_MAX_PCT);
        return true;
    }

    if (validB)
    {
        *pct_out = clampf(pctB, THROTTLE_MIN_PCT, THROTTLE_MAX_PCT);
        return true;
    }

    return false; // should not reach here
}

/* ---------------------------------------------------------------------------
 * Checksum for 0xBF Data0 (B0_Validation)
 * -------------------------------------------------------------------------*/

static uint8_t bf_compute_data0(uint8_t dlc,
                                uint8_t d1, uint8_t d2,
                                uint8_t d3, uint8_t d4,
                                uint8_t d5, uint8_t d6, uint8_t d7)
{
    #define B(v, bit) (((v) >> (bit)) & 1u)

    // Output bit 0
    uint8_t b0 = (uint8_t)(
        B(d1, 0) ^ B(d1, 2) ^
        B(d4, 7) ^
        B(d5, 0) ^ B(d5, 1) ^
        B(d6, 2) ^ B(d6, 5) ^ B(d6, 7) ^
        B(d7, 0) ^ B(d7, 4) ^ B(d7, 5) ^ B(d7, 6)
    );

    // Output bit 1
    uint8_t b1 = (uint8_t)(
        B(dlc, 3) ^
        B(d1, 1) ^ B(d1, 3) ^
        B(d3, 0) ^
        B(d5, 0) ^ B(d5, 1) ^ B(d5, 2) ^
        B(d6, 3) ^ B(d6, 6) ^
        B(d7, 1) ^ B(d7, 5) ^ B(d7, 6) ^ B(d7, 7)
    );

    // Output bit 2
    uint8_t b2 = (uint8_t)(
        B(dlc, 3) ^
        B(d1, 0) ^
        B(d4, 6) ^ B(d4, 7) ^
        B(d5, 0) ^ B(d5, 2) ^ B(d5, 3) ^
        B(d6, 0) ^ B(d6, 2) ^ B(d6, 4) ^ B(d6, 5) ^
        B(d7, 0) ^ B(d7, 2) ^ B(d7, 4) ^ B(d7, 5) ^ B(d7, 7)
    );

    // Output bit 3
    uint8_t b3 = (uint8_t)(
        B(d1, 0) ^ B(d1, 1) ^ B(d1, 2) ^
        B(d3, 0) ^
        B(d5, 0) ^ B(d5, 3) ^ B(d5, 4) ^
        B(d6, 2) ^ B(d6, 3) ^ B(d6, 6) ^ B(d6, 7) ^
        B(d7, 0) ^ B(d7, 1) ^ B(d7, 3) ^ B(d7, 4)
    );

    // Output bit 4
    uint8_t b4 = (uint8_t)(
        B(d1, 0) ^ B(d1, 1) ^ B(d1, 3) ^
        B(d4, 6) ^ B(d4, 7) ^
        B(d5, 4) ^ B(d5, 5) ^
        B(d6, 0) ^ B(d6, 3) ^ B(d6, 4) ^ B(d6, 5) ^
        B(d7, 0) ^ B(d7, 1) ^ B(d7, 2) ^ B(d7, 6)
    );

    // Output bit 5
    uint8_t b5 = (uint8_t)(
        B(dlc, 3) ^
        B(d1, 1) ^ B(d1, 2) ^
        B(d3, 0) ^
        B(d4, 7) ^
        B(d5, 5) ^
        B(d6, 2) ^ B(d6, 4) ^ B(d6, 5) ^ B(d6, 6) ^
        B(d7, 1) ^ B(d7, 2) ^ B(d7, 3) ^ B(d7, 7)
    );

    // Output bit 6
    uint8_t b6 = (uint8_t)(
        B(d1, 0) ^ B(d1, 2) ^ B(d1, 3) ^
        B(d3, 0) ^
        B(d6, 0) ^ B(d6, 3) ^ B(d6, 5) ^ B(d6, 6) ^ B(d6, 7) ^
        B(d7, 2) ^ B(d7, 3) ^ B(d7, 4)
    );

    // Output bit 7
    uint8_t b7 = (uint8_t)(
        B(d1, 1) ^ B(d1, 3) ^
        B(d4, 6) ^
        B(d5, 0) ^
        B(d6, 0) ^ B(d6, 4) ^ B(d6, 6) ^ B(d6, 7) ^
        B(d7, 3) ^ B(d7, 4) ^ B(d7, 5)
    );

    #undef B

    uint8_t data0 =
        (uint8_t)((b0      ) |
                  (b1 << 1) |
                  (b2 << 2) |
                  (b3 << 3) |
                  (b4 << 4) |
                  (b5 << 5) |
                  (b6 << 6) |
                  (b7 << 7));

    return data0;
}

/* ---------------------------------------------------------------------------
 * Failsafe handling
 * -------------------------------------------------------------------------*/

static void send_keypad_illumination(gateway_mode_t mode); // forward declaration

static void enter_failsafe(const char *reason)
{
    if (!g_failsafe)
    {
        g_failsafe = true;

        // In failsafe we behave like clutch/pass-through mode,
        // but log that we’ve latched into it.
        g_mode = MODE_CLUTCH;
        writeLED(LED_1, false);  // LED off to indicate clutch mode

        // Immediately force 0% throttle in failsafe
        send_throttle_command(0);
        g_throttle_filtered  = 0.0f;

        // Reset dynamic state so a later recovery is clean
        g_have_last_clutch   = false;
        g_have_last_throttle = true;  // we have a known 0% now
        g_bf_seen_ever       = false;
        g_bf_timeout_counter = 0;
        // keep blink HB flags as-is (we still want to see if it comes back)

        printf("[FAILSAFE] Entering failsafe pass-through mode: %s\r\n", reason);

        // Immediately update keypad to failsafe indication
        send_keypad_illumination(g_mode);
    }
}

/* ---------------------------------------------------------------------------
 * Mode / keypad LED handling
 * -------------------------------------------------------------------------*/

static void send_keypad_illumination(gateway_mode_t mode)
{
    uint8_t data[8] = {0};

    // Static to detect changes and avoid spam
    static bool           s_led_state_valid   = false;
    static bool           s_led_last_failsafe = false;
    static gateway_mode_t s_led_last_mode     = MODE_CLUTCH;

    bool changed = false;

    if (g_failsafe)
    {
        // FAILSAFE:
        //  - Buttons 1 & 2: blinking red (using 0x315 LED Blink command)
        //  - Buttons 3 & 4: off
        //
        // We assume the Blink command uses the same color bit layout as the
        // static command (section 13 PKP-2200-SI manual):
        //   Byte0: 0 0 R4 R3 R2 R1
        //   Byte1: 0 0 G4 G3 G2 G1
        //   Byte2: 0 0 B4 B3 B2 B1
        //
        // So to blink red on 1 & 2 only: Byte0 = 0b00000011 = 0x03.

        if (!s_led_state_valid || !s_led_last_failsafe)
        {
            changed = true;
        }

        // 1) Ensure static LEDs are all off
        memset(data, 0, sizeof(data));
        send_message(CAN_3, false, ID_BLINK_KEYPAD_LED_STATIC, 8, data);

        // 2) Command blink: red on buttons 1 & 2 only
        memset(data, 0, sizeof(data));
        data[0] = 0x03;  // R1 + R2 blink, others off

        send_message(CAN_3, false, ID_BLINK_KEYPAD_LED_BLINK, 8, data);

        if (changed)
        {
            printf("[KEYPAD-LED] FAILSAFE blinking red on buttons 1&2, blink_data=[%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
                   data[0], data[1], data[2], data[3],
                   data[4], data[5], data[6], data[7]);
        }

        s_led_state_valid   = true;
        s_led_last_failsafe = true;
        s_led_last_mode     = mode;
        return;
    }

    // Normal mode LED strategy (Blink static LED command on 0x215):
    //
    // Byte 0: 0 0 R4 R3 R2 R1  (Red)
    // Byte 1: 0 0 G4 G3 G2 G1  (Green)
    // Byte 2: 0 0 B4 B3 B2 B1  (Blue)
    //
    //  - Buttons 3 & 4: no LEDs.
    //  - Button 2: yellow (R2+G2) in THROTTLE mode.
    //  - Button 1: blue in CLUTCH mode.

    // If we are *leaving* failsafe, clear any blink settings by
    // sending a blink frame with all bits zero once.
    if (s_led_state_valid && s_led_last_failsafe)
    {
        uint8_t blink_off[8] = {0};
        send_message(CAN_3, false, ID_BLINK_KEYPAD_LED_BLINK, 8, blink_off);
    }

    if (mode == MODE_CLUTCH)
    {
        // CLUTCH mode:
        //  - Button 1: blue
        //  - Button 2: off
        //  - Buttons 3 & 4: off
        //
        // data[2] = 0x01 => B1 on, others off.
        data[0] = 0x00;   // no red
        data[1] = 0x00;   // no green
        data[2] = 0x01;   // blue on button 1
    }
    else
    {
        // THROTTLE mode:
        //  - Button 2: yellow (Red2 + Green2)
        //  - Buttons 1,3,4: off
        //
        // R2 bit = 0x02 in byte0, G2 bit = 0x02 in byte1.
        data[0] = 0x02;   // R2 on
        data[1] = 0x02;   // G2 on
        data[2] = 0x00;   // no blue
    }

    if (!s_led_state_valid || s_led_last_failsafe || s_led_last_mode != mode)
    {
        changed = true;
    }

    send_message(CAN_3, false, ID_BLINK_KEYPAD_LED_STATIC, 8, data);

    if (changed)
    {
        printf("[KEYPAD-LED] mode=%s, static_data=[%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
               (mode == MODE_THROTTLE) ? "THROTTLE" : "CLUTCH",
               data[0], data[1], data[2], data[3],
               data[4], data[5], data[6], data[7]);
    }

    s_led_state_valid   = true;
    s_led_last_failsafe = false;
    s_led_last_mode     = mode;
}

static void gateway_set_mode(gateway_mode_t new_mode)
{
    // If we were in failsafe and driver explicitly requests throttle mode,
    // clear the failsafe latch and allow normal operation again.
    if (g_failsafe && new_mode == MODE_THROTTLE)
    {
        printf("[MODE] Throttle mode requested – clearing failsafe latch\r\n");
        g_failsafe           = false;
        g_have_last_throttle = false;
        g_have_last_clutch   = false;
        g_bf_seen_ever       = false;
        g_bf_timeout_counter = 0;
    }

    g_mode = new_mode;

    // Onboard GPIO LED: ON in throttle mode, OFF otherwise
    writeLED(LED_1, (g_mode == MODE_THROTTLE) && !g_failsafe);

    // If we are in clutch (or still in failsafe), immediately force 0% throttle.
    if (g_mode == MODE_CLUTCH || g_failsafe)
    {
        send_throttle_command(0);
        g_throttle_filtered  = 0.0f;
        g_have_last_throttle = true;
    }

    send_keypad_illumination(g_mode);

    if (g_mode == MODE_THROTTLE)
    {
        printf("[MODE] Switched to THROTTLE mode\r\n");
    }
    else
    {
        if (g_failsafe)
            printf("[MODE] In CLUTCH/Failsafe pass-through mode\r\n");
        else
            printf("[MODE] Switched to CLUTCH mode\r\n");
    }
}

/* ---------------------------------------------------------------------------
 * Startup Functions
 * -------------------------------------------------------------------------*/

void events_Startup()
{
    // Bitrates:
    //  - CAN1: 1 Mbps (steering wheel)
    //  - CAN2: 1 Mbps (car)
    //  - CAN3: 500 kbps (Blink keypad + DBWC)
    setupCANbus(CAN_1, 1000000, NORMAL_MODE);
    setupCANbus(CAN_2, 1000000, NORMAL_MODE);
    setupCANbus(CAN_3, 500000,  NORMAL_MODE);

    // Termination: CAN1 & CAN3 on, CAN2 off
    setCAN_Termination(CAN_1, true);
    setCAN_Termination(CAN_2, false);
    setCAN_Termination(CAN_3, true);

    startCANbus(CAN_1);
    startCANbus(CAN_2);
    startCANbus(CAN_3);

    // Safe default: start in clutch/pass-through mode.
    g_failsafe = false;
    g_prev_err_valid = false;
    gateway_set_mode(MODE_CLUTCH);

    printf("\r\n[CAN_TRIPLE] GT3 steering/throttle gateway starting...\r\n");
    printf("[CAN_TRIPLE] CAN1=1Mbps (term ON), CAN2=1Mbps (term OFF), CAN3=500kbps (term ON)\r\n");

    // Dump serialnumber
    printf("[INFO] serialnumber = %lu\r\n", (unsigned long)serialnumber);

    // Initial CAN error counters
    CAN_ErrorCounts e1 = getCANErrorCounts(CAN_1);
    CAN_ErrorCounts e2 = getCANErrorCounts(CAN_2);
    CAN_ErrorCounts e3 = getCANErrorCounts(CAN_3);

    printf("[CAN ERR] Startup: CAN1 Tx=%u Rx=%u Reset=%u | "
           "CAN2 Tx=%u Rx=%u Reset=%u | "
           "CAN3 Tx=%u Rx=%u Reset=%u\r\n",
           e1.TxErrorCounter, e1.RxErrorCounter, e1.BusResetCounter,
           e2.TxErrorCounter, e2.RxErrorCounter, e2.BusResetCounter,
           e3.TxErrorCounter, e3.RxErrorCounter, e3.BusResetCounter);

    // Prime previous snapshot
    g_prev_err_can1 = e1;
    g_prev_err_can2 = e2;
    g_prev_err_can3 = e3;
    g_prev_err_valid = true;
}

/* End Startup Functions */

void onSerialReceive(uint8_t *serialMessage)
{
    // UART debug hook
    // printf("%07.4f message received...\r\n",getTimestamp());
}

/* ---------------------------------------------------------------------------
 * Steering wheel 0xBF handler (ID_0x0BF from DBC)
 * -------------------------------------------------------------------------*/

static void handle_steering_0xBF_from_CAN1(CAN_Message *msg)
{
    uint8_t tx_data[8];
    memcpy(tx_data, msg->data, 8);

    // 0xBF timeout: every time we see one, reset the timer
    g_bf_timeout_counter = 0;
    g_bf_seen_ever       = true;

    // Basic sanity check: 0xBF should always be 8 bytes.
    if (msg->dlc != 8u)
    {
        enter_failsafe("0xBF DLC != 8");
    }

    // Decode Clutch_A and Clutch_B using Motorola semantics (DBC @0+ big-endian)
    uint16_t rawA = extract_bits_motorola(msg->data, CLUTCH_A_START_BIT, CLUTCH_A_LENGTH_BITS);
    uint16_t rawB = extract_bits_motorola(msg->data, CLUTCH_B_START_BIT, CLUTCH_B_LENGTH_BITS);

    // Derive clutch% from redundant sensors, with mutual plausibility
    float clutch_pct = 0.0f;

    if (!clutch_pair_to_percent(rawA, rawB, &clutch_pct))
    {
        printf("[CLU PLAUS] Implausible A/B: rawA=%u rawB=%u\r\n", rawA, rawB);
        enter_failsafe("0xBF clutch A/B redundant sensors implausible");
    }

    if (!clutch_pct_plausible(clutch_pct))
    {
        enter_failsafe("0xBF clutch rate/range implausible");
    }

    // Update debug snapshot for 500ms status
    g_dbg_rawA       = rawA;
    g_dbg_rawB       = rawB;
    g_dbg_clutch_pct = clutch_pct;
    g_dbg_have_bf    = true;

    // If failsafe is active, always behave as simple pass-through
    if (g_failsafe)
    {
        send_message(CAN_2,
                     msg->is_extended_id,
                     msg->arbitration_id,
                     msg->dlc,
                     msg->data);
        return;
    }

    if (g_mode == MODE_THROTTLE)
    {
        // Map clutch% -> throttle%, then filter + slew limit
        float throttle_pct = clampf(clutch_pct, THROTTLE_MIN_PCT, THROTTLE_MAX_PCT);

        // Filter using backend 1st-order LPF at ~0xBF rate
        if (!g_have_last_throttle)
        {
            g_throttle_filtered  = throttle_pct;
            g_have_last_throttle = true;
        }
        else
        {
            g_throttle_filtered = lowpass_filter_by_frequency(
                                      g_throttle_filtered,
                                      throttle_pct,
                                      THROTTLE_FILTER_TIME_CONST,
                                      BF_MESSAGE_RATE_HZ);
        }

        // Convert to integer, clamp
        uint8_t target_thr = (uint8_t)(g_throttle_filtered + 0.5f);
        if (target_thr > 100u)
            target_thr = 100u;

        // Slew limit
        uint8_t prev_thr = g_last_throttle_byte;
        int16_t delta    = (int16_t)target_thr - (int16_t)prev_thr;

        if (g_have_last_throttle)
        {
            if (delta > (int16_t)THROTTLE_MAX_STEP_PER_FRAME)
                target_thr = (uint8_t)(prev_thr + THROTTLE_MAX_STEP_PER_FRAME);
            else if (delta < -(int16_t)THROTTLE_MAX_STEP_PER_FRAME)
                target_thr = (uint8_t)(prev_thr - THROTTLE_MAX_STEP_PER_FRAME);
        }

        // Send throttle demand on CAN3, keep state in sync
        send_throttle_command(target_thr);

        // Freeze clutch bytes towards the car using the known 0% pattern
        tx_data[6] = (uint8_t)(CLUTCH_RAW_STATIC_0PCT >> 8);
        tx_data[7] = (uint8_t)(CLUTCH_RAW_STATIC_0PCT & 0xFF);

        // Recalculate validation byte (B0_Validation) based on modified bytes 1..7
        uint8_t new_validation = bf_compute_data0(msg->dlc,
                                                  tx_data[1], tx_data[2],
                                                  tx_data[3], tx_data[4],
                                                  tx_data[5], tx_data[6],
                                                  tx_data[7]);
        tx_data[0] = new_validation;
    }
    else
    {
        // In clutch mode, we do NOT send throttle here.
        // Periodic code (events_20Hz) will keep asserting 0% on 0x6DE.
        g_dbg_thr_cmd = 0;
        // 0xBF passed through unmodified in CLUTCH mode
    }

    // Forward to car on CAN2 (possibly modified depending on mode)
    send_message(CAN_2,
                 msg->is_extended_id,
                 msg->arbitration_id,
                 msg->dlc,
                 tx_data);
}

/* ---------------------------------------------------------------------------
 * CAN receive handler
 * -------------------------------------------------------------------------*/

void onReceive(CAN_Message Message)
{
    // CAN1: steering wheel to car
    if (Message.Bus == CAN_1)
    {
        if (!Message.is_extended_id && Message.arbitration_id == ID_STEERING_CLUTCH)
        {
            handle_steering_0xBF_from_CAN1(&Message);
        }
        else
        {
            // Straight-through for all other IDs, no per-frame logging
            send_message(CAN_2,
                         Message.is_extended_id,
                         Message.arbitration_id,
                         Message.dlc,
                         Message.data);
        }
        return;
    }

    // CAN2: car to steering wheel (always pure mirror, no spam logging)
    if (Message.Bus == CAN_2)
    {
        send_message(CAN_1,
                     Message.is_extended_id,
                     Message.arbitration_id,
                     Message.dlc,
                     Message.data);
        return;
    }

    // CAN3: Blink keypad + DBWC throttle controller
    if (Message.Bus == CAN_3)
    {
        // Heartbeat from keypad
        if (!Message.is_extended_id &&
            Message.arbitration_id == ID_BLINK_HEARTBEAT)
        {
            g_blink_hb_miss_counter = 0;
            g_blink_hb_seen         = true;
            // printf("[KEYPAD] Heartbeat 0x715 received\r\n");
        }

        // Button I/O
        if (!Message.is_extended_id &&
            Message.arbitration_id == ID_BLINK_KEYPAD_IO &&
            Message.dlc >= 1)
        {
            uint8_t button = Message.data[0];

            // Log raw packet once
            printf("[KEYPAD] 0x195 data0=0x%02X raw=[%02X %02X %02X %02X %02X]\r\n",
                   button,
                   Message.data[0], Message.data[1], Message.data[2],
                   Message.data[3], Message.data[4]);

            bool handled = false;

            if (button == BTN_ALL_RELEASED)
            {
                // Valid "all buttons released" frame – do not treat as unknown.
                printf("[KEYPAD] All buttons released (0x00 frame)\r\n");
                handled = true;
            }

            if (button & BTN_CLUTCH_MODE)
            {
                printf("[KEYPAD] Button 1 (clutch) pressed\r\n");
                gateway_set_mode(MODE_CLUTCH);
                handled = true;
            }

            if (button & BTN_THROTTLE_MODE)
            {
                printf("[KEYPAD] Button 2 (throttle) pressed\r\n");
                gateway_set_mode(MODE_THROTTLE);
                handled = true;
            }

            if (button & BTN_3_MASK)
            {
                // Button 3 in use for diagnostics only – no action
                printf("[KEYPAD] Button 3 pressed (no action)\r\n");
                handled = true;
            }

            if (button & BTN_4_MASK)
            {
                // Button 4 in use for diagnostics only – no action
                printf("[KEYPAD] Button 4 pressed (no action)\r\n");
                handled = true;
            }

            if (!handled)
            {
                printf("[KEYPAD] Unknown button code 0x%02X on 0x195\r\n", button);
            }
        }

        return;
    }
}

/* ---------------------------------------------------------------------------
 * Periodic event hooks
 * -------------------------------------------------------------------------*/

/* Run 2000Hz Functions here */
void events_2000Hz()
{
}

/* Run 1000Hz Functions here */
void events_1000Hz()
{
}

/* Run 500Hz Functions here */
void events_500Hz()
{
}

/* Run 200Hz Functions here */
void events_200Hz()
{
}

/* Run 100Hz Functions here */
void events_100Hz()
{
    // 0xBF timeout: if we stop seeing the clutch message for 0.5s,
    // force 0% throttle and failsafe
    if (g_bf_seen_ever && !g_failsafe)
    {
        if (g_bf_timeout_counter < 0xFFFFu)
            g_bf_timeout_counter++;

        if (g_bf_timeout_counter == BF_TIMEOUT_TICKS_100HZ)
        {
            printf("[TIMEOUT] 0xBF clutch message timeout, commanding 0%% and entering failsafe\r\n");

            // 1) Command 0% throttle once
            send_throttle_command(0);

            // 2) Enter failsafe
            enter_failsafe("0xBF timeout (no steering wheel clutch msg)");
        }
    }
}

/* Run 50Hz Functions here */
void events_50Hz()
{
}

/* Run 20Hz Functions here */
void events_20Hz()
{
    // 20 Hz (every 50ms): in clutch or failsafe, keep reasserting 0% throttle
    if (g_mode == MODE_CLUTCH || g_failsafe)
    {
        send_throttle_command(0);
        g_throttle_filtered  = 0.0f;
        g_have_last_throttle = true;
    }
}

/* Run 10Hz Functions here */
void events_10Hz()
{
}

/* Run 5Hz Functions here */
void events_5Hz()
{
    // 5 Hz = every 200ms. Periodically refresh keypad LEDs 
    send_keypad_illumination(g_mode);

    // Heartbeat timeout handling for Blink keypad
    if (g_blink_hb_seen && !g_failsafe)
    {
        if (g_blink_hb_miss_counter < 0xFFFFu)
            g_blink_hb_miss_counter++;

        if (g_blink_hb_miss_counter == BLINK_HB_TIMEOUT_TICKS_5HZ)
        {
            printf("[TIMEOUT] Blink keypad heartbeat timeout (0x715), entering failsafe\r\n");
            enter_failsafe("Blink keypad heartbeat timeout (0x715)");
        }
    }
}

/* Run 2Hz Functions here */
void events_2Hz()
{
    // 2 Hz = every 500ms: compact status line instead of per-frame spam
    if (g_dbg_have_bf)
    {
        printf("[BF STAT] mode=%s failsafe=%u rawA=%u rawB=%u clutch_pct=%.1f thr_cmd=%u\r\n",
               (g_mode == MODE_THROTTLE) ? "THR" : "CLU",
               (unsigned)g_failsafe,
               g_dbg_rawA, g_dbg_rawB,
               (double)g_dbg_clutch_pct,
               (unsigned)g_dbg_thr_cmd);
    }
}

/* Run 1Hz Functions here */
void events_1Hz()
{
    // Read current error counts for each CAN bus
    CAN_ErrorCounts err1 = getCANErrorCounts(CAN_1);
    CAN_ErrorCounts err2 = getCANErrorCounts(CAN_2);
    CAN_ErrorCounts err3 = getCANErrorCounts(CAN_3);

    if (g_prev_err_valid)
    {
        bool change1 =
            (err1.TxErrorCounter != g_prev_err_can1.TxErrorCounter) ||
            (err1.RxErrorCounter != g_prev_err_can1.RxErrorCounter) ||
            (err1.BusResetCounter != g_prev_err_can1.BusResetCounter);

        bool change2 =
            (err2.TxErrorCounter != g_prev_err_can2.TxErrorCounter) ||
            (err2.RxErrorCounter != g_prev_err_can2.RxErrorCounter) ||
            (err2.BusResetCounter != g_prev_err_can2.BusResetCounter);

        bool change3 =
            (err3.TxErrorCounter != g_prev_err_can3.TxErrorCounter) ||
            (err3.RxErrorCounter != g_prev_err_can3.RxErrorCounter) ||
            (err3.BusResetCounter != g_prev_err_can3.BusResetCounter);

        if ((change1 || change2 || change3) && !g_failsafe)
        {
            printf("[CAN ERR] CAN1: Tx=%u Rx=%u Reset=%u | "
                   "CAN2: Tx=%u Rx=%u Reset=%u | "
                   "CAN3: Tx=%u Rx=%u Reset=%u\r\n",
                   err1.TxErrorCounter, err1.RxErrorCounter, err1.BusResetCounter,
                   err2.TxErrorCounter, err2.RxErrorCounter, err2.BusResetCounter,
                   err3.TxErrorCounter, err3.RxErrorCounter, err3.BusResetCounter);
        }

        // Existing reset detection -> failsafe
        if (!g_failsafe)
        {
            bool reset1 = (err1.BusResetCounter > g_prev_err_can1.BusResetCounter);
            bool reset2 = (err2.BusResetCounter > g_prev_err_can2.BusResetCounter);
            bool reset3 = (err3.BusResetCounter > g_prev_err_can3.BusResetCounter);

            if (reset1 || reset2 || reset3)
            {
                printf("[CAN ERR] Detected CAN bus reset: "
                       "CAN1 ΔRst=%d CAN2 ΔRst=%d CAN3 ΔRst=%d – entering failsafe\r\n",
                       (int)(err1.BusResetCounter - g_prev_err_can1.BusResetCounter),
                       (int)(err2.BusResetCounter - g_prev_err_can2.BusResetCounter),
                       (int)(err3.BusResetCounter - g_prev_err_can3.BusResetCounter));

                enter_failsafe("CAN bus reset detected (possible bus-off)");
            }
        }
    }

    // Store for next comparison
    g_prev_err_can1 = err1;
    g_prev_err_can2 = err2;
    g_prev_err_can3 = err3;
    g_prev_err_valid = true;
}
