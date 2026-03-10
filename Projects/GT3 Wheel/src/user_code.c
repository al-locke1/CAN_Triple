/*
 * user_code.c - CAN Triple
 *
 * Buses:
 *  - CAN1: Wheel @ 1Mbps, termination ON
 *  - CAN2: Car   @ 1Mbps, termination OFF
 *  - CAN3: Blink keypad + DBWC @ 500kbps, termination ON
 *
 * Modes:
 *  - Clutch mode: relay CAN1 <-> CAN2 verbatim
 *  - Throttle mode:
 *      - Relay CAN1 <-> CAN2 verbatim EXCEPT Wheel->Car ID 0xBF:
 *          * Freeze RIGHT paddle (clutch paddle 2) fully toward car:
 *              - Freeze BOTH right paddle sensors:
 *                  R1: Motorola startbit 43 len 10, raw0=224
 *                  R2: Motorola startbit 49 len 10, raw0=759
 *              - Freeze ONLY right-only status bits:
 *                  bits: 40,41,42,43,49,60,61 (Intel bit numbering)
 *                  baseline values: b40=1,b41=1,b42=0,b43=0,b49=1,b60=1,b61=1
 *              - Recompute CRC in Data0 (CRC-8 poly 0x1D init 0x61 over Data1..Data7)
 *          * Compute throttle from LIVE right paddle sensors (same two fields):
 *              - R1 endpoints: 0%=224, 100%=791
 *              - R2 endpoints: 0%=759, 100%=203
 *            Send CAN3 ID 0x6DE Byte0 = 0..100 at 200 Hz.
 *
 *      * Linearise paddle after computing the paddle percent.
 *      * Anchor: 15% paddle -> 50% throttle
 *      * Keep 0->0 and 100->100.
 */

#include "user_code.h"
#include "backend_functions.h"
#include "main.h"
#include "snprintf.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define WHEEL_BUS   CAN_1
#define CAR_BUS     CAN_2
#define IO_BUS      CAN_3

#define ID_WHEEL_BF          0x0BFu
#define ID_BLINK_INPUT       0x195u
#define ID_BLINK_OUTPUT      0x215u
#define ID_THROTTLE_DEMAND   0x6DEu

static const uint8_t BLINK_ILLUM_CLUTCH[8]   = {0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
static const uint8_t BLINK_ILLUM_THROTTLE[8] = {0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00};

/* -------------------- Right paddle sensors  --------------------
 * Two right paddle signals in ID 0xBF are Motorola (big-endian) 10-bit fields:
 * R1: startbit 43 len 10  (0%=224, 100%=791)  increasing
 * R2: startbit 49 len 10  (0%=759, 100%=203)  decreasing
 */
#define R1_MOT_STARTBIT   43u
#define R2_MOT_STARTBIT   49u
#define R_MOT_LEN         10u

#define R1_RAW_0          224u
#define R1_RAW_100        791u
#define R2_RAW_0          759u
#define R2_RAW_100        203u

/* Right-only status bits to freeze (Intel bit numbering: byte*8 + bit) */
#define BIT40_BASE 1u
#define BIT41_BASE 1u
#define BIT42_BASE 0u
#define BIT43_BASE 0u
#define BIT49_BASE 1u
#define BIT60_BASE 1u
#define BIT61_BASE 1u

/* Throttle conditioning */
#define PLAUS_DIFF_PCT        10u   /* max disagreement between sensors before clamp */
#define THROTTLE_DEADBAND     1u   /* percent: clamp to 0 below this */
#define THROTTLE_ALPHA_Q8     128u  /* IIR alpha (Q8): 128/256 = 0.5 */

/* -------------------- State -------------------- */
typedef enum {
    MODE_CLUTCH = 0,
    MODE_THROTTLE = 1
} operation_mode_t;

static volatile operation_mode_t g_mode = MODE_CLUTCH;
static volatile bool g_throttle_mode_active = false;

/* Unfiltered and filtered throttle percent */
static volatile uint8_t g_throttle_pct = 0;
static volatile uint8_t g_throttle_pct_filt = 0;

/* -------------------- CRC helper -------------------- */
/* CRC-8: poly 0x1D, init 0x61, xorout 0x00, no reflection
 * computed over Data1..Data7 and stored in Data0.
 */
static uint8_t bf_crc8_data0(const uint8_t d[8])
{
    uint8_t crc = 0x61;
    for (int i = 1; i <= 7; i++)
    {
        crc ^= d[i];
        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80u) crc = (uint8_t)(((uint8_t)(crc << 1)) ^ 0x1Du);
            else             crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* -------------------- Intel single-bit helpers -------------------- */
static uint8_t get_intel_bit(const uint8_t data[8], uint8_t bitpos)
{
    uint8_t byte = (uint8_t)(bitpos / 8u);
    uint8_t bit  = (uint8_t)(bitpos % 8u);
    return (uint8_t)((data[byte] >> bit) & 0x01u);
}

static void set_intel_bit(uint8_t data[8], uint8_t bitpos, uint8_t val)
{
    uint8_t byte = (uint8_t)(bitpos / 8u);
    uint8_t bit  = (uint8_t)(bitpos % 8u);
    uint8_t mask = (uint8_t)(1u << bit);
    if (val) data[byte] |= mask;
    else     data[byte] &= (uint8_t)~mask;
}

/* -------------------- Motorola bitfield helpers --------------------
 * DBC-style Motorola traversal: startbit points at the MSB of the signal.
 * Next bits proceed toward lower bit index within byte; when bit==0, move to next byte and bit=7.
 */
static uint16_t get_motorola_field_u16(const uint8_t data[8], uint8_t startbit, uint8_t length)
{
    uint8_t byte = (uint8_t)(startbit / 8u);
    uint8_t bit  = (uint8_t)(startbit % 8u);

    uint16_t val = 0;
    uint8_t  cur_byte = byte;
    uint8_t  cur_bit  = bit;

    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t b = (uint8_t)((data[cur_byte] >> cur_bit) & 0x01u);
        val = (uint16_t)((val << 1) | b);

        if (cur_bit == 0u) {
            cur_byte++;
            cur_bit = 7u;
            if (cur_byte >= 8u) break;
        } else {
            cur_bit--;
        }
    }
    return val;
}

static void set_motorola_field_u16(uint8_t data[8], uint8_t startbit, uint8_t length, uint16_t value)
{
    uint8_t byte = (uint8_t)(startbit / 8u);
    uint8_t bit  = (uint8_t)(startbit % 8u);

    uint8_t  cur_byte = byte;
    uint8_t  cur_bit  = bit;

    /* Write MSB-first into the signal positions. */
    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t bit_from_value = (uint8_t)((value >> (length - 1u - i)) & 0x01u);
        uint8_t mask = (uint8_t)(1u << cur_bit);

        if (bit_from_value) data[cur_byte] |= mask;
        else                data[cur_byte] &= (uint8_t)~mask;

        if (cur_bit == 0u) {
            cur_byte++;
            cur_bit = 7u;
            if (cur_byte >= 8u) break;
        } else {
            cur_bit--;
        }
    }
}

/* -------------------- Throttle mapping -------------------- */
static uint8_t map_raw_to_percent(uint16_t raw, uint16_t raw0, uint16_t raw100)
{
    if (raw0 == raw100) return 0;

    int pct;
    if (raw0 < raw100) {
        /* increasing */
        pct = (int)(((int)raw - (int)raw0) * 100) / ((int)raw100 - (int)raw0);
    } else {
        /* decreasing */
        pct = (int)(((int)raw0 - (int)raw) * 100) / ((int)raw0 - (int)raw100);
    }

    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

/* Remap paddle percent -> throttle percent to compensate for non-linear sensor response.
 * Anchor based on ~50% physical travel produces ~15% computed paddle percent.
 * Piecewise linear:
 *   [0..15]   maps to [0..50]
 *   [15..100] maps to [50..100]
 */
static uint8_t remap_paddle_pct_to_throttle(uint8_t p)
{
    if (p == 0u) return 0u;
    if (p >= 100u) return 100u;

    if (p <= 15u) {
        uint16_t y = ((uint16_t)p * 50u + 7u) / 15u;   /* rounding */
        if (y > 50u) y = 50u;
        return (uint8_t)y;
    } else {
        uint16_t y = 50u + (((uint16_t)(p - 15u) * 50u + 42u) / 85u); /* rounding */
        if (y > 100u) y = 100u;
        return (uint8_t)y;
    }
}

static uint8_t compute_right_paddle_pct(const uint8_t d[8])
{
    uint16_t r1 = get_motorola_field_u16(d, (uint8_t)R1_MOT_STARTBIT, (uint8_t)R_MOT_LEN);
    uint16_t r2 = get_motorola_field_u16(d, (uint8_t)R2_MOT_STARTBIT, (uint8_t)R_MOT_LEN);

    uint8_t p1 = map_raw_to_percent(r1, (uint16_t)R1_RAW_0, (uint16_t)R1_RAW_100);
    uint8_t p2 = map_raw_to_percent(r2, (uint16_t)R2_RAW_0, (uint16_t)R2_RAW_100);

    uint8_t diff = (p1 > p2) ? (p1 - p2) : (p2 - p1);

    uint8_t pct;
    if (diff > (uint8_t)PLAUS_DIFF_PCT) {
        /* safer = lower to prevent spikes */
        pct = (p1 < p2) ? p1 : p2;
    } else {
        pct = (uint8_t)(((uint16_t)p1 + (uint16_t)p2 + 1u) / 2u);
    }

    /* Noise gate at rest: clamp only when near zero */
    if (pct <= 1u) pct = 0;
    return pct;
}

static void freeze_right_paddle_to_zero(uint8_t d[8])
{
    /* Freeze both motorola sensor fields to 0% */
    set_motorola_field_u16(d, (uint8_t)R1_MOT_STARTBIT, (uint8_t)R_MOT_LEN, (uint16_t)R1_RAW_0);
    set_motorola_field_u16(d, (uint8_t)R2_MOT_STARTBIT, (uint8_t)R_MOT_LEN, (uint16_t)R2_RAW_0);

    /* Freeze only proven right-only status bits to baseline */
    set_intel_bit(d, 40u, (uint8_t)BIT40_BASE);
    set_intel_bit(d, 41u, (uint8_t)BIT41_BASE);
    set_intel_bit(d, 42u, (uint8_t)BIT42_BASE);
    set_intel_bit(d, 43u, (uint8_t)BIT43_BASE);
    set_intel_bit(d, 49u, (uint8_t)BIT49_BASE);
    set_intel_bit(d, 60u, (uint8_t)BIT60_BASE);
    set_intel_bit(d, 61u, (uint8_t)BIT61_BASE);
}

/* -------------------- CAN helpers -------------------- */
static void send_blink_illum(operation_mode_t mode)
{
    uint8_t d[8];
    if (mode == MODE_CLUTCH) memcpy(d, BLINK_ILLUM_CLUTCH, 8);
    else                     memcpy(d, BLINK_ILLUM_THROTTLE, 8);

    send_message(IO_BUS, false, ID_BLINK_OUTPUT, 8, d);
}

static void send_throttle_percent(uint8_t percent_0_100)
{
    uint8_t d[8] = {0};
    d[0] = percent_0_100;
    send_message(IO_BUS, false, ID_THROTTLE_DEMAND, 8, d);
}

static void bridge_message(uint8_t out_bus, const CAN_Message *in, const uint8_t out_data[8])
{
    send_message(out_bus,
                 in->is_extended_id,
                 in->arbitration_id,
                 in->dlc,
                 (uint8_t*)out_data);
}

/* -------------------- Startup -------------------- */
void events_Startup()
{
    setupCANbus(WHEEL_BUS, 1000000, NORMAL_MODE);
    setCAN_Termination(WHEEL_BUS, true);
    startCANbus(WHEEL_BUS);

    setupCANbus(CAR_BUS, 1000000, NORMAL_MODE);
    setCAN_Termination(CAR_BUS, false);
    startCANbus(CAR_BUS);

    setupCANbus(IO_BUS, 500000, NORMAL_MODE);
    setCAN_Termination(IO_BUS, true);
    startCANbus(IO_BUS);

    g_mode = MODE_CLUTCH;
    g_throttle_mode_active = false;
    g_throttle_pct = 0;
    g_throttle_pct_filt = 0;
    send_blink_illum(g_mode);
}

void onSerialReceive(uint8_t *serialMessage)
{
    (void)serialMessage;
}

/* -------------------- Rx handler -------------------- */
void onReceive(CAN_Message Message)
{
    /* CAN3: Blink keypad */
    if (Message.Bus == IO_BUS)
    {
        if (Message.arbitration_id == ID_BLINK_INPUT && Message.dlc >= 1)
        {
            if (Message.data[0] == 0x01u) {
                g_mode = MODE_CLUTCH;
                g_throttle_mode_active = false;
                g_throttle_pct = 0;
                g_throttle_pct_filt = 0;
                send_blink_illum(g_mode);
            } else if (Message.data[0] == 0x02u) {
                g_mode = MODE_THROTTLE;
                g_throttle_mode_active = true;
                g_throttle_pct = 0;
                g_throttle_pct_filt = 0;
                send_blink_illum(g_mode);
            }
        }
        return;
    }

    /* CAN1/CAN2 bridge */
    if (Message.Bus == WHEEL_BUS || Message.Bus == CAR_BUS)
    {
        uint8_t out_bus = (Message.Bus == WHEEL_BUS) ? CAR_BUS : WHEEL_BUS;

        uint8_t out_data[8];
        memcpy(out_data, Message.data, 8);

        /* Modify only wheel->car 0xBF in throttle mode */
        if (g_mode == MODE_THROTTLE &&
            Message.Bus == WHEEL_BUS &&
            out_bus == CAR_BUS &&
            Message.arbitration_id == ID_WHEEL_BF &&
            Message.dlc == 8)
        {
            /* Compute paddle percent from live right paddle sensors */
            uint8_t paddle_pct = compute_right_paddle_pct(Message.data);

            /* Apply non-linear remap for more linear paddle feel */
            g_throttle_pct = remap_paddle_pct_to_throttle(paddle_pct);

            /* Freeze right paddle fully toward the car */
            freeze_right_paddle_to_zero(out_data);

            /* Fix checksum */
            out_data[0] = bf_crc8_data0(out_data);
        }

        bridge_message(out_bus, &Message, out_data);
        return;
    }
}

/* -------------------- Periodic tasks -------------------- */
void events_2000Hz() {}
void events_1000Hz() {}
void events_500Hz()  {}

/* Send throttle at fixed rate, lightly filtered */
void events_200Hz()
{
    if (!g_throttle_mode_active) return;

    uint8_t x = (uint8_t)g_throttle_pct;
    uint8_t y = (uint8_t)g_throttle_pct_filt;

    int16_t err = (int16_t)x - (int16_t)y;
    int16_t dy  = (int16_t)((err * (int16_t)THROTTLE_ALPHA_Q8) / 256);

    int16_t y2 = (int16_t)y + dy;
    if (y2 < 0) y2 = 0;
    if (y2 > 100) y2 = 100;

    g_throttle_pct_filt = (uint8_t)y2;
    send_throttle_percent((uint8_t)g_throttle_pct_filt);
}

void events_100Hz()  {}
void events_50Hz()   {}
void events_20Hz()   {}
void events_10Hz()   {}

void events_5Hz()
{
    toggleLED(LED_1);
}

void events_2Hz() {}
void events_1Hz() {}
