/*
 * ESL IR waveform transmitter — Bruce/ESP32-S3 specific implementation.
 * See esl_ir.h for why this is a full rewrite rather than a port, and
 * THIRD_PARTY.md for the TagTinker attribution this timing is derived from.
 *
 * ---------------------------------------------------------------------
 * Timing derivation (from TagTinker's ir/tagtinker_ir.c, STM32WB55 TIM1):
 *
 *   Carrier: TIM1 ARR=50 (CARRIER_ARR = 51-1), CCR=25, PWM2 mode, driven
 *   from the 64MHz APB2 timer clock (Flipper Zero's standard system
 *   clock). Period = (ARR+1) = 51 timer counts.
 *     carrier_freq = 64,000,000 / 51 = 1,254,901.9607... Hz
 *     duty         = CCR / (ARR+1) = 25/51 ~= 49.0%  (we use 50%, see below)
 *
 *   PP4 symbol timing, in 64MHz cycles (from PP4_BURST_CYCLES and
 *   pp4_gap_cycles[] in tagtinker_ir.c), converted to microseconds:
 *     burst        = 2581  cycles / 64e6 = 40.328125   us
 *     gap symbol 0 = 3871  cycles / 64e6 = 60.484375   us
 *     gap symbol 1 = 11612 cycles / 64e6 = 181.4375    us
 *     gap symbol 2 = 7741  cycles / 64e6 = 120.953125  us
 *     gap symbol 3 = 15483 cycles / 64e6 = 241.921875  us
 *
 * These microsecond values are converted below to RMT ticks at a 10MHz
 * (100ns) tick resolution, chosen so the worst-case rounding error is
 * <0.1% of any symbol's duration (see ESL_IR_RESOLUTION_HZ). The 49%
 * carrier duty from CCR/ARR is approximated as 50% (RMT's carrier duty
 * is a float 0..1); this is a documented approximation, everything else
 * above is exact register-derived math, not a guess.
 * ---------------------------------------------------------------------
 */

#include "esl_ir.h"

#include "../ir/ir_utils.h"
#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <globals.h>
#include <vector>

// RMT tick resolution: 10MHz -> 1 tick = 100ns.
#define ESL_IR_RESOLUTION_HZ 10000000

// Carrier: 64,000,000 / 51, rounded to the nearest Hz.
#define ESL_IR_CARRIER_HZ 1254902
#define ESL_IR_CARRIER_DUTY 0.5f

// Burst/gap durations in 100ns ticks (see derivation above).
#define ESL_PP4_BURST_TICKS 403 // 40.3us  (exact: 40.328125us)
static const uint16_t ESL_PP4_GAP_TICKS[4] = {
    605,  // symbol 0: 60.5us  (exact: 60.484375us)
    1814, // symbol 1: 181.4us (exact: 181.4375us)
    1210, // symbol 2: 121.0us (exact: 120.953125us)
    2419, // symbol 3: 241.9us (exact: 241.921875us)
};

// RMT duration fields are 15-bit (max 32767 ticks); every value above is
// far below that, so no splitting is needed here (unlike rf_encoder.cpp,
// which handles multi-millisecond RF pulses).

static rmt_channel_handle_t esl_ir_channel = nullptr;
static rmt_encoder_handle_t esl_ir_encoder = nullptr;
static bool esl_ir_ready = false;
static volatile bool esl_ir_busy = false;
static volatile bool esl_ir_stop_requested = false;

bool esl_ir_init(void) {
    if (esl_ir_ready) return true;

    setup_ir_pin(bruceConfigPins.irTx, OUTPUT);

    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num = (gpio_num_t)bruceConfigPins.irTx;
    tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz = ESL_IR_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols = 64;
    tx_cfg.trans_queue_depth = 4;
    tx_cfg.flags.invert_out = false;
    tx_cfg.flags.with_dma = false;

    if (rmt_new_tx_channel(&tx_cfg, &esl_ir_channel) != ESP_OK) {
        esl_ir_channel = nullptr;
        return false;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    if (rmt_new_copy_encoder(&copy_cfg, &esl_ir_encoder) != ESP_OK) {
        rmt_del_channel(esl_ir_channel);
        esl_ir_channel = nullptr;
        esl_ir_encoder = nullptr;
        return false;
    }

    rmt_carrier_config_t carrier_cfg = {};
    carrier_cfg.frequency_hz = ESL_IR_CARRIER_HZ;
    carrier_cfg.duty_cycle = ESL_IR_CARRIER_DUTY;
    carrier_cfg.flags.polarity_active_low = false;
    if (rmt_apply_carrier(esl_ir_channel, &carrier_cfg) != ESP_OK) {
        rmt_del_encoder(esl_ir_encoder);
        rmt_del_channel(esl_ir_channel);
        esl_ir_channel = nullptr;
        esl_ir_encoder = nullptr;
        return false;
    }

    if (rmt_enable(esl_ir_channel) != ESP_OK) {
        rmt_del_encoder(esl_ir_encoder);
        rmt_del_channel(esl_ir_channel);
        esl_ir_channel = nullptr;
        esl_ir_encoder = nullptr;
        return false;
    }

    esl_ir_stop_requested = false;
    esl_ir_ready = true;
    return true;
}

void esl_ir_deinit(void) {
    if (!esl_ir_ready) return;
    esl_ir_stop();
    rmt_disable(esl_ir_channel);
    rmt_del_encoder(esl_ir_encoder);
    rmt_del_channel(esl_ir_channel);
    esl_ir_channel = nullptr;
    esl_ir_encoder = nullptr;
    esl_ir_ready = false;
}

// Encodes one PP4 frame into RMT symbols: 4 x 2-bit symbols per byte
// (LSB-of-remaining-byte first, matching TagTinker's `current_byte & 0x03`
// then `>>= 2`), each symbol = carrier burst + gap[symbol]; a final
// closing burst with no following gap ends the frame (matches the
// original's trailing carrier_on()/delay/carrier_off() with no extra
// wait).
static void esl_pp4_build_symbols(const uint8_t* data, size_t len, std::vector<rmt_symbol_word_t>& syms) {
    syms.clear();
    syms.reserve(len * 4 + 1);
    bool pendingLow = false;

    auto push_half = [&](uint8_t level, uint16_t ticks) {
        if (!pendingLow) {
            rmt_symbol_word_t s = {};
            s.level0 = level;
            s.duration0 = ticks;
            syms.push_back(s);
            pendingLow = true;
        } else {
            rmt_symbol_word_t& s = syms.back();
            s.level1 = level;
            s.duration1 = ticks;
            pendingLow = false;
        }
    };

    for (size_t byte_idx = 0; byte_idx < len; byte_idx++) {
        uint8_t current_byte = data[byte_idx];
        for (int sym = 0; sym < 4; sym++) {
            uint8_t symbol = current_byte & 0x03;
            current_byte >>= 2;
            push_half(1, ESL_PP4_BURST_TICKS);
            push_half(0, ESL_PP4_GAP_TICKS[symbol]);
        }
    }
    // Final closing burst, no trailing gap (duration1 left at 0 ends the
    // RMT transmission right after this on-pulse).
    push_half(1, ESL_PP4_BURST_TICKS);
}

bool esl_ir_transmit(const uint8_t* data, size_t len, uint16_t repeats_raw, uint8_t delay) {
    if (!esl_ir_ready) return false;
    if (!data || len == 0 || len > 255) return false;

    esl_ir_stop_requested = false;
    esl_ir_busy = true;
    uint32_t repeats = repeats_raw & 0x7FFF;

    std::vector<rmt_symbol_word_t> syms;
    esl_pp4_build_symbols(data, len, syms);

    bool ok = true;
    for (uint32_t rep = 0; rep <= repeats; rep++) {
        if (esl_ir_stop_requested) {
            ok = false;
            break;
        }

        rmt_transmit_config_t txc = {};
        txc.loop_count = 0;
        txc.flags.eot_level = 0; // leave the IR LED off when done

        esp_err_t err = rmt_transmit(esl_ir_channel, esl_ir_encoder, syms.data(),
                                      syms.size() * sizeof(rmt_symbol_word_t), &txc);
        if (err != ESP_OK) {
            ok = false;
            break;
        }
        err = rmt_tx_wait_all_done(esl_ir_channel, 2000);
        if (err != ESP_OK) {
            ok = false;
            break;
        }

        if (rep < repeats) {
            if (delay > 0) delayMicroseconds((uint32_t)delay * 500U);
            // Yield to the RTOS periodically to avoid starving other tasks
            // during long repeat counts (mirrors the original's watchdog-
            // friendly furi_delay_ms(1) every 5 repeats).
            if ((rep % 5U) == 4U) vTaskDelay(1);
        }
    }

    esl_ir_busy = false;
    return ok;
}

bool esl_ir_is_busy(void) { return esl_ir_busy; }

void esl_ir_stop(void) { esl_ir_stop_requested = true; }
