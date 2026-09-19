/*
 * ESL IR waveform transmitter — Bruce/ESP32-S3 specific.
 *
 * This is a FULL REWRITE, not a port. TagTinker's original driver
 * (ir/tagtinker_ir.c) bit-bangs the Flipper Zero's STM32WB55 TIM1
 * timer + DWT cycle counter directly and cannot run on ESP32 hardware
 * or API. Only the PROTOCOL-REQUIRED TIMING VALUES were carried over
 * (carrier frequency, PP4 burst/gap durations), read from the original
 * register values (see esl_ir.cpp for the derivation of each constant).
 *
 * Bruce's existing IR stack (src/modules/ir/, IRremoteESP8266 fork) is a
 * software GPIO bit-banger (delayMicroseconds toggling) meant for
 * ~30-56kHz remote-control carriers; it cannot reliably generate the
 * ~1.25MHz carrier this ESL protocol requires. This module instead uses
 * the ESP32 RMT peripheral directly, following the same driver/rmt_tx.h
 * pattern Bruce already uses for precise waveform generation in
 * src/modules/rf/protocols/rf_encoder.cpp.
 *
 * Uses Bruce's board-configured IR TX pin (bruceConfigPins.irTx) —
 * no hard-coded GPIO number.
 *
 * For AUTHORIZED research/testing on hardware you own only.
 */

#ifndef __ESL_IR_H__
#define __ESL_IR_H__

#include <stddef.h>
#include <stdint.h>

// true on success. Safe to call again if already initialized (no-op).
bool esl_ir_init(void);

void esl_ir_deinit(void);

// Sends `data[0..len)` PP4-encoded over IR, `repeats` extra times after the
// first (so repeats=0 sends exactly one frame), with a `delay` x 500us gap
// between repetitions. Returns false on invalid args, uninitialized driver,
// RMT failure, or if stopped early via esl_ir_stop().
bool esl_ir_transmit(const uint8_t* data, size_t len, uint16_t repeats, uint8_t delay);

bool esl_ir_is_busy(void);

void esl_ir_stop(void);

#endif // __ESL_IR_H__
