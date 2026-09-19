/*
 * ESL image transmission sequence.
 *
 * The ORDER, REPEAT COUNTS and INTER-STAGE DELAYS below are protocol
 * knowledge, not presentation: they come from TagTinker
 * (https://github.com/i12bp8/TagTinker, GPL-3.0-only),
 * scenes/tagtinker_scene_transmit.c -- tx_send_full_payload() /
 * tx_send_payload_frames() for the general dot-matrix path and
 * tx_send_color26_payload() for the "Color 2.6" path, which uses a
 * different schedule entirely. See THIRD_PARTY.md.
 *
 * This layer sits between esl_protocol (pure, no hardware) and esl_ir
 * (ESP32 RMT). It deliberately knows nothing about Bruce's UI: callers
 * pass hooks for progress reporting and cancellation, so the same
 * sequence can be driven from a menu, a serial command or a test.
 *
 * For AUTHORIZED research/testing on hardware you own only.
 */

#ifndef __ESL_SEQUENCE_H__
#define __ESL_SEQUENCE_H__

#include "esl_protocol.h"

enum EslSequenceResult {
    EslSequenceOk = 0,
    EslSequenceBadArgs,
    EslSequenceCancelled,
    EslSequenceTxFailed,
};

// Stage identifiers reported through EslSequenceHooks::on_progress, so the
// caller can label progress without this layer knowing any display strings.
enum EslSequenceStage {
    EslStageWake = 0, // ping (general) or wake (Color 2.6)
    EslStageParams,
    EslStageData,
    EslStageRefresh,
};

struct EslSequenceHooks {
    // Polled between transmit batches. Return false to abort the sequence
    // (esl_sequence_send_image then returns EslSequenceCancelled).
    bool (*should_continue)(void* ctx);
    // done/total are frames within the current stage.
    void (*on_progress)(void* ctx, EslSequenceStage stage, uint32_t done, uint32_t total);
    void* ctx;
};

// TagTinker's default data-frame repeat setting (tagtinker_app.c), clamped
// there to 1..10. Passed as `repeats` to the IR layer, so the tag actually
// sees this many EXTRA sends on top of the first.
#define ESL_DEFAULT_DATA_FRAME_REPEATS 2
#define ESL_MIN_DATA_FRAME_REPEATS 1
#define ESL_MAX_DATA_FRAME_REPEATS 10

// Runs wake/ping -> image parameters -> image data -> refresh for `payload`.
// The caller owns esl_ir_init()/esl_ir_deinit() around this.
// `profile` selects the schedule (Color 2.6 vs general) and, for Color 2.6,
// forces the wire geometry the tag expects.
EslSequenceResult esl_sequence_send_image(
    const EslTagProfile* profile, const uint8_t plid[4], const EslImagePayload* payload,
    uint8_t page, uint16_t width, uint16_t height, uint16_t pos_x, uint16_t pos_y,
    uint8_t data_frame_repeats, const EslSequenceHooks* hooks
);

#endif // __ESL_SEQUENCE_H__
