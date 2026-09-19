/*
 * ESL image transmission sequence.
 * See esl_sequence.h and THIRD_PARTY.md for TagTinker attribution.
 *
 * Every constant in this file is transcribed from TagTinker's
 * scenes/tagtinker_scene_transmit.c rather than chosen here:
 *
 *   General dot-matrix (tx_send_full_payload / tx_send_payload_frames):
 *     ping(repeats=80) -> 50ms -> image params(repeats=15) -> 50ms
 *     -> N data frames(repeats=app->data_frame_repeats, default 2), with
 *        a 1ms pause after every 32nd frame "to avoid tag overflow"
 *     -> 50ms -> refresh(repeats=20)
 *
 *   Color 2.6 (tx_send_color26_payload):
 *     wake(repeats=400) -> 50ms -> image params(repeats=1) -> 50ms
 *     -> N data frames(repeats=1) spaced 50ms apart
 *     -> 50ms -> refresh(repeats=1)
 *     and the parameter frame always carries the 152x296 WIRE geometry
 *     at position 0,0, regardless of the glass being landscape.
 *
 * Note the IR layer's `repeats` argument means EXTRA sends, so a stage
 * described upstream as repeats=80 puts 81 frames on the wire. The
 * batching below preserves those totals exactly while still letting the
 * caller cancel partway through a long burst.
 */

#include "esl_sequence.h"

#include "esl_ir.h"
#include <Arduino.h>

namespace {

constexpr uint32_t kStageGapMs = 50;

// Long bursts (the 401-frame Color 2.6 wake in particular) are issued in
// batches so should_continue() gets polled regularly; esl_ir_transmit()
// itself blocks for the whole run it is given.
constexpr uint32_t kBatchFrames = 20;

// Upstream repeat values, expressed as TOTAL frames on the wire.
constexpr uint32_t kPingFrames = 80 + 1;
constexpr uint32_t kWakeFramesColor26 = 400 + 1;
constexpr uint32_t kParamFramesGeneral = 15 + 1;
constexpr uint32_t kParamFramesColor26 = 1 + 1;
constexpr uint32_t kRefreshFramesGeneral = 20 + 1;
constexpr uint32_t kRefreshFramesColor26 = 1 + 1;
constexpr uint32_t kDataFramesColor26 = 1 + 1;

// General path pauses 1ms after every 32nd data frame.
constexpr size_t kGeneralDataPauseEvery = 32;

// `delay` argument to esl_ir_transmit, in units of 500us between repeats
// (TagTinker passes 1 for every frame it sends).
constexpr uint8_t kInterRepeatDelayUnits = 1;

bool hooks_continue(const EslSequenceHooks* hooks) {
    if (!hooks || !hooks->should_continue) return true;
    return hooks->should_continue(hooks->ctx);
}

void hooks_progress(const EslSequenceHooks* hooks, EslSequenceStage stage, uint32_t done, uint32_t total) {
    if (hooks && hooks->on_progress) hooks->on_progress(hooks->ctx, stage, done, total);
}

// Sends exactly `totalFrames` copies of one frame, in cancel-friendly
// batches, preserving the upstream frame count.
EslSequenceResult send_repeated(
    const uint8_t* frame, size_t len, uint32_t totalFrames, EslSequenceStage stage,
    const EslSequenceHooks* hooks
) {
    for (uint32_t sent = 0; sent < totalFrames;) {
        if (!hooks_continue(hooks)) return EslSequenceCancelled;
        uint32_t remaining = totalFrames - sent;
        uint32_t batch = remaining < kBatchFrames ? remaining : kBatchFrames;
        hooks_progress(hooks, stage, sent, totalFrames);
        // repeats = batch - 1 because the first send is not a "repeat".
        if (!esl_ir_transmit(frame, len, (uint16_t)(batch - 1), kInterRepeatDelayUnits)) {
            return EslSequenceTxFailed;
        }
        sent += batch;
    }
    hooks_progress(hooks, stage, totalFrames, totalFrames);
    return EslSequenceOk;
}

} // namespace

EslSequenceResult esl_sequence_send_image(
    const EslTagProfile* profile, const uint8_t plid[4], const EslImagePayload* payload,
    uint8_t page, uint16_t width, uint16_t height, uint16_t pos_x, uint16_t pos_y,
    uint8_t data_frame_repeats, const EslSequenceHooks* hooks
) {
    if (!profile || !plid || !payload || !payload->data || payload->byte_count == 0) {
        return EslSequenceBadArgs;
    }
    if (payload->byte_count % ESL_IMAGE_DATA_BYTES_PER_FRAME != 0) return EslSequenceBadArgs;
    // byte_count is a 16-bit field in the parameter frame.
    if (payload->byte_count > 0xFFFFU) return EslSequenceBadArgs;

    if (data_frame_repeats < ESL_MIN_DATA_FRAME_REPEATS) data_frame_repeats = ESL_MIN_DATA_FRAME_REPEATS;
    if (data_frame_repeats > ESL_MAX_DATA_FRAME_REPEATS) data_frame_repeats = ESL_MAX_DATA_FRAME_REPEATS;

    const bool color26 = esl_profile_needs_wh_swap(profile);
    uint8_t frame[ESL_MAX_FRAME_SIZE];
    size_t len = 0;
    EslSequenceResult res = EslSequenceOk;

    // --- Stage 1: wake the tag -------------------------------------------
    // Color 2.6 uses the PrecIR-style wake (0x17); every other dot-matrix
    // tag is woken with a ping (0x97) on the general path.
    if (color26) {
        len = esl_make_wake_frame(frame, plid);
        res = send_repeated(frame, len, kWakeFramesColor26, EslStageWake, hooks);
    } else {
        len = esl_make_ping_frame(frame, plid);
        res = send_repeated(frame, len, kPingFrames, EslStageWake, hooks);
    }
    if (res != EslSequenceOk) return res;
    delay(kStageGapMs);

    // --- Stage 2: image parameters ---------------------------------------
    if (color26) {
        // The Color 2.6 parameter frame always describes the 152x296 wire
        // image at 0,0, and its image slot lives on page 2 by default.
        len = esl_make_image_param_frame(
            frame, plid, (uint16_t)payload->byte_count, payload->comp_type,
            esl_color26_resolve_page(page), ESL_COLOR26_WIRE_W, ESL_COLOR26_WIRE_H, 0, 0
        );
        res = send_repeated(frame, len, kParamFramesColor26, EslStageParams, hooks);
    } else {
        len = esl_make_image_param_frame(
            frame, plid, (uint16_t)payload->byte_count, payload->comp_type, page, width, height,
            pos_x, pos_y
        );
        res = send_repeated(frame, len, kParamFramesGeneral, EslStageParams, hooks);
    }
    if (res != EslSequenceOk) return res;
    delay(kStageGapMs);

    // --- Stage 3: image data ---------------------------------------------
    const size_t frameCount = payload->byte_count / ESL_IMAGE_DATA_BYTES_PER_FRAME;
    const uint32_t dataFrames = color26 ? kDataFramesColor26 : (uint32_t)data_frame_repeats + 1U;

    for (size_t i = 0; i < frameCount; i++) {
        if (!hooks_continue(hooks)) return EslSequenceCancelled;
        hooks_progress(hooks, EslStageData, (uint32_t)i, (uint32_t)frameCount);
        len = esl_make_image_data_frame(
            frame, plid, (uint16_t)i, &payload->data[i * ESL_IMAGE_DATA_BYTES_PER_FRAME]
        );
        if (!esl_ir_transmit(frame, len, (uint16_t)(dataFrames - 1), kInterRepeatDelayUnits)) {
            return EslSequenceTxFailed;
        }
        if ((i + 1) < frameCount) {
            // Color 2.6 spaces every data frame; the general path only
            // pauses briefly every 32 frames to avoid overflowing the tag.
            if (color26) delay(kStageGapMs);
            else if (((i + 1) % kGeneralDataPauseEvery) == 0) delay(1);
        }
    }
    hooks_progress(hooks, EslStageData, (uint32_t)frameCount, (uint32_t)frameCount);
    delay(kStageGapMs);

    // --- Stage 4: refresh the e-paper ------------------------------------
    if (!hooks_continue(hooks)) return EslSequenceCancelled;
    len = esl_make_refresh_frame(frame, plid);
    res = send_repeated(
        frame, len, color26 ? kRefreshFramesColor26 : kRefreshFramesGeneral, EslStageRefresh, hooks
    );
    return res;
}
