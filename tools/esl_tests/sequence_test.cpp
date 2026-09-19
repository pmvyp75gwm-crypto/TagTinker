// Host-side verification of esl_sequence.cpp's transmit schedules against
// the values transcribed from TagTinker's scene_transmit.c.
// esl_ir_transmit is faked here so every frame put "on the wire" is recorded.
#include "esl_protocol.h"
#include "esl_sequence.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

struct TxCall {
    std::vector<uint8_t> frame;
    uint16_t repeats;
    uint8_t delayUnits;
    uint32_t frames() const { return (uint32_t)repeats + 1U; }
    uint8_t cmd() const { return frame.size() > 5 ? frame[5] : 0; }
    uint8_t mcuSub() const { return frame.size() > 9 ? frame[9] : 0; }
};

static std::vector<TxCall> g_tx;
static bool g_fail_after = false;
static uint32_t g_fail_at = 0;

bool esl_ir_transmit(const uint8_t* data, size_t len, uint16_t repeats, uint8_t delay) {
    if (g_fail_after && g_tx.size() >= g_fail_at) return false;
    TxCall c;
    c.frame.assign(data, data + len);
    c.repeats = repeats;
    c.delayUnits = delay;
    g_tx.push_back(c);
    return true;
}
bool esl_ir_init(void) { return true; }
void esl_ir_deinit(void) {}
bool esl_ir_is_busy(void) { return false; }
void esl_ir_stop(void) {}

static int failures = 0;
#define CHECK(cond, msg)                                                                           \
    do {                                                                                           \
        if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; }                                  \
        else { printf("  ok:   %s\n", msg); }                                                      \
    } while (0)

// Total frames on the wire for calls whose command byte matches.
static uint32_t framesWithCmd(uint8_t cmd) {
    uint32_t n = 0;
    for (auto& c : g_tx)
        if (c.cmd() == cmd) n += c.frames();
    return n;
}
static uint32_t framesWithMcuSub(uint8_t sub) {
    uint32_t n = 0;
    for (auto& c : g_tx)
        if (c.cmd() == 0x34 && c.mcuSub() == sub) n += c.frames();
    return n;
}
static uint32_t callsWithMcuSub(uint8_t sub) {
    uint32_t n = 0;
    for (auto& c : g_tx)
        if (c.cmd() == 0x34 && c.mcuSub() == sub) n++;
    return n;
}

static EslImagePayload makePayload(size_t dataFrames) {
    EslImagePayload p{};
    p.byte_count = dataFrames * ESL_IMAGE_DATA_BYTES_PER_FRAME;
    p.data = (uint8_t*)calloc(p.byte_count, 1);
    p.comp_type = 2;
    return p;
}

static const EslTagProfile* profileByType(uint16_t type) {
    for (size_t i = 0; i < esl_profile_count(); i++) {
        const EslTagProfile* p = esl_profile_at(i);
        if (p->type_code == type) return p;
    }
    return nullptr;
}

int main() {
    const uint8_t plid[4] = {1, 2, 3, 4};
    const size_t kFrames = 100;

    // ---- General dot-matrix path (SmartTag HD S, 1317, 152x152) ----------
    printf("\n== General dot-matrix schedule ==\n");
    const EslTagProfile* gen = profileByType(1317);
    EslImagePayload pay = makePayload(kFrames);
    g_tx.clear();
    EslSequenceResult r = esl_sequence_send_image(
        gen, plid, &pay, 0, gen->width, gen->height, 0, 0, ESL_DEFAULT_DATA_FRAME_REPEATS, nullptr
    );
    CHECK(r == EslSequenceOk, "sequence completes");
    CHECK(framesWithCmd(0x97) == 81, "wake stage is ping 0x97 x81 (upstream repeats=80)");
    CHECK(framesWithCmd(0x17) == 0, "general path does NOT use the 0x17 wake frame");
    CHECK(framesWithMcuSub(0x05) == 16, "image params x16 (upstream repeats=15)");
    CHECK(callsWithMcuSub(0x20) == kFrames, "one transmit call per image data frame");
    CHECK(framesWithMcuSub(0x20) == kFrames * 3,
          "each data frame sent x3 (upstream default data_frame_repeats=2)");
    CHECK(framesWithMcuSub(0x01) == 21, "refresh x21 (upstream repeats=20)");

    // Stage ordering: ping, then params, then data, then refresh.
    size_t iPing = 0, iParam = 0, iFirstData = 0, iRefresh = 0;
    for (size_t i = 0; i < g_tx.size(); i++) {
        if (g_tx[i].cmd() == 0x97) iPing = i;
        else if (g_tx[i].cmd() == 0x34 && g_tx[i].mcuSub() == 0x05) iParam = i;
        else if (g_tx[i].cmd() == 0x34 && g_tx[i].mcuSub() == 0x20 && iFirstData == 0) iFirstData = i;
        else if (g_tx[i].cmd() == 0x34 && g_tx[i].mcuSub() == 0x01) iRefresh = i;
    }
    CHECK(iPing < iParam && iParam < iFirstData && iFirstData < iRefresh,
          "stage order is wake -> params -> data -> refresh");

    // Param frame must carry this profile's own geometry.
    for (auto& c : g_tx) {
        if (c.cmd() == 0x34 && c.mcuSub() == 0x05) {
            uint16_t w = (uint16_t)((c.frame[15] << 8) | c.frame[16]);
            uint16_t h = (uint16_t)((c.frame[17] << 8) | c.frame[18]);
            CHECK(w == 152 && h == 152, "general param frame carries the profile geometry");
            break;
        }
    }
    // Data frame indices must be sequential from 0.
    bool idxOk = true;
    uint16_t expectIdx = 0;
    for (auto& c : g_tx) {
        if (c.cmd() == 0x34 && c.mcuSub() == 0x20) {
            uint16_t idx = (uint16_t)((c.frame[10] << 8) | c.frame[11]);
            if (idx != expectIdx++) idxOk = false;
        }
    }
    CHECK(idxOk, "image data frame indices run 0..N-1 in order");
    free(pay.data);

    // ---- Color 2.6 path (1626) -------------------------------------------
    printf("\n== Color 2.6 schedule ==\n");
    const EslTagProfile* c26 = profileByType(ESL_TYPE_SMARTAG_COLOR_26);
    EslImagePayload pay2 = makePayload(kFrames);
    g_tx.clear();
    r = esl_sequence_send_image(c26, plid, &pay2, 0, 999, 999, 7, 7,
                                ESL_DEFAULT_DATA_FRAME_REPEATS, nullptr);
    CHECK(r == EslSequenceOk, "sequence completes");
    CHECK(framesWithCmd(0x17) == 401, "wake stage is 0x17 x401 (upstream repeats=400)");
    CHECK(framesWithCmd(0x97) == 0, "Color 2.6 does NOT use the ping frame");
    CHECK(framesWithMcuSub(0x05) == 2, "image params x2 (upstream repeats=1)");
    CHECK(framesWithMcuSub(0x20) == kFrames * 2, "each data frame sent x2 (upstream repeats=1)");
    CHECK(framesWithMcuSub(0x01) == 2, "refresh x2 (upstream repeats=1)");
    for (auto& c : g_tx) {
        if (c.cmd() == 0x34 && c.mcuSub() == 0x05) {
            uint8_t page = c.frame[14];
            uint16_t w = (uint16_t)((c.frame[15] << 8) | c.frame[16]);
            uint16_t h = (uint16_t)((c.frame[17] << 8) | c.frame[18]);
            uint16_t px = (uint16_t)((c.frame[19] << 8) | c.frame[20]);
            uint16_t py = (uint16_t)((c.frame[21] << 8) | c.frame[22]);
            CHECK(w == ESL_COLOR26_WIRE_W && h == ESL_COLOR26_WIRE_H,
                  "Color 2.6 param frame forces 152x296 wire geometry, ignoring caller dims");
            CHECK(px == 0 && py == 0, "Color 2.6 param frame forces position 0,0");
            CHECK(page == 2, "page 0 resolves to the Color 2.6 image slot (page 2)");
            break;
        }
    }
    free(pay2.data);

    printf("\n== Page resolution helper ==\n");
    CHECK(esl_color26_resolve_page(0) == 2, "page 0 -> 2");
    CHECK(esl_color26_resolve_page(1) == 2, "page 1 -> 2");
    CHECK(esl_color26_resolve_page(5) == 5, "explicit page 5 left alone");
    CHECK(esl_color26_resolve_page(9) == 7, "page > 7 clamps to 7");

    printf("\n== Wire->glass mapping ==\n");
    uint16_t bx = 0, by = 0;
    esl_color26_proto_to_glass(ESL_COLOR26_WIRE_W, 0, 0, &bx, &by);
    CHECK(bx == 0 && by == ESL_COLOR26_WIRE_W - 1, "wire (0,0) maps to glass (0,151)");
    esl_color26_proto_to_glass(ESL_COLOR26_WIRE_W, 151, 295, &bx, &by);
    CHECK(bx == 295 && by == 0, "wire (151,295) maps to glass (295,0)");

    printf("\n== Cancellation & failure ==\n");
    EslImagePayload pay3 = makePayload(kFrames);
    g_tx.clear();
    EslSequenceHooks hooks{};
    hooks.should_continue = [](void*) { return g_tx.size() < 3; };
    r = esl_sequence_send_image(gen, plid, &pay3, 0, 152, 152, 0, 0,
                                ESL_DEFAULT_DATA_FRAME_REPEATS, &hooks);
    CHECK(r == EslSequenceCancelled, "should_continue()==false aborts with Cancelled");
    CHECK(g_tx.size() < 10, "abort happens promptly, not after the whole burst");

    g_tx.clear();
    g_fail_after = true;
    g_fail_at = 2;
    r = esl_sequence_send_image(gen, plid, &pay3, 0, 152, 152, 0, 0,
                                ESL_DEFAULT_DATA_FRAME_REPEATS, nullptr);
    CHECK(r == EslSequenceTxFailed, "IR transmit failure propagates as TxFailed");
    g_fail_after = false;

    printf("\n== Argument validation ==\n");
    EslImagePayload bad{};
    r = esl_sequence_send_image(gen, plid, &bad, 0, 152, 152, 0, 0, 2, nullptr);
    CHECK(r == EslSequenceBadArgs, "empty payload rejected");
    EslImagePayload misaligned = makePayload(2);
    misaligned.byte_count -= 1; // no longer a multiple of 20
    r = esl_sequence_send_image(gen, plid, &misaligned, 0, 152, 152, 0, 0, 2, nullptr);
    CHECK(r == EslSequenceBadArgs, "payload not frame-aligned rejected");
    free(misaligned.data);
    r = esl_sequence_send_image(nullptr, plid, &pay3, 0, 152, 152, 0, 0, 2, nullptr);
    CHECK(r == EslSequenceBadArgs, "null profile rejected");
    free(pay3.data);

    printf("\n%s (%d failure(s))\n\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}
