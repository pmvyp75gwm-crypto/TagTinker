// Host-side verification of: test pattern -> ESL image encoding ->
// protocol frame generation -> PP4 IR waveform generation.
// Compiles natively (no ESP32 needed); mirrors esl_ir.cpp's PP4 builder.
#include "esl_protocol.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

// Host mirror of ESP-IDF's rmt_symbol_word_t bitfield layout.
struct rmt_symbol_word_t {
    unsigned duration0 : 15;
    unsigned level0 : 1;
    unsigned duration1 : 15;
    unsigned level1 : 1;
};

#define ESL_PP4_BURST_TICKS 403
static const uint16_t ESL_PP4_GAP_TICKS[4] = {605, 1814, 1210, 2419};

// Copy of esl_ir.cpp's esl_pp4_build_symbols (kept identical on purpose).
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
    push_half(1, ESL_PP4_BURST_TICKS);
}

static int failures = 0;
#define CHECK(cond, msg)                                                                           \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("  FAIL: %s\n", msg);                                                           \
            failures++;                                                                            \
        } else {                                                                                   \
            printf("  ok:   %s\n", msg);                                                           \
        }                                                                                          \
    } while (0)

int main() {
    const uint8_t plid[4] = {0x11, 0x22, 0x33, 0x44};
    uint8_t frame[ESL_MAX_FRAME_SIZE];

    printf("\n== CRC ==\n");
    // CRC of the frame body must appear little-endian in the last two bytes.
    size_t wlen = esl_make_wake_frame(frame, plid);
    uint16_t crc = esl_crc16(frame, wlen - 2);
    CHECK((frame[wlen - 2] == (crc & 0xFF)) && (frame[wlen - 1] == ((crc >> 8) & 0xFF)),
          "wake frame CRC16 is appended LSB-first");
    CHECK(esl_crc16(nullptr, 0) == 0x8408, "CRC seed is 0x8408 for empty input");

    printf("\n== Frame layout / lengths ==\n");
    CHECK(wlen == 34, "wake frame is 34 bytes (6 hdr + 4 pad + 22 x 0x01 + 2 crc)");
    CHECK(frame[0] == ESL_PROTO_DM, "wake frame starts with DM proto byte 0x85");
    CHECK(memcmp(&frame[1], plid, 4) == 0, "PLID is embedded at bytes 1..4");
    CHECK(frame[5] == 0x17, "wake command byte is 0x17");

    size_t plen = esl_make_ping_frame(frame, plid);
    CHECK(plen == 32, "ping frame is 32 bytes");
    CHECK(frame[5] == 0x97, "ping command byte is 0x97");

    size_t rlen = esl_make_refresh_frame(frame, plid);
    CHECK(rlen == 30, "refresh frame is 30 bytes");
    CHECK(frame[5] == 0x34, "MCU frames use command 0x34");
    CHECK(frame[9] == 0x01, "refresh MCU sub-command is 0x01");

    size_t iplen = esl_make_image_param_frame(frame, plid, 1234, 2, 2, 152, 296, 0, 0);
    CHECK(iplen == 34, "image param frame is 34 bytes");
    CHECK(frame[9] == 0x05, "image param MCU sub-command is 0x05");
    CHECK((frame[10] == (1234 >> 8)) && (frame[11] == (1234 & 0xFF)), "byte_count is big-endian");

    uint8_t chunk[ESL_IMAGE_DATA_BYTES_PER_FRAME];
    memset(chunk, 0xA5, sizeof(chunk));
    size_t idlen = esl_make_image_data_frame(frame, plid, 7, chunk);
    CHECK(idlen == 34, "image data frame is 34 bytes");
    CHECK(frame[9] == 0x20, "image data MCU sub-command is 0x20");
    CHECK((frame[10] == 0) && (frame[11] == 7), "frame index is big-endian");

    printf("\n== Profiles ==\n");
    CHECK(esl_profile_count() == 39, "39 built-in tag profiles");
    CHECK(esl_type_is_known(ESL_TYPE_SMARTAG_COLOR_26), "Color 2.6 (1626) is a known type");
    CHECK(!esl_type_is_known(9999), "unknown type code is rejected");
    CHECK(esl_type_needs_wh_swap(ESL_TYPE_SMARTAG_COLOR_26), "Color 2.6 needs W/H swap");
    CHECK(!esl_type_needs_wh_swap(1317), "SmartTag HD S does not need W/H swap");

    uint16_t gw = 0, gh = 0;
    const EslTagProfile* c26 = nullptr;
    for (size_t i = 0; i < esl_profile_count(); i++) {
        const EslTagProfile* p = esl_profile_at(i);
        if (p->type_code == ESL_TYPE_SMARTAG_COLOR_26) { c26 = p; }
    }
    CHECK(c26 != nullptr, "Color 2.6 profile is retrievable by index");
    if (c26) {
        esl_profile_glass_size(c26, &gw, &gh);
        CHECK(gw == ESL_COLOR26_GLASS_W && gh == ESL_COLOR26_GLASS_H,
              "Color 2.6 glass size is 296x152 while wire size stays 152x296");
        CHECK(c26->width == ESL_COLOR26_WIRE_W && c26->height == ESL_COLOR26_WIRE_H,
              "Color 2.6 wire size is 152x296");
    }

    printf("\n== Barcode parsing ==\n");
    // 17 digits: XX AAAAA BBBBB TTTT X ; type code is digits [12..16)
    const char* bc = "0000100002001639";
    CHECK(!esl_is_barcode_valid(bc), "16-digit barcode is rejected (needs 17)");
    const char* bc17 = "00001000020016390";
    CHECK(esl_is_barcode_valid(bc17), "17-digit barcode accepted");
    uint16_t type = 0;
    CHECK(esl_barcode_to_type(bc17, &type) && type == 1639, "type code parsed from digits 12..15");
    uint8_t out_plid[4] = {0};
    CHECK(esl_barcode_to_plid(bc17, out_plid), "PLID derived from barcode");
    EslTagProfile prof{};
    CHECK(esl_barcode_to_profile(bc17, &prof) && prof.known && prof.width == 152,
          "profile resolved from barcode (1639 -> SmartTag HD S Red 152x152)");

    // Cases the barcode entry screen must reject as "unsupported profile".
    EslTagProfile p2{};
    CHECK(esl_barcode_to_profile("00001000020013170", &p2) && p2.kind == EslTagKindDotMatrix &&
              p2.width == 152 && p2.height == 152,
          "known dot-matrix barcode (1317) accepted");
    EslTagProfile p3{};
    CHECK(esl_barcode_to_profile("00001000020012060", &p3) && p3.kind == EslTagKindSegment &&
              p3.width == 0,
          "segment tag (1206) resolves but has no image geometry -> UI rejects");
    EslTagProfile p4{};
    CHECK(!esl_barcode_to_profile("00001000020099990", &p4),
          "unknown type code (9999) rejected as unsupported profile");
    CHECK(!p4.known, "rejected profile is not marked known");
    EslTagProfile p5{};
    CHECK(esl_barcode_to_profile("00001000020016260", &p5) &&
              p5.type_code == ESL_TYPE_SMARTAG_COLOR_26 && esl_profile_needs_wh_swap(&p5),
          "Color 2.6 barcode (1626) selects the swapped-geometry profile");

    // Two different tags must yield two different PLIDs.
    uint8_t plidA[4] = {0}, plidB[4] = {0};
    esl_barcode_to_plid("00001000020013170", plidA);
    esl_barcode_to_plid("00002000030013170", plidB);
    CHECK(memcmp(plidA, plidB, 4) != 0, "different barcodes derive different PLIDs");

    printf("\n== Test pattern -> image encoding ==\n");
    const uint16_t W = 152, H = 152;
    std::vector<uint8_t> pixels((size_t)W * H);
    for (uint16_t y = 0; y < H; y++)
        for (uint16_t x = 0; x < W; x++) pixels[(size_t)y * W + x] = (((x / 8) + (y / 8)) % 2) ? 1 : 0;

    EslImagePayload payload{};
    bool enc = esl_encode_image_payload(pixels.data(), W, H, false, EslCompressionAuto, &payload);
    CHECK(enc && payload.data != nullptr, "checkerboard encodes successfully");
    CHECK(payload.byte_count % ESL_IMAGE_DATA_BYTES_PER_FRAME == 0,
          "payload is padded to a whole number of 20-byte data frames");
    CHECK(payload.comp_type == 2, "8px checkerboard picks RLE (comp_type 2) via Auto mode");
    size_t frameCount = payload.byte_count / ESL_IMAGE_DATA_BYTES_PER_FRAME;
    printf("        (payload %zu bytes -> %zu data frames)\n", payload.byte_count, frameCount);
    CHECK(frameCount > 0, "at least one data frame produced");

    // Raw mode must yield exactly one bit per pixel, padded up to frame size.
    EslImagePayload rawp{};
    esl_encode_image_payload(pixels.data(), W, H, false, EslCompressionRaw, &rawp);
    size_t rawBitsNeeded = (size_t)W * H;
    CHECK(rawp.comp_type == 0, "forced raw mode reports comp_type 0");
    CHECK(rawp.byte_count * 8 >= rawBitsNeeded &&
              (rawp.byte_count * 8 - rawBitsNeeded) < ESL_IMAGE_DATA_BYTES_PER_FRAME * 8,
          "raw payload holds 1 bit/pixel plus < one frame of padding");
    esl_free_image_payload(&rawp);

    printf("\n== Protocol frames -> PP4 waveform ==\n");
    std::vector<rmt_symbol_word_t> syms;
    esl_make_wake_frame(frame, plid);
    esl_pp4_build_symbols(frame, wlen, syms);
    // Each byte -> 4 symbols (burst+gap = 2 half-pulses each) + 1 closing burst.
    CHECK(syms.size() == wlen * 4 + 1, "symbol count = len*4 + 1 closing burst");
    CHECK(syms[0].level0 == 1 && syms[0].duration0 == ESL_PP4_BURST_TICKS,
          "waveform starts with a 403-tick (40.3us) carrier burst");
    CHECK(syms[0].level1 == 0, "burst is followed by a carrier-off gap");
    CHECK(syms.back().duration1 == 0,
          "final symbol's second half is empty (frame ends on the closing burst)");
    CHECK(syms.back().level0 == 1 && syms.back().duration0 == ESL_PP4_BURST_TICKS,
          "final half-pulse is the closing burst");

    // Verify the gap actually selected for each 2-bit symbol of byte 0.
    uint8_t b0 = frame[0]; // 0x85 = 0b10000101 -> symbols (LSB-first pairs): 1,1,0,2
    uint8_t expect[4];
    uint8_t tmp = b0;
    for (int i = 0; i < 4; i++) {
        expect[i] = tmp & 0x03;
        tmp >>= 2;
    }
    bool gapsOk = true;
    for (int i = 0; i < 4; i++) {
        if (syms[i].duration1 != ESL_PP4_GAP_TICKS[expect[i]]) gapsOk = false;
    }
    CHECK(gapsOk, "each 2-bit symbol (LSB-first) selects its matching gap duration");

    // Total airtime sanity: sum of all half-pulses at 100ns/tick.
    double us = 0;
    for (auto& s : syms) us += (s.duration0 + s.duration1) * 0.1;
    printf("        (wake frame airtime: %.1f us over %zu symbols)\n", us, syms.size());
    CHECK(us > 1000 && us < 60000, "wake frame airtime is in a plausible ms-scale range");

    esl_free_image_payload(&payload);

    printf("\n%s (%d failure(s))\n\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}
