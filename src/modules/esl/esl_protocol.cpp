/*
 * ESL wire protocol implementation.
 *
 * Ported from TagTinker (https://github.com/i12bp8/TagTinker),
 * protocol/tagtinker_proto.c, licensed GNU GPL-3.0-only.
 * Copyright (C) the TagTinker contributors.
 * See THIRD_PARTY.md. SPDX-License-Identifier: GPL-3.0-only
 *
 * The CRC, frame layouts, profile table and RLE bit-packing below are a
 * direct, hardware-independent port: no Flipper/furi API was used by the
 * original code in these functions. Only the enclosing type names were
 * renamed (TagTinker* -> Esl*) and the unused/unimplemented upstream
 * declarations (tagtinker_build_image_sequence, tagtinker_rle_compress,
 * which have no body anywhere in the TagTinker source tree) were dropped.
 */

#include "esl_protocol.h"

#include <stdlib.h>
#include <string.h>

// --- Profile table (ported verbatim as data; ESL tag model catalogue) ---
// Stored directly as EslTagProfile so esl_profile_at() can hand out stable
// pointers into the table (a shared static scratch struct would alias
// across calls). Trailing `true` is EslTagProfile::known.

static const EslTagProfile PROFILE_TABLE[] = {
    {1206, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E2 HCS",         0, true},
    {1207, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E2 HCN",         4, true},
    {1217, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E5 HCS",         2, true},
    {1219, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E5 HCN",         1, true},
    {1240, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E4 HCS",         3, true},
    {1241, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E4 HCN",         0, true},
    {1242, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E4 HCN FZ",      0, true},
    {1243, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E4 HCW",         0, true},
    {1265, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "Continuum E5 HCS",         2, true},
    {1275, 320, 192, EslTagKindDotMatrix, EslTagColorMono,   "DM110",                    0, true},
    {1276, 320, 140, EslTagKindDotMatrix, EslTagColorMono,   "DM90",                     0, true},
    {1291, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "FVL Promoline 3-16",       0, true},
    {1300, 172, 72,  EslTagKindDotMatrix, EslTagColorMono,   "DM3370",                   0, true},
    {1314, 400, 300, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD110",           0, true},
    {1315, 296, 128, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD L",            0, true},
    {1317, 152, 152, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD S",            0, true},
    {1318, 208, 112, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD M",            0, true},
    {1319, 800, 480, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD200",           0, true},
    {1322, 152, 152, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD S",            0, true},
    {1324, 208, 112, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD M FZ",         0, true},
    {1327, 208, 112, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD M Red",        0, true},
    {1328, 296, 128, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD L Red",        0, true},
    {1336, 400, 300, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD110 Red",       0, true},
    {1339, 152, 152, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD S Red",        0, true},
    {1340, 800, 480, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD200 Red",       0, true},
    {1344, 296, 128, EslTagKindDotMatrix, EslTagColorYellow, "SmartTag HD L Yellow",     0, true},
    {1346, 800, 480, EslTagKindDotMatrix, EslTagColorYellow, "SmartTag HD200 Yellow",    0, true},
    {1348, 264, 176, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD T Red",        0, true},
    {1349, 264, 176, EslTagKindDotMatrix, EslTagColorYellow, "SmartTag HD T Yellow",     0, true},
    {1351, 648, 480, EslTagKindDotMatrix, EslTagColorMono,   "SmartTag HD150",           0, true},
    {1353, 648, 480, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD150 Red",       0, true},
    {1354, 648, 480, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD150 Red",       0, true},
    {1370, 296, 128, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD L Red (2021)", 0, true},
    {1371, 648, 480, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD150 Red (2021)",0, true},
    {1510, 0,   0,   EslTagKindSegment,   EslTagColorMono,   "SmartTag E5 M",            1, true},
    {ESL_TYPE_SMARTAG_COLOR_26, ESL_COLOR26_WIRE_W, ESL_COLOR26_WIRE_H,
     EslTagKindDotMatrix, EslTagColorRed, "SmartTAG Color 2.6",                          0, true},
    {1627, 296, 128, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD L Red",        0, true},
    {1628, 296, 128, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD L Red",        0, true},
    {1639, 152, 152, EslTagKindDotMatrix, EslTagColorRed,    "SmartTag HD S Red",        0, true},
};

#define PROFILE_TABLE_COUNT (sizeof(PROFILE_TABLE) / sizeof(PROFILE_TABLE[0]))

static const EslTagProfile* find_profile_entry(uint16_t type_code) {
    for (size_t i = 0; i < PROFILE_TABLE_COUNT; i++) {
        if (PROFILE_TABLE[i].type_code == type_code) return &PROFILE_TABLE[i];
    }
    return nullptr;
}

size_t esl_profile_count(void) { return PROFILE_TABLE_COUNT; }

const EslTagProfile* esl_profile_at(size_t index) {
    if (index >= PROFILE_TABLE_COUNT) return nullptr;
    return &PROFILE_TABLE[index];
}

bool esl_type_needs_wh_swap(uint16_t type_code) { return type_code == ESL_TYPE_SMARTAG_COLOR_26; }

bool esl_profile_needs_wh_swap(const EslTagProfile* profile) {
    return profile && esl_type_needs_wh_swap(profile->type_code);
}

void esl_profile_glass_size(const EslTagProfile* profile, uint16_t* width, uint16_t* height) {
    if (!profile || !width || !height) return;
    if (esl_profile_needs_wh_swap(profile)) {
        *width = ESL_COLOR26_GLASS_W;
        *height = ESL_COLOR26_GLASS_H;
        return;
    }
    *width = profile->width;
    *height = profile->height;
}

// --- CRC -----------------------------------------------------------------

uint16_t esl_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0x8408;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

static size_t terminate(uint8_t* buf, size_t len) {
    uint16_t crc = esl_crc16(buf, len);
    buf[len] = crc & 0xFF;
    buf[len + 1] = (crc >> 8) & 0xFF;
    return len + 2;
}

static size_t raw_frame(uint8_t* buf, uint8_t proto, const uint8_t plid[4], uint8_t cmd) {
    buf[0] = proto;
    memcpy(&buf[1], plid, 4);
    buf[5] = cmd;
    return 6;
}

static size_t mcu_frame(uint8_t* buf, const uint8_t plid[4], uint8_t cmd) {
    size_t p = raw_frame(buf, ESL_PROTO_DM, plid, 0x34);
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    buf[p++] = cmd;
    return p;
}

static void append_word(uint8_t* buf, size_t* p, uint16_t value) {
    buf[(*p)++] = (value >> 8) & 0xFF;
    buf[(*p)++] = value & 0xFF;
}

// --- Barcode / profile -----------------------------------------------------

bool esl_is_barcode_valid(const char* barcode) {
    if (!barcode || strlen(barcode) != 17) return false;
    return true;
}

bool esl_barcode_to_plid(const char* barcode, uint8_t plid[4]) {
    if (!barcode || strlen(barcode) != 17) return false;
    uint64_t a = 0, b = 0;
    for (int i = 2; i < 7; i++) a = a * 10 + (barcode[i] - '0');
    for (int i = 7; i < 12; i++) b = b * 10 + (barcode[i] - '0');

    uint64_t id = (a << 16) | b;
    plid[0] = id & 0xFF; // LSB first
    plid[1] = (id >> 8) & 0xFF;
    plid[2] = (id >> 16) & 0xFF;
    plid[3] = (id >> 24) & 0xFF;
    return true;
}

bool esl_type_is_known(uint16_t type_code) { return find_profile_entry(type_code) != nullptr; }

bool esl_barcode_to_type(const char* barcode, uint16_t* type_code) {
    if (!barcode || strlen(barcode) != 17 || !type_code) return false;
    uint16_t type = 0;
    for (int i = 12; i < 16; i++) type = type * 10 + (barcode[i] - '0');
    *type_code = type;
    return true;
}

bool esl_barcode_to_profile(const char* barcode, EslTagProfile* profile) {
    if (!profile) return false;
    memset(profile, 0, sizeof(*profile));
    uint16_t type_code = 0;
    if (!esl_barcode_to_type(barcode, &type_code)) return false;
    profile->type_code = type_code;
    const EslTagProfile* entry = find_profile_entry(type_code);
    if (!entry) return false;
    *profile = *entry;
    return true;
}

// --- Frame builders --------------------------------------------------------

size_t esl_make_ping_frame(uint8_t* buf, const uint8_t plid[4]) {
    size_t p = raw_frame(buf, ESL_PROTO_DM, plid, 0x97);
    buf[p++] = 0x01;
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    for (int i = 0; i < 20; i++) buf[p++] = 0x01;
    return terminate(buf, p);
}

size_t esl_make_wake_frame(uint8_t* buf, const uint8_t plid[4]) {
    // PrecIR / PriceHax wake: cmd 0x17 and 22 trailing 0x01 bytes.
    size_t p = raw_frame(buf, ESL_PROTO_DM, plid, 0x17);
    buf[p++] = 0x01;
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    for (int i = 0; i < 22; i++) buf[p++] = 0x01;
    return terminate(buf, p);
}

size_t esl_make_refresh_frame(uint8_t* buf, const uint8_t plid[4]) {
    size_t p = mcu_frame(buf, plid, 0x01);
    for (int i = 0; i < 18; i++) buf[p++] = 0x00;
    return terminate(buf, p);
}

size_t esl_make_image_param_frame(
    uint8_t* buf, const uint8_t plid[4], uint16_t byte_count, uint8_t comp_type, uint8_t page,
    uint16_t width, uint16_t height, uint16_t pos_x, uint16_t pos_y
) {
    size_t p = mcu_frame(buf, plid, 0x05);
    append_word(buf, &p, byte_count);
    buf[p++] = 0x00;
    buf[p++] = comp_type;
    buf[p++] = page;
    append_word(buf, &p, width);
    append_word(buf, &p, height);
    append_word(buf, &p, pos_x);
    append_word(buf, &p, pos_y);
    append_word(buf, &p, 0x0000);
    buf[p++] = 0x88;
    append_word(buf, &p, 0x0000);
    for (int i = 0; i < 4; i++) buf[p++] = 0x00;
    return terminate(buf, p);
}

size_t esl_make_image_data_frame(
    uint8_t* buf, const uint8_t plid[4], uint16_t index,
    const uint8_t data[ESL_IMAGE_DATA_BYTES_PER_FRAME]
) {
    size_t p = mcu_frame(buf, plid, 0x20);
    append_word(buf, &p, index);
    memcpy(&buf[p], data, ESL_IMAGE_DATA_BYTES_PER_FRAME);
    p += ESL_IMAGE_DATA_BYTES_PER_FRAME;
    return terminate(buf, p);
}

size_t esl_build_broadcast_page_frame(uint8_t* buf, uint8_t page, bool forever, uint16_t duration) {
    const uint8_t plid[4] = {0};
    size_t p = raw_frame(buf, ESL_PROTO_DM, plid, 0x06);
    buf[p++] = ((page & 7) << 3) | 0x01 | (forever ? 0x80 : 0x00);
    buf[p++] = 0x00;
    buf[p++] = 0x00;
    buf[p++] = (duration >> 8) & 0xFF;
    buf[p++] = duration & 0xFF;
    return terminate(buf, p);
}

// --- RLE bit-packing / image payload ----------------------------------------

struct EslBitWriter {
    uint8_t* data;
    size_t bit_pos;
};

static inline void bit_writer_append(EslBitWriter* writer, uint8_t bit) {
    size_t byte_idx = writer->bit_pos / 8U;
    size_t bit_idx = 7U - (writer->bit_pos % 8U);
    if (bit) writer->data[byte_idx] |= (uint8_t)(1U << bit_idx);
    writer->bit_pos++;
}

static size_t record_run_bit_length(uint32_t count) {
    size_t bits = 0;
    do {
        bits++;
        count >>= 1;
    } while (count);
    return (bits * 2U) - 1U;
}

static void bit_writer_append_run(EslBitWriter* writer, uint32_t count) {
    uint8_t bits[32];
    int n = 0;
    uint32_t v = count;
    while (v) {
        bits[n++] = v & 1U;
        v >>= 1;
    }
    for (int i = 0; i < n / 2; i++) {
        uint8_t t = bits[i];
        bits[i] = bits[n - 1 - i];
        bits[n - 1 - i] = t;
    }
    for (int i = 1; i < n; i++) bit_writer_append(writer, 0U);
    for (int i = 0; i < n; i++) bit_writer_append(writer, bits[i]);
}

static inline uint8_t plane_pixel_at(const uint8_t* p1, const uint8_t* p2, size_t count, size_t idx) {
    return (idx < count) ? p1[idx] : p2[idx - count];
}

#define DATA_BITS_PER_FRAME (ESL_IMAGE_DATA_BYTES_PER_FRAME * 8U)

static size_t esl_rle_fn_bit_length(EslPixelAtFn pixel_at, void* ctx, size_t total) {
    if (!pixel_at || total == 0) return 0;
    size_t bit_len = 1U;
    uint8_t run_pixel = pixel_at(0, ctx);
    uint32_t run_count = 1;
    for (size_t i = 1; i < total; i++) {
        uint8_t pix = pixel_at(i, ctx);
        if (pix == run_pixel) {
            run_count++;
        } else {
            bit_len += record_run_bit_length(run_count);
            run_pixel = pix;
            run_count = 1;
        }
    }
    if (run_count > 0U) bit_len += record_run_bit_length(run_count);
    return bit_len;
}

static void esl_pack_fn_raw(EslPixelAtFn pixel_at, void* ctx, size_t total, uint8_t* out) {
    EslBitWriter writer = {.data = out, .bit_pos = 0};
    for (size_t i = 0; i < total; i++) bit_writer_append(&writer, pixel_at(i, ctx));
}

static void esl_pack_fn_rle(EslPixelAtFn pixel_at, void* ctx, size_t total, uint8_t* out) {
    if (total == 0) return;
    EslBitWriter writer = {.data = out, .bit_pos = 0};
    uint8_t run_pixel = pixel_at(0, ctx);
    uint32_t run_count = 1;
    bit_writer_append(&writer, run_pixel);
    for (size_t i = 1; i < total; i++) {
        uint8_t pix = pixel_at(i, ctx);
        if (pix == run_pixel) {
            run_count++;
        } else {
            bit_writer_append_run(&writer, run_count);
            run_pixel = pix;
            run_count = 1;
        }
    }
    if (run_count > 0U) bit_writer_append_run(&writer, run_count);
}

bool esl_encode_fn_payload(
    EslPixelAtFn pixel_at, void* ctx, size_t total, EslCompressionMode mode, EslImagePayload* payload
) {
    if (!pixel_at || !payload || total == 0) return false;
    memset(payload, 0, sizeof(*payload));
    size_t comp_len = esl_rle_fn_bit_length(pixel_at, ctx, total);
    bool use_compressed = (mode == EslCompressionRle) ||
                           (mode == EslCompressionAuto && comp_len > 0U && comp_len < total);
    size_t src_len = use_compressed ? comp_len : total;
    size_t padded_bits =
        src_len + ((DATA_BITS_PER_FRAME - (src_len % DATA_BITS_PER_FRAME)) % DATA_BITS_PER_FRAME);
    uint8_t* data = (uint8_t*)calloc(padded_bits / 8U, 1);
    if (!data) return false;
    if (use_compressed) esl_pack_fn_rle(pixel_at, ctx, total, data);
    else esl_pack_fn_raw(pixel_at, ctx, total, data);
    payload->data = data;
    payload->byte_count = padded_bits / 8U;
    payload->comp_type = use_compressed ? 2U : 0U;
    return true;
}

struct EslPlaneCtx {
    const uint8_t* p1;
    const uint8_t* p2;
    size_t count;
};

static uint8_t esl_plane_pixel_cb(size_t idx, void* ctx) {
    const EslPlaneCtx* c = (const EslPlaneCtx*)ctx;
    return plane_pixel_at(c->p1, c->p2, c->count, idx);
}

static bool esl_encode_planes_payload(
    const uint8_t* p1, const uint8_t* p2, size_t count, EslCompressionMode mode, EslImagePayload* payload
) {
    if (!p1 || !payload) return false;
    EslPlaneCtx ctx = {.p1 = p1, .p2 = p2, .count = count};
    size_t total = p2 ? (count * 2U) : count;
    return esl_encode_fn_payload(esl_plane_pixel_cb, &ctx, total, mode, payload);
}

bool esl_encode_image_payload(
    const uint8_t* pixels, uint16_t width, uint16_t height, bool color_clear, EslCompressionMode mode,
    EslImagePayload* payload
) {
    size_t count = (size_t)width * height;
    uint8_t* second = nullptr;
    if (color_clear) {
        second = (uint8_t*)malloc(count);
        if (!second) return false;
        memset(second, 1, count);
    }
    bool ok = esl_encode_planes_payload(pixels, second, count, mode, payload);
    free(second);
    return ok;
}

void esl_free_image_payload(EslImagePayload* payload) {
    if (payload && payload->data) {
        free(payload->data);
        payload->data = nullptr;
    }
}
