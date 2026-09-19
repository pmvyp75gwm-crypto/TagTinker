/*
 * ESL (Electronic Shelf Label) wire protocol.
 *
 * Ported from TagTinker (https://github.com/i12bp8/TagTinker),
 * protocol/tagtinker_proto.{c,h}, licensed GNU GPL-3.0-only.
 * Copyright (C) the TagTinker contributors.
 *
 * This file keeps ONLY the hardware-independent protocol logic: CRC,
 * frame layout, tag profile table, barcode parsing and image payload
 * (raw/RLE) encoding. It has no dependency on Bruce's IR/RMT layer or
 * on Flipper Zero APIs. See THIRD_PARTY.md for full attribution and
 * SPDX-License-Identifier: GPL-3.0-only for the porting rationale
 * (combined into Bruce, AGPL-3.0-or-later, per GPLv3 section 13).
 *
 * For AUTHORIZED research/testing on hardware you own only.
 */

#ifndef __ESL_PROTOCOL_H__
#define __ESL_PROTOCOL_H__

#include <stddef.h>
#include <stdint.h>

// Frame protocol IDs (byte 0 of a raw frame). DM = dot-matrix tags,
// SEG = segment/LCD tags. Only DM (image) tags are handled by this module.
#define ESL_PROTO_DM 0x85
#define ESL_PROTO_SEG 0x84

#define ESL_MAX_FRAME_SIZE 96
#define ESL_IMAGE_DATA_BYTES_PER_FRAME 20U

// "Color 2.6" glass is physically landscape 296x152, but the wire/image
// header for it is authored as 152x296 (short x long) and rotated on
// the wire. This one type code needs special width/height handling.
#define ESL_TYPE_SMARTAG_COLOR_26 1626
#define ESL_COLOR26_WIRE_W 152U
#define ESL_COLOR26_WIRE_H 296U
#define ESL_COLOR26_GLASS_W 296U
#define ESL_COLOR26_GLASS_H 152U

#define ESL_CUSTOM_SIZE_MIN 8U
#define ESL_CUSTOM_SIZE_MAX 800U
#define ESL_CUSTOM_SIZE_STEP 8U

// Printed tag barcodes are exactly 17 digits (TagTinker: TAGTINKER_BC_LEN).
#define ESL_BARCODE_LEN 17

enum EslTagKind {
    EslTagKindUnknown = 0,
    EslTagKindDotMatrix,
    EslTagKindSegment,
};

enum EslTagColor {
    EslTagColorMono = 0,
    EslTagColorRed,
    EslTagColorYellow,
};

struct EslTagProfile {
    uint16_t type_code;
    uint16_t width;
    uint16_t height;
    EslTagKind kind;
    EslTagColor color;
    const char* model_name;
    uint8_t pl_bit_def;
    bool known;
};

enum EslCompressionMode {
    EslCompressionAuto = 0,
    EslCompressionRaw,
    EslCompressionRle,
};

struct EslImagePayload {
    uint8_t* data;
    size_t byte_count;
    uint8_t comp_type;
};

// One frame builder call fills `buf` (caller-owned, >= ESL_MAX_FRAME_SIZE
// bytes) and returns the frame length in bytes (including the CRC16).
typedef uint8_t (*EslPixelAtFn)(size_t idx, void* ctx);

// --- CRC ---------------------------------------------------------------
uint16_t esl_crc16(const uint8_t* data, size_t len);

// --- Barcode / profile ---------------------------------------------------
bool esl_is_barcode_valid(const char* barcode);
bool esl_barcode_to_plid(const char* barcode, uint8_t plid[4]);
bool esl_barcode_to_type(const char* barcode, uint16_t* type_code);
bool esl_type_is_known(uint16_t type_code);
bool esl_barcode_to_profile(const char* barcode, EslTagProfile* profile);

// Number of built-in profiles and direct indexed access (for menu listing).
size_t esl_profile_count(void);
const EslTagProfile* esl_profile_at(size_t index);

bool esl_type_needs_wh_swap(uint16_t type_code);
bool esl_profile_needs_wh_swap(const EslTagProfile* profile);
void esl_profile_glass_size(const EslTagProfile* profile, uint16_t* width, uint16_t* height);

// Store-used Color 2.6 tags keep the barcode on page 1, so page 2 is the
// image slot; unspecified pages (0 or 1) map there, explicit 2-7 are left
// alone. (TagTinker: tagtinker_color26_resolve_page)
uint8_t esl_color26_resolve_page(uint8_t page);

// Wire (px, py) on 152x296 -> glass (bx, by) on 296x152.
// (TagTinker: tagtinker_color26_proto_to_glass)
void esl_color26_proto_to_glass(
    uint16_t proto_w, uint16_t px, uint16_t py, uint16_t* bx, uint16_t* by
);

// --- Image payload encoding ---------------------------------------------
bool esl_encode_fn_payload(
    EslPixelAtFn pixel_at, void* ctx, size_t total_pixels, EslCompressionMode mode,
    EslImagePayload* payload
);
bool esl_encode_image_payload(
    const uint8_t* pixels, uint16_t width, uint16_t height, bool color_clear,
    EslCompressionMode mode, EslImagePayload* payload
);
void esl_free_image_payload(EslImagePayload* payload);

// --- Frame builders ------------------------------------------------------
// Tags need a wake ping before most addressed commands.
size_t esl_make_ping_frame(uint8_t* buf, const uint8_t plid[4]);
size_t esl_make_wake_frame(uint8_t* buf, const uint8_t plid[4]);
size_t esl_make_refresh_frame(uint8_t* buf, const uint8_t plid[4]);

size_t esl_make_image_param_frame(
    uint8_t* buf, const uint8_t plid[4], uint16_t byte_count, uint8_t comp_type, uint8_t page,
    uint16_t width, uint16_t height, uint16_t pos_x, uint16_t pos_y
);
size_t esl_make_image_data_frame(
    uint8_t* buf, const uint8_t plid[4], uint16_t frame_index,
    const uint8_t data_bytes[ESL_IMAGE_DATA_BYTES_PER_FRAME]
);

// Broadcast frames address every listening tag (no PLID).
size_t esl_build_broadcast_page_frame(uint8_t* buf, uint8_t page, bool forever, uint16_t duration);

#endif // __ESL_PROTOCOL_H__
