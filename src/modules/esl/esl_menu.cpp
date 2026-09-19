#include "esl_menu.h"

#include "esl_ir.h"
#include "esl_protocol.h"

#include "core/display.h"
#include "core/mykeyboard.h"
#include <Arduino.h>
#include <globals.h>
#include <vector>

namespace {

// Repeat counts / inter-stage gaps below match the "Color 2.6" image-upload
// path in TagTinker's scenes/tagtinker_scene_transmit.c
// (tx_send_color26_payload / TX_COLOR26_WAKE_REPEATS), which was the one
// orchestration path fully read during porting. Applied here as the
// generic default for all dot-matrix profiles; TODO before relying on this
// for anything beyond diagnostics: confirm against upstream whether other
// profiles want the (unread) general tx_send_image_start repeat=15 path
// instead.
constexpr uint16_t kWakeRepeats = 400;
constexpr uint32_t kStageGapMs = 50;
// The wake burst is ~400 repeats of a ~19ms frame (several seconds of
// airtime). esl_ir_transmit() blocks for the whole run, so it is issued in
// batches: the repeat total stays faithful to upstream while Back stays
// responsive between batches.
constexpr uint16_t kWakeBatch = 20;

bool g_have_profile = false;
EslTagProfile g_profile{};
uint8_t g_plid[4] = {0, 0, 0, 0};

String plidToHex() {
    char buf[9];
    snprintf(buf, sizeof(buf), "%02X%02X%02X%02X", g_plid[0], g_plid[1], g_plid[2], g_plid[3]);
    return String(buf);
}

bool parsePlidHex(const String& hex, uint8_t out[4]) {
    if (hex.length() != 8) return false;
    for (int i = 0; i < 8; i++) {
        char c = hex[i];
        bool isHex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        if (!isHex) return false;
    }
    for (int i = 0; i < 4; i++) {
        out[i] = (uint8_t)strtoul(hex.substring(i * 2, i * 2 + 2).c_str(), nullptr, 16);
    }
    return true;
}

// 8px checkerboard: a synthetic, hardware-independent pattern used only to
// exercise the encode -> frame -> waveform pipeline end to end without
// requiring a real target's saved image (per the "safe diagnostic mode
// first" requirement).
bool buildCheckerboard(uint16_t w, uint16_t h, uint8_t** out) {
    if (w == 0 || h == 0 || w > ESL_CUSTOM_SIZE_MAX || h > ESL_CUSTOM_SIZE_MAX) return false;
    size_t count = (size_t)w * h;
    uint8_t* pixels = (uint8_t*)malloc(count);
    if (!pixels) return false;
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            pixels[(size_t)y * w + x] = (((x / 8) + (y / 8)) % 2) ? 1 : 0;
        }
    }
    *out = pixels;
    return true;
}

void drawStatusLine(const String& title, const String& line1, const String& line2) {
    drawMainBorderWithTitle(title);
    tft.setTextSize(FM);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.setCursor(10, 45);
    padprint(line1);
    tft.setCursor(10, 65);
    padprint(line2);
}

// Builds the checkerboard -> ESL payload -> frame set, then either reports
// a dry-run summary or actually transmits over IR. Centralizes every error
// path the requirements call out: unsupported profile, invalid dimensions,
// invalid/failed encode, IR init failure, transmit failure/cancel.
void runPipeline(bool dryRun) {
    if (!g_have_profile || !g_profile.known) {
        displayError("Select a tag profile first", true);
        return;
    }

    uint16_t w = g_profile.width;
    uint16_t h = g_profile.height;
    if (w == 0 || h == 0 || w > ESL_CUSTOM_SIZE_MAX || h > ESL_CUSTOM_SIZE_MAX) {
        displayError("Profile has no valid image\ndimensions (segment tag?)", true);
        return;
    }

    uint8_t* pixels = nullptr;
    if (!buildCheckerboard(w, h, &pixels)) {
        displayError("Invalid image dimensions\nor out of memory", true);
        return;
    }

    EslImagePayload payload{};
    bool encOk = esl_encode_image_payload(pixels, w, h, false, EslCompressionAuto, &payload);
    free(pixels);
    if (!encOk || payload.data == nullptr || payload.byte_count == 0 ||
        (payload.byte_count % ESL_IMAGE_DATA_BYTES_PER_FRAME) != 0) {
        displayError("Invalid data: image encoding\nfailed", true);
        esl_free_image_payload(&payload);
        return;
    }

    // byte_count travels as a 16-bit field in the image parameter frame, so a
    // larger payload cannot be described on the wire (it would truncate).
    if (payload.byte_count > 0xFFFFU) {
        displayError("Image too large for protocol\n(payload > 64KB)", true);
        esl_free_image_payload(&payload);
        return;
    }

    size_t frameCount = payload.byte_count / ESL_IMAGE_DATA_BYTES_PER_FRAME;

    if (dryRun) {
        drawMainBorderWithTitle("ESL Dry Run");
        tft.setTextSize(FM);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.setCursor(10, 45);
        padprint(g_profile.model_name);
        tft.setCursor(10, 60);
        padprint(String(w) + "x" + String(h) + "  comp=" + String(payload.comp_type));
        tft.setCursor(10, 75);
        padprint(String(payload.byte_count) + " bytes, " + String(frameCount) + " frames");
        tft.setCursor(10, 90);
        padprint("PLID " + plidToHex());
        tft.setCursor(10, 110);
        padprint("No IR transmitted.");
        delay(50);
        while (!check(EscPress) && !check(SelPress)) delay(10);
        esl_free_image_payload(&payload);
        return;
    }

    if (!esl_ir_init()) {
        displayError("IR initialization failed", true);
        esl_free_image_payload(&payload);
        return;
    }

    uint8_t frame[ESL_MAX_FRAME_SIZE];
    bool ok = true;
    bool cancelled = false;

    size_t len = esl_make_wake_frame(frame, g_plid);
    // Upstream sends repeats=400, i.e. 401 frames total.
    const uint16_t wakeFrames = kWakeRepeats + 1;
    for (uint16_t sent = 0; sent < wakeFrames;) {
        if (check(EscPress)) {
            cancelled = true;
            ok = false;
            break;
        }
        uint16_t remaining = (uint16_t)(wakeFrames - sent);
        uint16_t batch = remaining < kWakeBatch ? remaining : kWakeBatch;
        drawStatusLine(
            "ESL Transmit", "Waking target...",
            String(sent) + " / " + String(wakeFrames) + "  (BACK cancels)"
        );
        // repeats = batch - 1, since esl_ir_transmit always sends one frame
        // plus `repeats` extra ones.
        ok = esl_ir_transmit(frame, len, (uint16_t)(batch - 1), 1);
        if (!ok) break;
        sent = (uint16_t)(sent + batch);
    }
    if (ok) delay(kStageGapMs);

    if (ok) {
        drawStatusLine("ESL Transmit", "Sending image params...", "Press BACK to cancel");
        len = esl_make_image_param_frame(frame, g_plid, (uint16_t)payload.byte_count, payload.comp_type, 2, w, h, 0, 0);
        ok = esl_ir_transmit(frame, len, 1, 1);
        delay(kStageGapMs);
    }

    for (size_t i = 0; ok && i < frameCount; i++) {
        if (check(EscPress)) {
            cancelled = true;
            ok = false;
            break;
        }
        drawStatusLine(
            "ESL Transmit", "Sending image data...",
            String(i + 1) + " / " + String(frameCount) + "  (BACK cancels)"
        );
        len = esl_make_image_data_frame(frame, g_plid, (uint16_t)i, &payload.data[i * ESL_IMAGE_DATA_BYTES_PER_FRAME]);
        ok = esl_ir_transmit(frame, len, 1, 1);
        if (ok && (i + 1) < frameCount) delay(kStageGapMs);
    }

    if (ok) {
        delay(kStageGapMs);
        drawStatusLine("ESL Transmit", "Refreshing display...", "");
        len = esl_make_refresh_frame(frame, g_plid);
        ok = esl_ir_transmit(frame, len, 1, 1);
    }

    esl_ir_deinit();
    esl_free_image_payload(&payload);

    if (cancelled) {
        displayWarning("Transmission cancelled", true);
    } else if (!ok) {
        displayError("Transmission failed", true);
    } else {
        displaySuccess("Test pattern sent", true);
    }
}

void pickProfile() {
    options.clear();
    for (size_t i = 0; i < esl_profile_count(); i++) {
        const EslTagProfile* p = esl_profile_at(i);
        if (!p || p->kind != EslTagKindDotMatrix || p->width == 0 || p->height == 0) continue;
        EslTagProfile copy = *p; // captured by value into the lambda below
        String label = String(p->model_name) + " " + String(p->width) + "x" + String(p->height);
        options.push_back({label, [copy]() {
                                g_profile = copy;
                                g_have_profile = true;
                            }});
    }
    if (options.empty()) {
        displayError("No dot-matrix profiles built in", true);
        return;
    }
    options.push_back({"Back", []() {}});
    loopOptions(options, MENU_TYPE_SUBMENU, "Select ESL Tag Model");
}

void setTargetPlid() {
    String hex = hex_keyboard(plidToHex(), 8, "Target PLID (8 hex, raw bytes):");
    hex.trim();
    hex.toUpperCase();
    uint8_t parsed[4];
    if (hex.length() == 0) return; // user cancelled
    if (!parsePlidHex(hex, parsed)) {
        displayError("Invalid PLID: need exactly\n8 hex characters", true);
        return;
    }
    memcpy(g_plid, parsed, 4);
}

} // namespace

void EslMenu() {
    while (true) {
        String status = g_have_profile ? String(g_profile.model_name) : String("no profile");
        options = {
            {"Select Tag Model",         pickProfile                    },
            {"Set Target PLID (hex)",    setTargetPlid                  },
            {"Dry Run (no TX)",          [=]() { runPipeline(true); }  },
            {"Send Test Pattern",        [=]() { runPipeline(false); } },
            {"Back",                     []() {}                       },
        };
        int idx = loopOptions(
            options, MENU_TYPE_SUBMENU,
            ("ESL Research: " + status + "  PLID " + plidToHex()).c_str()
        );
        // loopOptions returns -1 (no operation run) when the user pressed
        // Back/Esc; it returns the chosen index, with that item's operation
        // already executed, otherwise. Re-show this menu unless the user
        // backed out or explicitly chose "Back".
        if (idx < 0 || options[idx].label == "Back") break;
    }
}
