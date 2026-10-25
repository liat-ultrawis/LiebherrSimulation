// Focused self tests for frame encoding/decoding and numeric/bit progression.
// Run: LiebherrPlcSimulator.exe --selftest   (exit code 0 = all passed)
#pragma once

#include "plc_protocol.h"
#include <cstdio>

namespace plc105 {

static int g_testFailures = 0;
#define PLC_CHECK(cond) do { if (!(cond)) { ++g_testFailures; \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

inline void testValuePacking() {
    const uint16_t samples[] = { 0, 1, 2, 3, 127, 128, 129, 255, 256, 257, 511, 512,
                                 1000, 3599, 9999, 16384, 32766, SENSOR_ERROR };
    for (int i = 0; i < NUM_VALUES; ++i) {
        for (uint16_t v : samples) {
            uint8_t f[PLC_FRAME_LEN] = {};
            packValue(f, i, v);
            PLC_CHECK(unpackValue(f, i) == v);
            for (int b = 1; b <= 26; ++b) PLC_CHECK((f[b] & 0x80) == 0);
        }
    }
    // Exhaustive round trip for Value 04 (lifting height)
    for (uint32_t v = 0; v <= SENSOR_ERROR; ++v) {
        uint8_t f[PLC_FRAME_LEN] = {};
        packValue(f, 4, (uint16_t)v);
        if (unpackValue(f, 4) != v) { PLC_CHECK(false); break; }
    }
    // Known byte positions (spec §5/§6): DB1-7 / DB9-15 / DB0 / DB8
    {
        uint8_t f[PLC_FRAME_LEN] = {};
        packValue(f, 0, 3);       // DB0=1, DB1=1
        PLC_CHECK(f[5] == 0x01 && f[6] == 0x00 && f[23] == 0x01);
    }
    {
        uint8_t f[PLC_FRAME_LEN] = {};
        packValue(f, 0, 256);     // DB8 only
        PLC_CHECK(f[5] == 0x00 && f[6] == 0x00 && f[23] == 0x02);
    }
    {
        uint8_t f[PLC_FRAME_LEN] = {};
        packValue(f, 6, 104);     // wind 10.4 m/s = 0b1101000
        PLC_CHECK(f[17] == 0x34 && f[18] == 0x00 && f[25] == 0x00);
    }
    {
        uint8_t f[PLC_FRAME_LEN] = {};
        packValue(f, 8, 0x0101);  // Value 08 DB0+DB8 -> byte 25 bits 4,5
        PLC_CHECK(f[25] == 0x30 && f[21] == 0x00 && f[22] == 0x00);
    }
    {
        uint8_t f[PLC_FRAME_LEN] = {};
        packValue(f, 5, 0x0200);  // DB9 -> load high byte bit0
        PLC_CHECK(f[16] == 0x01 && f[15] == 0x00 && f[24] == 0x00);
    }
}

inline void testPlcFrame() {
    PlcFrameData d = {};
    for (int b = 0; b < NUM_PLC_BITS; ++b) d.bits[b] = (b % 2) == 0;
    for (int i = 0; i < NUM_VALUES; ++i) d.values[i] = (uint16_t)(1234 * (i + 1) % 32767);
    uint8_t f[PLC_FRAME_LEN];
    buildPlcFrame(d, f);
    PLC_CHECK(f[0] == STX && f[27] == ETX);
    PLC_CHECK(f[26] == xorChecksum(f, 1, 25));
    PLC_CHECK((f[1] & 0x80) == 0 && (f[3] == 0));   // byte 3 is all "free"
    PlcFrameData r = {};
    std::string err;
    PLC_CHECK(decodePlcFrame(f, PLC_FRAME_LEN, r, err));
    for (int b = 0; b < NUM_PLC_BITS; ++b) PLC_CHECK(r.bits[b] == d.bits[b]);
    for (int i = 0; i < NUM_VALUES; ++i) PLC_CHECK(r.values[i] == d.values[i]);

    // Single bit positions
    PlcFrameData z = {};
    z.bits[1] = true;   // ControlSystemOn  -> byte1 bit1
    z.bits[12] = true;  // BusConnectionOk  -> byte4 bit6
    buildPlcFrame(z, f);
    PLC_CHECK(f[1] == 0x02 && f[2] == 0x00 && f[4] == 0x40);
    PLC_CHECK(f[26] == (0x02 ^ 0x40));

    f[26] ^= 0x01;
    PLC_CHECK(!decodePlcFrame(f, PLC_FRAME_LEN, r, err));
}

inline void testAcsFrame() {
    const uint8_t ok[8] = { 0xC0, 0x05, 0x00, 0x08, 0x00, 0x00, 0x0D, 0x80 };
    std::string err;
    PLC_CHECK(validateAcsFrame(ok, 8, err));
    std::string s = formatAcsDecoded(ok);
    PLC_CHECK(s.find("StopSlewRight = 1") != std::string::npos);
    PLC_CHECK(s.find("StopSlewLeft = 0") != std::string::npos);
    PLC_CHECK(s.find("SlewLeftStep1 = 1") != std::string::npos);
    PLC_CHECK(s.find("StopHookRaising = 1") != std::string::npos);
    PLC_CHECK(s.find("StopHookLowering = 0") != std::string::npos);

    uint8_t bad[8];
    std::memcpy(bad, ok, 8); bad[6] = 0x0C;
    PLC_CHECK(!validateAcsFrame(bad, 8, err) && err.find("checksum") != std::string::npos);
    std::memcpy(bad, ok, 8); bad[7] = 0x00;
    PLC_CHECK(!validateAcsFrame(bad, 8, err) && err.find("ETX") != std::string::npos);
    std::memcpy(bad, ok, 8); bad[0] = 0x00;
    PLC_CHECK(!validateAcsFrame(bad, 8, err) && err.find("STX") != std::string::npos);
    PLC_CHECK(!validateAcsFrame(ok, 7, err) && err.find("length") != std::string::npos);
}

inline void testProgression() {
    NumericParam p;
    p.autoRun = true; p.minValue = 10; p.maxValue = 20; p.step = 4;
    std::string err;
    PLC_CHECK(p.validate(err));
    p.reset();
    const uint16_t expect[] = { 10, 14, 18, 20, 10, 14 };
    for (uint16_t e : expect) { PLC_CHECK(p.value() == e); p.advance(); }

    // Full protocol range, wrap at 32766 without overflow
    p.minValue = 32760; p.maxValue = VALUE_MAX; p.step = 5; p.reset();
    const uint16_t expect2[] = { 32760, 32765, 32766, 32760 };
    for (uint16_t e : expect2) { PLC_CHECK(p.value() == e); p.advance(); }

    // min == max stays constant
    p.minValue = p.maxValue = 7; p.step = 1; p.reset();
    for (int k = 0; k < 3; ++k) { PLC_CHECK(p.value() == 7); p.advance(); }

    // Fixed mode
    NumericParam f; f.fixedValue = 555; f.reset();
    for (int k = 0; k < 3; ++k) { PLC_CHECK(f.value() == 555); f.advance(); }

    // Validation
    NumericParam v; v.autoRun = true; v.minValue = 5; v.maxValue = 4; v.step = 1;
    PLC_CHECK(!v.validate(err));
    v.minValue = 0; v.step = 0;
    PLC_CHECK(!v.validate(err));
    v.step = 1; v.maxValue = 32767;
    PLC_CHECK(!v.validate(err));

    // Bit auto toggle every 2 cycles
    BitParam b; b.value = false; b.autoToggle = true; b.periodCycles = 2; b.reset();
    const bool expectB[] = { false, false, true, true, false };
    for (bool e : expectB) { PLC_CHECK(b.current == e); b.advance(); }
    BitParam c; c.value = true; c.reset();
    for (int k = 0; k < 3; ++k) { PLC_CHECK(c.current); c.advance(); }
}

inline void testTxLogDecodesFrame() {
    // Configuration says one thing, frame bytes say another: log must follow the bytes.
    PlcFrameData cfg = {};
    cfg.values[6] = 52;                 // WindSpeed 5.2 m/s configured
    uint8_t f[PLC_FRAME_LEN];
    buildPlcFrame(cfg, f);
    std::string s = formatTxFrameDecoded(f, PLC_FRAME_LEN);
    PLC_CHECK(s.find("WindSpeed = 5.2 m/s") != std::string::npos);
    PLC_CHECK(s.find("INVALID") == std::string::npos);

    // Patch the transmitted bytes to wind = 104 (10.4 m/s) with a valid checksum
    packValue(f, 6, 104);
    f[26] = xorChecksum(f, 1, 25);
    s = formatTxFrameDecoded(f, PLC_FRAME_LEN);
    PLC_CHECK(s.find("WindSpeed = 10.4 m/s") != std::string::npos);
    PLC_CHECK(s.find("WindSpeed = 5.2") == std::string::npos);

    // DB0 / DB8 contribute to decoded value (LiftingHeight raw 257 = 25.7 m)
    PlcFrameData h = {};
    h.values[4] = 257;
    buildPlcFrame(h, f);
    PLC_CHECK(formatTxFrameDecoded(f, PLC_FRAME_LEN).find("LiftingHeight = 25.7 m") != std::string::npos);

    // Bits are decoded from the bytes
    f[1] |= 0x02; f[26] = xorChecksum(f, 1, 25);
    PLC_CHECK(formatTxFrameDecoded(f, PLC_FRAME_LEN).find("ControlSystemOn = 1") != std::string::npos);

    // Validation errors are reported
    f[26] ^= 0x01;
    s = formatTxFrameDecoded(f, PLC_FRAME_LEN);
    PLC_CHECK(s.find("INVALID") != std::string::npos && s.find("checksum") != std::string::npos);
    f[26] ^= 0x01; f[27] = 0x00;
    PLC_CHECK(formatTxFrameDecoded(f, PLC_FRAME_LEN).find("ETX") != std::string::npos);
    PLC_CHECK(formatTxFrameDecoded(f, 27).find("length") != std::string::npos);
}

inline void testEngToRaw() {
    uint16_t r = 0;
    std::string err;
    // Valid, exact at resolution (incl. typical FP representations)
    PLC_CHECK(engToRaw(0, 359.9, r, err) && r == 3599);   // slewing angle /10
    PLC_CHECK(engToRaw(0, 0.0, r, err) && r == 0);
    PLC_CHECK(engToRaw(6, 10.4, r, err) && r == 104);     // wind /10
    PLC_CHECK(engToRaw(5, 2.13, r, err) && r == 213);     // load /100
    PLC_CHECK(engToRaw(5, 0.29, r, err) && r == 29);
    PLC_CHECK(engToRaw(1, 0.1, r, err) && r == 100);      // slewing speed /1000
    PLC_CHECK(engToRaw(4, 3276.6, r, err) && r == 32766); // protocol max
    PLC_CHECK(engToRaw(8, 32766, r, err) && r == 32766);  // Value 08 /1
    PLC_CHECK(engToRaw(4, 0.1, r, err) && r == 1);        // single step

    // Negative
    PLC_CHECK(!engToRaw(0, -0.1, r, err) && err.find("negative") != std::string::npos);
    PLC_CHECK(!engToRaw(0, -0.04, r, err) && err.find("negative") != std::string::npos);
    // Not representable at resolution
    PLC_CHECK(!engToRaw(0, 0.04, r, err) && err.find("resolution") != std::string::npos);
    PLC_CHECK(!engToRaw(0, 12.35, r, err) && err.find("resolution") != std::string::npos);
    PLC_CHECK(!engToRaw(5, 2.135, r, err) && err.find("resolution") != std::string::npos);
    PLC_CHECK(!engToRaw(1, 0.0004, r, err) && err.find("resolution") != std::string::npos);
    PLC_CHECK(!engToRaw(8, 1.5, r, err) && err.find("resolution") != std::string::npos);
    // Above protocol max
    PLC_CHECK(!engToRaw(4, 3276.7, r, err) && err.find("maximum") != std::string::npos);
    PLC_CHECK(!engToRaw(8, 32767, r, err) && err.find("maximum") != std::string::npos);
    // Non-finite
    PLC_CHECK(!engToRaw(0, std::nan(""), r, err));
    PLC_CHECK(!engToRaw(0, HUGE_VAL, r, err));

    // Converted Min/Max/Step keep progression behaviour (wind 0.0 .. 1.0 step 0.3)
    NumericParam p; p.autoRun = true;
    PLC_CHECK(engToRaw(6, 0.0, p.minValue, err));
    PLC_CHECK(engToRaw(6, 1.0, p.maxValue, err));
    PLC_CHECK(engToRaw(6, 0.3, p.step, err));
    PLC_CHECK(p.validate(err));
    p.reset();
    const uint16_t expect[] = { 0, 3, 6, 9, 10, 0 };
    for (uint16_t e : expect) { PLC_CHECK(p.value() == e); p.advance(); }
}

inline int runSelfTests() {
    g_testFailures = 0;
    testValuePacking();
    testPlcFrame();
    testAcsFrame();
    testProgression();
    testTxLogDecodesFrame();
    testEngToRaw();
    if (g_testFailures == 0) std::printf("SELFTEST PASSED\n");
    else std::printf("SELFTEST FAILED: %d check(s)\n", g_testFailures);
    return g_testFailures == 0 ? 0 : 1;
}

} // namespace plc105
