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

inline bool motionNear(double a, double b) { return std::fabs(a - b) < 1e-9; }

// Every motion speed/rate is the original value divided by SPEED_SCALE exactly once.
inline void testSpeedScaling() {
    PLC_CHECK(motionNear(kSlewMotion.maxSpeed * SPEED_SCALE, 0.8));
    PLC_CHECK(motionNear(kSlewMotion.acceleration * SPEED_SCALE, 0.4));
    PLC_CHECK(motionNear(kSlewMotion.deceleration * SPEED_SCALE, 0.4));
    PLC_CHECK(motionNear(kTrolleyMotion.maxSpeed * SPEED_SCALE, 1.8));
    PLC_CHECK(motionNear(kTrolleyMotion.acceleration * SPEED_SCALE, 0.5));
    PLC_CHECK(motionNear(kTrolleyMotion.deceleration * SPEED_SCALE, 0.5));
    // Ramp times unchanged by the scaling (slew 2 s, trolley 3.6 s to max)
    PLC_CHECK(motionNear(kSlewMotion.maxSpeed / kSlewMotion.acceleration, 2.0));
    PLC_CHECK(motionNear(kTrolleyMotion.maxSpeed / kTrolleyMotion.acceleration, 3.6));
    // SLOW steps stay proportional to the scaled max (fraction applied once, not rescaled)
    uint8_t f[8] = { STX, 0x00, 0x00, 0x00, 0x10, 0x03, 0x00, ETX };
    f[6] = xorChecksum(f, 1, 5);
    MotionState w; w.reset(0.0, 0.0, 0.0, 0.0);
    for (int k = 0; k < 100; ++k) w.update(1, 0, 0.13, decodeAcsRestrictions(f));
    PLC_CHECK(motionNear(w.slew.velocity, 0.25 * 0.8 / SPEED_SCALE));
    // Transmitted Value 01 at full slew speed: 0.8/3.9 1/min -> raw 205
    PlcFrameData d = {};
    w.slew.velocity = kSlewMotion.maxSpeed;
    w.apply(d);
    PLC_CHECK(d.values[VAL_SLEW_SPEED] == 205);
}

inline void testMotion() {
    const MotionParams& sp = kSlewMotion;
    const double vmax = sp.maxSpeed, acc = sp.acceleration, dec = sp.deceleration;

    // Acceleration from zero is time based
    AxisMotion a;
    a.update(1, 0.25, sp); PLC_CHECK(motionNear(a.velocity, acc * 0.25));
    a.update(1, 0.25, sp); PLC_CHECK(motionNear(a.velocity, acc * 0.5));

    // Reaching max speed and staying there
    for (int k = 0; k < 50; ++k) { a.update(1, 0.13, sp); PLC_CHECK(a.velocity <= vmax + 1e-12); }
    PLC_CHECK(motionNear(a.velocity, vmax));

    // Deceleration after release (gradual, then stopped)
    a.update(0, 1.0, sp); PLC_CHECK(motionNear(a.velocity, vmax - dec));
    a.update(0, 1.5, sp); PLC_CHECK(a.velocity == 0.0);
    a.update(0, 1.0, sp); PLC_CHECK(a.velocity == 0.0);

    // Direction reversal goes through zero
    a.velocity = vmax;
    a.update(-1, 1.0, sp); PLC_CHECK(motionNear(a.velocity, vmax - dec));
    a.update(-1, 1.5, sp); PLC_CHECK(a.velocity == 0.0);
    a.update(-1, 1.0, sp); PLC_CHECK(motionNear(a.velocity, -acc));
    a.velocity = vmax;
    bool sawZero = false;
    for (int k = 0; k < 100; ++k) {
        a.update(-1, 0.13, sp);
        if (a.velocity == 0.0) sawZero = true;
        if (a.velocity < 0.0) PLC_CHECK(sawZero);
    }
    PLC_CHECK(motionNear(a.velocity, -vmax));

    // Simultaneous opposite keys -> target 0
    PLC_CHECK(motionCommand(true, true) == 0);
    PLC_CHECK(motionCommand(false, false) == 0);
    PLC_CHECK(motionCommand(true, false) == 1);
    PLC_CHECK(motionCommand(false, true) == -1);
    a.velocity = vmax;
    a.update(motionCommand(true, true), 1.0, sp); PLC_CHECK(motionNear(a.velocity, vmax - dec));
    AxisMotion z;
    z.update(motionCommand(true, true), 1.0, sp); PLC_CHECK(z.velocity == 0.0);

    // Large dt is clamped; zero/negative dt changes nothing
    MotionState m; m.reset(0.0, 0.0, 0.0, 100.0);
    m.update(1, 1, 10.0);
    PLC_CHECK(motionNear(m.slew.velocity, kSlewMotion.acceleration * MOTION_MAX_DT_S));
    PLC_CHECK(motionNear(m.trolley.velocity, kTrolleyMotion.acceleration * MOTION_MAX_DT_S));
    MotionState n; n.reset(10.0, 3.0, 0.0, 6.0);
    n.update(1, 1, 0.0); n.update(1, 1, -1.0);
    PLC_CHECK(n.slew.velocity == 0.0 && n.trolley.velocity == 0.0);
    PLC_CHECK(motionNear(n.slewAngleDeg, 10.0) && motionNear(n.trolleyPosM, 3.0));

    // Trolley limits (configured range 2.0 .. 6.0 m)
    MotionState t; t.reset(0.0, 5.0, 2.0, 6.0);
    for (int k = 0; k < 200; ++k) { t.update(0, 1, 0.13); PLC_CHECK(t.trolleyPosM <= 6.0); }
    PLC_CHECK(t.trolleyPosM == 6.0 && t.trolley.velocity == 0.0);
    PlcFrameData d = {};
    t.apply(d);
    PLC_CHECK(d.values[VAL_TROLLEY_DISTANCE] == 60 && !d.bits[BIT_TROLLEY_FORWARD]);
    for (int k = 0; k < 400; ++k) { t.update(0, -1, 0.13); PLC_CHECK(t.trolleyPosM >= 2.0); }
    PLC_CHECK(t.trolleyPosM == 2.0 && t.trolley.velocity == 0.0);
    t.reset(0.0, 9.0, 2.0, 6.0); PLC_CHECK(t.trolleyPosM == 6.0);

    // Slew angle wrapping (max 0.8/3.9 1/min -> 6*max deg/s)
    MotionState w; w.reset(359.5, 0.0, 0.0, 0.0);
    w.slew.velocity = kSlewMotion.maxSpeed;
    w.update(1, 0, 0.5);
    PLC_CHECK(std::fabs(w.slewAngleDeg - (kSlewMotion.maxSpeed * 3.0 - 0.5)) < 1e-6);
    d = {};
    w.apply(d);
    PLC_CHECK(d.values[VAL_SLEW_ANGLE] == 1 && d.values[VAL_SLEW_SPEED] == 205);
    PLC_CHECK(d.bits[BIT_SLEW_RIGHT] && !d.bits[BIT_SLEW_LEFT]);
    w.reset(0.5, 0.0, 0.0, 0.0);
    w.slew.velocity = -kSlewMotion.maxSpeed;
    w.update(-1, 0, 0.5);
    d = {};
    w.apply(d);
    PLC_CHECK(d.values[VAL_SLEW_ANGLE] == 3599 && d.bits[BIT_SLEW_LEFT] && !d.bits[BIT_SLEW_RIGHT]);
    w.reset(725.0, 0.0, 0.0, 0.0); PLC_CHECK(std::fabs(w.slewAngleDeg - 5.0) < 1e-9);

    // Zero velocity: no movement, no direction bits; other fields untouched
    MotionState s; s.reset(123.4, 3.0, 0.0, 6.0);
    for (int k = 0; k < 10; ++k) s.update(0, 0, 0.13);
    PLC_CHECK(std::fabs(s.slewAngleDeg - 123.4) < 1e-9 && s.trolleyPosM == 3.0);
    d = {};
    d.bits[1] = true; d.bits[BIT_SLEW_RIGHT] = true; d.values[5] = 213;
    s.apply(d);
    PLC_CHECK(!d.bits[BIT_SLEW_RIGHT] && !d.bits[BIT_SLEW_LEFT]);
    PLC_CHECK(!d.bits[BIT_TROLLEY_FORWARD] && !d.bits[BIT_TROLLEY_BACKWARD]);
    PLC_CHECK(d.values[VAL_SLEW_ANGLE] == 1234 && d.values[VAL_SLEW_SPEED] == 0 && d.values[VAL_TROLLEY_DISTANCE] == 30);
    PLC_CHECK(d.bits[1] && d.values[5] == 213);

    // Motion values survive the unchanged frame encoding
    uint8_t f[PLC_FRAME_LEN];
    buildPlcFrame(d, f);
    PlcFrameData r = {};
    std::string err;
    PLC_CHECK(decodePlcFrame(f, PLC_FRAME_LEN, r, err) && r.values[VAL_SLEW_ANGLE] == 1234);
}

inline void makeAcsFrame(uint8_t f[8], uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5) {
    f[0] = STX; f[1] = b1; f[2] = b2; f[3] = b3; f[4] = b4; f[5] = b5;
    f[6] = xorChecksum(f, 1, 5); f[7] = ETX;
}

inline void testRestrictions() {
    uint8_t f[8];
    std::string err;
    const double vt = kTrolleyMotion.maxSpeed;
    const double dvt = kTrolleyMotion.deceleration * 0.5;    // velocity change per 0.5 s step

    // Decoding + priority STOP > SLOW > NORMAL
    makeAcsFrame(f, 0x05, 0x00, 0x08, 0x00, 0x00);          // StopSlewRight (+ step/hook bits ignored)
    PLC_CHECK(validateAcsFrame(f, 8, err));
    MotionLimits L = decodeAcsRestrictions(f);
    PLC_CHECK(L.slewRight == 0.0 && L.slewLeft == 1.0 && L.trolleyOut == 1.0 && L.trolleyIn == 1.0);
    makeAcsFrame(f, 0x00, 0x14, 0x00, 0x00, 0x00);          // Stop + Limit trolley out -> STOP
    L = decodeAcsRestrictions(f);
    PLC_CHECK(L.trolleyOut == 0.0 && L.trolleyIn == 1.0);
    makeAcsFrame(f, 0x00, 0x20, 0x00, 0x10, 0x07);          // limit in 25%, slew slow 25%
    L = decodeAcsRestrictions(f);
    PLC_CHECK(motionNear(L.trolleyIn, 0.25) && L.trolleyOut == 1.0);
    PLC_CHECK(motionNear(L.slewRight, 0.25) && motionNear(L.slewLeft, 0.25));
    makeAcsFrame(f, 0x00, 0x00, 0x00, 0x40, 0x00);          // Stop all motions
    L = decodeAcsRestrictions(f);
    PLC_CHECK(L.slewRight == 0.0 && L.slewLeft == 0.0 && L.trolleyOut == 0.0 && L.trolleyIn == 0.0);

    // STOP trolley out while key held: gradual deceleration, then stays stopped
    makeAcsFrame(f, 0x00, 0x04, 0x00, 0x00, 0x00);
    L = decodeAcsRestrictions(f);
    MotionState m; m.reset(0.0, 50.0, 0.0, 100.0);
    m.trolley.velocity = vt;                                 // full speed out
    m.update(0, 1, 0.5, L); PLC_CHECK(motionNear(m.trolley.velocity, vt - dvt));
    double prev = m.trolley.velocity;
    for (int k = 0; k < 100; ++k) {
        m.update(0, 1, 0.13, L);
        PLC_CHECK(m.trolley.velocity <= prev && m.trolley.velocity >= 0.0);
        prev = m.trolley.velocity;
    }
    PLC_CHECK(m.trolley.velocity == 0.0);
    PlcFrameData d = {};
    m.apply(d);
    PLC_CHECK(!d.bits[BIT_TROLLEY_FORWARD] && !d.bits[BIT_TROLLEY_BACKWARD]);

    // Restriction removed while key still held -> accelerates again (no jump)
    MotionLimits normal;
    m.update(0, 1, 0.5, normal); PLC_CHECK(motionNear(m.trolley.velocity, kTrolleyMotion.acceleration * 0.5));
    for (int k = 0; k < 100; ++k) m.update(0, 1, 0.13, normal);
    PLC_CHECK(motionNear(m.trolley.velocity, vt));

    // SLOW trolley out (50%): decelerate to 50% of max and stay limited
    makeAcsFrame(f, 0x00, 0x10, 0x00, 0x00, 0x00);
    L = decodeAcsRestrictions(f);
    const double trolleySlow = 0.5 * vt;
    m.update(0, 1, 0.5, L); PLC_CHECK(motionNear(m.trolley.velocity, vt - dvt));
    for (int k = 0; k < 100; ++k) { m.update(0, 1, 0.13, L); PLC_CHECK(m.trolley.velocity >= trolleySlow - 1e-12); }
    PLC_CHECK(motionNear(m.trolley.velocity, trolleySlow));
    // SLOW from standstill: accelerates only up to the limit
    MotionState s0; s0.reset(0.0, 50.0, 0.0, 100.0);
    for (int k = 0; k < 100; ++k) { s0.update(0, 1, 0.13, L); PLC_CHECK(s0.trolley.velocity <= trolleySlow + 1e-12); }
    PLC_CHECK(motionNear(s0.trolley.velocity, trolleySlow));
    // SLOW removed while held -> back to full speed
    for (int k = 0; k < 100; ++k) m.update(0, 1, 0.13, normal);
    PLC_CHECK(motionNear(m.trolley.velocity, vt));

    // SLOW slewing (25% of max)
    makeAcsFrame(f, 0x00, 0x00, 0x00, 0x10, 0x03);
    L = decodeAcsRestrictions(f);
    const double slewSlow = 0.25 * kSlewMotion.maxSpeed;
    MotionState w; w.reset(0.0, 0.0, 0.0, 0.0);
    w.slew.velocity = kSlewMotion.maxSpeed;
    w.update(1, 0, 0.05, L);
    PLC_CHECK(motionNear(w.slew.velocity, kSlewMotion.maxSpeed - kSlewMotion.deceleration * 0.05));
    for (int k = 0; k < 30; ++k) { w.update(1, 0, 0.13, L); PLC_CHECK(w.slew.velocity >= slewSlow - 1e-12); }
    PLC_CHECK(motionNear(w.slew.velocity, slewSlow));

    // Opposite directions are independent: STOP slew right does not affect left
    makeAcsFrame(f, 0x01, 0x00, 0x00, 0x00, 0x00);
    L = decodeAcsRestrictions(f);
    MotionState o; o.reset(0.0, 0.0, 0.0, 0.0);
    for (int k = 0; k < 10; ++k) { o.update(1, 0, 0.13, L); PLC_CHECK(o.slew.velocity == 0.0); }
    o.update(-1, 0, 0.25, L); PLC_CHECK(motionNear(o.slew.velocity, -kSlewMotion.acceleration * 0.25));
    for (int k = 0; k < 50; ++k) o.update(-1, 0, 0.13, L);
    PLC_CHECK(motionNear(o.slew.velocity, -kSlewMotion.maxSpeed));

    // STOP trolley in: moving in decelerates to 0, moving out still allowed at full speed
    makeAcsFrame(f, 0x00, 0x08, 0x00, 0x00, 0x00);
    L = decodeAcsRestrictions(f);
    MotionState t; t.reset(0.0, 50.0, 0.0, 100.0);
    t.trolley.velocity = -vt;
    t.update(0, -1, 0.5, L); PLC_CHECK(motionNear(t.trolley.velocity, -(vt - dvt)));
    prev = t.trolley.velocity;
    for (int k = 0; k < 100; ++k) {
        t.update(0, -1, 0.13, L);
        PLC_CHECK(t.trolley.velocity >= prev && t.trolley.velocity <= 0.0);
        prev = t.trolley.velocity;
    }
    PLC_CHECK(t.trolley.velocity == 0.0);
    for (int k = 0; k < 100; ++k) t.update(0, 1, 0.13, L);
    PLC_CHECK(motionNear(t.trolley.velocity, vt));
}

inline int runSelfTests() {
    g_testFailures = 0;
    testValuePacking();
    testPlcFrame();
    testAcsFrame();
    testProgression();
    testTxLogDecodesFrame();
    testEngToRaw();
    testSpeedScaling();
    testMotion();
    testRestrictions();
    if (g_testFailures == 0) std::printf("SELFTEST PASSED\n");
    else std::printf("SELFTEST FAILED: %d check(s)\n", g_testFailures);
    return g_testFailures == 0 ? 0 : 1;
}
} // namespace plc105
