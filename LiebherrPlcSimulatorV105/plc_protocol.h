// Liebherr protocol V1.05 - frame encoding / decoding / parameter progression.
// Source of truth: protocol1.05-spec.md
// Pure logic, no I/O. Header-only (included by main.cpp and the self tests).
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>

namespace plc105 {

// ---------------- Framing constants (spec §4, §5, §7, §8, §10) ----------------
const uint8_t  STX = 0xC0;
const uint8_t  ETX = 0x80;
const size_t   PLC_FRAME_LEN = 28;   // PLC -> ACS
const size_t   ACS_FRAME_LEN = 8;    // ACS -> PLC
const uint16_t VALUE_MAX = 32766;    // 0 - 07FFEh (spec §6)
const uint16_t SENSOR_ERROR = 0x7FFF; // spec §6

// ---------------- Bit field definitions ----------------
struct BitDef {
    int byteIdx;
    int bit;
    const char* name;  // short name used in the log
    const char* desc;  // protocol description
};

// PLC -> ACS bits that have a defined function (spec §5). Bits marked "free" are always 0.
static const BitDef kPlcBits[] = {
    {1, 0, "OverrideSignal",     "1.State/0 override signal from PLC"},
    {1, 1, "ControlSystemOn",    "1.State/1 control system ON"},
    {1, 2, "FreeSlewing",        "1.State/2 crane is in free slewing"},
    {1, 3, "SlewingGearRight",   "1.State/3 slewing gear right"},
    {1, 4, "SlewingGearLeft",    "1.State/4 slewing gear left"},
    {1, 5, "TrolleyForward",     "1.State/5 trolley forw. / jib outside (HCL)"},
    {1, 6, "TrolleyBackward",    "1.State/6 trolley backw. / jib inside (HCL)"},
    {2, 0, "TravelGearForward",  "2.State/0 travel gear forward"},
    {2, 1, "TravelGearBackward", "2.State/1 travel gear backward"},
    {2, 2, "HookUpward",         "2.State/2 hook direction upward"},
    {2, 3, "HookDownward",       "2.State/3 hook direction downward"},
    {4, 5, "PositionDataValid",  "2.Res/5 position data valid"},
    {4, 6, "BusConnectionOk",    "2.Res/6 bus PLC <-> CSM485 o.k."},
};
const int NUM_PLC_BITS = (int)(sizeof(kPlcBits) / sizeof(kPlcBits[0]));

// ACS -> PLC bits (spec §7). Byte 4 bits 0-3 and byte 5 bits 3-6 are "free".
static const BitDef kAcsBits[] = {
    {1, 0, "StopSlewRight",           "Stopp slewing right"},
    {1, 1, "StopSlewLeft",            "Stopp slewing left"},
    {1, 2, "SlewLeftStep1",           "Slewing left in first step"},
    {1, 3, "SlewRightStep1",          "Slewing right in first step"},
    {1, 4, "SlewStep2",               "Slew in second step"},
    {1, 5, "SlewStep3",               "Slew in third step"},
    {1, 6, "SlewStep4",               "Slew in fourth step"},
    {2, 0, "SlewStep5",               "Slew in fifth step"},
    {2, 1, "CloseSlewingBrake",       "Close the slewing brake"},
    {2, 2, "StopTrolleyForward",      "Stopp forward trolleying / outside luffing"},
    {2, 3, "StopTrolleyBackward",     "Stopp backward trolleying / inside luffing"},
    {2, 4, "LimitTrolleyForward",     "limit forward trolleying / jib outside luffing speed"},
    {2, 5, "LimitTrolleyBackward",    "limit backward trolleying / jib inside luffing speed"},
    {2, 6, "StopTravelForward",       "Stopp forward travelling"},
    {3, 0, "StopTravelBackward",      "Stopp backward travelling"},
    {3, 1, "LimitTravelForward",      "limit forward travelling speed"},
    {3, 2, "LimitTravelBackward",     "limit backward travelling speed"},
    {3, 3, "StopHookRaising",         "Stopp hook raising"},
    {3, 4, "StopHookLowering",        "Stopp hook lowering"},
    {3, 5, "LimitHookRaising",        "limit hook raising speed"},
    {3, 6, "LimitHookLowering",       "limit hook lowering speed"},
    {4, 4, "SlewSlowToBorder",        "Limit set value of slewing gear to slew slowly to border"},
    {4, 5, "AntiCollisionOverridden", "anti-collision system is overridden (by key)"},
    {4, 6, "StopAllMotions",          "Stopp all motions and activate the brakes"},
    {5, 0, "SlewLimitInfoBit0",       "Info bit0 to limit slewing speed"},
    {5, 1, "SlewLimitInfoBit1",       "Info bit1 to limit slewing speed"},
    {5, 2, "PreLimitSpeedInfo",       "max. speed in pre-limitswitch sector 0=50%, 1=25%"},
};
const int NUM_ACS_BITS = (int)(sizeof(kAcsBits) / sizeof(kAcsBits[0]));

// ---------------- Analog values (spec §5 bytes 5-25, §6 format) ----------------
// Scaling taken literally from the spec "Format" column:
//   359.9 deg -> 3599, 0.1 1/min -> 100, 99.9 m -> 999, 90.0 deg -> 900,
//   999.9 m -> 9999, 2.13 t -> 213, 10.4 m/s -> 104, 99.9 m -> 999.
struct ValueDef {
    const char* name;
    const char* desc;
    double divisor;    // engineering = raw / divisor
    int decimals;
    const char* unit;  // UTF-8
};

const int NUM_VALUES = 9;
static const ValueDef kPlcValues[NUM_VALUES] = {
    {"SlewingAngle",       "Value 00 slewing angle",        10.0,   1, "\xC2\xB0"},
    {"SlewingSpeed",       "Value 01 slewing speed",        1000.0, 3, "1/min"},
    {"TrolleyDistance",    "Value 02 trolley distance",     10.0,   1, "m"},
    {"LuffingAngle",       "Value 03 luffing angle",        10.0,   1, "\xC2\xB0"},
    {"LiftingHeight",      "Value 04 lifting height",       10.0,   1, "m"},
    {"Load",               "Value 05 load",                 100.0,  2, "t"},
    {"WindSpeed",          "Value 06 wind speed",           10.0,   1, "m/s"},
    {"TravelGearPosition", "Value 07 travel gear position", 10.0,   1, "m"},
    {"Value08",            "Value 08 free",                 1.0,    0, ""},
};

// ---------------- Decoded PLC frame content ----------------
struct PlcFrameData {
    bool bits[NUM_PLC_BITS];
    uint16_t values[NUM_VALUES];
};

// Value i: low byte = 5+2i (DB1-7), high byte = 6+2i (DB9-15),
// DB0/DB8 in byte 23 + i/3, bit pair (i%3)*2 / (i%3)*2+1.
inline void packValue(uint8_t* f, int i, uint16_t v) {
    f[5 + 2 * i] = (uint8_t)((v >> 1) & 0x7F);
    f[6 + 2 * i] = (uint8_t)((v >> 9) & 0x7F);
    const int fb = 23 + i / 3;
    const int sh = (i % 3) * 2;
    f[fb] = (uint8_t)(f[fb] & ~(3u << sh));
    if (v & 0x0001) f[fb] = (uint8_t)(f[fb] | (1u << sh));
    if (v & 0x0100) f[fb] = (uint8_t)(f[fb] | (2u << sh));
}

inline uint16_t unpackValue(const uint8_t* f, int i) {
    const int fb = 23 + i / 3;
    const int sh = (i % 3) * 2;
    uint16_t v = (uint16_t)(((f[5 + 2 * i] & 0x7Fu) << 1) | ((f[6 + 2 * i] & 0x7Fu) << 9));
    if (f[fb] & (1u << sh)) v |= 0x0001;
    if (f[fb] & (2u << sh)) v |= 0x0100;
    return v;
}

// XOR of data bytes [first..last], bit7 forced to 0 (spec §9)
inline uint8_t xorChecksum(const uint8_t* p, int first, int last) {
    uint8_t x = 0;
    for (int i = first; i <= last; ++i) x ^= p[i];
    return (uint8_t)(x & 0x7F);
}

inline void buildPlcFrame(const PlcFrameData& d, uint8_t out[PLC_FRAME_LEN]) {
    std::memset(out, 0, PLC_FRAME_LEN);
    out[0] = STX;
    for (int b = 0; b < NUM_PLC_BITS; ++b)
        if (d.bits[b]) out[kPlcBits[b].byteIdx] |= (uint8_t)(1u << kPlcBits[b].bit);
    for (int i = 0; i < NUM_VALUES; ++i) packValue(out, i, d.values[i]);
    out[26] = xorChecksum(out, 1, 25);
    out[27] = ETX;
}

// Validates framing/checksum and decodes. Returns false + reason on invalid frame.
inline bool decodePlcFrame(const uint8_t* p, size_t n, PlcFrameData& d, std::string& err) {
    err.clear();
    if (n != PLC_FRAME_LEN) { err = "length " + std::to_string(n) + " != 28"; return false; }
    if (p[0] != STX) err += "missing STX; ";
    if (p[27] != ETX) err += "missing ETX; ";
    for (int i = 1; i <= 26; ++i)
        if (p[i] & 0x80) { err += "bit7 set in byte " + std::to_string(i) + "; "; }
    if (xorChecksum(p, 1, 25) != p[26]) err += "checksum mismatch; ";
    for (int b = 0; b < NUM_PLC_BITS; ++b)
        d.bits[b] = (p[kPlcBits[b].byteIdx] >> kPlcBits[b].bit) & 1;
    for (int i = 0; i < NUM_VALUES; ++i) d.values[i] = unpackValue(p, i);
    return err.empty();
}

inline std::string formatValue(int i, uint16_t raw) {
    if (raw == SENSOR_ERROR) return "SENSOR-ERROR";
    char buf[64];
    const ValueDef& vd = kPlcValues[i];
    if (vd.unit[0])
        std::snprintf(buf, sizeof(buf), "%.*f %s", vd.decimals, raw / vd.divisor, vd.unit);
    else
        std::snprintf(buf, sizeof(buf), "%u", (unsigned)raw);
    return buf;
}

inline std::string formatPlcDecoded(const PlcFrameData& d) {
    // One parameter per line; the log adds the 3-space indent before the first line.
    std::string s;
    for (int i = 0; i < NUM_VALUES; ++i) {
        s += kPlcValues[i].name; s += " = "; s += formatValue(i, d.values[i]); s += "\n   ";
    }
    for (int b = 0; b < NUM_PLC_BITS; ++b) {
        s += kPlcBits[b].name; s += d.bits[b] ? " = 1" : " = 0";
        if (b + 1 < NUM_PLC_BITS) s += "\n   ";
    }
    return s;
}

// Decodes the actual transmitted bytes (not the configuration) for TX logging.
inline std::string formatTxFrameDecoded(const uint8_t* p, size_t n) {
    PlcFrameData d = {};
    std::string err;
    const bool ok = decodePlcFrame(p, n, d, err);
    if (n != PLC_FRAME_LEN) return "INVALID: " + err;
    std::string s = formatPlcDecoded(d);
    if (!ok) s = "INVALID: " + err + "\n   " + s;
    return s;
}

// Engineering value -> raw protocol value at the field's resolution (spec §6 scaling, unchanged).
// Rejects non-finite, negative, non-representable (beyond floating-point tolerance) and > 32766.
inline bool engToRaw(int i, double x, uint16_t& raw, std::string& err) {
    const ValueDef& vd = kPlcValues[i];
    char buf[160];
    if (!std::isfinite(x)) { err = "not a finite number"; return false; }
    if (x < 0.0) {
        std::snprintf(buf, sizeof(buf), "%g is negative (allowed: >= 0)", x);
        err = buf; return false;
    }
    const double scaled = x * vd.divisor;
    const double r = std::floor(scaled + 0.5);
    if (std::fabs(scaled - r) > 1e-6 * (r > 1.0 ? r : 1.0)) {
        std::snprintf(buf, sizeof(buf), "%.10g is not a multiple of the protocol resolution %.*f %s",
            x, vd.decimals, 1.0 / vd.divisor, vd.unit);
        err = buf; return false;
    }
    if (r > VALUE_MAX) {
        std::snprintf(buf, sizeof(buf), "%.10g exceeds protocol maximum %.*f %s (raw 32766)",
            x, vd.decimals, VALUE_MAX / vd.divisor, vd.unit);
        err = buf; return false;
    }
    raw = (uint16_t)r;
    return true;
}

// ---------------- ACS frame ----------------
inline bool validateAcsFrame(const uint8_t* p, size_t n, std::string& err) {
    err.clear();
    if (n != ACS_FRAME_LEN) { err = "length " + std::to_string(n) + " != 8"; return false; }
    if (p[0] != STX) err += "missing STX (byte0=" + std::to_string(p[0]) + "); ";
    if (p[7] != ETX) err += "missing ETX; ";
    for (int i = 1; i <= 6; ++i)
        if (p[i] & 0x80) err += "bit7 set in byte " + std::to_string(i) + "; ";
    const uint8_t cs = xorChecksum(p, 1, 5);
    if (cs != p[6]) {
        char b[64];
        std::snprintf(b, sizeof(b), "checksum mismatch (got %02X, expected %02X); ", p[6], cs);
        err += b;
    }
    if (!err.empty() && err.size() >= 2) err.erase(err.size() - 2);
    return err.empty();
}

inline std::string formatAcsDecoded(const uint8_t* p) {
    // One parameter per line; the log adds the 3-space indent before the first line.
    std::string s;
    for (int b = 0; b < NUM_ACS_BITS; ++b) {
        s += kAcsBits[b].name;
        s += ((p[kAcsBits[b].byteIdx] >> kAcsBits[b].bit) & 1) ? " = 1\n   " : " = 0\n   ";
    }
    // spec section 7 comment: byte5 bit0/bit1 -> limit for set value (active with byte4 bit4)
    static const char* kLimit[4] = { "10%", "15%", "20%", "25%" };
    s += "SlewLimit = "; s += kLimit[p[5] & 0x03];
    s += (p[4] & 0x10) ? " (active)" : " (inactive)";
    s += "\n   PreLimitMaxSpeed = "; s += (p[5] & 0x04) ? "25%" : "50%";
    return s;
}

// ---------------- Parameter configuration / automatic progression ----------------
struct BitParam {
    bool value = false;          // configured state
    bool autoToggle = false;
    uint32_t periodCycles = 10;  // toggle every N TX cycles
    bool current = false;
    uint32_t count = 0;

    void reset() { current = value; count = 0; }
    void advance() {
        if (!autoToggle || periodCycles == 0) return;
        if (++count >= periodCycles) { count = 0; current = !current; }
    }
};

struct NumericParam {
    bool autoRun = false;
    uint16_t fixedValue = 0;
    uint16_t minValue = 0;
    uint16_t maxValue = 0;
    uint16_t step = 1;
    uint16_t current = 0;

    bool validate(std::string& err) const {
        if (fixedValue > VALUE_MAX || minValue > VALUE_MAX || maxValue > VALUE_MAX) {
            err = "value exceeds 32766"; return false;
        }
        if (autoRun) {
            if (minValue > maxValue) { err = "Min > Max"; return false; }
            if (step == 0) { err = "Step must be >= 1"; return false; }
        }
        return true;
    }
    void reset() { current = autoRun ? minValue : fixedValue; }
    uint16_t value() const { return autoRun ? current : fixedValue; }
    // Min -> Max in steps; Max is always reached exactly, then restart at Min.
    void advance() {
        if (!autoRun) return;
        if (current >= maxValue) { current = minValue; return; }
        uint32_t n = (uint32_t)current + step;
        current = (uint16_t)(n > maxValue ? maxValue : n);
    }
};

struct SimConfig {
    BitParam bits[NUM_PLC_BITS];
    NumericParam values[NUM_VALUES];

    void reset() {
        for (auto& b : bits) b.reset();
        for (auto& v : values) v.reset();
    }
    PlcFrameData frameData() const {
        PlcFrameData d;
        for (int b = 0; b < NUM_PLC_BITS; ++b) d.bits[b] = bits[b].current;
        for (int i = 0; i < NUM_VALUES; ++i) d.values[i] = values[i].value();
        return d;
    }
    void advance() {
        for (auto& b : bits) b.advance();
        for (auto& v : values) v.advance();
    }
};

} // namespace plc105
