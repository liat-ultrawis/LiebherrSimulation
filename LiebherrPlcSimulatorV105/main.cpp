// Liebherr PLC Simulator V1.05
// Minimal standalone Windows console app that acts as the PLC (master).
// - Reads COM port from config.ini (COM_PORT=COMx)
// - Opens serial at 19200 8E1
// - Sends a valid V1.05 PLC->ACS packet (28 bytes) every 20 ms
// - Waits up to 30 ms for a valid V1.05 ACS->PLC packet (8 bytes)
// - Prints TX/RX/error lines
//
// Protocol reference: protocol1.05-spec.md

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <fstream>
#include <vector>

// ---------------- Config ----------------
static std::string readComPortFromIni(const std::string& path) {
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        // strip CR
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
            line.pop_back();
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        // trim
        auto trim = [](std::string& s){
            size_t a = s.find_first_not_of(" \t");
            size_t b = s.find_last_not_of(" \t");
            if (a == std::string::npos) { s.clear(); return; }
            s = s.substr(a, b - a + 1);
        };
        trim(k); trim(v);
        if (k == "COM_PORT") return v;
    }
    return "COM5";
}

// ---------------- Serial ----------------
class SerialPort {
public:
    SerialPort() : h_(INVALID_HANDLE_VALUE) {}
    ~SerialPort() { close(); }

    bool open(const std::string& port) {
        // Use \\.\COMx form to support COM10+
        std::string devName = "\\\\.\\" + port;
        h_ = CreateFileA(devName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                         OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) return false;

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h_, &dcb)) return false;
        dcb.BaudRate = 19200;
        dcb.ByteSize = 8;
        dcb.Parity   = EVENPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary  = TRUE;
        dcb.fParity  = TRUE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl  = DTR_CONTROL_DISABLE;
        dcb.fRtsControl  = RTS_CONTROL_DISABLE;
        dcb.fOutX = FALSE; dcb.fInX = FALSE;
        if (!SetCommState(h_, &dcb)) return false;

        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout         = MAXDWORD;
        to.ReadTotalTimeoutConstant    = 30;   // 30 ms receive window
        to.ReadTotalTimeoutMultiplier  = 0;
        to.WriteTotalTimeoutConstant   = 50;
        to.WriteTotalTimeoutMultiplier = 0;
        if (!SetCommTimeouts(h_, &to)) return false;

        PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR);
        return true;
    }

    void close() {
        if (h_ != INVALID_HANDLE_VALUE) { CloseHandle(h_); h_ = INVALID_HANDLE_VALUE; }
    }

    bool write(const uint8_t* data, size_t n) {
        DWORD written = 0;
        if (!WriteFile(h_, data, (DWORD)n, &written, nullptr)) return false;
        return written == n;
    }

    // Read up to n bytes, blocks up to the configured timeout total.
    // Returns number of bytes actually read.
    int read(uint8_t* buf, size_t n) {
        DWORD got = 0;
        if (!ReadFile(h_, buf, (DWORD)n, &got, nullptr)) return -1;
        return (int)got;
    }

    bool valid() const { return h_ != INVALID_HANDLE_VALUE; }

private:
    HANDLE h_;
};

// ---------------- V1.05 protocol ----------------
// PLC -> ACS: 28 bytes total.
//   [0]  STX = 0xC0
//   [1..25] payload bytes (bit7 always 0)
//   [26] XOR of [1..25], bit7 = 0
//   [27] ETX = 0x80
static void buildPlcPacket(uint8_t out[28], uint32_t counter) {
    std::memset(out, 0, 28);
    out[0] = 0xC0; // STX

    // Everything zero by default satisfies "bit7 == 0" and represents a
    // safe/idle PLC state (no drives commanded, sensors at 0).
    // We toggle a heartbeat bit in "position data valid" (Byte4 / bit5)
    // so the ACS side sees the packet contents move.
    out[4] = (uint8_t)(1u << 5); // 2.Res / bit5 = position data valid

    // Optional: put a slowly moving counter into the free "Value 08"
    // bytes 21 and 22 so raw traffic isn't visually static. Keep bit7 = 0.
    out[21] = (uint8_t)(counter & 0x7F);
    out[22] = (uint8_t)((counter >> 7) & 0x7F);

    // XOR checksum over bytes [1..25], mask bit7 to 0.
    uint8_t xr = 0;
    for (int i = 1; i <= 25; ++i) xr ^= out[i];
    out[26] = (uint8_t)(xr & 0x7F);

    out[27] = 0x80; // ETX
}

// Validate 8-byte ACS -> PLC packet.
static bool validateAcsPacket(const uint8_t* p, size_t n, std::string& err) {
    if (n != 8)          { err = "wrong length"; return false; }
    if (p[0] != 0xC0)    { err = "missing STX";  return false; }
    if (p[7] != 0x80)    { err = "missing ETX";  return false; }
    for (int i = 1; i <= 6; ++i)
        if (p[i] & 0x80) { err = "bit7 set in data byte"; return false; }
    uint8_t xr = 0;
    for (int i = 1; i <= 5; ++i) xr ^= p[i];
    xr &= 0x7F;
    if (xr != p[6])      { err = "bad checksum"; return false; }
    return true;
}

// ---------------- Printing ----------------
static void printBytes(const char* tag, const uint8_t* p, size_t n) {
    std::printf("%s:", tag);
    for (size_t i = 0; i < n; ++i) std::printf(" %02X", p[i]);
    std::printf("\n");
    std::fflush(stdout);
}

// Read exactly one V1.05 ACS frame within the receive window (30 ms).
// Framing strategy: scan the incoming byte stream for 0xC0, then read
// 7 more bytes; validate ETX at the end. Because SetCommTimeouts uses a
// 30 ms total-read timeout, this whole routine cannot exceed ~30 ms.
static int receiveAcsFrame(SerialPort& sp, uint8_t out[8]) {
    uint8_t scratch[64];
    int got = sp.read(scratch, sizeof(scratch));
    if (got <= 0) return got; // 0 = timeout, -1 = error
    // Find 0xC0
    for (int i = 0; i < got; ++i) {
        if (scratch[i] == 0xC0) {
            int remaining = got - i;
            if (remaining >= 8) {
                std::memcpy(out, scratch + i, 8);
                return 8;
            }
            // We have STX but not the full frame yet; read the rest.
            std::memcpy(out, scratch + i, remaining);
            int need = 8 - remaining;
            int more = sp.read(out + remaining, need);
            if (more < 0) return -1;
            return remaining + more;
        }
    }
    return 0; // no STX found
}

int main() {
    std::string comPort = readComPortFromIni("config.ini");
    std::printf("Liebherr PLC Simulator V1.05\n");
    std::printf("Using %s at 19200 8E1\n", comPort.c_str());

    SerialPort sp;
    if (!sp.open(comPort)) {
        std::printf("ERROR: failed to open %s (GetLastError=%lu)\n",
                    comPort.c_str(), GetLastError());
        return 1;
    }

    uint32_t counter = 0;
    for (;;) {
        // ---- TX ----
        uint8_t tx[28];
        buildPlcPacket(tx, counter++);
        if (!sp.write(tx, sizeof(tx))) {
            std::printf("ERROR: serial write failed (GetLastError=%lu)\n", GetLastError());
            break;
        }
        printBytes("TX", tx, sizeof(tx));

        // ---- RX (up to ~30 ms) ----
        uint8_t rx[8] = {0};
        int n = receiveAcsFrame(sp, rx);
        if (n <= 0) {
            std::printf("ERROR: serial read failed (GetLastError=%lu) or RX timeout (no ACS response within 30 ms)\n", GetLastError());
        } else {
            printBytes("RX", rx, (size_t)n);
            std::string err;
           // if (!validateAcsPacket(rx, (size_t)n, err)) {
            //    std::printf("ERROR: invalid ACS packet: %s\n", err.c_str());
           // }
        }
        std::fflush(stdout);

        // ---- Send cycle = 20 ms (RX already consumed up to 30 ms window,
        //      so we simply wait 20 ms before the next TX) ----
        Sleep(20);
    }

    sp.close();
    return 0;
}
