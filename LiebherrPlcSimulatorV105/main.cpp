// Liebherr PLC Simulator V1.05
// Single-threaded precise timing: one TX every 50 ms, one RX attempt per cycle using overlapped I/O.
// - Reads COM port from config.ini (COM_PORT=COMx)
// - Opens serial at 19200 8E1 (unchanged)
// - Sends one valid 28-byte PLC->ACS frame exactly on steady_clock deadlines 50 ms apart
// - Immediately starts ONE overlapped ReadFile and waits until either it completes or the next deadline occurs
// - If ReadFile completes before the deadline print RX once; otherwise cancel the pending read and send next TX
// - No per-cycle COMMTIMEOUTS changes, no additional threads, no RX validation/interpretation
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
#include <chrono>
#include <thread>

using steady_clock = std::chrono::steady_clock;
using ms = std::chrono::milliseconds;

// ---------------- Config ----------------
static std::string readComPortFromIni(const std::string& path) {
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
            line.pop_back();
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        auto trim = [](std::string& s) {
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

// ---------------- Serial (overlapped support) ----------------
class SerialPort {
public:
    SerialPort() : h_(INVALID_HANDLE_VALUE) {}
    ~SerialPort() { close(); }

    bool open(const std::string& port) {
        // Use \\.\COMx form and open overlapped
        std::string devName = "\\\\.\\" + port;
        h_ = CreateFileA(devName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
            nullptr);
        if (h_ == INVALID_HANDLE_VALUE) return false;

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h_, &dcb)) return false;
        dcb.BaudRate = 19200;
        dcb.ByteSize = 8;
        dcb.Parity = EVENPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fParity = TRUE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fOutX = FALSE; dcb.fInX = FALSE;
        if (!SetCommState(h_, &dcb)) return false;

        // Set initial COM timeouts (not changed per-cycle). Overlapped reads will be managed by the loop.
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutConstant = 30;
        to.ReadTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant = 50;
        to.WriteTotalTimeoutMultiplier = 0;
        if (!SetCommTimeouts(h_, &to)) return false;

        PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR);
        return true;
    }

    void close() {
        if (h_ != INVALID_HANDLE_VALUE) { CloseHandle(h_); h_ = INVALID_HANDLE_VALUE; }
    }

    // synchronous write implemented on an overlapped handle (blocks until complete)
    bool writeSync(const uint8_t* data, size_t n) {
        if (h_ == INVALID_HANDLE_VALUE) return false;
        OVERLAPPED ov;
        ZeroMemory(&ov, sizeof(ov));
        ov.hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        if (!ov.hEvent) return false;

        DWORD written = 0;
        BOOL ok = WriteFile(h_, data, (DWORD)n, &written, &ov);
        if (!ok) {
            DWORD e = GetLastError();
            if (e == ERROR_IO_PENDING) {
                if (!GetOverlappedResult(h_, &ov, &written, TRUE)) {
                    CloseHandle(ov.hEvent);
                    return false;
                }
            }
            else {
                CloseHandle(ov.hEvent);
                return false;
            }
        }
        CloseHandle(ov.hEvent);
        return written == n;
    }

    // Start one overlapped read and wait up to timeoutMs for completion.
    // Returns: >0 number of bytes read, 0 = timeout (no bytes), -1 = error.
    // Behavior notes (implements safe cancellation + stale-RX discard):
    //  - If read completes before timeout -> return bytes (>0).
    //  - If timeout occurs:
    //      - CancelIoEx is called.
    //      - Wait for the overlapped operation to complete/cancel before closing event/returning.
    //      - Any bytes that completed after the deadline are discarded (PurgeComm RX) to avoid them being reported in the next cycle.
    //      - ERROR_OPERATION_ABORTED originating from our CancelIoEx is treated as normal timeout (return 0).
    //      - Any other error is reported as -1.
    int readOnceUntil(uint8_t* buf, size_t bufSize, unsigned timeoutMs) {
        if (h_ == INVALID_HANDLE_VALUE) return -1;
        OVERLAPPED ov;
        ZeroMemory(&ov, sizeof(ov));
        ov.hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        if (!ov.hEvent) return -1;

        DWORD bytesRead = 0;
        BOOL ok = ReadFile(h_, buf, (DWORD)bufSize, &bytesRead, &ov);
        if (ok) {
            // completed immediately
            CloseHandle(ov.hEvent);
            return (int)bytesRead;
        }

        DWORD err = GetLastError();
        if (err != ERROR_IO_PENDING) {
            CloseHandle(ov.hEvent);
            return -1;
        }

        // wait for either completion or timeoutMs
        DWORD waitRes = WaitForSingleObject(ov.hEvent, timeoutMs);
        if (waitRes == WAIT_OBJECT_0) {
            // Read completed before deadline.
            DWORD got = 0;
            if (!GetOverlappedResult(h_, &ov, &got, FALSE)) {
                DWORD gle = GetLastError();
                CloseHandle(ov.hEvent);
                // If operation was aborted unexpectedly, treat as error unless it's our cancellation (should not happen here).
                if (gle == ERROR_OPERATION_ABORTED) return 0;
                return -1;
            }
            CloseHandle(ov.hEvent);
            return (int)got;
        }
        else if (waitRes == WAIT_TIMEOUT) {
            // Deadline reached: cancel the pending read, then wait for completion/cancellation
            CancelIoEx(h_, &ov);

            // WAIT: ensure completion/cancellation finishes before we destroy ov/event/buf.
            // Safety priority: wait indefinitely to let cancellation complete. This may delay the next TX if system is slow,
            // but avoids unsafe destruction of overlapped/event while driver still references them.
            DWORD waitRes2 = WaitForSingleObject(ov.hEvent, INFINITE);
            if (waitRes2 == WAIT_OBJECT_0) {
                // get final result without blocking
                DWORD got = 0;
                if (!GetOverlappedResult(h_, &ov, &got, FALSE)) {
                    DWORD gle = GetLastError();
                    CloseHandle(ov.hEvent);
                    // If operation was cancelled by us, treat as normal timeout and discard any bytes.
                    if (gle == ERROR_OPERATION_ABORTED) {
                        // Discard any stale RX data that may have arrived after the deadline but before cancel completed.
                        PurgeComm(h_, PURGE_RXCLEAR);
                        return 0;
                    }
                    // other error -> real serial error
                    PurgeComm(h_, PURGE_RXCLEAR);
                    return -1;
                }
                // If GetOverlappedResult succeeded after cancellation, we intentionally discard those bytes
                // because they arrived too late (they completed after deadline).
                (void)got; // ignore
                PurgeComm(h_, PURGE_RXCLEAR);
                CloseHandle(ov.hEvent);
                return 0;
            }
            else {
                // Wait failure; attempt to cancel and treat as error
                CancelIoEx(h_, &ov);
                CloseHandle(ov.hEvent);
                return -1;
            }
        }
        else {
            // Wait error
            CancelIoEx(h_, &ov);
            // wait for it to finish to be safe
            WaitForSingleObject(ov.hEvent, INFINITE);
            CloseHandle(ov.hEvent);
            return -1;
        }
    }

    bool discardRx() {
        if (h_ == INVALID_HANDLE_VALUE) return false;
        return PurgeComm(h_, PURGE_RXCLEAR) ? true : false;
    }

    bool valid() const { return h_ != INVALID_HANDLE_VALUE; }

private:
    HANDLE h_;
};

// ---------------- V1.05 protocol (TX with configurable realistic values) ----------------
static void buildPlcPacket(uint8_t out[28], uint32_t counter) {
    std::memset(out, 0, 28);
    out[0] = 0xC0; // STX
    out[4] = (uint8_t)(1u << 5); // position data valid

    // Simulate changing values based on counter for realistic movement
    // Slewing angle: 0–32766 decimal, scaling 359.9° ? 3599
    // Range: 0–3599 (0–359.9°), cycle through 0–180° continuously
    uint16_t slew_angle = (counter % 1800) * 2;  // 0–3599 over 1800 cycles
    out[5] = (uint8_t)(slew_angle & 0x7F);        // DB1-7 (bits 1-7)
    out[6] = (uint8_t)((slew_angle >> 8) & 0x7F); // DB9-15 (bits 9-15)
    if (slew_angle & 0x0001) out[23] |= 0x01; // DB0 ? byte 23 bit 0
    if (slew_angle & 0x0100) out[23] |= 0x02; // DB8 ? byte 23 bit 1

    // Slewing speed: 0–32766 decimal, scaling 0.1 1/min ? 100
    // Range: 0–100 (0–10 1/min), vary 0–50 1/min
    uint16_t slew_speed = (counter % 500) % 51;  // 0–50 over 500 cycles
    out[7] = (uint8_t)(slew_speed & 0x7F);        // DB1-7 (bits 1-7)
    out[8] = (uint8_t)((slew_speed >> 8) & 0x7F); // DB9-15 (bits 9-15)
    if (slew_speed & 0x0001) out[23] |= 0x04; // DB0 ? byte 23 bit 2
    if (slew_speed & 0x0100) out[23] |= 0x08; // DB8 ? byte 23 bit 3

    // Trolley distance: 0–32766 decimal, scaling 99.9 m ? 999
    // Range: 0–999 (0–99.9 m), vary 0–50 m
    uint16_t trolley_dist = (counter % 500) % 501;  // 0–500 over 500 cycles
    out[9] = (uint8_t)(trolley_dist & 0x7F);        // DB1-7 (bits 1-7)
    out[10] = (uint8_t)((trolley_dist >> 8) & 0x7F); // DB9-15 (bits 9-15)
    if (trolley_dist & 0x0001) out[23] |= 0x10; // DB0 ? byte 23 bit 4
    if (trolley_dist & 0x0100) out[23] |= 0x20; // DB8 ? byte 23 bit 5

    // Wind speed: 0–32766 decimal, scaling 10.4 m/s ? 104
    // Range: 0–104 (0–10.4 m/s), vary 0–52 m/s
    uint16_t wind = (counter % 520) % 105;  // 0–104 over 520 cycles
    out[17] = (uint8_t)(wind & 0x7F);        // DB1-7 (bits 1-7)
    out[18] = (uint8_t)((wind >> 8) & 0x7F); // DB9-15 (bits 9-15)
    if (wind & 0x0001) out[25] |= 0x01; // DB0 ? byte 25 bit 0
    if (wind & 0x0100) out[25] |= 0x02; // DB8 ? byte 25 bit 1

    out[21] = (uint8_t)(counter & 0x7F);
    out[22] = (uint8_t)((counter >> 7) & 0x7F);
    uint8_t xr = 0;
    for (int i = 1; i <= 25; ++i) xr ^= out[i];
    out[26] = (uint8_t)(xr & 0x7F);
    out[27] = 0x80; // ETX
}
// ---------------- Timestamp helper (monotonic, ms precision) ----------------
static steady_clock::time_point g_start_time;
static void formatElapsed(char* buf, size_t buflen, steady_clock::time_point tp) {
    using namespace std::chrono;
    auto elapsed = duration_cast<std::chrono::milliseconds>(tp - g_start_time).count();
    long long seconds = elapsed / 1000;
    int ms_part = (int)(elapsed % 1000);
    _snprintf_s(buf, buflen, _TRUNCATE, "%lld.%03d", seconds, ms_part);
}

// ---------------- Printing (TX/RX only) ----------------
static void printHexLine(const char* dir, steady_clock::time_point ts, const uint8_t* p, size_t n) {
    char tbuf[32];
    formatElapsed(tbuf, sizeof(tbuf), ts);
    std::printf("%s [%s]", dir, tbuf);
    for (size_t i = 0; i < n; ++i) std::printf(" %02X", p[i]);
    std::printf("\n");
    std::fflush(stdout);
}

// ---------------- main (single-threaded periodic loop with overlapped RX) ----------------
int main() {
    std::string comPort = readComPortFromIni("config.ini");
    std::printf("Liebherr PLC Simulator V1.05\n");
    std::printf("Using %s at 19200 8E1\n", comPort.c_str());
    std::fflush(stdout);

    SerialPort sp;
    if (!sp.open(comPort)) {
        std::printf("ERROR: failed to open %s (GetLastError=%lu)\n", comPort.c_str(), GetLastError());
        return 1;
    }

    // start monotonic reference
    g_start_time = steady_clock::now();

    const ms period(50);
    auto next_tx = steady_clock::now(); // first signal at now
    uint32_t counter = 0;

    for (;;) {
        // wait until the scheduled absolute TX deadline
        std::this_thread::sleep_until(next_tx);

        // 1) Send exactly one PLC frame and timestamp send-start
        uint8_t tx[28];
        buildPlcPacket(tx, counter++);
        auto tx_ts = steady_clock::now();

        if (!sp.writeSync(tx, sizeof(tx))) {
            std::printf("ERROR: serial write failed (GetLastError=%lu)\n", GetLastError());
            break;
        }

        // 2) Print TX line
        printHexLine("TX", tx_ts, tx, sizeof(tx));

        // 3) Listen for ONE RX response only, up to the remaining time before next deadline
        auto deadline = next_tx + period;
        auto now_after_tx = steady_clock::now();
        auto remain_ms_ll = std::chrono::duration_cast<ms>(deadline - now_after_tx).count();
        if (remain_ms_ll > 0) {
            unsigned remain_ms = (unsigned)remain_ms_ll;
            uint8_t rxbuf[256];
            int r = sp.readOnceUntil(rxbuf, sizeof(rxbuf), remain_ms);
            if (r < 0) {
                std::printf("ERROR: serial read failed (GetLastError=%lu)\n", GetLastError());
                break;
            }
            else if (r > 0) {
                auto rx_ts = steady_clock::now();
                printHexLine("RX", rx_ts, rxbuf, (size_t)r);
                // STOP listening for further RX this cycle; we did exactly one read
            }
            // r == 0 => timed out before deadline, print nothing (and stale RX already discarded inside readOnceUntil)
        }

        // 5) Advance to next absolute deadline (no catch-up bursts)
        next_tx += period;
        // loop repeats; sleep_until(next_tx) at top ensures absolute periodic schedule
    }

    sp.close();
    return 0;
}