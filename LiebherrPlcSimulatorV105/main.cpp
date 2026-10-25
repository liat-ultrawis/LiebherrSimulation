// Liebherr PLC Simulator V1.05
// GUI simulator: one TX every 130 ms (absolute steady_clock deadlines), ACS response received between transmissions.
// - Reads COM port from config.ini (COM_PORT=COMx)
// - Opens serial at 19200 8E1 (unchanged SerialPort implementation)
// - PLC frame built from GUI parameters (plc_protocol.h), checksum calculated automatically
// - Serial communication runs in a worker thread, independent of GUI rendering
// - Every TX/RX is logged as raw hex + decoded parameter names/values
// - "--selftest" runs the encoding/decoding/progression tests
//
// Protocol reference: protocol1.05-spec.md

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cwchar>
#include <string>
#include <fstream>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>

#include "plc_protocol.h"
#include "plc_selftest.h"

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

// ---------------- Timestamp helper (monotonic, ms precision) ----------------
static steady_clock::time_point g_start_time;
static void formatElapsed(char* buf, size_t buflen, steady_clock::time_point tp) {
    using namespace std::chrono;
    auto elapsed = duration_cast<std::chrono::milliseconds>(tp - g_start_time).count();
    long long seconds = elapsed / 1000;
    int ms_part = (int)(elapsed % 1000);
    _snprintf_s(buf, buflen, _TRUNCATE, "%lld.%03d", seconds, ms_part);
}

// ---------------- Log formatting (TX/RX raw hex line) ----------------
static std::string formatHexLine(const char* dir, steady_clock::time_point ts, const uint8_t* p, size_t n) {
    char tbuf[32];
    formatElapsed(tbuf, sizeof(tbuf), ts);
    std::string s = std::string(dir) + " [" + tbuf + "]";
    char hb[4];
    for (size_t i = 0; i < n; ++i) { _snprintf_s(hb, sizeof(hb), _TRUNCATE, " %02X", p[i]); s += hb; }
    return s;
}

// ---------------- Shared state worker <-> GUI ----------------
static const UINT WM_APP_STOPPED = WM_APP + 1;
static const size_t MAX_PENDING_LOG = 4000;

static std::mutex g_logMx;
static std::deque<std::string> g_log;
static std::atomic<bool> g_stopReq(false);
static HWND g_hwnd = nullptr;

static void pushLog(const std::string& s) {
    std::lock_guard<std::mutex> lk(g_logMx);
    g_log.push_back(s);
    while (g_log.size() > MAX_PENDING_LOG) g_log.pop_front();
}

// ---------------- Communication worker (TX every 130 ms, RX in between) ----------------
static void simWorker(plc105::SimConfig cfg, std::string comPort) {
    SerialPort sp;
    if (!sp.open(comPort)) {
        char b[128];
        _snprintf_s(b, sizeof(b), _TRUNCATE, "ERROR: failed to open %s (GetLastError=%lu)", comPort.c_str(), GetLastError());
        pushLog(b);
        PostMessageW(g_hwnd, WM_APP_STOPPED, 0, 0);
        return;
    }
    pushLog("Opened " + comPort + " at 19200 8E1, TX period 130 ms");

    g_start_time = steady_clock::now();
    cfg.reset();

    const ms period(130);
    auto next_tx = steady_clock::now();

    while (!g_stopReq.load()) {
        std::this_thread::sleep_until(next_tx);
        if (g_stopReq.load()) break;

        // 1) Build and send PLC frame from current parameters
        const plc105::PlcFrameData data = cfg.frameData();
        uint8_t tx[plc105::PLC_FRAME_LEN];
        plc105::buildPlcFrame(data, tx);
        auto tx_ts = steady_clock::now();
        if (!sp.writeSync(tx, sizeof(tx))) {
            char b[96];
            _snprintf_s(b, sizeof(b), _TRUNCATE, "ERROR: serial write failed (GetLastError=%lu)", GetLastError());
            pushLog(b);
            break;
        }
        pushLog(formatHexLine("TX", tx_ts, tx, sizeof(tx)) + "\n   " + plc105::formatTxFrameDecoded(tx, sizeof(tx)));
        cfg.advance();

        // 2) Receive ACS response until the next TX deadline (accumulate partial reads)
        const auto deadline = next_tx + period;
        uint8_t acc[64];
        size_t accN = 0;
        bool fatal = false;
        steady_clock::time_point rx_ts;
        for (;;) {
            auto remain = std::chrono::duration_cast<ms>(deadline - steady_clock::now()).count();
            if (remain <= 0 || accN >= sizeof(acc)) break;
            int r = sp.readOnceUntil(acc + accN, sizeof(acc) - accN, (unsigned)remain);
            if (r < 0) {
                char b[96];
                _snprintf_s(b, sizeof(b), _TRUNCATE, "ERROR: serial read failed (GetLastError=%lu)", GetLastError());
                pushLog(b);
                fatal = true;
                break;
            }
            if (r > 0) {
                if (accN == 0) rx_ts = steady_clock::now();
                accN += (size_t)r;
                // complete once a full 8-byte frame starting at STX is available
                const uint8_t* s = (const uint8_t*)std::memchr(acc, plc105::STX, accN);
                if (s && (size_t)(acc + accN - s) >= plc105::ACS_FRAME_LEN) break;
            }
        }
        if (fatal) break;

        if (accN == 0) {
            char tbuf[32];
            formatElapsed(tbuf, sizeof(tbuf), steady_clock::now());
            pushLog(std::string("RX [") + tbuf + "] no response");
        }
        else {
            std::string line = formatHexLine("RX", rx_ts, acc, accN);
            const uint8_t* s = (const uint8_t*)std::memchr(acc, plc105::STX, accN);
            if (!s) {
                line += "\n   INVALID: no STX (C0h) found";
            }
            else {
                const size_t off = (size_t)(s - acc);
                const size_t avail = accN - off;
                std::string err;
                if (off > 0) line += "\n   NOTE: " + std::to_string(off) + " byte(s) before STX ignored";
                if (avail < plc105::ACS_FRAME_LEN) {
                    line += "\n   INVALID: incomplete frame (" + std::to_string(avail) + " of 8 bytes)";
                }
                else {
                    if (!plc105::validateAcsFrame(s, plc105::ACS_FRAME_LEN, err))
                        line += "\n   INVALID: " + err;
                    if (avail > plc105::ACS_FRAME_LEN)
                        line += "\n   NOTE: " + std::to_string(avail - plc105::ACS_FRAME_LEN) + " extra byte(s) after frame";
                    line += "\n   " + plc105::formatAcsDecoded(s);
                }
            }
            pushLog(line);
        }

        // 3) Next absolute deadline; no catch-up bursts if we fell far behind
        next_tx += period;
        if (steady_clock::now() > next_tx + period) next_tx = steady_clock::now();
    }

    sp.close();
    pushLog("Port closed, simulation stopped");
    PostMessageW(g_hwnd, WM_APP_STOPPED, 0, 0);
}

// ---------------- GUI ----------------
enum : int {
    ID_START = 100, ID_STOP = 101, ID_STATUS = 102, ID_LOG = 103, ID_CLEAR = 104,
    ID_BIT_ON = 1000,
    ID_VAL_MIN = 2200, ID_VAL_MAX = 2300, ID_VAL_STEP = 2400,
};
static const UINT_PTR LOG_TIMER = 1;

// Value 08 is "free" in the spec: not shown in the GUI, always transmitted with this default raw value.
static const int VALUE08_INDEX = 8;
static const uint16_t VALUE08_DEFAULT = 0;
static const int NUM_GUI_VALUES = plc105::NUM_VALUES - 1; // Values 00..07

// Default Max per value: spec §6 "Format" reference values; Value 08 (free) uses the protocol maximum.
static const uint16_t kDefaultMax[plc105::NUM_VALUES] = { 3599, 100, 999, 900, 9999, 213, 104, 999, plc105::VALUE_MAX };

static std::string g_comPort;
static std::thread g_worker;
static bool g_running = false;
static HFONT g_font = nullptr;
static std::vector<HWND> g_paramCtrls;

static std::wstring toWide(const std::string& s) {
    if (s.empty()) return std::wstring();
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), nullptr, 0);
    std::wstring w((size_t)n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), &w[0], n);
    return w;
}

static HWND mkCtrl(HWND parent, const wchar_t* cls, const wchar_t* text, DWORD style, int x, int y, int w, int h, int id) {
    HWND c = CreateWindowExW(0, cls, text, WS_CHILD | WS_VISIBLE | style, x, y, w, h,
        parent, (HMENU)(INT_PTR)id, GetModuleHandleW(nullptr), nullptr);
    SendMessageW(c, WM_SETFONT, (WPARAM)g_font, TRUE);
    return c;
}

static std::wstring fmtEng(int i, uint16_t raw) {
    wchar_t b[32];
    const auto& vd = plc105::kPlcValues[i];
    swprintf(b, 32, L"%.*f", vd.decimals, raw / vd.divisor);
    return b;
}

static void createControls(HWND hwnd) {
    int y = 10;
    mkCtrl(hwnd, L"BUTTON", L"START", BS_PUSHBUTTON, 10, y, 90, 28, ID_START);
    mkCtrl(hwnd, L"BUTTON", L"STOP", BS_PUSHBUTTON, 110, y, 90, 28, ID_STOP);
    mkCtrl(hwnd, L"BUTTON", L"Clear log", BS_PUSHBUTTON, 210, y, 90, 28, ID_CLEAR);
    mkCtrl(hwnd, L"STATIC", L"", SS_LEFT, 310, y + 6, 330, 20, ID_STATUS);
    EnableWindow(GetDlgItem(hwnd, ID_STOP), FALSE);

    // Bit parameters
    y = 50;
    mkCtrl(hwnd, L"STATIC", L"PLC bit parameters (ON/OFF)", SS_LEFT, 10, y, 300, 18, 0);
    y += 20;
    for (int b = 0; b < plc105::NUM_PLC_BITS; ++b) {
        const auto& bd = plc105::kPlcBits[b];
        wchar_t label[160];
        swprintf(label, 160, L"B%d.%d  %s", bd.byteIdx, bd.bit, toWide(bd.desc).c_str());
        HWND on = mkCtrl(hwnd, L"BUTTON", label, BS_AUTOCHECKBOX, 10, y, 400, 20, ID_BIT_ON + b);
        g_paramCtrls.push_back(on);
        const bool defOn = (b == 1 || b == 11 || b == 12); // ControlSystemOn, PositionDataValid, BusConnectionOk
        SendMessageW(on, BM_SETCHECK, defOn ? BST_CHECKED : BST_UNCHECKED, 0);
        y += 22;
    }

    // Numeric parameters (engineering units, converted to raw by spec scaling)
    y += 12;
    mkCtrl(hwnd, L"STATIC", L"PLC numeric parameters", SS_LEFT, 10, y, 200, 18, 0);
    mkCtrl(hwnd, L"STATIC", L"Min", SS_LEFT, 300, y, 70, 18, 0);
    mkCtrl(hwnd, L"STATIC", L"Max", SS_LEFT, 380, y, 70, 18, 0);
    mkCtrl(hwnd, L"STATIC", L"Step", SS_LEFT, 460, y, 70, 18, 0);
    y += 20;
    for (int i = 0; i < NUM_GUI_VALUES; ++i) {
        const auto& vd = plc105::kPlcValues[i];
        std::wstring label = toWide(vd.name);
        if (vd.unit[0]) label += L" [" + toWide(vd.unit) + L"]";
        mkCtrl(hwnd, L"STATIC", label.c_str(), SS_LEFT, 10, y + 2, 280, 20, 0);
        g_paramCtrls.push_back(mkCtrl(hwnd, L"EDIT", fmtEng(i, 0).c_str(), WS_BORDER | ES_AUTOHSCROLL, 300, y, 70, 20, ID_VAL_MIN + i));
        g_paramCtrls.push_back(mkCtrl(hwnd, L"EDIT", fmtEng(i, kDefaultMax[i]).c_str(), WS_BORDER | ES_AUTOHSCROLL, 380, y, 70, 20, ID_VAL_MAX + i));
        g_paramCtrls.push_back(mkCtrl(hwnd, L"EDIT", fmtEng(i, 1).c_str(), WS_BORDER | ES_AUTOHSCROLL, 460, y, 70, 20, ID_VAL_STEP + i));
        y += 24;
    }
    mkCtrl(hwnd, L"STATIC",
        L"Values in engineering units (spec \u00A76 scaling). Raw range 0..32766. Each TX: Min \u2192 Max by Step, then restart at Min. Constant value: set Min = Max.",
        SS_LEFT, 10, y + 6, 620, 36, 0);

    HWND log = mkCtrl(hwnd, L"EDIT", L"",
        WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY, 650, 10, 600, 640, ID_LOG);
    SendMessageW(log, EM_SETLIMITTEXT, 0, 0);
}

static void setStatus(HWND hwnd, const std::wstring& s) { SetWindowTextW(GetDlgItem(hwnd, ID_STATUS), s.c_str()); }

static void setRunningUi(HWND hwnd, bool running) {
    for (HWND c : g_paramCtrls) EnableWindow(c, running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(hwnd, ID_START), running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(hwnd, ID_STOP), running ? TRUE : FALSE);
}

static bool readEngField(HWND hwnd, int id, int i, uint16_t& raw, std::wstring& err) {
    wchar_t buf[64];
    GetWindowTextW(GetDlgItem(hwnd, id), buf, 64);
    std::wstring s(buf);
    size_t a = s.find_first_not_of(L" \t");
    size_t b = s.find_last_not_of(L" \t");
    s = (a == std::wstring::npos) ? std::wstring() : s.substr(a, b - a + 1);
    if (s.empty()) { err = L"empty value"; return false; }
    if (s[0] == L'-') { err = L"\"" + s + L"\" is negative (allowed: >= 0)"; return false; }

    // Exact decimal parse: digits[.digits] only; no sign, exponent, hex, inf/nan.
    const auto& vd = plc105::kPlcValues[i];
    unsigned long long intPart = 0, frac = 0;
    int fracDigits = 0;
    bool anyDigit = false, dot = false, excessNonZero = false;
    for (wchar_t c : s) {
        if (c == L'.' && !dot) { dot = true; continue; }
        if (c < L'0' || c > L'9') { err = L"not a valid non-negative decimal number: \"" + s + L"\""; return false; }
        anyDigit = true;
        const unsigned d = (unsigned)(c - L'0');
        if (!dot) {
            intPart = intPart * 10 + d;
            if (intPart > 1000000ULL) { err = L"\"" + s + L"\" exceeds protocol maximum (raw 32766)"; return false; }
        }
        else if (fracDigits < vd.decimals) { frac = frac * 10 + d; ++fracDigits; }
        else if (d != 0) excessNonZero = true;
    }
    if (!anyDigit) { err = L"not a valid non-negative decimal number: \"" + s + L"\""; return false; }
    if (excessNonZero) {
        wchar_t m[160];
        swprintf(m, 160, L"\"%s\" is not representable: protocol resolution is %.*f", s.c_str(), vd.decimals, 1.0 / vd.divisor);
        err = m; return false;
    }
    unsigned long long scale = 1;
    for (int k = 0; k < vd.decimals; ++k) scale *= 10;
    for (int k = fracDigits; k < vd.decimals; ++k) frac *= 10;
    const unsigned long long r = intPart * scale + frac;
    if (r > plc105::VALUE_MAX) {
        wchar_t m[160];
        swprintf(m, 160, L"\"%s\" exceeds protocol maximum %.*f (raw 32766)", s.c_str(), vd.decimals, plc105::VALUE_MAX / vd.divisor);
        err = m; return false;
    }
    raw = (uint16_t)r;
    return true;
}

static bool readConfigFromUi(HWND hwnd, plc105::SimConfig& cfg, std::wstring& err) {
    for (int b = 0; b < plc105::NUM_PLC_BITS; ++b) {
        auto& p = cfg.bits[b];
        p.value = SendMessageW(GetDlgItem(hwnd, ID_BIT_ON + b), BM_GETCHECK, 0, 0) == BST_CHECKED;
        p.autoToggle = false;
        p.periodCycles = 1;
    }
    for (int i = 0; i < NUM_GUI_VALUES; ++i) {
        auto& p = cfg.values[i];
        const std::wstring nm = toWide(plc105::kPlcValues[i].name);
        std::wstring e;
        p.autoRun = true;
        if (!readEngField(hwnd, ID_VAL_MIN + i, i, p.minValue, e)) { err = nm + L" Min: " + e; return false; }
        if (!readEngField(hwnd, ID_VAL_MAX + i, i, p.maxValue, e)) { err = nm + L" Max: " + e; return false; }
        if (!readEngField(hwnd, ID_VAL_STEP + i, i, p.step, e)) { err = nm + L" Step: " + e; return false; }
        p.fixedValue = p.minValue;
        std::string ve;
        if (!p.validate(ve)) { err = nm + L": " + toWide(ve); return false; }
    }
    {
        auto& v8 = cfg.values[VALUE08_INDEX];
        v8.autoRun = false;
        v8.fixedValue = v8.minValue = v8.maxValue = VALUE08_DEFAULT;
        v8.step = 1;
    }
    return true;
}

static void drainLog(HWND hwnd) {
    std::deque<std::string> lines;
    {
        std::lock_guard<std::mutex> lk(g_logMx);
        lines.swap(g_log);
    }
    if (lines.empty()) return;
    std::string text;
    for (auto& l : lines) {
        for (char c : l) { if (c == '\n') text += "\r\n"; else text += c; }
        text += "\r\n";
    }
    HWND log = GetDlgItem(hwnd, ID_LOG);
    SendMessageW(log, WM_SETREDRAW, FALSE, 0);
    int len = GetWindowTextLengthW(log);
    if (len > 300000) { // keep the edit control small
        SendMessageW(log, EM_SETSEL, 0, len / 2);
        SendMessageW(log, EM_REPLACESEL, FALSE, (LPARAM)L"");
        len = GetWindowTextLengthW(log);
    }
    SendMessageW(log, EM_SETSEL, len, len);
    SendMessageW(log, EM_REPLACESEL, FALSE, (LPARAM)toWide(text).c_str());
    SendMessageW(log, WM_SETREDRAW, TRUE, 0);
    SendMessageW(log, EM_SCROLLCARET, 0, 0);
    InvalidateRect(log, nullptr, TRUE);
}

static LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE:
        g_hwnd = hwnd;
        createControls(hwnd);
        setStatus(hwnd, L"Stopped - port " + toWide(g_comPort) + L" (config.ini)");
        SetTimer(hwnd, LOG_TIMER, 100, nullptr);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case ID_START: {
            if (g_running) return 0;
            plc105::SimConfig cfg;
            std::wstring err;
            if (!readConfigFromUi(hwnd, cfg, err)) {
                MessageBoxW(hwnd, err.c_str(), L"Invalid parameter", MB_OK | MB_ICONWARNING);
                return 0;
            }
            g_stopReq = false;
            g_running = true;
            setRunningUi(hwnd, true);
            setStatus(hwnd, L"Running on " + toWide(g_comPort) + L" - TX every 130 ms");
            g_worker = std::thread(simWorker, cfg, g_comPort);
            return 0;
        }
        case ID_STOP:
            if (g_running) {
                g_stopReq = true;
                EnableWindow(GetDlgItem(hwnd, ID_STOP), FALSE);
                setStatus(hwnd, L"Stopping...");
            }
            return 0;
        case ID_CLEAR:
            SetWindowTextW(GetDlgItem(hwnd, ID_LOG), L"");
            return 0;
        }
        break;
    case WM_APP_STOPPED:
        if (g_worker.joinable()) g_worker.join();
        g_running = false;
        drainLog(hwnd);
        setRunningUi(hwnd, false);
        setStatus(hwnd, L"Stopped - port " + toWide(g_comPort) + L" (config.ini)");
        return 0;
    case WM_TIMER:
        if (wp == LOG_TIMER) drainLog(hwnd);
        return 0;
    case WM_CLOSE:
        g_stopReq = true;
        if (g_worker.joinable()) g_worker.join();
        DestroyWindow(hwnd);
        return 0;
    case WM_DESTROY:
        KillTimer(hwnd, LOG_TIMER);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

static int runGui() {
    HINSTANCE hi = GetModuleHandleW(nullptr);
    NONCLIENTMETRICSW ncm{};
    ncm.cbSize = sizeof(ncm);
    SystemParametersInfoW(SPI_GETNONCLIENTMETRICS, sizeof(ncm), &ncm, 0);
    g_font = CreateFontIndirectW(&ncm.lfMessageFont);

    WNDCLASSW wc{};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = hi;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wc.lpszClassName = L"LiebherrPlcSim105";
    RegisterClassW(&wc);

    HWND hwnd = CreateWindowExW(0, wc.lpszClassName, L"Liebherr PLC Simulator V1.05",
        WS_OVERLAPPEDWINDOW & ~(WS_MAXIMIZEBOX | WS_THICKFRAME),
        CW_USEDEFAULT, CW_USEDEFAULT, 1280, 700, nullptr, nullptr, hi, nullptr);
    if (!hwnd) return 1;
    ShowWindow(hwnd, SW_SHOW);

    MSG m;
    while (GetMessageW(&m, nullptr, 0, 0) > 0) {
        TranslateMessage(&m);
        DispatchMessageW(&m);
    }
    if (g_font) DeleteObject(g_font);
    return 0;
}

// ---------------- main ----------------
int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--selftest") == 0) return plc105::runSelfTests();

    g_comPort = readComPortFromIni("config.ini");

    // Hide the console window when it was created only for this process (launched from Explorer).
    DWORD procs[2];
    if (GetConsoleProcessList(procs, 2) == 1) FreeConsole();

    return runGui();
}