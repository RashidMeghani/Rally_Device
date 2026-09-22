#include "FileUtil.h"

namespace FileUtil {

bool LineReader::refill() {
    if (_pos < _have) return true;      // still serving the current block
    const int n = _file.read(_buf, BUF_LEN);
    if (n <= 0) { _have = 0; _pos = 0; return false; }
    _have = (size_t)n;
    _pos = 0;
    return true;
}

bool LineReader::readLine(char* out, size_t outLen) {
    size_t i = 0;
    bool any = false;

    while (refill()) {
        const char c = (char)_buf[_pos++];
        any = true;
        if (c == '\n') break;
        if (c == '\r') continue;
        // Overlong lines are truncated, not overflowed: the caller's buffer
        // is sized for the format it expects, and a malformed file must not
        // be able to write past it.
        if (i < outLen - 1) out[i++] = c;
    }

    out[i] = '\0';
    return any;
}

} // namespace FileUtil
