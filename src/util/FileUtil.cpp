#include "FileUtil.h"

namespace FileUtil {

bool readLine(File& f, char* out, size_t outLen) {
    size_t i = 0;
    bool any = false;
    while (f.available()) {
        int c = f.read();
        if (c < 0) break;
        any = true;
        if (c == '\n') break;
        if (c == '\r') continue;
        if (i < outLen - 1) out[i++] = (char)c;
    }
    out[i] = '\0';
    return any;
}

} // namespace FileUtil
