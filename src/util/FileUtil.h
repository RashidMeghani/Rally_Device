// Small shared SD-file helpers. Kept separate from NmeaUtil because these
// are about reading files, not about NMEA.
#pragma once

#include <FS.h>
#include <cstddef>

namespace FileUtil {

// Reads one line into `out` (NUL-terminated, CR/LF stripped). Returns
// false at EOF with nothing read. Lines longer than outLen are truncated
// rather than overflowing.
bool readLine(File& f, char* out, size_t outLen);

} // namespace FileUtil
