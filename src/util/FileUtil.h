// Small shared SD-file helpers. Kept separate from NmeaUtil because these
// are about reading files, not about NMEA.
#pragma once

#include <FS.h>
#include <cstddef>
#include <cstdint>

namespace FileUtil {

// Line reader with its own block buffer.
//
// The obvious implementation - File::read() one byte at a time - is what
// this replaces, and it was costing real time on every path that streams a
// file. Each single-byte read goes through the FS object, the FAT layer and
// the SD driver, so the per-call overhead dwarfs the byte itself. Reading a
// sector at a time and serving the line from RAM turns thousands of those
// calls into one.
//
// It matters most where the most bytes are read:
//   - the full-route reacquisition scan after a reset, which streams EVERY
//     segment file (hundreds of KB on a long route);
//   - the ReferenceMap index build, which streams the whole recon capture;
//   - resolving each geofence point's heading at boot.
//
// BUF_LEN is one SD sector, so each refill maps to a single underlying read
// rather than straddling two. The buffer lives in the reader, so a reader
// is a ~512-byte object - fine on the stack for the short-lived scans here,
// but do not create one per loop iteration.
class LineReader {
public:
    explicit LineReader(File& f) : _file(f) {}

    // Reads one line into `out` (NUL-terminated, CR/LF stripped). Returns
    // false at EOF with nothing read. Lines longer than outLen are
    // truncated rather than overflowing.
    bool readLine(char* out, size_t outLen);

private:
    static constexpr size_t BUF_LEN = 512;

    File& _file;
    uint8_t _buf[BUF_LEN];
    size_t _have = 0;   // bytes currently in _buf
    size_t _pos = 0;    // next byte to serve

    // Refills the buffer. Returns false once the file is exhausted.
    bool refill();
};

} // namespace FileUtil
