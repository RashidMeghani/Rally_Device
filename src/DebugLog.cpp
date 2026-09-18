#include "../include/DebugLog.h"

namespace DebugLog {

// Start from the compiled-in defaults: boot logging has to work before
// ConfigManager has had a chance to read NVS. applyConfig() then replaces
// these with the stored (and web-settable) values.
bool serialEnabled = AppConst::DEBUG_SERIAL_DEFAULT;
bool serialRawNmea = AppConst::DEBUG_SERIAL_RAW_NMEA_DEFAULT;

void applyConfig(bool enabled, bool rawNmea) {
    serialEnabled = enabled;
    serialRawNmea = rawNmea;
}

} // namespace DebugLog
