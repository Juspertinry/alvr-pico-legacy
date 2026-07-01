#include "foveation.h"
#include "alvr_client_core.h"   // alvr_get_settings_json
#include "alvr_ext.h"           // alvr_get_settings_json_bounded (fork-only)
#include <cstdio>
#include <cstring>
#include <cstdlib>   // atof

// Parse a float that follows a JSON key, e.g. "center_size_x":0.66 -> 0.66.
// Lightweight (no JSON lib): finds "<key>": and atof's what follows.
// Single forward pass -- search from *cursor (the keys are serialized in struct
// order, so each is found just past the previous one), advancing *cursor on a hit.
// If a key isn't ahead of the cursor (server reordered/omitted it), fall back to a
// full scan from `json` so correctness never depends on the ordering.
static float jsonFloatFwd(const char *json, const char **cursor, const char *key, float dflt) {
    char pat[64];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(*cursor, pat);
    if (!p && *cursor != json) p = strstr(json, pat);   // fallback: full scan
    if (!p) return dflt;
    *cursor = p + n;
    return (float) atof(*cursor);
}

// Read the server's foveation params from the (StreamingStarted-updated) settings
// JSON into out[6] = {csx,csy,shx,shy,erx,ery}. `sj` is static (off-stack);
// parsed in one forward pass (see jsonFloatFwd). Event-time only (config changes),
// not a per-frame path.
void readFoveationParams(float out[6]) {
    static char sj[65536];
    alvr_get_settings_json_bounded(sj, sizeof(sj));   // bounded; NUL-terminated within cap
    const char *c = sj;
    out[0] = jsonFloatFwd(sj, &c, "center_size_x", 0.66f);
    out[1] = jsonFloatFwd(sj, &c, "center_size_y", 0.6f);
    out[2] = jsonFloatFwd(sj, &c, "center_shift_x", 0.4f);
    out[3] = jsonFloatFwd(sj, &c, "center_shift_y", 0.1f);
    out[4] = jsonFloatFwd(sj, &c, "edge_ratio_x", 6.0f);
    out[5] = jsonFloatFwd(sj, &c, "edge_ratio_y", 6.0f);
}
