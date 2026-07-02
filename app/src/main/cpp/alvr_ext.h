#pragma once
#include <cstdint>
// Fork-only additions to the ALVR client_core C API (NOT in the stock
// alvr_client_core.h). Declared separately so the upstream header stays
// pristine and regenerable. See c_api.rs / eq.rs in our client_core fork.

// set per-eye openness (0=closed..1=open) to be sent as eye blink with the next
// alvr_send_tracking.
extern "C" void alvr_send_eye_openness(float left_openness, float right_openness);
// 16-band audio EQ: push band gains (dB) to the DSP that runs in the audio
// playback callback. The lobby EQ panel owns gains + persistence.
extern "C" void alvr_set_eq_gains(const float *gains, int count);
extern "C" void alvr_set_eq_enabled(bool enabled);
// Client streaming diagnostics: fills 6 floats (ms)
// [total,decode,decoderQ,render,vsyncQ,frameInterval].
extern "C" bool alvr_get_client_stats(float *out);
// Like alvr_get_frame but BLOCKS up to timeout_ns for a freshly decoded frame
// (lets the render loop sleep on the decoder instead of busy-polling).
// Returns true + writes outputs if a frame arrived in time, false on timeout.
extern "C" bool alvr_get_frame_timeout(uint64_t *out_timestamp_ns,
                                       void **out_buffer_ptr,
                                       uint64_t timeout_ns);

// Manual-lobby power save: when paused, decoded NALs are discarded before the
// MediaCodec so the HW decoder idles (server keeps sending; only client decode
// stops). Resuming (paused=false) requests a fresh IDR so playback restarts from a
// clean keyframe. The decoder object stays alive across the toggle (no teardown).
extern "C" void alvr_set_decoder_paused(bool paused);

// BOUNDED variants of the stock buffer-filling getters. The upstream
// alvr_hud_message / alvr_get_settings_json / alvr_get_decoder_config copy the
// full internal (server-controlled) length into the caller buffer with no size
// argument -> a payload larger than the caller's fixed buffer overflows it.
// These take the buffer capacity and never write past it; each returns the FULL
// length so the caller can detect truncation (return > cap) and reject. The
// string variants always NUL-terminate within `cap`. ALWAYS prefer these.
extern "C" uint64_t alvr_hud_message_bounded(char *message_buffer, uint64_t cap);
extern "C" uint64_t alvr_get_settings_json_bounded(char *out_buffer, uint64_t cap);
extern "C" uint64_t alvr_get_decoder_config_bounded(char *out_buffer, uint64_t cap);
