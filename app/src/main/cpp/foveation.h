#pragma once
// Read the server's foveation params from the (StreamingStarted-updated) ALVR
// settings JSON. Used at STREAMING_STARTED to bake the de-foveation pipeline and
// at REAL_TIME_CONFIG to detect a mid-session foveation change.

// Fills out[6] = {center_size_x, center_size_y, center_shift_x, center_shift_y,
//                 edge_ratio_x, edge_ratio_y}.
void readFoveationParams(float out[6]);
