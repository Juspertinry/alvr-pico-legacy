#pragma once
// Lobby panel builders that sit on the UI kit: the streaming diagnostics overlay.
// Reads shared knobs from app_state; each builder emits pos.xyz+rgb geometry into
// a vertex vector (panel-local metres).
#include <vector>
#include "ui_kit.h"   // UiRect + widgets

// ---- streaming diagnostics overlay (bottom-middle, panel-local) ----
// page 1 = ALVR pipeline metrics; page 2 = system (CPU/GPU/heat) telemetry.
void buildDiagOverlay(std::vector<float> &v, int page);
// Low-battery pop-up card geometry (panel-local metres): a coloured background
// card + "LOW BATTERY" + "NN% REMAINING", tinted by severity (<=5 red, else amber).
void buildBatteryWarn(std::vector<float> &v, int pct);
