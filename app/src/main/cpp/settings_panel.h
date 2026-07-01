#pragma once
// Single unified lobby SETTINGS window, now DATA-DRIVEN. The window chrome (panel
// background, sidebar tabs, header, close button, scroll + clip + scrollbar) lives
// here; the CONTENT of each category is a list of MenuItems (see menu_model.h) that
// this panel lays out, draws, and hit-tests generically. Categories are dynamic --
// the sidebar shows one tab per MenuCategory in the model, so adding a category (or
// an item) needs no layout math here. Panel-local metres, +x right / +y up.
#include <vector>
#include "ui_kit.h"
#include "menu_model.h"

// ---- session state (not persisted) ----------------------------------------
extern bool gSettingsOpen;   // panel shown
extern int  gSettingsCat;    // active category = index into settingsModel()

// The menu model (core categories + any extras). Built once on first use; item
// values are read live, so it never needs rebuilding. Use this for the category
// count, names, and content.
MenuModel &settingsModel();
int  settingsNumCats();                  // settingsModel().size()
void settingsClampCat();                 // keep gSettingsCat in range

// Per-category scroll offset (metres). Sized to the model; index by gSettingsCat.
extern std::vector<float> gSettingsScroll;
float &settingsScroll();                  // scroll for the active category

// ---- panel geometry (panel-local metres) ----------------------------------
constexpr float kSetPanelL = -0.66f, kSetPanelR =  0.62f;
constexpr float kSetPanelTop = 0.46f, kSetPanelBot = -0.46f;
constexpr float kSetHdrY   =  0.40f;
constexpr UiRect kSetClose = { 0.575f, 0.40f, 0.07f, 0.07f };
// content viewport (the scrolling region)
constexpr float kCtX0 = -0.42f, kCtX1 = 0.60f;
constexpr float kCtTop = 0.34f, kCtBot = -0.42f;
constexpr float kSetContentOffX = 0.09f;
constexpr float kSetViewportH = kCtTop - kCtBot;
// reference scrollbar (drawn only, never interactive)
constexpr float kSbX = 0.585f, kSbW = 0.012f;
// sidebar tabs: auto-stacked on the left, one per category.
UiRect settingsTabRect(int i);

// ---- measure / build ------------------------------------------------------
// Measure the active category, clamp its scroll, and return the offset that maps
// builder-local content coords into panel-local + the total content height.
void settingsMeasure(float &offX, float &offY, float &contentH);
// Emit the whole panel (bg + sidebar + header + close + scroll-clipped content +
// scrollbar). `content` highlights the hovered item; tabHover/closeHover the chrome.
void buildSettingsPanel(std::vector<float> &v, float offX, float offY, float contentH,
                        const MenuHover &content, int tabHover, bool closeHover);
