#pragma once
// Data-driven lobby menu model. Instead of hand-placing every control with magic
// panel-local coordinates, a category is just a LIST of items; this engine
// auto-stacks them top-to-bottom and provides ONE
// generic builder / hit-test / input-apply over the list. Adding a control is one
// MenuItem pushed into a category -- no positions, no per-widget hit-test, no
// per-widget action switch. Reorder by moving lines; insert anywhere and the rest
// reflows.
//
// Two escape hatches keep the complex multi-widget panels working unchanged: an
// item of kind MK_CUSTOM owns its own sub-layout (used for the 16-band EQ and the
// input playground), reusing their existing geometry builders verbatim.
//
// All coords are panel-local metres (+x right / +y up), centred on x=0, laid out
// from a top Y downward. The settings panel translates + scroll-clips the result.
#include <vector>
#include <functional>
#include "ui_kit.h"

enum MenuKind { MK_TOGGLE, MK_STEPPER, MK_FADER, MK_BUTTON, MK_DROPDOWN, MK_CUSTOM };

// Which sub-part of an item the pointer is over. For MK_CUSTOM, `part` is item-
// defined (the custom hit/act lambdas agree on its meaning). `grab` = this hover
// arms a press/drag (vs. empty space, which scrolls the page).
struct MenuHover { int item = -1; int part = 0; bool grab = false; };

struct MenuItem {
    MenuKind kind = MK_TOGGLE;
    const char *label = "";

    // Live value binding. toggle: 0/1. fader: 0..1 fraction. stepper: raw value.
    std::function<float()>     get;
    std::function<void(float)> set;     // store the (already-clamped) value

    // Stepper / fader range.
    float vmin = 0.0f, vmax = 1.0f, vstep = 0.1f;

    // MK_DROPDOWN: the choices (get() returns the selected index, set() picks one).
    // dropOpen is runtime expand state (the model persists, so it lives here).
    std::vector<const char *> options;
    bool dropOpen = false;
    // Optional value readout (stepper shows it; fader may). Writes into buf.
    std::function<void(char *, int)> valueText;

    // Side-effect hooks (all optional).
    std::function<void()> onChange;     // after any committed value change (save / set a dirty flag)
    std::function<void()> onCommit;     // fader drag released (persist)
    std::function<void()> onClick;      // MK_BUTTON pressed

    // MK_CUSTOM escape hatch: the item is the SOLE occupant of its category and
    // positions itself in builder-local coords (reserves `customH` of height).
    float customH = 0.0f;
    std::function<void(std::vector<float> &, const MenuHover &)> cBuild;
    std::function<void(float cx, float cy, MenuHover &)>          cHit;
    std::function<void(const MenuHover &, bool click, bool grab, float cx, float cy)> cAct;
};

struct MenuCategory {
    const char *name = "";
    std::vector<MenuItem> items;
    bool custom = false;            // true => items[0] is a self-positioning MK_CUSTOM
};
using MenuModel = std::vector<MenuCategory>;

// ---- auto-layout ----------------------------------------------------------
// Top Y of the content stack (first row's top). Shared with the panel viewport.
constexpr float kMenuTopY = 0.30f;
float menuRowHeight(const MenuItem &it);
float menuRowTop(const MenuCategory &c, int i);   // top Y of row i (builder-local)
float menuContentH(const MenuCategory &c);         // total stack height

// ---- generic passes (all builder-local) -----------------------------------
void     menuBuild(std::vector<float> &v, const MenuCategory &c, const MenuHover &h);
MenuHover menuHit(const MenuCategory &c, float cx, float cy);
// Apply pointer input to the hovered item. Manages hold-to-repeat (steppers) and
// fader drag/commit internally. `click` = press edge, `grab` = held this frame.
void     menuApply(int catId, MenuCategory &c, const MenuHover &h,
                   bool click, bool grab, float cx, float cy);
// Quantized hash of every item's current value (for the render-on-demand sig).
unsigned menuValueSig(const MenuCategory &c);
