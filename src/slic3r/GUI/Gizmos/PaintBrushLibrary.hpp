///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#ifndef slic3r_PaintBrushLibrary_hpp_
#define slic3r_PaintBrushLibrary_hpp_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::GUI
{

// A brush tip mask plus the pressure curves that change how hard the dab is.
// Mask values are 0 (no paint) to 255 (full). Pressure curves are used only when
// the preset says that sensor is driven by pen pressure.
struct PaintBrushTip
{
    std::string name;
    int width{0};
    int height{0};
    std::vector<uint8_t> mask;
    // Fraction of the dab diameter between stamps along a stroke.
    float spacing{0.4f};
    float opacity_value{1.f};
    float flow_value{1.f};
    bool opacity_uses_pressure{false};
    bool flow_uses_pressure{false};
    bool size_uses_pressure{false};
    std::vector<std::pair<float, float>> opacity_curve;
    std::vector<std::pair<float, float>> flow_curve;
    std::vector<std::pair<float, float>> size_curve;

    // Bilinear sample. u grows to the right, v grows downward. Both are 0..1 inside the dab.
    float mask_at(float u, float v) const;
    // Mask times opacity and flow. `pressure` is 0..1; a mouse passes 1.
    float coverage(float u, float v, float pressure) const;
    // 1 when the preset does not scale with pressure.
    float size_factor(float pressure) const;
};

struct PaintBrushEntry
{
    std::string label;
    std::string key;
    // Empty inner means `path` is the brush file. Otherwise `path` is a zip bundle.
    std::string path;
    std::string inner;
};

// Krita presets and brush tips, plus GIMP brushes, from the usual install locations.
std::vector<PaintBrushEntry> scan_installed_paint_brushes();
bool load_paint_brush(const PaintBrushEntry &entry, PaintBrushTip &out);

} // namespace Slic3r::GUI

#endif
