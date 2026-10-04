///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#ifndef slic3r_MixedColorPalette_hpp_
#define slic3r_MixedColorPalette_hpp_

#include "Color.hpp"
#include "FilamentOptics.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace Slic3r
{

struct ColorMixingRecipe;

struct MixedColor
{
    int id = 0;               // Stable palette ID (1-based, never reused)
    std::string name;         // User-friendly name
    ColorRGB target_color;    // What the user wants
    ColorRGB predicted_color; // What K-M predicts (computed, not serialized)
    float delta_e = 0.0f;     // Quality metric (computed, not serialized)

    // Repeating layer pattern: physical filament indices (0-based)
    // e.g., {0, 0, 2} = filament 0, filament 0, filament 2, repeat.
    // Coextruded entries store one slot per tool, in the order used for the wall shift.
    std::vector<int> layer_pattern;

    bool enabled = true;
    bool user_override = false; // User manually set the pattern
    bool auto_generated = false;
    // Walls of these tools are shifted apart on purpose, the way a wrong tool offset
    // makes a layer blend look coextruded. Rotation spins which color faces which way.
    bool coextruded = false;
    float coex_rotation_deg = 0.f;
};

class MixedColorPalette
{
public:
    const std::vector<MixedColor> &colors() const { return m_colors; }
    const MixedColor *find_by_id(int id) const;
    int num_enabled() const;

    // Auto-generate pairwise blends from physical filaments.
    // steps_per_pair: blend ratios to generate (e.g., 5 = 10%, 25%, 50%, 75%, 90%)
    // max_cycle_length: max layers in a blend pattern
    void auto_generate(const std::vector<FilamentOptics> &filaments, float layer_height, int steps_per_pair = 5,
                       int max_cycle_length = 8);

    // Add a user-defined color by target. Returns the new ID.
    int add_custom(const ColorRGB &target, const std::string &name = "");

    // Remove a color (marks as disabled, preserves ID stability)
    void remove(int id);

    // Clear all colors and reset ID counter
    void clear();

    // Rotation for one coextruded swatch, in degrees. Ignored for ordinary blends.
    void set_coex_rotation(size_t index, float degrees);

    // After the palette is rebuilt, put back rotations stored on painted recipes.
    // Match is by the ordered tool list, so a filament reload does not snap them to 0.
    void apply_saved_coex_rotations(const std::vector<ColorMixingRecipe> &recipes);

    // Recompute predicted colors and patterns for all entries.
    // Call when filament colors, TD, or layer height change.
    void recompute(const std::vector<FilamentOptics> &filaments, float layer_height, int max_cycle_length = 8);

    // How many physical filaments are referenced by the palette
    int max_physical_filament() const;

    // Number of pure-filament entries at the start of colors(). The first N entries of the
    // palette are the pure filaments (one per loaded filament); entries after that are blends.
    size_t num_pure_filaments() const { return m_num_pure_filaments; }

    // Find the palette entry best matching the requested 24-bit RGB color (0x00RRGGBB).
    // Uses luminance-weighted sRGB Euclidean distance (R*0.30, G*0.59, B*0.11) -- not Lab,
    // because Lab compresses the blue-purple region enough that blue ranks "closer" to magenta
    // than cyan, which mismatches what filament actually produces on the bed. No solid-preference
    // bias: if the user picked a blend, the slicer uses that blend rather than collapsing to a
    // nearby pure filament. Returns the index into colors(), or -1 if the palette is empty.
    int find_best_match(uint32_t rgb) const;

    // Hash for the slicer-side palette cache. Combines filament colors, TDs, and layer height.
    // Two calls with equal inputs always produce equal keys; differing inputs produce
    // different keys with high probability.
    static uint64_t cache_key(const std::vector<FilamentOptics> &filaments, float layer_height);

private:
    std::vector<MixedColor> m_colors;
    size_t m_num_pure_filaments = 0; // Count of pure filament entries at start
    int m_next_id = 1;

    // Generate a name from component filaments and ratio
    static std::string generate_name(int filament_a, int filament_b, float ratio_b);
};

} // namespace Slic3r

#endif // slic3r_MixedColorPalette_hpp_
