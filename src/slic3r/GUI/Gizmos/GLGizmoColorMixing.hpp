///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#ifndef slic3r_GLGizmoColorMixing_hpp_
#define slic3r_GLGizmoColorMixing_hpp_

#include "GLGizmoPainterBase.hpp"

#include "libslic3r/Color.hpp"
#include "libslic3r/FilamentOptics.hpp"
#include "libslic3r/MixedColorPalette.hpp"
#include "slic3r/GUI/I18N.hpp"

#include <optional>

namespace Slic3r::GUI
{

class GLGizmoColorMixing : public GLGizmoPainterBase
{
public:
    GLGizmoColorMixing(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id)
        : GLGizmoPainterBase(parent, icon_filename, sprite_id)
    {
    }
    ~GLGizmoColorMixing() override = default;

    void render_painter_gizmo() override;

    // Override data_changed to detect filament color changes.
    void data_changed(bool is_serializing) override;

    // Eyedropper: Alt+click picks the state under the cursor into the active brush slot
    // instead of painting, so the user can re-use a color they've already placed without
    // hunting for a matching swatch in the palette.
    bool gizmo_event(SLAGizmoEventType action, const Vec2d &mouse_position, bool shift_down, bool alt_down,
                     bool control_down) override;

protected:
    // Renders painted triangles via TriangleSelectorMmGui (mm_color_preview / mm_gouraud shader).
    void render_triangles(const Selection &selection) const override;

    void on_render_input_window(float x, float y, float bottom_limit) override;
    std::string on_get_name() const override;

    wxString handle_snapshot_action_name(bool control_down, Button button_down) const override;

    std::string get_gizmo_entering_text() const override { return _u8L("Entering color mixing"); }
    std::string get_gizmo_leaving_text() const override { return _u8L("Leaving color mixing"); }
    std::string get_action_snapshot_name() const override { return _u8L("Color mixing painting"); }

    ColorRGBA get_cursor_sphere_left_button_color() const override;
    ColorRGBA get_cursor_sphere_right_button_color() const override;

    TriangleStateType get_left_button_state_type() const override;
    TriangleStateType get_right_button_state_type() const override;

private:
    bool on_init() override;
    bool on_is_selectable() const override { return true; }

    void update_model_object() const override;
    void update_from_model_object() override;

    // Build per-volume TriangleSelectorMmGui instances from the model's color_mixing_facets.
    void init_model_triangle_selectors();

    void on_opening() override;
    void on_shutdown() override;
    PainterGizmoType get_painter_type() const override;

    // Picture projected onto the faces that point at the chosen side. Pixels are snapped
    // to the virtual-color palette when the user applies it.
    struct MappedImage
    {
        std::string filename;
        int width{0};
        int height{0};
        std::vector<unsigned char> rgb;
        std::vector<unsigned char> alpha;
    };
    bool load_mapped_image();
    // mesh_id < 0 paints every model part in the chosen direction.
    // A seed facet paints that face and the connected surfaces that point the same way.
    void apply_mapped_image(int mesh_id = -1, int seed_facet = -1);
    std::optional<MappedImage> m_mapped_image;
    int m_image_projection{0};
    float m_image_detail_mm{0.6f};
    // Armed only by "Place on next click". A loaded picture does not steal paint strokes.
    bool m_place_image_armed{false};

    // Palette management.
    void init_palette();
    std::vector<FilamentOptics> get_current_filament_optics() const;
    float get_current_layer_height() const;
    bool palette_inputs_changed() const;
    std::vector<FilamentOptics> m_filament_optics;
    float m_layer_height = 0.f;
    MixedColorPalette m_palette;

    // Color selection: indices into m_modified_colors for left/right brush.
    size_t m_first_selected_color_idx = 0;
    size_t m_second_selected_color_idx = 1;

    // Render colors handed to TriangleSelectorMmGui (one entry per painted state).
    std::vector<ColorRGBA> m_original_colors;
    std::vector<ColorRGBA> m_modified_colors;

    // Eraser toggle. When active, paint strokes override to NONE so they clear color back
    // to "unpainted" rather than applying a brush color.
    bool m_eraser_mode = false;

    // Rebuild m_modified_colors so that every painted state has a valid render color. Entries
    // for which a volume recipe exists are rendered via find_best_match against the current
    // runtime palette (preserves painted intent even when filaments change); entries without a
    // recipe fall back to the runtime palette entry at that index. Must cover the
    // maximum state value used across all volumes, otherwise out-of-range states render as
    // extruder 0 by TriangleSelectorMmGui's fallback.
    void rebuild_modified_colors();

    std::map<std::string, std::string> m_desc;
};

} // namespace Slic3r::GUI

#endif // slic3r_GLGizmoColorMixing_hpp_
