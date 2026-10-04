///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#include "GLGizmoColorMixing.hpp"
#include "TriangleSelectorMmGui.hpp"

#include "libslic3r/Model.hpp"
#include "libslic3r/ColorMixer.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/NotificationManager.hpp"
#include "slic3r/GUI/format.hpp"
#include "slic3r/GUI/EventTypes.hpp"
#include "slic3r/GUI/EventBridge.hpp"

#include <wx/filedlg.h>
#include <wx/msgdlg.h>

#include <gdk-pixbuf/gdk-pixbuf.h>

#if SLIC3R_OPENGL_ES
#include <glad/gles2.h>
#else
#include <glad/gl.h>
#endif

namespace Slic3r::GUI
{

TriangleStateType GLGizmoColorMixing::get_left_button_state_type() const
{
    // Eraser mode overrides the left-click state to NONE so painting clears existing color
    // assignments back to "unpainted" rather than applying a new color.
    if (m_eraser_mode)
        return TriangleStateType::NONE;
    return TriangleStateType(m_first_selected_color_idx + 1);
}

TriangleStateType GLGizmoColorMixing::get_right_button_state_type() const
{
    // The pen's eraser end clears paint back to the volume's own filament.
    if (m_stylus_eraser || m_eraser_mode)
        return TriangleStateType::NONE;
    return TriangleStateType(m_second_selected_color_idx + 1);
}

bool GLGizmoColorMixing::on_init()
{
    m_shortcut_key = WXK_CONTROL_Y;

    m_desc["clipping_of_view"] = _u8L("Clipping of view") + ": ";
    m_desc["reset_direction"] = _u8L("Reset direction");
    m_desc["cursor_size"] = _u8L("Brush size") + ": ";
    m_desc["cursor_type"] = _u8L("Brush shape");
    m_desc["remove_all"] = _u8L("Clear all");
    m_desc["circle"] = _u8L("Circle");
    m_desc["sphere"] = _u8L("Sphere");
    m_desc["pointer"] = _u8L("Triangles");
    m_desc["tool_type"] = _u8L("Tool type");
    m_desc["tool_brush"] = _u8L("Brush");
    m_desc["tool_smart_fill"] = _u8L("Smart fill");
    m_desc["tool_bucket_fill"] = _u8L("Bucket fill");
    m_desc["tool_height_range"] = _u8L("Height range");
    m_desc["smart_fill_angle"] = _u8L("Smart fill angle");
    m_desc["bucket_fill_angle"] = _u8L("Bucket fill angle");
    m_desc["height_range_z_range"] = _u8L("Height range");
    m_desc["split_triangles"] = _u8L("Split triangles");

    // Color-coded direction hints
    m_desc["paint_caption"] = _u8L("Left mouse button") + ": ";
    m_desc["paint_action"] = _u8L("Paint color");
    m_desc["erase_caption"] =
#ifdef __APPLE__
        _u8L("Cmd + Left mouse button") + ": ";
#else
        _u8L("Ctrl + Left mouse button") + ": ";
#endif
    m_desc["erase_action"] = _u8L("Remove color");
    m_desc["alt_caption"] = _u8L("Alt + Mouse wheel") + ": ";
    m_desc["alt_brush"] = _u8L("Change brush size");
    m_desc["alt_fill"] = _u8L("Change angle");
    m_desc["alt_height_range"] = _u8L("Change height range");
    m_desc["pick_caption"] = _u8L("Alt + Left mouse button") + ": ";
    m_desc["pick_action"] = _u8L("Pick color");

    return true;
}

void GLGizmoColorMixing::on_opening()
{
    // Generate palette on first open or when filament settings / layer height changed
    if (m_filament_optics.empty() || palette_inputs_changed())
        init_palette();

    refresh_brush_catalog();
    m_old_mo_id = ObjectID();
}

void GLGizmoColorMixing::refresh_brush_catalog()
{
    const std::string selected = (m_brush_choice > 0 && m_brush_choice <= int(m_brush_entries.size()))
                                     ? m_brush_entries[size_t(m_brush_choice - 1)].key
                                     : std::string{};
    m_brush_entries = scan_installed_paint_brushes();
    m_brush_labels.clear();
    m_brush_labels.emplace_back(_u8L("Solid"));
    m_brush_choice = 0;
    for (size_t i = 0; i < m_brush_entries.size(); ++i)
    {
        m_brush_labels.push_back(m_brush_entries[i].label);
        if (!selected.empty() && m_brush_entries[i].key == selected)
            m_brush_choice = int(i) + 1;
    }
    if (m_brush_choice == 0)
        m_brush_tip.reset();
}

static Vec2d brush_world_to_screen(const Camera &camera, const Vec3d &world)
{
    const std::array<int, 4> &viewport = camera.get_viewport();
    const Eigen::Vector4d clip =
        camera.get_projection_matrix().matrix() *
        (camera.get_view_matrix().matrix() * Eigen::Vector4d(world.x(), world.y(), world.z(), 1.0));
    if (!(std::abs(clip.w()) > 1e-8))
        return Vec2d(1.0e10, 1.0e10);
    const double inv_w = 1.0 / clip.w();
    const double ndc_x = clip.x() * inv_w;
    const double ndc_y = clip.y() * inv_w;
    const double x = viewport[0] + (ndc_x + 1.0) * 0.5 * viewport[2];
    const double gl_y = (ndc_y + 1.0) * 0.5 * viewport[3];
    return Vec2d(x, double(viewport[3]) - gl_y);
}

static uint32_t pack_rgb(const ColorRGB &color)
{
    return (uint32_t(color.r_uchar()) << 16) | (uint32_t(color.g_uchar()) << 8) | uint32_t(color.b_uchar());
}

// Squared distance from a point to a triangle (Ericson, Real-Time Collision Detection).
static double point_triangle_distance_sq(const Vec3d &point, const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    const Vec3d ab = b - a;
    const Vec3d ac = c - a;
    const Vec3d ap = point - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
        return ap.squaredNorm();

    const Vec3d bp = point - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
        return bp.squaredNorm();

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        const double v = d1 / (d1 - d3);
        return (ap - ab * v).squaredNorm();
    }

    const Vec3d cp = point - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
        return cp.squaredNorm();

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        const double w = d2 / (d2 - d6);
        return (ap - ac * w).squaredNorm();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (bp - (c - b) * w).squaredNorm();
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return (ap - ab * v - ac * w).squaredNorm();
}

bool GLGizmoColorMixing::stamp_paint_brush(const Vec2d &mouse_position, bool second_color, bool clear_to_default,
                                           bool hard_clear)
{
    if (!m_brush_tip || m_triangle_selectors.empty() || m_modified_colors.empty())
        return false;

    const Camera &camera = m_parent.get_camera();
    const Selection &selection = m_parent.get_selection();
    ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
    if (mo == nullptr || selection.get_instance_idx() < 0 ||
        selection.get_instance_idx() >= int(mo->instances.size()))
        return false;
    const ModelInstance *mi = mo->instances[selection.get_instance_idx()];

    std::vector<Transform3d> trafos;
    std::vector<const ModelVolume *> volumes;
    for (const ModelVolume *mv : mo->volumes)
    {
        if (!mv->is_model_part())
            continue;
        trafos.emplace_back(mi->get_transformation().get_matrix() * mv->get_matrix());
        volumes.push_back(mv);
    }
    update_raycast_cache(mouse_position, camera, trafos);
    if (m_rr.mesh_id < 0 || m_rr.mesh_id >= int(m_triangle_selectors.size()) ||
        m_rr.mesh_id >= int(volumes.size()))
        return m_brush_stamp_valid;

    const PaintBrushTip &tip = *m_brush_tip;
    const float pressure = m_stylus_pressure >= 0.f ? m_stylus_pressure : 1.f;
    float radius = m_cursor_radius;
    if (m_stylus_pressure >= 0.f)
        radius = tip.size_uses_pressure ? m_cursor_radius * tip.size_factor(pressure) : effective_brush_radius();
    radius = std::max(radius, get_cursor_radius_min());
    const float radius_px = float(double(radius) * camera.get_zoom());
    if (!(radius_px > 1.f))
        return false;

    const float step = std::max(tip.spacing, 0.35f) * radius_px * 2.f;
    std::vector<Vec2d> dabs;
    if (!m_brush_stamp_valid)
    {
        dabs.push_back(mouse_position);
    }
    else
    {
        const Vec2d delta = mouse_position - m_brush_stamp_cursor;
        const double distance = delta.norm();
        if (distance < double(step) * 0.5)
            return true;
        const int count = std::min(8, std::max(1, int(distance / double(step))));
        for (int i = 1; i <= count; ++i)
            dabs.push_back(m_brush_stamp_cursor + delta * (double(i) / double(count)));
    }

    const int mesh_id = m_rr.mesh_id;
    const Transform3d &trafo = trafos[size_t(mesh_id)];
    const ModelVolume *volume = volumes[size_t(mesh_id)];
    const indexed_triangle_set &its = volume->mesh().its;
    const Vec3d hit_world = trafo * m_rr.hit.cast<double>();
    Vec3d toward_camera = camera.get_position() - hit_world;
    if (toward_camera.squaredNorm() < 1e-12)
        return false;
    toward_camera.normalize();

    const size_t paint_idx = second_color ? m_second_selected_color_idx : m_first_selected_color_idx;
    const ColorRGB paint_color =
        paint_idx < m_modified_colors.size()
            ? ColorRGB(m_modified_colors[paint_idx].r(), m_modified_colors[paint_idx].g(), m_modified_colors[paint_idx].b())
            : ColorRGB::WHITE();
    ColorRGB default_color = paint_color;
    if (const auto *painted = dynamic_cast<const TriangleSelectorMmGui *>(m_triangle_selectors[size_t(mesh_id)].get()))
        default_color = ColorRGB(painted->default_volume_color().r(), painted->default_volume_color().g(),
                                  painted->default_volume_color().b());
    const size_t default_extruder =
        ModelVolume::get_extruder_color_idx(*volume, std::max(1, int(m_original_colors.size())));

    auto color_of = [&](TriangleStateType state) -> ColorRGB
    {
        if (state == TriangleStateType::NONE)
            return default_color;
        const size_t idx = size_t(state) - 1;
        if (idx < m_modified_colors.size())
            return ColorRGB(m_modified_colors[idx].r(), m_modified_colors[idx].g(), m_modified_colors[idx].b());
        return default_color;
    };

    const double reach = double(radius) * 1.35;
    const double reach_sq = reach * reach;
    std::vector<unsigned char> facet_mask(its.indices.size(), 0);
    const int seed = int(m_rr.facet);
    for (int face = 0; face < int(its.indices.size()); ++face)
    {
        if (face == seed)
        {
            facet_mask[size_t(face)] = 1;
            continue;
        }
        const Vec3f &v0 = its.vertices[its.indices[size_t(face)][0]];
        const Vec3f &v1 = its.vertices[its.indices[size_t(face)][1]];
        const Vec3f &v2 = its.vertices[its.indices[size_t(face)][2]];
        const Vec3d w0 = trafo * v0.cast<double>();
        const Vec3d w1 = trafo * v1.cast<double>();
        const Vec3d w2 = trafo * v2.cast<double>();
        if (point_triangle_distance_sq(hit_world, w0, w1, w2) <= reach_sq)
            facet_mask[size_t(face)] = 1;
    }

    int assigned = 0;
    for (const Vec2d &dab : dabs)
    {
        assigned += m_triangle_selectors[size_t(mesh_id)]->paint_planar_image(
            trafo, toward_camera, 0.15f, std::max(radius / 8.f, 0.05f),
            [&](const Vec3d &world_point, TriangleStateType current) -> std::optional<TriangleStateType>
            {
                const Vec2d screen = brush_world_to_screen(camera, world_point);
                const float u = float((screen.x() - dab.x()) / double(radius_px * 2.f) + 0.5);
                const float v = float((screen.y() - dab.y()) / double(radius_px * 2.f) + 0.5);
                if (u < 0.f || u > 1.f || v < 0.f || v > 1.f)
                    return std::nullopt;
                const float alpha = tip.coverage(u, v, pressure);
                if (alpha < 0.03f)
                    return std::nullopt;
                // Shift clears in the shape of the tip. A firm eraser dab clears back to unpainted.
                if (hard_clear && alpha > 0.25f)
                    return TriangleStateType::NONE;
                if (clear_to_default && alpha > 0.92f)
                    return TriangleStateType::NONE;
                if (!clear_to_default && alpha > 0.97f)
                    return TriangleStateType(int(paint_idx) + 1);
                const ColorRGB under = color_of(current);
                const ColorRGB over = clear_to_default ? default_color : paint_color;
                const ColorRGB mixed = under * (1.f - alpha) + over * alpha;
                const int best = m_palette.find_best_match(pack_rgb(mixed));
                if (best < 0)
                    return std::nullopt;
                if (clear_to_default && size_t(best) == default_extruder)
                    return TriangleStateType::NONE;
                return TriangleStateType(best + 1);
            },
            &facet_mask);
    }
    (void) assigned;
    m_brush_stamp_cursor = mouse_position;
    m_brush_stamp_valid = true;
    m_triangle_selectors[size_t(mesh_id)]->request_update_render_data();
    m_parent.set_as_dirty();
    return true;
}

void GLGizmoColorMixing::on_shutdown()
{
    m_parent.use_slope(false);
    m_parent.toggle_model_objects_visibility(true);
}

bool GLGizmoColorMixing::load_mapped_image()
{
    wxFileDialog dialog(m_parent.get_wxglcanvas_parent(), _L("Select an image to map"), "", "",
                        "Image files (*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff)|*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff",
                        wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dialog.ShowModal() != wxID_OK)
        return false;

    // wx's JPEG loader longjmps on a bad file and then calls through an uninitialized
    // source, which is the crash. GdkPixbuf reports the failure instead.
    // utf8_str() does not own its bytes, so the path has to stay alive as a wxString.
    const wxString file_path = dialog.GetPath();
    const std::string utf8 = file_path.utf8_string();
    GError *error = nullptr;
    GdkPixbuf *pixels = gdk_pixbuf_new_from_file_at_scale(utf8.c_str(), 512, 512, TRUE, &error);
    if (pixels == nullptr)
    {
        const wxString message = error ? wxString::FromUTF8(error->message) : _L("Could not load that image.");
        if (error)
            g_error_free(error);
        wxMessageBox(message, _L("Map image"), wxOK | wxICON_ERROR);
        return false;
    }

    const int width = gdk_pixbuf_get_width(pixels);
    const int height = gdk_pixbuf_get_height(pixels);
    const int channels = gdk_pixbuf_get_n_channels(pixels);
    const int stride = gdk_pixbuf_get_rowstride(pixels);
    const bool usable = gdk_pixbuf_get_colorspace(pixels) == GDK_COLORSPACE_RGB && width > 0 && height > 0 &&
                        (channels == 3 || channels == 4) && gdk_pixbuf_get_bits_per_sample(pixels) == 8;
    if (!usable)
    {
        g_object_unref(pixels);
        wxMessageBox(_L("Could not load that image."), _L("Map image"), wxOK | wxICON_ERROR);
        return false;
    }

    MappedImage mapped;
    mapped.filename = dialog.GetFilename().ToStdString();
    mapped.width = width;
    mapped.height = height;
    const size_t count = size_t(width) * size_t(height);
    mapped.rgb.resize(count * 3);
    if (channels == 4)
        mapped.alpha.resize(count);
    const guint8 *src = gdk_pixbuf_get_pixels(pixels);
    for (int y = 0; y < height; ++y)
    {
        const guint8 *row = src + size_t(y) * size_t(stride);
        for (int x = 0; x < width; ++x)
        {
            const guint8 *pixel = row + size_t(x) * size_t(channels);
            const size_t index = size_t(y) * size_t(width) + size_t(x);
            mapped.rgb[index * 3] = pixel[0];
            mapped.rgb[index * 3 + 1] = pixel[1];
            mapped.rgb[index * 3 + 2] = pixel[2];
            if (channels == 4)
                mapped.alpha[index] = pixel[3];
        }
    }
    g_object_unref(pixels);
    m_mapped_image = std::move(mapped);
    m_parent.set_as_dirty();
    return true;
}

void GLGizmoColorMixing::apply_mapped_image(int mesh_id, int seed_facet)
{
    if (!m_mapped_image || m_triangle_selectors.empty())
        return;
    if (m_palette.colors().empty())
    {
        m_parent.get_notification_manager()->push_notification(
            _u8L("Load filaments before mapping an image. The picture is matched to those virtual colors."));
        return;
    }
    ModelObject *mo = m_c->selection_info()->model_object();
    if (mo == nullptr || mo->instances.empty())
        return;

    const MappedImage &image = *m_mapped_image;
    // Each pixel keeps the virtual color it matched, including 2-way and 3-way mixes.
    // The paint preview shows that blend. Slicing still lays down the mix's filaments.
    std::vector<int> quantized(size_t(image.width) * size_t(image.height), -1);
    for (int i = 0; i < image.width * image.height; ++i)
    {
        if (!image.alpha.empty() && image.alpha[size_t(i)] < 128)
            continue;
        const unsigned char *px = image.rgb.data() + size_t(i) * 3;
        const uint32_t rgb = (uint32_t(px[0]) << 16) | (uint32_t(px[1]) << 8) | uint32_t(px[2]);
        quantized[size_t(i)] = m_palette.find_best_match(rgb);
    }

    int instance_idx = m_parent.get_selection().get_instance_idx();
    if (instance_idx < 0 || instance_idx >= int(mo->instances.size()))
        instance_idx = 0;

    struct Projection
    {
        Vec3d face;
        Vec3d right;
        Vec3d up;
    };
    const Camera &camera = m_parent.get_camera();
    Projection projection;
    std::vector<unsigned char> facet_mask;
    const bool place_on_face = mesh_id >= 0 && seed_facet >= 0;
    float min_dot = 0.35f;
    if (place_on_face)
    {
        const ModelVolume *target = nullptr;
        int part_index = -1;
        for (const ModelVolume *mv : mo->volumes)
        {
            if (!mv->is_model_part())
                continue;
            ++part_index;
            if (part_index == mesh_id)
            {
                target = mv;
                break;
            }
        }
        if (target == nullptr)
        {
            m_parent.get_notification_manager()->push_notification(_u8L("Click a face of the model."));
            return;
        }
        const indexed_triangle_set &its = target->mesh().its;
        if (seed_facet >= int(its.indices.size()))
        {
            m_parent.get_notification_manager()->push_notification(_u8L("Click a face of the model."));
            return;
        }
        const Transform3d trafo = mo->instances[instance_idx]->get_transformation().get_matrix() * target->get_matrix();
        if (std::abs(trafo.linear().determinant()) < 1e-18)
            return;
        const Eigen::Matrix3d normal_matrix = trafo.linear().inverse().transpose();
        const std::vector<Vec3f> face_normals = its_face_normals(its);
        const auto world_normal = [&](size_t face) -> Vec3d
        {
            Vec3d normal = normal_matrix * face_normals[face].cast<double>();
            const double length = normal.norm();
            if (length <= 1e-12)
                return Vec3d::Zero();
            normal /= length;
            return normal;
        };
        const Vec3d seed_normal = world_normal(size_t(seed_facet));
        if (seed_normal.squaredNorm() < 0.5)
            return;

        const Vec3d camera_right = camera.get_dir_right();
        const Vec3d camera_up = camera.get_dir_up();
        Vec3d right = camera_right - seed_normal * seed_normal.dot(camera_right);
        if (right.squaredNorm() < 1e-8)
        {
            const Vec3d hint = std::abs(seed_normal.z()) < 0.9 ? Vec3d::UnitZ() : Vec3d::UnitX();
            right = hint.cross(seed_normal);
        }
        right.normalize();
        Vec3d up = camera_up - seed_normal * seed_normal.dot(camera_up);
        up -= right * up.dot(right);
        if (up.squaredNorm() < 1e-8)
            up = seed_normal.cross(right);
        up.normalize();
        projection = {seed_normal, right, up};

        // 35 degrees from the clicked face. A flat side is kept whole; a curve stays a local patch.
        const double min_align = std::cos(35.0 * 0.017453292519943295);
        const std::vector<Vec3i> neighbors = its_face_neighbors(its);
        facet_mask.assign(its.indices.size(), 0);
        std::vector<int> pending;
        pending.push_back(seed_facet);
        facet_mask[size_t(seed_facet)] = 1;
        while (!pending.empty())
        {
            const int face = pending.back();
            pending.pop_back();
            if (face < 0 || face >= int(neighbors.size()))
                continue;
            for (int corner = 0; corner < 3; ++corner)
            {
                const int next = neighbors[size_t(face)][corner];
                if (next < 0 || next >= int(facet_mask.size()) || facet_mask[size_t(next)] != 0)
                    continue;
                if (world_normal(size_t(next)).dot(seed_normal) < min_align)
                    continue;
                facet_mask[size_t(next)] = 1;
                pending.push_back(next);
            }
        }
        min_dot = 0.5f;
    }
    else
    {
        switch (m_image_projection)
        {
        case 2:
            projection = {Vec3d::UnitZ(), Vec3d::UnitX(), Vec3d::UnitY()};
            break;
        case 3:
            projection = {-Vec3d::UnitZ(), Vec3d::UnitX(), Vec3d::UnitY()};
            break;
        case 4:
            projection = {-Vec3d::UnitY(), Vec3d::UnitX(), Vec3d::UnitZ()};
            break;
        case 5:
            projection = {Vec3d::UnitY(), -Vec3d::UnitX(), Vec3d::UnitZ()};
            break;
        case 6:
            projection = {-Vec3d::UnitX(), Vec3d::UnitY(), Vec3d::UnitZ()};
            break;
        case 7:
            projection = {Vec3d::UnitX(), -Vec3d::UnitY(), Vec3d::UnitZ()};
            break;
        default:
            projection = {-camera.get_dir_forward(), camera.get_dir_right(), camera.get_dir_up()};
            break;
        }
        if (projection.face.squaredNorm() < 1e-12)
            return;
        projection.face.normalize();
        projection.right.normalize();
        projection.up.normalize();
    }

    const auto project_bounds = [&](const Transform3d &trafo, const indexed_triangle_set &its, double &umin,
                                     double &umax, double &vmin, double &vmax) -> bool
    {
        if (std::abs(trafo.linear().determinant()) < 1e-18)
            return false;
        const Eigen::Matrix3d normals = trafo.linear().inverse().transpose();
        const std::vector<Vec3f> face_normals = its_face_normals(its);
        umin = std::numeric_limits<double>::infinity();
        umax = -umin;
        vmin = umin;
        vmax = umax;
        bool any = false;
        for (size_t face = 0; face < its.indices.size(); ++face)
        {
            if (!facet_mask.empty() && (face >= facet_mask.size() || facet_mask[face] == 0))
                continue;
            Vec3d normal = normals * face_normals[face].cast<double>();
            const double length = normal.norm();
            if (!(length > 1e-12) || normal.dot(projection.face) / length < double(min_dot))
                continue;
            const Vec3i &tri = its.indices[face];
            for (int corner = 0; corner < 3; ++corner)
            {
                const Vec3d world = trafo * its.vertices[tri[corner]].cast<double>();
                const double u = world.dot(projection.right);
                const double v = world.dot(projection.up);
                umin = std::min(umin, u);
                umax = std::max(umax, u);
                vmin = std::min(vmin, v);
                vmax = std::max(vmax, v);
                any = true;
            }
        }
        return any && umax - umin > 1e-4 && vmax - vmin > 1e-4;
    };

    int selector_idx = -1;
    bool any_surface = false;
    for (const ModelVolume *mv : mo->volumes)
    {
        if (!mv->is_model_part())
            continue;
        ++selector_idx;
        if (selector_idx >= int(m_triangle_selectors.size()))
            break;
        if (place_on_face && selector_idx != mesh_id)
            continue;
        const Transform3d trafo = mo->instances[instance_idx]->get_transformation().get_matrix() * mv->get_matrix();
        double umin, umax, vmin, vmax;
        if (project_bounds(trafo, mv->mesh().its, umin, umax, vmin, vmax))
            any_surface = true;
    }
    if (!any_surface)
    {
        m_parent.get_notification_manager()->push_notification(
            place_on_face ? _u8L("That face is too small to place the image on.")
                          : _u8L("No surface faces that direction. Aim the view, or pick another side."));
        return;
    }

    m_place_image_armed = false;
    m_parent.take_gizmo_snapshot(_u8L("Map image"));
    int assigned = 0;
    selector_idx = -1;
    for (const ModelVolume *mv : mo->volumes)
    {
        if (!mv->is_model_part())
            continue;
        ++selector_idx;
        if (selector_idx >= int(m_triangle_selectors.size()))
            break;
        if (place_on_face && selector_idx != mesh_id)
            continue;
        const Transform3d trafo = mo->instances[instance_idx]->get_transformation().get_matrix() * mv->get_matrix();
        double umin, umax, vmin, vmax;
        if (!project_bounds(trafo, mv->mesh().its, umin, umax, vmin, vmax))
            continue;
        const double u_span = umax - umin;
        const double v_span = vmax - vmin;
        assigned += m_triangle_selectors[selector_idx]->paint_planar_image(
            trafo, projection.face, min_dot, m_image_detail_mm,
            [&](const Vec3d &world_point, TriangleStateType) -> std::optional<TriangleStateType>
            {
                const double u = (world_point.dot(projection.right) - umin) / u_span;
                const double v = (world_point.dot(projection.up) - vmin) / v_span;
                if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
                    return std::nullopt;
                const int x = std::clamp(int(std::lround(u * (image.width - 1))), 0, image.width - 1);
                const int y = std::clamp(int(std::lround((1.0 - v) * (image.height - 1))), 0, image.height - 1);
                const int palette_idx = quantized[size_t(y * image.width + x)];
                if (palette_idx < 0)
                    return std::nullopt;
                return TriangleStateType(palette_idx + 1);
            },
            facet_mask.empty() ? nullptr : &facet_mask);
        m_triangle_selectors[selector_idx]->request_update_render_data();
    }

    update_model_object();
    m_parent.set_as_dirty();
    if (assigned == 0)
        m_parent.get_notification_manager()->push_notification(
            _u8L("The image did not cover any facing surface. Transparent pixels are left unpainted."));
}

std::string GLGizmoColorMixing::on_get_name() const
{
    // Toolbar hover tooltip. Detailed help lives in the swatch tooltips inside the popup.
    return _u8L("Color mixing painting");
}

PainterGizmoType GLGizmoColorMixing::get_painter_type() const
{
    return PainterGizmoType::COLOR_MIXING;
}

void GLGizmoColorMixing::init_palette()
{
    if (!m_parent.preset_bundle())
        return;

    m_filament_optics = get_current_filament_optics();
    m_layer_height = get_current_layer_height();
    m_original_colors.clear();
    m_original_colors.reserve(m_filament_optics.size());
    for (const FilamentOptics &fo : m_filament_optics)
        m_original_colors.emplace_back(fo.color.r(), fo.color.g(), fo.color.b(), 1.0f);

    m_palette.clear();
    if (m_filament_optics.size() >= 2)
        m_palette.auto_generate(m_filament_optics, m_layer_height, 12, 10);
    if (const ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr)
    {
        for (const ModelVolume *mv : mo->volumes)
            if (mv->is_model_part() && !mv->color_mixing_palette.empty())
            {
                m_palette.apply_saved_coex_rotations(mv->color_mixing_palette);
                break;
            }
    }

    // Preserve the user's picked swatch indices across regeneration so a mid-paint filament
    // edit doesn't reset their brush selection. rebuild_modified_colors uses find_best_match
    // for recipe-bound positions, so the swatch the user had picked still shows the closest
    // achievable color at the same position. Only fall back to defaults if the saved indices
    // would be out of range after regeneration.
    const size_t saved_first = m_first_selected_color_idx;
    const size_t saved_second = m_second_selected_color_idx;

    rebuild_modified_colors();

    if (m_modified_colors.empty())
    {
        m_first_selected_color_idx = 0;
        m_second_selected_color_idx = 0;
    }
    else
    {
        m_first_selected_color_idx = (saved_first < m_modified_colors.size()) ? saved_first : 0;
        m_second_selected_color_idx = (saved_second < m_modified_colors.size())
                                          ? saved_second
                                          : std::min((size_t) 1, m_modified_colors.size() - 1);
    }
}

// Read per-extruder color + TD directly from the selected filament presets. Avoids
// PresetBundle::full_config(), which rebuilds a full merged DynamicPrintConfig on every call
// -- expensive enough to matter when polled from render paths. TD for an unconfigured slot
// falls back to DEFAULT_FILAMENT_TD.
std::vector<FilamentOptics> GLGizmoColorMixing::get_current_filament_optics() const
{
    std::vector<FilamentOptics> optics;
    if (!m_parent.preset_bundle())
        return optics;

    const auto &extruder_colors = m_parent.get_extruder_colors_from_plater_config();
    const auto &extruders_filaments = m_parent.preset_bundle()->extruders_filaments;

    optics.reserve(extruder_colors.size());
    for (size_t i = 0; i < extruder_colors.size(); ++i)
    {
        const auto &rgba = extruder_colors[i];
        float td = DEFAULT_FILAMENT_TD;
        if (i < extruders_filaments.size())
        {
            if (const Preset *preset = extruders_filaments[i].get_selected_preset())
            {
                const auto *td_opt = preset->config.option<ConfigOptionFloats>("filament_transmission_distance");
                if (td_opt && !td_opt->values.empty())
                    td = (float) td_opt->values[0];
            }
        }
        optics.emplace_back(ColorRGB(rgba.r(), rgba.g(), rgba.b()), td);
    }
    return optics;
}

// Active print preset's layer_height. Drives the dither-stack opacity simulation so the
// predicted_color in the picker matches what the printer lays down. Reads the edited preset
// config directly -- no full_config merge, cheap to call from event handlers. First-layer
// height is intentionally ignored: it's almost always thicker than the rest and treating it
// specially would only skew predictions for one layer out of hundreds.
float GLGizmoColorMixing::get_current_layer_height() const
{
    if (!m_parent.preset_bundle())
        return 0.2f;
    const auto *opt = m_parent.preset_bundle()->prints.get_edited_preset().config.option<ConfigOptionFloat>(
        "layer_height");
    return (opt && opt->value > 0.0) ? (float) opt->value : 0.2f;
}

bool GLGizmoColorMixing::palette_inputs_changed() const
{
    auto current = get_current_filament_optics();
    if (current.size() != m_filament_optics.size())
        return true;
    for (size_t i = 0; i < current.size(); ++i)
        if (current[i] != m_filament_optics[i])
            return true;
    if (std::abs(get_current_layer_height() - m_layer_height) > 1e-4f)
        return true;
    return false;
}

void GLGizmoColorMixing::data_changed(bool is_serializing)
{
    GLGizmoPainterBase::data_changed(is_serializing);

    if (m_state != On)
        return;

    if (palette_inputs_changed())
    {
        this->init_palette();
        this->init_model_triangle_selectors();
    }
}

// Eyedropper: Alt+Left / Alt+Right picks the state under the cursor into the first/second
// brush slot. Falls through to the base for every other action so the existing paint, line,
// eraser, and fill paths keep working.
bool GLGizmoColorMixing::gizmo_event(SLAGizmoEventType action, const Vec2d &mouse_position, bool shift_down,
                                     bool alt_down, bool control_down)
{
    const bool is_pick = alt_down && !shift_down && !control_down &&
                         (action == SLAGizmoEventType::LeftDown || action == SLAGizmoEventType::RightDown);
    // Placement is a separate one-shot. Loading a picture must not turn paint clicks into it.
    const bool place_image = m_place_image_armed && m_mapped_image && m_image_projection == 0 && !alt_down &&
                             !control_down && action == SLAGizmoEventType::LeftDown;
    if (place_image)
    {
        if (m_triangle_selectors.empty())
            return true;
        const Selection &selection = m_parent.get_selection();
        const ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
        if (!mo || selection.get_instance_idx() < 0)
            return true;
        const ModelInstance *mi = mo->instances[selection.get_instance_idx()];
        std::vector<Transform3d> trafo_matrices;
        for (const ModelVolume *mv : mo->volumes)
            if (mv->is_model_part())
                trafo_matrices.emplace_back(mi->get_transformation().get_matrix() * mv->get_matrix());
        update_raycast_cache(mouse_position, m_parent.get_camera(), trafo_matrices);
        if (m_rr.mesh_id >= 0 && m_rr.mesh_id < int(m_triangle_selectors.size()))
            apply_mapped_image(m_rr.mesh_id, int(m_rr.facet));
        return true;
    }
    if (action == SLAGizmoEventType::LeftUp || action == SLAGizmoEventType::RightUp)
        m_brush_stamp_valid = false;

    const bool brush_stamp = m_brush_tip && m_tool_type == ToolType::BRUSH &&
                             m_cursor_type != TriangleSelector::CursorType::POINTER && !control_down && !is_pick;
    if (brush_stamp &&
        (action == SLAGizmoEventType::LeftDown || action == SLAGizmoEventType::RightDown ||
         (action == SLAGizmoEventType::Dragging && pressed_button() != Button::None)))
    {
        if (action == SLAGizmoEventType::LeftDown)
            remember_pressed_button(Button::Left);
        else if (action == SLAGizmoEventType::RightDown)
            remember_pressed_button(Button::Right);
        const bool clear = shift_down || m_stylus_eraser || m_eraser_mode;
        const bool second = pressed_button() == Button::Right && !clear;
        if (stamp_paint_brush(mouse_position, second, clear, shift_down))
            return true;
    }

    if (!is_pick)
        return GLGizmoPainterBase::gizmo_event(action, mouse_position, shift_down, alt_down, control_down);

    if (m_triangle_selectors.empty())
        return false;

    const Selection &selection = m_parent.get_selection();
    const ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
    if (!mo || selection.get_instance_idx() < 0)
        return false;
    const ModelInstance *mi = mo->instances[selection.get_instance_idx()];

    std::vector<Transform3d> trafo_matrices;
    for (const ModelVolume *mv : mo->volumes)
        if (mv->is_model_part())
            trafo_matrices.emplace_back(mi->get_transformation().get_matrix() * mv->get_matrix());

    update_raycast_cache(mouse_position, m_parent.get_camera(), trafo_matrices);
    if (m_rr.mesh_id < 0 || m_rr.mesh_id >= (int) m_triangle_selectors.size())
        return false;

    const TriangleStateType state = m_triangle_selectors[m_rr.mesh_id]->get_triangle_leaf_state(int(m_rr.facet));
    if (state == TriangleStateType::NONE)
        // Nothing painted under the cursor; ignore the click so the user can try another spot.
        return true;

    const size_t idx = size_t(state) - 1;
    if (idx >= m_modified_colors.size())
        return true;

    if (action == SLAGizmoEventType::LeftDown)
        m_first_selected_color_idx = idx;
    else
        m_second_selected_color_idx = idx;

    m_parent.set_as_dirty();
    return true;
}

void GLGizmoColorMixing::init_model_triangle_selectors()
{
    const ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
    if (!mo)
        return;

    m_triangle_selectors.clear();

    for (const ModelVolume *mv : mo->volumes)
    {
        if (!mv->is_model_part())
            continue;

        const TriangleMesh *mesh = &mv->mesh();
        const int extruders_count = (int) m_original_colors.size();
        const size_t extruder_idx = ModelVolume::get_extruder_color_idx(*mv, extruders_count);
        ColorRGBA default_color = (extruder_idx < m_original_colors.size()) ? m_original_colors[extruder_idx]
                                                                            : ColorRGBA(0.5f, 0.5f, 0.5f, 1.0f);

        m_triangle_selectors.emplace_back(
            std::make_unique<TriangleSelectorMmGui>(*mesh, m_modified_colors, default_color));
        m_triangle_selectors.back()->deserialize(mv->color_mixing_facets.get_data(), false);
        m_triangle_selectors.back()->request_update_render_data();
    }
}

void GLGizmoColorMixing::update_from_model_object()
{
    wxBusyCursor wait;

    if (palette_inputs_changed())
        this->init_palette();

    // Defensive: seed recipes from the current palette for any painted volume that arrives
    // without a recipe table (foreign 3MF). find_best_match later snaps each painted region
    // to the closest achievable color.
    if (const ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr)
    {
        for (ModelVolume *mv : const_cast<ModelObject *>(mo)->volumes)
        {
            if (!mv->is_model_part())
                continue;
            if (mv->is_color_mixing_painted() && mv->color_mixing_palette.empty())
                ensure_color_mixing_recipes_for_used_states(m_palette, *mv);
        }
    }

    this->init_model_triangle_selectors();
}

void GLGizmoColorMixing::update_model_object() const
{
    bool updated = false;
    ModelObject *mo = m_c->selection_info()->model_object();
    int idx = -1;
    for (ModelVolume *mv : mo->volumes)
    {
        if (!mv->is_model_part())
            continue;
        ++idx;
        const bool facets_updated = mv->color_mixing_facets.set(*m_triangle_selectors[idx]);
        updated |= facets_updated;
        // After a paint commit, sync every currently-used state's recipe to the current
        // palette's predicted_color so recipe.rgb tracks the swatch the user just clicked.
        if (facets_updated && !mv->color_mixing_facets.empty())
            ensure_color_mixing_recipes_for_used_states(m_palette, *mv);
    }

    if (updated)
    {
        const ModelObjectPtrs &mos = m_parent.get_model()->objects;
        m_parent.event_poster()->postEvent(CanvasEventType::UpdateInfoItems,
                                           int(std::find(mos.begin(), mos.end(), mo) - mos.begin()));
        m_parent.event_poster()->postEvent(CanvasEventType::ScheduleBackgroundProcess);

        // First-paint welcome notification, once per app installation. The persistent banner in
        // the gizmo popup carries the same message at lower volume; this one fires on the user's
        // very first paint so they read it at least once before forming expectations.
        if (m_parent.app_config() && m_parent.app_config()->get("color_mixing_warning_shown") != "1")
        {
            m_parent.get_notification_manager()->push_notification(
                _u8L("Color mixing painted. Blends look best on near-vertical walls. Top surfaces, "
                     "steep overhangs, and the base layers may show a single filament rather than the "
                     "blended target. Base layer behavior is configurable in Print Settings -> Layer "
                     "height."));
            m_parent.app_config()->set("color_mixing_warning_shown", "1");
        }
    }
}

void GLGizmoColorMixing::render_painter_gizmo()
{
    const Selection &selection = m_parent.get_selection();

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));

    render_triangles(selection);
    m_c->object_clipper()->render_cut();
    m_c->instances_hider()->render_cut();
    render_cursor();

    glsafe(::glDisable(GL_BLEND));
}

void GLGizmoColorMixing::render_triangles(const Selection &selection) const
{
    ClippingPlaneDataWrapper clp_data = this->get_clipping_plane_data();
    auto *shader = m_parent.get_shader("mm_color_preview");
    if (!shader)
        shader = m_parent.get_shader("mm_gouraud"); // fallback
    if (!shader)
        return;
    shader->start_using();
    shader->set_uniform("clipping_plane", clp_data.clp_dataf);
    shader->set_uniform("z_range", clp_data.z_range);
    ScopeGuard guard(
        [shader]()
        {
            if (shader)
                shader->stop_using();
        });

    const ModelObject *mo = m_c->selection_info()->model_object();
    int mesh_id = -1;
    for (const ModelVolume *mv : mo->volumes)
    {
        if (!mv->is_model_part())
            continue;
        ++mesh_id;

        const Transform3d trafo_matrix =
            mo->instances[selection.get_instance_idx()]->get_transformation().get_matrix() * mv->get_matrix();

        const bool is_left_handed = trafo_matrix.matrix().determinant() < 0.0;
        if (is_left_handed)
            glsafe(::glFrontFace(GL_CW));

        const Camera &camera = m_parent.get_camera();
        const Transform3d &view_matrix = camera.get_view_matrix();
        shader->set_uniform("view_model_matrix", view_matrix * trafo_matrix);
        shader->set_uniform("projection_matrix", camera.get_projection_matrix());
        const Matrix3d view_normal_matrix = view_matrix.matrix().block(0, 0, 3, 3) *
                                            trafo_matrix.matrix().block(0, 0, 3, 3).inverse().transpose();
        shader->set_uniform("view_normal_matrix", view_normal_matrix);
        shader->set_uniform("volume_world_matrix", trafo_matrix);
        shader->set_uniform("volume_mirrored", is_left_handed);

        m_triangle_selectors[mesh_id]->set_shader_getters([this](const std::string &name)
                                                          { return m_parent.get_shader(name); },
                                                          [this]() { return m_parent.get_current_shader(); });
        m_triangle_selectors[mesh_id]->render(m_imgui, trafo_matrix, m_parent.get_camera());

        if (is_left_handed)
            glsafe(::glFrontFace(GL_CCW));
    }
}

ColorRGBA GLGizmoColorMixing::get_cursor_sphere_left_button_color() const
{
    if (m_first_selected_color_idx < m_modified_colors.size())
    {
        ColorRGBA color = m_modified_colors[m_first_selected_color_idx];
        color.a(0.25f);
        return color;
    }
    return {0.0f, 0.0f, 1.0f, 0.25f};
}

ColorRGBA GLGizmoColorMixing::get_cursor_sphere_right_button_color() const
{
    if (m_stylus_eraser)
    {
        if (m_rr.mesh_id >= 0 && m_rr.mesh_id < int(m_triangle_selectors.size()))
        {
            if (const auto *painted = dynamic_cast<const TriangleSelectorMmGui *>(
                    m_triangle_selectors[m_rr.mesh_id].get()))
            {
                ColorRGBA color = painted->default_volume_color();
                color.a(0.25f);
                return color;
            }
        }
        return {0.75f, 0.75f, 0.75f, 0.25f};
    }
    if (m_second_selected_color_idx < m_modified_colors.size())
    {
        ColorRGBA color = m_modified_colors[m_second_selected_color_idx];
        color.a(0.25f);
        return color;
    }
    return {1.0f, 0.0f, 0.0f, 0.25f};
}

void GLGizmoColorMixing::on_render_input_window(float x, float y, float bottom_limit)
{
    if (!m_c->selection_info()->model_object())
        return;

    // Anchor the popup's bottom-right corner at (x, bottom_limit) -- pivot (1.0, 1.0) tells
    // ImGui to treat the given point as the bottom-right of the window rather than the
    // top-left. bottom_limit is the viewport's usable bottom, which coincides with the
    // bottom of the vertical gizmo toolbar. This keeps the popup visually glued to the
    // color mixing icon (which lives at the bottom of the toolbar) regardless of popup
    // height. ImGui handles the "size not known yet" first-frame case because the pivot is
    // applied after auto-resize measures the window's content.
    set_side_flyout_pos(x, bottom_limit, 1.0f);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
                                    ImGuiWindowFlags_NoCollapse;
    ImGuiPureWrap::begin(get_name(), window_flags);

    // Scale the swatch size with the font so it stays proportional across DPI settings.
    // Hardcoding 24.0f made swatches render as tiny black squares on macOS Retina because
    // the rest of the popup scales with m_imgui->scaled() and ImGui's ColorButton fell
    // below a usable size. 1.5 * font_size ~= 24 px at typical Windows DPI.
    const float swatch_size = m_imgui->scaled(1.5f);
    const float slider_width = m_imgui->scaled(10.f);

    // --- Color-coded direction hints ---
    {
        float caption_max = 0.f;
        for (const std::string &t : {"paint", "erase", "pick", "alt"})
            caption_max = std::max(caption_max, ImGuiPureWrap::calc_text_size(m_desc[t + "_caption"]).x);
        caption_max += m_imgui->scaled(1.f);

        auto draw_hint = [&](const std::string &caption, const std::string &text)
        {
            ImGuiPureWrap::text_colored(ImGuiPureWrap::COL_ORANGE_LIGHT, caption);
            ImGui::SameLine(caption_max);
            ImGuiPureWrap::text(text);
        };

        draw_hint(m_desc.at("paint_caption"), m_desc.at("paint_action"));
        draw_hint(m_desc.at("erase_caption"), m_desc.at("erase_action"));
        draw_hint(m_desc.at("pick_caption"), m_desc.at("pick_action"));
        std::string alt_text = (m_tool_type == ToolType::BRUSH)          ? m_desc.at("alt_brush")
                               : (m_tool_type == ToolType::HEIGHT_RANGE) ? m_desc.at("alt_height_range")
                                                                         : m_desc.at("alt_fill");
        draw_hint(m_desc.at("alt_caption"), alt_text);
    }

    ImGui::Separator();

    // --- Limitation banner. Sets expectations every time the gizmo opens so users don't paint
    // a top face or shallow overhang and then think the dither is broken when it shows a single
    // filament instead of the blend. The base-layer behavior is configured separately in Print
    // Settings, so it isn't called out here. ---
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("%s", _u8L("Color blends on near-vertical walls by changing filament each layer "
                                   "and pulling a low-share layer's outer wall inward so the color "
                                   "underneath stays visible. A top face of a 2-way or 3-way color dithers "
                                   "those filaments along the infill lines and the walls around that top, "
                                   "in the same proportion. Steep overhangs may still show one filament.")
                                  .c_str());
    ImGui::PopTextWrapPos();

    ImGui::Separator();

    // --- Achievable color swatches (auto-fill width) ---
    if (!m_modified_colors.empty())
    {
        // Eraser toggle sits on the same line as the swatches header. Painting with eraser
        // active overrides the brush state to NONE so strokes clear color back to unpainted.
        ImGuiPureWrap::text(_u8L("Achievable Colors"));
        ImGui::SameLine();
        ImGui::Checkbox(_u8L("Eraser").c_str(), &m_eraser_mode);

        // Reserve space for the vertical scrollbar so the rightmost swatch isn't clipped.
        const float scrollbar_w = ImGui::GetStyle().ScrollbarSize;
        const float avail_width = ImGui::GetContentRegionAvail().x - scrollbar_w;
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const int cols = std::max(1, (int) ((avail_width + spacing) / (swatch_size + spacing)));

        float max_palette_height = swatch_size * 6.0f + ImGui::GetStyle().ItemSpacing.y * 6.0f;
        ImGui::BeginChild("##palette_scroll", ImVec2(0, max_palette_height), false);

        // Nudge the grid in by 2 px so the 2-px selection outline (drawn 1 px outside each
        // swatch) has room to render on the leftmost swatch in a row without getting clipped
        // by the child window's content edge.
        ImGui::Indent(2.0f);

        // Tier each entry by unique-filament count in its layer_pattern. 1 = pure filament,
        // 2 = 2-way blend, 3+ = 3-way blend. Recipe-only entries past the runtime palette
        // size fall back to tier 1 so they're always visible.
        auto entry_tier = [this](size_t idx) -> int
        {
            if (idx < m_palette.colors().size() && m_palette.colors()[idx].coextruded)
                return 4;
            if (idx >= m_palette.colors().size())
                return 1;
            const auto &pat = m_palette.colors()[idx].layer_pattern;
            if (pat.empty())
                return 1;
            std::vector<int> uniq(pat.begin(), pat.end());
            std::sort(uniq.begin(), uniq.end());
            uniq.erase(std::unique(uniq.begin(), uniq.end()), uniq.end());
            return std::min(3, (int) uniq.size());
        };

        // Bucket indices into the three tiers, preserving generation order within each tier.
        std::array<std::vector<size_t>, 4> tiers;
        for (size_t i = 0; i < m_modified_colors.size(); ++i)
            tiers[entry_tier(i) - 1].push_back(i);

        const std::array<std::string, 4> tier_labels = {_u8L("Single colors"), _u8L("2-way colors"),
                                                        _u8L("3-way colors"), _u8L("Coextruded")};

        bool first_section = true;
        for (int t = 0; t < 4; ++t)
        {
            if (tiers[t].empty())
                continue;
            if (!first_section)
                ImGui::Spacing();
            first_section = false;
            ImGui::TextDisabled("%s", tier_labels[t].c_str());

            for (size_t k = 0; k < tiers[t].size(); ++k)
            {
                const size_t i = tiers[t][k];
                const auto &rgba = m_modified_colors[i];
                ImVec4 col(rgba.r(), rgba.g(), rgba.b(), 1.0f);
                ImGui::PushID((int) i);

                if (m_first_selected_color_idx == i)
                {
                    ImVec2 cursor = ImGui::GetCursorScreenPos();
                    ImGui::GetWindowDrawList()->AddRect(ImVec2(cursor.x - 1, cursor.y - 1),
                                                        ImVec2(cursor.x + swatch_size + 1, cursor.y + swatch_size + 1),
                                                        IM_COL32(255, 255, 255, 255), 0.0f, 0, 2.0f);
                }

                if (ImGui::ColorButton("##swatch", col, ImGuiColorEditFlags_NoTooltip,
                                       ImVec2(swatch_size, swatch_size)))
                    m_first_selected_color_idx = i;

                if (ImGui::IsItemHovered() && i < m_palette.colors().size())
                {
                    const auto &mc = m_palette.colors()[i];
                    ImGui::BeginTooltip();
                    ImGui::Text("%s", mc.name.c_str());
                    if (mc.coextruded)
                        ImGui::TextDisabled("%s", _u8L("Shifts these tools' walls apart so the colors show together. "
                                                       "Rotation turns which color faces which way.")
                                                      .c_str());
                    else if (mc.layer_pattern.size() <= 1)
                        ImGui::TextDisabled("%s", _u8L("Single filament -- no swaps").c_str());
                    else
                        ImGui::TextDisabled("%s", GUI::format(_L("Repeats every %1% layers (%2% swaps per cycle)"),
                                                              (int) mc.layer_pattern.size(),
                                                              (int) mc.layer_pattern.size() - 1)
                                                      .c_str());
                    ImGui::EndTooltip();
                }

                ImGui::PopID();

                if (((int) (k + 1) % cols) != 0 && k < tiers[t].size() - 1)
                    ImGui::SameLine();
            }
        }

        ImGui::EndChild();

        if (m_first_selected_color_idx < m_palette.colors().size() &&
            m_palette.colors()[m_first_selected_color_idx].coextruded)
        {
            float rotation = m_palette.colors()[m_first_selected_color_idx].coex_rotation_deg;
            ImGui::AlignTextToFramePadding();
            ImGuiPureWrap::text(_u8L("Wall rotation"));
            ImGui::SameLine();
            ImGui::PushItemWidth(slider_width);
            if (m_imgui->slider_float("##coex_rotation", &rotation, 0.f, 360.f, "%.0f°", 1.f, true,
                                      _u8L("Which color faces which way")))
            {
                m_palette.set_coex_rotation(m_first_selected_color_idx, rotation);
                if (ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr)
                {
                    for (ModelVolume *mv : mo->volumes)
                    {
                        if (m_first_selected_color_idx >= mv->color_mixing_palette.size())
                            continue;
                        ColorMixingRecipe &rec = mv->color_mixing_palette[m_first_selected_color_idx];
                        if (rec.is_coextruded())
                            rec.coex_rotation_deg = m_palette.colors()[m_first_selected_color_idx].coex_rotation_deg;
                    }
                }
            }
            ImGui::PopItemWidth();
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextDisabled("%s", _u8L("Two colors shift along the line between them. A third color shifts off "
                                           "the midpoint of those two.")
                                        .c_str());
            ImGui::PopTextWrapPos();
        }
    }

    ImGui::Separator();

    // --- Tool type (label above, centered two-row layout) ---
    ImGuiPureWrap::text(m_desc.at("tool_type"));
    ImGui::NewLine();

    auto set_tool = [&](ToolType t)
    {
        m_tool_type = t;
        for (auto &ts : m_triangle_selectors)
        {
            ts->seed_fill_unselect_all_triangles();
            ts->request_update_render_data();
        }
    };

    // First row: Brush, Smart fill, Bucket fill (centered)
    {
        float row1_w = ImGuiPureWrap::calc_text_size(m_desc["tool_brush"]).x +
                       ImGuiPureWrap::calc_text_size(m_desc["tool_smart_fill"]).x +
                       ImGuiPureWrap::calc_text_size(m_desc["tool_bucket_fill"]).x + m_imgui->scaled(7.5f);
        float offset = (ImGui::GetContentRegionAvail().x - row1_w) * 0.5f;
        ImGui::SameLine(std::max(0.f, offset));
    }
    if (ImGuiPureWrap::radio_button(m_desc["tool_brush"], m_tool_type == ToolType::BRUSH))
        set_tool(ToolType::BRUSH);
    ImGui::SameLine();
    if (ImGuiPureWrap::radio_button(m_desc["tool_smart_fill"], m_tool_type == ToolType::SMART_FILL))
        set_tool(ToolType::SMART_FILL);
    ImGui::SameLine();
    if (ImGuiPureWrap::radio_button(m_desc["tool_bucket_fill"], m_tool_type == ToolType::BUCKET_FILL))
        set_tool(ToolType::BUCKET_FILL);

    // Second row: Height range (centered)
    {
        float hr_w = ImGuiPureWrap::calc_text_size(m_desc["tool_height_range"]).x + m_imgui->scaled(2.5f);
        float offset = (ImGui::GetContentRegionAvail().x - hr_w) * 0.5f;
        ImGui::NewLine();
        ImGui::SameLine(std::max(0.f, offset));
        if (ImGuiPureWrap::radio_button(m_desc["tool_height_range"], m_tool_type == ToolType::HEIGHT_RANGE))
            set_tool(ToolType::HEIGHT_RANGE);
    }

    ImGui::Separator();

    // --- Tool-specific settings ---
    if (m_tool_type == ToolType::BRUSH)
    {
        // Brush shape (label above, centered)
        ImGuiPureWrap::text(m_desc.at("cursor_type"));
        ImGui::NewLine();
        {
            float row_w = ImGuiPureWrap::calc_text_size(m_desc["sphere"]).x +
                          ImGuiPureWrap::calc_text_size(m_desc["circle"]).x +
                          ImGuiPureWrap::calc_text_size(m_desc["pointer"]).x + m_imgui->scaled(7.5f);
            float offset = (ImGui::GetContentRegionAvail().x - row_w) * 0.5f;
            ImGui::SameLine(std::max(0.f, offset));
        }
        if (ImGuiPureWrap::radio_button(m_desc["sphere"], m_cursor_type == TriangleSelector::CursorType::SPHERE))
            m_cursor_type = TriangleSelector::CursorType::SPHERE;
        ImGui::SameLine();
        if (ImGuiPureWrap::radio_button(m_desc["circle"], m_cursor_type == TriangleSelector::CursorType::CIRCLE))
            m_cursor_type = TriangleSelector::CursorType::CIRCLE;
        ImGui::SameLine();
        if (ImGuiPureWrap::radio_button(m_desc["pointer"], m_cursor_type == TriangleSelector::CursorType::POINTER))
            m_cursor_type = TriangleSelector::CursorType::POINTER;

        if (m_brush_labels.empty())
            refresh_brush_catalog();
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(_u8L("Brush tip"));
        ImGui::SameLine();
        int brush_choice = m_brush_choice;
        if (ImGuiPureWrap::combo("##paint_brush_tip", m_brush_labels, brush_choice, 0, 0.f, m_imgui->scaled(14.f)) &&
            brush_choice != m_brush_choice)
        {
            m_brush_choice = brush_choice;
            m_brush_stamp_valid = false;
            if (m_brush_choice <= 0 || m_brush_choice > int(m_brush_entries.size()))
            {
                m_brush_choice = 0;
                m_brush_tip.reset();
            }
            else
            {
                PaintBrushTip loaded;
                if (!load_paint_brush(m_brush_entries[size_t(m_brush_choice - 1)], loaded))
                {
                    m_parent.get_notification_manager()->push_notification(
                        _u8L("Could not read that brush. The solid brush is still available."));
                    m_brush_choice = 0;
                    m_brush_tip.reset();
                }
                else
                    m_brush_tip = std::move(loaded);
            }
        }
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", _u8L("A soft or pressure-faded dab blends over the color already there, then snaps "
                                       "to the nearest virtual filament.")
                                    .c_str());
        ImGui::PopTextWrapPos();

        m_imgui->disabled_begin(m_cursor_type != TriangleSelector::CursorType::SPHERE &&
                                m_cursor_type != TriangleSelector::CursorType::CIRCLE);

        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(m_desc.at("cursor_size"));
        ImGui::SameLine();
        ImGui::PushItemWidth(slider_width);
        m_imgui->slider_float("##cursor_radius", &m_cursor_radius, CursorRadiusMin, CursorRadiusMax, "%.2f", 1.0f, true,
                              _u8L("Alt + Mouse wheel"));

        ImGuiPureWrap::checkbox(m_desc["split_triangles"], m_triangle_splitting_enabled);

        m_imgui->disabled_end();

        ImGui::Separator();
    }
    else if (m_tool_type == ToolType::SMART_FILL || m_tool_type == ToolType::BUCKET_FILL)
    {
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(m_tool_type == ToolType::SMART_FILL ? m_desc.at("smart_fill_angle")
                                                                : m_desc.at("bucket_fill_angle"));
        ImGui::SameLine();
        ImGui::PushItemWidth(slider_width);
        float &fill_angle = (m_tool_type == ToolType::SMART_FILL) ? m_smart_fill_angle : m_bucket_fill_angle;
        if (m_imgui->slider_float("##fill_angle", &fill_angle, SmartFillAngleMin, SmartFillAngleMax, "%.f\xC2\xB0",
                                  1.0f, true, _u8L("Alt + Mouse wheel")))
        {
            for (auto &ts : m_triangle_selectors)
            {
                ts->seed_fill_unselect_all_triangles();
                ts->request_update_render_data();
            }
        }

        ImGui::Separator();
    }
    else if (m_tool_type == ToolType::HEIGHT_RANGE)
    {
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(m_desc.at("height_range_z_range"));
        ImGui::SameLine();
        ImGui::PushItemWidth(slider_width);
        if (m_imgui->slider_float("##height_range_z_range", &m_height_range_z_range, HeightRangeZRangeMin,
                                  HeightRangeZRangeMax, "%.2f mm", 1.0f, true, _u8L("Alt + Mouse wheel")))
        {
            for (auto &ts : m_triangle_selectors)
            {
                ts->seed_fill_unselect_all_triangles();
                ts->request_update_render_data();
            }
        }

        ImGui::Separator();
    }

    ImGui::Separator();
    ImGuiPureWrap::text(_u8L("Map image"));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("%s",
                        m_image_projection == 0
                            ? _u8L("Place on next click, then click a face. Brush painting stays "
                                   "available until you do. Each pixel becomes the closest virtual "
                                   "color, including 2-way and 3-way mixes. Connected surfaces that "
                                   "point the same way are included. Transparent parts of a PNG are "
                                   "left unpainted.")
                                  .c_str()
                            : _u8L("Projects a picture onto the surfaces that face the chosen side. "
                                   "Each pixel becomes the closest virtual color, including 2-way "
                                   "and 3-way mixes. Transparent parts of a PNG are left unpainted.")
                                  .c_str());
    ImGui::PopTextWrapPos();
    // The file dialog cannot open inside this draw. A mouse-up redraw is what called us,
    // and a modal dialog from here re-enters that redraw.
    if (ImGuiPureWrap::button(m_mapped_image ? _u8L("Change image") : _u8L("Load image")))
    {
        static bool load_pending = false;
        if (!load_pending)
        {
            load_pending = true;
            wxWindow *parent = m_parent.get_wxglcanvas_parent();
            parent->CallAfter([this]()
                              {
                                  this->load_mapped_image();
                                  load_pending = false;
                              });
        }
    }
    if (m_mapped_image)
    {
        ImGui::SameLine();
        ImGuiPureWrap::text(m_mapped_image->filename);
        const std::vector<std::string> projections = {_u8L("Clicked face"), _u8L("Current view"), _u8L("Top"),
                                                      _u8L("Bottom"),       _u8L("Front"),        _u8L("Back"),
                                                      _u8L("Left"),         _u8L("Right")};
        if (ImGuiPureWrap::combo(_u8L("Side"), projections, m_image_projection, 0, 0.0f, slider_width) &&
            m_image_projection != 0)
            m_place_image_armed = false;
        ImGui::AlignTextToFramePadding();
        ImGuiPureWrap::text(_u8L("Detail"));
        ImGui::SameLine();
        ImGui::PushItemWidth(slider_width);
        m_imgui->slider_float("##image_detail", &m_image_detail_mm, 0.3f, 4.0f, "%.2f mm", 1.0f, true,
                              _u8L("Smaller follows the picture more closely"));
        if (m_image_projection == 0)
        {
            if (ImGuiPureWrap::button(m_place_image_armed ? _u8L("Click the model… (cancel)")
                                                         : _u8L("Place on next click")))
                m_place_image_armed = !m_place_image_armed;
        }
        else if (ImGuiPureWrap::button(_u8L("Apply image")))
            apply_mapped_image();
    }

    ImGui::Separator();
    const float clip_left = std::max(ImGuiPureWrap::calc_text_size(m_desc.at("clipping_of_view")).x,
                                     ImGuiPureWrap::calc_text_size(m_desc.at("reset_direction")).x) +
                            m_imgui->scaled(1.5f);
    ImGui::AlignTextToFramePadding();
    ImGuiPureWrap::text(m_desc.at("clipping_of_view"));
    auto clp_dist = float(m_c->object_clipper()->get_position());
    ImGui::SameLine(clip_left);
    ImGui::PushItemWidth(slider_width);
    if (m_imgui->slider_float("##clp_dist", &clp_dist, 0.f, 1.f, "%.2f", 1.0f, true, _u8L("Ctrl + Mouse wheel")))
        m_c->object_clipper()->set_position_by_ratio(clp_dist, true);

    ImGui::Separator();
    if (ImGuiPureWrap::button(m_desc.at("remove_all")))
    {
        m_parent.take_gizmo_snapshot(_u8L("Clear color mixing"));
        ModelObject *mo = m_c->selection_info()->model_object();
        int idx = -1;
        for (ModelVolume *mv : mo->volumes)
        {
            if (!mv->is_model_part())
                continue;
            ++idx;
            m_triangle_selectors[idx]->reset();
            m_triangle_selectors[idx]->request_update_render_data();
            // Wipe the recipe table too, not just the facets. Keeping a stale recipe
            // table around after "Clear all" caused new paint to land on recipe slots
            // populated by a previous session's palette, which rendered as wrong colors
            // even though the gizmo's find_best_match view still looked correct.
            mv->color_mixing_palette.clear();
        }
        update_model_object();
        m_parent.set_as_dirty();
    }

    ImGuiPureWrap::end();
}

wxString GLGizmoColorMixing::handle_snapshot_action_name(bool control_down, Button button_down) const
{
    return control_down ? _L("Remove color mixing") : _L("Paint color mixing");
}

void GLGizmoColorMixing::rebuild_modified_colors()
{
    ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;

    // Determine the size needed. m_modified_colors must be large enough to index every painted
    // state across all volumes, otherwise TriangleSelectorMmGui silently falls back to color 0
    // (extruder 0) for out-of-range states, which manifests as paint rendering as the wrong
    // color when the palette shrinks (e.g., after removing a filament).
    size_t total = m_palette.colors().size();
    if (mo)
    {
        for (const ModelVolume *mv : mo->volumes)
        {
            if (!mv->is_model_part())
                continue;
            total = std::max(total, mv->color_mixing_palette.size());
        }
    }

    m_modified_colors.clear();
    m_modified_colors.reserve(total);

    // Find the first volume with a recipe table. All painted volumes share the same recipe
    // table contents (the gizmo mirrors the runtime palette into each), so picking any
    // painted volume's table is equivalent for swatch rendering.
    const std::vector<ColorMixingRecipe> *recipes = nullptr;
    if (mo)
    {
        for (const ModelVolume *mv : mo->volumes)
        {
            if (mv->is_model_part() && !mv->color_mixing_palette.empty())
            {
                recipes = &mv->color_mixing_palette;
                break;
            }
        }
    }

    for (size_t i = 0; i < total; ++i)
    {
        // Prefer the recipe-snapped color so painted intent survives palette resizes. If no
        // recipe exists for this index yet (pre-paint or picker-only slot), fall back to the
        // runtime palette entry.
        if (recipes && i < recipes->size())
        {
            const ColorMixingRecipe &rec = (*recipes)[i];
            if (rec.is_locked() && (int) rec.extruder_lock < (int) m_filament_optics.size() && rec.extruder_lock >= 0)
            {
                const ColorRGB &c = m_filament_optics[rec.extruder_lock].color;
                m_modified_colors.emplace_back(c.r(), c.g(), c.b(), 1.0f);
                continue;
            }
            int best = m_palette.find_best_match(rec.rgb);
            if (best >= 0 && best < (int) m_palette.colors().size())
            {
                const MixedColor &mc = m_palette.colors()[best];
                m_modified_colors.emplace_back(mc.predicted_color.r(), mc.predicted_color.g(), mc.predicted_color.b(),
                                               1.0f);
                continue;
            }
            // No palette match (palette empty) -- render the recipe rgb literally.
            float r = float((rec.rgb >> 16) & 0xFF) / 255.f;
            float g = float((rec.rgb >> 8) & 0xFF) / 255.f;
            float b = float(rec.rgb & 0xFF) / 255.f;
            m_modified_colors.emplace_back(r, g, b, 1.0f);
        }
        else if (i < m_palette.colors().size())
        {
            const MixedColor &mc = m_palette.colors()[i];
            m_modified_colors.emplace_back(mc.predicted_color.r(), mc.predicted_color.g(), mc.predicted_color.b(),
                                           1.0f);
        }
        else
        {
            // Defensive: unreachable under normal flow.
            m_modified_colors.emplace_back(0.5f, 0.5f, 0.5f, 1.0f);
        }
    }

    if (m_modified_colors.empty())
        m_modified_colors.emplace_back(0.5f, 0.5f, 0.5f, 1.0f);

    m_first_selected_color_idx = std::min(m_first_selected_color_idx, m_modified_colors.size() - 1);
    m_second_selected_color_idx = std::min(m_second_selected_color_idx, m_modified_colors.size() - 1);
}

} // namespace Slic3r::GUI
