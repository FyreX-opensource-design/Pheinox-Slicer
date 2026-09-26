///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
#include "../GCode.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "../Geometry/ArcWelder.hpp"

#include "../ClipperUtils.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../Surface.hpp"

namespace Slic3r
{
namespace
{

enum FeatureKind : int
{
    fkNone = -1,
    fkExternal = 0,
    fkPerimeter,
    fkOverhang,
    fkInfill,
    fkSolid,
    fkTop,
    fkBridge,
    fkSupport,
    fkSupportInterface,
};

int feature_kind(const GCodeExtrusionRole role)
{
    switch (role)
    {
    case GCodeExtrusionRole::ExternalPerimeter:
    case GCodeExtrusionRole::Serpentine:
        return fkExternal;
    case GCodeExtrusionRole::Perimeter:
    case GCodeExtrusionRole::InterlockingPerimeter:
        return fkPerimeter;
    case GCodeExtrusionRole::OverhangPerimeter:
    case GCodeExtrusionRole::SerpentineOverhang:
        return fkOverhang;
    case GCodeExtrusionRole::InternalInfill:
    case GCodeExtrusionRole::GapFill:
        return fkInfill;
    case GCodeExtrusionRole::SolidInfill:
        return fkSolid;
    case GCodeExtrusionRole::TopSolidInfill:
        return fkTop;
    case GCodeExtrusionRole::BridgeInfill:
        return fkBridge;
    case GCodeExtrusionRole::SupportMaterial:
        return fkSupport;
    case GCodeExtrusionRole::SupportMaterialInterface:
        return fkSupportInterface;
    default:
        return fkNone;
    }
}

void append_user_gcode(std::string &out, const std::string &text)
{
    if (text.find_first_not_of(" \t\r\n") == std::string::npos)
        return;
    out += text;
    if (text.back() != '\n')
        out += '\n';
}

Point point_inside(const ExPolygon &ex)
{
    const ExPolygons inset = offset_ex(ex, -float(scale_(1.)));
    if (!inset.empty() && !inset.front().contour.points.empty())
    {
        const Point centroid = inset.front().contour.centroid();
        if (ex.contains(centroid))
            return centroid;
        return inset.front().contour.points.front();
    }
    if (!ex.contour.points.empty())
    {
        const Point centroid = ex.contour.centroid();
        if (ex.contains(centroid))
            return centroid;
        return ex.contour.points.front();
    }
    return Point(0, 0);
}

} // namespace

double GCodeGenerator::feature_flow_multiplier(const GCodeExtrusionRole role) const
{
    const double flow = this->feature_flow_of(feature_kind(role));
    return flow > 0. ? flow : 1.;
}

int GCodeGenerator::feature_temperature_of(const int feature) const
{
    if (feature < 0 || m_writer.extruder() == nullptr)
        return 0;
    const size_t tool = m_writer.extruder()->id();
    const ConfigOptionInts *option = nullptr;
    switch (feature)
    {
    case fkExternal:
        option = &m_config.feature_temp_external_perimeter;
        break;
    case fkPerimeter:
        option = &m_config.feature_temp_perimeter;
        break;
    case fkOverhang:
        option = &m_config.feature_temp_overhang_perimeter;
        break;
    case fkInfill:
        option = &m_config.feature_temp_infill;
        break;
    case fkSolid:
        option = &m_config.feature_temp_solid_infill;
        break;
    case fkTop:
        option = &m_config.feature_temp_top_solid_infill;
        break;
    case fkBridge:
        option = &m_config.feature_temp_bridge;
        break;
    case fkSupport:
        option = &m_config.feature_temp_support;
        break;
    case fkSupportInterface:
        option = &m_config.feature_temp_support_interface;
        break;
    default:
        return 0;
    }
    return option->get_at(tool);
}

double GCodeGenerator::feature_flow_of(const int feature) const
{
    if (feature < 0 || m_writer.extruder() == nullptr)
        return 0.;
    const size_t tool = m_writer.extruder()->id();
    const ConfigOptionFloats *option = nullptr;
    switch (feature)
    {
    case fkExternal:
        option = &m_config.feature_flow_external_perimeter;
        break;
    case fkPerimeter:
        option = &m_config.feature_flow_perimeter;
        break;
    case fkOverhang:
        option = &m_config.feature_flow_overhang_perimeter;
        break;
    case fkInfill:
        option = &m_config.feature_flow_infill;
        break;
    case fkSolid:
        option = &m_config.feature_flow_solid_infill;
        break;
    case fkTop:
        option = &m_config.feature_flow_top_solid_infill;
        break;
    case fkBridge:
        option = &m_config.feature_flow_bridge;
        break;
    case fkSupport:
        option = &m_config.feature_flow_support;
        break;
    case fkSupportInterface:
        option = &m_config.feature_flow_support_interface;
        break;
    default:
        return 0.;
    }
    return option->get_at(tool);
}

std::string GCodeGenerator::feature_custom_gcode(const int feature, const bool start) const
{
    if (feature < 0 || m_writer.extruder() == nullptr)
        return {};
    const size_t tool = m_writer.extruder()->id();
    const ConfigOptionStrings *option = nullptr;
    switch (feature)
    {
    case fkExternal:
        option = start ? &m_config.feature_gcode_start_external_perimeter : &m_config.feature_gcode_end_external_perimeter;
        break;
    case fkPerimeter:
        option = start ? &m_config.feature_gcode_start_perimeter : &m_config.feature_gcode_end_perimeter;
        break;
    case fkOverhang:
        option = start ? &m_config.feature_gcode_start_overhang_perimeter : &m_config.feature_gcode_end_overhang_perimeter;
        break;
    case fkInfill:
        option = start ? &m_config.feature_gcode_start_infill : &m_config.feature_gcode_end_infill;
        break;
    case fkSolid:
        option = start ? &m_config.feature_gcode_start_solid_infill : &m_config.feature_gcode_end_solid_infill;
        break;
    case fkTop:
        option = start ? &m_config.feature_gcode_start_top_solid_infill : &m_config.feature_gcode_end_top_solid_infill;
        break;
    case fkBridge:
        option = start ? &m_config.feature_gcode_start_bridge : &m_config.feature_gcode_end_bridge;
        break;
    case fkSupport:
        option = start ? &m_config.feature_gcode_start_support : &m_config.feature_gcode_end_support;
        break;
    case fkSupportInterface:
        option = start ? &m_config.feature_gcode_start_support_interface : &m_config.feature_gcode_end_support_interface;
        break;
    default:
        return {};
    }
    return option->get_at(tool);
}

int GCodeGenerator::feature_baseline_temperature() const
{
    if (m_writer.extruder() == nullptr)
        return 0;
    const size_t tool = m_writer.extruder()->id();
    if (this->on_first_layer())
    {
        const int first = m_config.first_layer_temperature.get_at(tool);
        if (first > 0)
            return first;
    }
    const int other = m_config.temperature.get_at(tool);
    if (other > 0)
        return other;
    return m_config.first_layer_temperature.get_at(tool);
}

Vec2d GCodeGenerator::bed_mm_to_gcode(const Vec2d &bed) const
{
    Vec2d offset = Vec2d::Zero();
    if (m_writer.extruder() != nullptr)
        offset = m_config.extruder_offset.get_at(m_writer.extruder()->id());
    return bed + m_origin - offset;
}

std::optional<Vec2d> GCodeGenerator::feature_infill_wait_xy() const
{
    if (m_layer == nullptr)
        return {};

    struct Candidate
    {
        const ExPolygon *ex = nullptr;
        double area = 0.;
        bool here = false;
    };
    const auto consider = [this](Candidate &best, const ExPolygon &ex)
    {
        if (ex.contour.points.empty())
            return;
        const double area = std::abs(ex.area());
        const bool here = this->last_position && ex.contains(*this->last_position);
        if (best.ex == nullptr || (here && !best.here) || (here == best.here && area > best.area))
            best = Candidate{&ex, area, here};
    };

    Candidate sparse;
    Candidate solid;
    for (const LayerRegion *region : m_layer->regions())
    {
        if (region == nullptr)
            continue;
        for (const Surface &surface : region->fill_surfaces().surfaces)
        {
            if (surface.is_bridge())
                continue;
            if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                consider(sparse, surface.expolygon);
            else if (surface.is_solid())
                consider(solid, surface.expolygon);
        }
    }

    const Candidate &chosen = sparse.ex != nullptr ? sparse : solid;
    if (chosen.ex == nullptr)
        return {};
    if (chosen.here && this->last_position)
        return this->point_to_gcode(*this->last_position);
    return this->point_to_gcode(point_inside(*chosen.ex));
}

std::string GCodeGenerator::emit_feature_temperature(const int temperature, const bool lock)
{
    if (m_writer.extruder() == nullptr || temperature <= 0)
    {
        if (!lock && m_feature_temp_locked)
        {
            m_feature_temp_locked = false;
            return ";FEATURE_TEMP_END\n";
        }
        return {};
    }

    if (m_feature_commanded_temp < 0)
    {
        if (m_feature_temp_baseline <= 0)
            m_feature_temp_baseline = this->feature_baseline_temperature();
        if (temperature == m_feature_temp_baseline)
        {
            m_feature_commanded_temp = temperature;
            return {};
        }
    }
    else if (temperature == m_feature_commanded_temp)
        return {};

    if (lock && m_feature_temp_baseline <= 0)
        m_feature_temp_baseline = this->feature_baseline_temperature();

    const size_t tool = m_writer.extruder()->id();
    const bool flow_temp = m_config.flow_temp_enabled.get_at(tool);
    const FeatureTempWait mode = m_config.feature_temp_wait.value;
    const bool do_wait = mode != FeatureTempWait::Off && (lock || !flow_temp);

    std::string out;
    if (lock && !m_feature_temp_locked)
    {
        out += ";FEATURE_TEMP_BEGIN\n";
        m_feature_temp_locked = true;
    }
    else if (!lock && m_feature_temp_locked)
    {
        out += ";FEATURE_TEMP_END\n";
        m_feature_temp_locked = false;
    }

    std::vector<Vec2d> path;
    const Vec3d start = m_writer.get_position();
    const bool can_travel = do_wait && this->last_position && start.z() > 0.;
    if (can_travel && mode == FeatureTempWait::PurgeBucket)
    {
        for (const Vec2d &waypoint : m_config.feature_purge_approach.values)
            path.push_back(this->bed_mm_to_gcode(waypoint));
        path.push_back(this->bed_mm_to_gcode(m_config.feature_purge_bucket.value));
        if (path.size() == 1 && (path.front() - start.head<2>()).norm() < 1.)
            path.clear();
    }
    else if (can_travel && mode == FeatureTempWait::OverInfill)
    {
        if (const std::optional<Vec2d> infill = this->feature_infill_wait_xy())
        {
            if ((*infill - start.head<2>()).norm() > 1.)
                path.emplace_back(*infill);
        }
    }

    const double lift = std::max(0., m_config.retract_lift.get_at(tool));
    const double z_hop = start.z() + lift;
    const auto travel = [this, lift, z_hop](const Vec2d &xy, const bool hop)
    {
        if (hop && lift > 0.)
            return m_writer.travel_to_xyz(Vec3d(xy.x(), xy.y(), z_hop), "feature temperature wait");
        return m_writer.travel_to_xy(xy, "feature temperature wait");
    };

    if (!path.empty())
    {
        if (m_writer.extruder()->retracted() <= 0.)
            out += m_writer.retract();
        for (const Vec2d &point : path)
            out += travel(point, true);
        if (lift > 0.)
            out += m_writer.travel_to_xyz(Vec3d(path.back().x(), path.back().y(), start.z()), "feature temperature wait");
        out += m_writer.unretract();
    }

    out += m_writer.set_temperature(static_cast<unsigned>(temperature), do_wait, static_cast<int>(tool));

    if (!path.empty() && mode == FeatureTempWait::PurgeBucket && m_config.feature_purge_length.value > 0.)
    {
        double purge = m_config.feature_purge_length.value;
        if (m_config.use_volumetric_e.value)
            purge *= m_writer.extruder()->filament_crossection();
        out += m_writer.extrude_to_xy(path.back(), purge, "purge bucket");
    }

    if (!path.empty())
    {
        out += m_writer.retract();
        if (lift > 0.)
            out += m_writer.travel_to_xyz(Vec3d(path.back().x(), path.back().y(), z_hop), "feature temperature wait");
        for (size_t i = path.size() - 1; i-- > 0;)
            out += travel(path[i], true);
        if (lift > 0.)
            out += m_writer.travel_to_xyz(start, "feature temperature wait");
        else
            out += m_writer.travel_to_xy(start.head<2>(), "feature temperature wait");
        out += m_writer.unretract();
    }

    m_feature_commanded_temp = temperature;
    return out;
}

GCodeGenerator::ConicalBand GCodeGenerator::conical_band(const Point &) const
{
    ConicalBand band;
    if (m_layer == nullptr || m_layer->object() == nullptr || m_config.spiral_vase.value)
        return band;

    const PrintObject *object = m_layer->object();
    ConicalSlicing mode = ConicalSlicing::Off;
    if (m_conical_region != nullptr)
        mode = m_conical_region->config().conical_slicing.value;
    else if (!object->conical_slicing_mixed())
        mode = object->conical_slicing_mode();
    if (mode == ConicalSlicing::Off)
        return band;

    // Walls sit outside the infill surfaces, so this must not require the point to
    // fall inside a slice. Skirt and brim are filtered by the caller.
    double step = m_layer->height;
    for (const LayerRegion *region : m_layer->regions())
    {
        if (region == nullptr)
            continue;
        const double wall_height = region->region().config().outer_wall_layer_height.value;
        if (wall_height > 0.)
        {
            step = wall_height;
            break;
        }
    }

    // Axis is the object's centered origin, the same origin the mesh warp used.
    // z_shift is the mesh sink plus the gap from the nozzle height down to the slice plane.
    band.axis = Vec2d::Zero();
    band.radius = object->conical_radius_mm();
    const double shift = (mode == ConicalSlicing::Inward && object->conical_slicing_mixed())
                             ? object->conical_z_shift_inward_mm()
                             : object->conical_z_shift_mm();
    band.z_shift = shift + (m_layer->slice_z - m_layer->print_z);
    band.z_step = std::max(0.05, 2. * step);
    band.sign = mode == ConicalSlicing::Inward ? 1. : -1.;
    band.active = true;
    return band;
}

double GCodeGenerator::conical_dz(const ConicalBand &band, const Point &point)
{
    if (!band.active)
        return 0.;
    const Vec2d at(unscale<double>(point.x()), unscale<double>(point.y()));
    // Radius in the original model. Slices were scaled back by cos(45°) after the warp.
    const double r = (at - band.axis).norm();
    // Inverse of the slice warp, so this layer lands on the original surface.
    // Outward warp was z' = z + r - z_shift. Inward warp was z' = z + (R - r) - z_shift.
    if (band.sign < 0.)
        return -r + band.z_shift;
    return r - band.radius + band.z_shift;
}

double GCodeGenerator::conical_z_offset_mm(const Point &point) const
{
    return conical_dz(this->conical_band(point), point);
}

bool GCodeGenerator::rewrite_conical_path(const Geometry::ArcWelder::Path &in, const double height_mm,
                                          Geometry::ArcWelder::Path &out) const
{
    if (in.size() < 2 || height_mm <= 1e-6)
        return false;
    const ConicalBand band = this->conical_band(in.front().point);
    if (!band.active)
        return false;

    out.clear();
    out.reserve(in.size());
    Geometry::ArcWelder::Segment first = in.front();
    first.radius = 0.f;
    first.height_fraction += float(conical_dz(band, first.point) / height_mm);
    out.push_back(first);

    for (size_t i = 1; i < in.size(); ++i)
    {
        const Geometry::ArcWelder::Segment &src_prev = in[i - 1];
        const Geometry::ArcWelder::Segment &src = in[i];
        const Vec2d from(double(src_prev.point.x()), double(src_prev.point.y()));
        const Vec2d to(double(src.point.x()), double(src.point.y()));
        const double len_mm = unscale<double>((to - from).norm());
        // At 45°, Z changes at most as fast as XY, so a piece this long changes Z by at most z_step.
        const int pieces = std::max(1, int(std::ceil(len_mm / band.z_step)));
        for (int k = 1; k <= pieces; ++k)
        {
            const double u = double(k) / double(pieces);
            const Vec2d at = from + (to - from) * u;
            Geometry::ArcWelder::Segment sample = src;
            sample.point = Point(coord_t(std::lround(at.x())), coord_t(std::lround(at.y())));
            sample.radius = 0.f;
            const float hf = float(src_prev.height_fraction + (src.height_fraction - src_prev.height_fraction) * u);
            sample.height_fraction = hf + float(conical_dz(band, sample.point) / height_mm);
            out.push_back(sample);
        }
    }
    return out.size() >= 2;
}

std::string GCodeGenerator::feature_transition(const GCodeExtrusionRole new_role)
{
    if (m_writer.extruder() == nullptr)
        return {};
    const int next = feature_kind(new_role);
    if (next == m_feature_index)
        return {};

    std::string out;
    append_user_gcode(out, this->feature_custom_gcode(m_feature_index, false));
    const int next_temp = this->feature_temperature_of(next);
    if (m_feature_temp_locked && next_temp <= 0)
    {
        const int baseline = m_feature_temp_baseline > 0 ? m_feature_temp_baseline : this->feature_baseline_temperature();
        out += this->emit_feature_temperature(baseline, false);
    }
    if (next_temp > 0)
        out += this->emit_feature_temperature(next_temp, true);
    append_user_gcode(out, this->feature_custom_gcode(next, true));
    m_feature_index = next;
    return out;
}

void GCodeGenerator::ensure_conical_band_grid()
{
    if (m_conical_grid.ready)
        return;
    const double first_h = std::max(0.05, m_config.first_layer_height.value);
    double lh = m_config.layer_height.value;
    if (m_layer != nullptr && m_layer->object() != nullptr)
        lh = m_layer->object()->config().layer_height.value;
    lh = std::max(0.05, lh);
    // The first slice is the first layer plus one normal layer. Later slices are two normal layers.
    m_conical_grid.layer_height = lh;
    m_conical_grid.first_top = m_config.z_offset.value + first_h + lh;
    m_conical_grid.span = 2. * lh;
    m_conical_grid.ready = true;
}

int GCodeGenerator::conical_band_index(double z) const
{
    if (!m_conical_grid.ready || z <= m_conical_grid.first_top + 1e-4)
        return 0;
    return 1 + int(std::floor((z - m_conical_grid.first_top - 1e-4) / m_conical_grid.span));
}

void GCodeGenerator::queue_conical_extrusion(const ExtrusionAttributes &attribs,
                                             const Geometry::ArcWelder::Path &path, const std::string_view description,
                                             const double speed, const EmitModifiers &emit_modifiers)
{
    if (path.size() < 2 || m_writer.extruder() == nullptr)
        return;
    this->ensure_conical_band_grid();

    const double bead = attribs.height > 1e-6 ? double(attribs.height) : double(m_last_height);
    Geometry::ArcWelder::Path zpath = path;
    for (Geometry::ArcWelder::Segment &seg : zpath)
    {
        const double z = double(m_last_layer_z) + (double(seg.height_fraction) - 1.) * bead;
        seg.height_fraction = float(z);
        seg.radius = 0.f;
    }

    auto band_top = [this](int band) {
        if (band <= 0)
            return m_conical_grid.first_top;
        return m_conical_grid.first_top + band * m_conical_grid.span;
    };
    auto push_unique = [](Geometry::ArcWelder::Path &dst, const Geometry::ArcWelder::Segment &seg)
    {
        if (!dst.empty() && dst.back().point == seg.point &&
            std::abs(dst.back().height_fraction - seg.height_fraction) < 1e-4f)
            return;
        dst.push_back(seg);
    };
    auto enqueue = [&](int band, Geometry::ArcWelder::Path piece)
    {
        if (piece.size() < 2)
            return;
        ConicalQueuedExtrusion item;
        item.band = band;
        item.support = attribs.role.is_support();
        item.object = m_layer ? m_layer->object() : nullptr;
        item.region = item.support ? nullptr : m_conical_region;
        const bool region_off =
            item.region == nullptr || item.region->config().conical_slicing.value == ConicalSlicing::Off;
        item.horizontal =
            item.support || (item.object != nullptr && item.object->conical_slicing_mixed() && region_off);
        item.layer = m_layer;
        item.instance_idx = m_current_instance.instance_idx;
        item.extruder_id = m_writer.extruder()->id();
        item.origin = m_origin;
        item.attributes = attribs;
        item.path = std::move(piece);
        item.z_key = std::numeric_limits<float>::infinity();
        for (const Geometry::ArcWelder::Segment &seg : item.path)
            item.z_key = std::min(item.z_key, seg.height_fraction);
        item.speed = speed;
        item.description = std::string(description);
        item.emit_modifiers = emit_modifiers;
        m_conical_queue.push_back(std::move(item));
    };

    Geometry::ArcWelder::Path current;
    int band = conical_band_index(zpath.front().height_fraction);
    current.push_back(zpath.front());
    for (size_t i = 1; i < zpath.size(); ++i)
    {
        const Geometry::ArcWelder::Segment &src = zpath[i];
        const double z1 = src.height_fraction;
        const int dest = conical_band_index(z1);
        if (dest == band)
        {
            push_unique(current, src);
            continue;
        }

        Geometry::ArcWelder::Segment from = current.back();
        double z_from = from.height_fraction;
        int guard = 0;
        while (band != dest && guard++ < 32)
        {
            const bool climbing = dest > band;
            const double edge = climbing ? band_top(band) : band_top(band - 1);
            const double dz = z1 - z_from;
            double u = std::abs(dz) < 1e-9 ? 1. : (edge - z_from) / dz;
            u = std::clamp(u, 0., 1.);
            Geometry::ArcWelder::Segment mid = src;
            const double x = double(from.point.x()) + (double(src.point.x()) - double(from.point.x())) * u;
            const double y = double(from.point.y()) + (double(src.point.y()) - double(from.point.y())) * u;
            mid.point = Point(coord_t(std::lround(x)), coord_t(std::lround(y)));
            mid.height_fraction = float(edge);
            mid.radius = 0.f;
            push_unique(current, mid);
            enqueue(band, std::move(current));
            current.clear();
            current.push_back(mid);
            band += climbing ? 1 : -1;
            z_from = edge;
            from = mid;
        }
        push_unique(current, src);
        band = dest;
    }
    enqueue(band, std::move(current));
}

std::string GCodeGenerator::flush_conical_bands(const Print &print)
{
    if (m_conical_queue.empty())
        return {};

    const bool mixed_horizontal = std::any_of(m_conical_queue.begin(), m_conical_queue.end(),
                                               [](const ConicalQueuedExtrusion &item)
                                               { return item.horizontal && !item.support; });
    // Within one layer height, keep the order paths were queued in. Sorting every fragment
    // by its exact Z breaks a cone wall into a travel between each bead. Paths a full layer
    // apart still print lower first, so the flat region is not buried under the cone.
    const double z_quantum = std::max(0.05, m_conical_grid.ready ? m_conical_grid.layer_height
                                                                 : m_config.layer_height.value);
    // Finish this band on one object before traveling to the next. Queue order otherwise
    // alternates objects on every source layer, so the nozzle hops across the plate.
    std::vector<std::pair<const PrintObject *, int>> object_order;
    object_order.reserve(8);
    for (const ConicalQueuedExtrusion &item : m_conical_queue)
    {
        const std::pair<const PrintObject *, int> key{item.object, item.instance_idx};
        if (std::find(object_order.begin(), object_order.end(), key) == object_order.end())
            object_order.push_back(key);
    }
    auto object_index = [&object_order](const ConicalQueuedExtrusion &item)
    {
        const std::pair<const PrintObject *, int> key{item.object, item.instance_idx};
        return int(std::find(object_order.begin(), object_order.end(), key) - object_order.begin());
    };
    std::stable_sort(m_conical_queue.begin(), m_conical_queue.end(),
                     [mixed_horizontal, z_quantum, &object_index](const ConicalQueuedExtrusion &a,
                                                                 const ConicalQueuedExtrusion &b)
                     {
                         if (a.band != b.band)
                             return a.band < b.band;
                         const int oa = object_index(a);
                         const int ob = object_index(b);
                         if (oa != ob)
                             return oa < ob;
                         if (mixed_horizontal)
                         {
                             const int qa = int(std::floor((double(a.z_key) + 1e-4) / z_quantum));
                             const int qb = int(std::floor((double(b.z_key) + 1e-4) / z_quantum));
                             if (qa != qb)
                                 return qa < qb;
                         }
                         // Supports at this height print before the object that sits on them.
                         return a.support && !b.support;
                     });

    std::string gcode;
    gcode.reserve(m_conical_queue.size() * 64);
    int current_band = std::numeric_limits<int>::min();
    const PrintObject *applied_object = nullptr;
    const PrintRegion *applied_region = nullptr;
    const double nominal_layer = m_conical_grid.ready ? m_conical_grid.layer_height : m_config.layer_height.value;

    auto begin_band = [&](int band, double zmin, double zmax)
    {
        double span = zmax - zmin;
        if (span < 1e-3)
            span = std::max(0.05, nominal_layer);
        m_last_layer_z = float(zmax);
        m_last_height = float(span);
        m_max_layer_z = std::max(m_max_layer_z, m_last_layer_z);

        if (!print.config().before_layer_gcode.value.empty() && band > 0)
        {
            DynamicConfig config;
            config.set_key_value("layer_num", new ConfigOptionInt(m_layer_index + 1));
            config.set_key_value("layer_z", new ConfigOptionFloat(zmax));
            config.set_key_value("max_layer_z", new ConfigOptionFloat(m_max_layer_z));
            gcode += this->placeholder_parser_process("before_layer_gcode", print.config().before_layer_gcode.value,
                                                      m_writer.extruder()->id(), &config) +
                     "\n";
        }

        gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) + ":" +
                 std::to_string(m_layer_index + 2) + "\n";
        gcode += std::string(";Z:") + float_to_string_decimal_point(zmax) + "\n";
        const double rounded_height = std::round(span * 10000.0) / 10000.0;
        gcode += std::string(";") + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) +
                 float_to_string_decimal_point(rounded_height) + "\n";
        const unsigned int progress_total = std::max(m_layer_count, (unsigned int) (m_layer_index + 2));
        gcode += m_writer.update_progress(++m_layer_index, progress_total);

        if (!print.config().layer_gcode.value.empty() && band > 0)
        {
            DynamicConfig config;
            config.set_key_value("layer_num", new ConfigOptionInt(m_layer_index));
            config.set_key_value("layer_z", new ConfigOptionFloat(zmax));
            config.set_key_value("max_layer_z", new ConfigOptionFloat(m_max_layer_z));
            gcode += this->placeholder_parser_process("layer_gcode", print.config().layer_gcode.value,
                                                      m_writer.extruder()->id(), &config) +
                     "\n";
        }

        if (band > 0 && !m_second_layer_things_done)
        {
            for (const Extruder &extruder : m_writer.extruders())
            {
                if (print.config().single_extruder_multi_material.value || m_ooze_prevention.enable)
                {
                    if (extruder.id() != m_writer.extruder()->id())
                        continue;
                }
                const int temperature = print.config().temperature.get_at(extruder.id());
                if (temperature > 0 && temperature != print.config().first_layer_temperature.get_at(extruder.id()))
                    gcode += m_writer.set_temperature(temperature, false, extruder.id());
            }
            const int num_extruders = int(print.config().nozzle_diameter.values.size());
            const int bed_temperature_extruder = print.config().bed_temperature_extruder;
            const bool use_first_extruder = bed_temperature_extruder <= 0 || bed_temperature_extruder > num_extruders;
            const int bed_ext_id = use_first_extruder ? int(m_writer.extruder()->id()) : bed_temperature_extruder - 1;
            const int bed_temperature = print.config().bed_temperature.get_at(bed_ext_id);
            if (bed_temperature > 0 && bed_temperature != print.config().first_layer_bed_temperature.get_at(bed_ext_id))
                gcode += m_writer.set_bed_temperature(bed_temperature);
            m_second_layer_things_done = true;
        }
        m_current_manual_fan_speed.reset();
        applied_object = nullptr;
        applied_region = nullptr;
    };

    size_t index = 0;
    while (index < m_conical_queue.size())
    {
        const int band = m_conical_queue[index].band;
        size_t end = index;
        double zmin = std::numeric_limits<double>::infinity();
        double zmax = -std::numeric_limits<double>::infinity();
        while (end < m_conical_queue.size() && m_conical_queue[end].band == band)
        {
            for (const Geometry::ArcWelder::Segment &seg : m_conical_queue[end].path)
            {
                const double z = seg.height_fraction;
                zmin = std::min(zmin, z);
                zmax = std::max(zmax, z);
            }
            ++end;
        }
        if (band != current_band)
        {
            begin_band(band, zmin, zmax);
            current_band = band;
        }
        const double layer_z = m_last_layer_z;

        for (size_t i = index; i < end; ++i)
        {
            ConicalQueuedExtrusion &item = m_conical_queue[i];
            if (item.object != nullptr)
            {
                const auto layers = item.object->layers();
                if (band == 0 && !layers.empty())
                    m_layer = layers.front();
                else if (item.layer != nullptr && item.layer->id() > 0)
                    m_layer = item.layer;
                else if (!layers.empty() && layers.size() > 1)
                    m_layer = layers[1];
                else
                    m_layer = item.layer;
            }
            m_object_layer_over_raft = false;
            if (item.object != applied_object || item.region != applied_region)
            {
                if (item.object != nullptr)
                    m_config.apply(item.object->config(), true);
                if (item.region != nullptr)
                    m_config.apply(item.region->config());
                applied_object = item.object;
                applied_region = item.region;
            }
            // Travels during this emit use the same cone as the queued path.
            m_conical_region = item.horizontal ? nullptr : item.region;
            if (m_writer.extruder() == nullptr || m_writer.extruder()->id() != item.extruder_id)
                gcode += this->set_extruder(item.extruder_id, layer_z);
            if (m_origin != item.origin)
                this->set_origin(item.origin);
            if (item.object != nullptr)
                m_current_instance = {item.object, item.instance_idx};
            // Object moves were queued without their exclude-object markers. The collection
            // pass already pointed the labeler at this instance, so emit the start here,
            // where the moves are actually written.
            if (item.object != nullptr && item.instance_idx >= 0 &&
                size_t(item.instance_idx) < item.object->instances().size())
            {
                this->m_label_objects.update(&item.object->instances()[size_t(item.instance_idx)]);
                gcode += this->m_label_objects.maybe_change_instance(m_writer);
            }

            // Flat paths (supports, and object regions outside a conical modifier) were sliced
            // at one Z. Cone paths are encoded against the bead height, which is what extrusion
            // multiplies back out. Encoding against the band span instead prints the cone at
            // half slope and sends travels to the top of the band.
            const float saved_layer_z = m_last_layer_z;
            const float saved_height = m_last_height;
            const double bead = item.attributes.height > 1e-6f ? double(item.attributes.height)
                                                               : std::max(1e-4, double(nominal_layer));
            if (item.horizontal)
            {
                const double z = item.path.front().height_fraction;
                for (Geometry::ArcWelder::Segment &seg : item.path)
                    seg.height_fraction = 1.f;
                m_last_layer_z = float(z);
            }
            else
            {
                for (Geometry::ArcWelder::Segment &seg : item.path)
                {
                    const double z = seg.height_fraction;
                    seg.height_fraction = float(1. + (z - layer_z) / bead);
                }
            }

            struct Guard
            {
                bool &flag;
                explicit Guard(bool &flag) : flag(flag) { flag = true; }
                ~Guard() { flag = false; }
            } guard{m_conical_rewrite};
            gcode += this->_extrude(item.attributes, item.path, item.description, item.speed, item.emit_modifiers);
            if (item.horizontal)
            {
                m_last_layer_z = saved_layer_z;
                m_last_height = saved_height;
            }
        }
        index = end;
    }

    this->set_origin(Vec2d::Zero());
    m_conical_region = nullptr;
    m_conical_queue.clear();
    m_conical_grid = {};
    return gcode;
}

GCodeGenerator::ConicalEmissionSnapshot GCodeGenerator::capture_conical_emission_state() const
{
    ConicalEmissionSnapshot snap;
    snap.axis = m_writer.axis_state();
    snap.config = m_config;
    snap.last_position = last_position;
    snap.origin = m_origin;
    snap.layer = m_layer;
    snap.object_layer_over_raft = m_object_layer_over_raft;
    snap.last_height = m_last_height;
    snap.last_layer_z = m_last_layer_z;
    snap.max_layer_z = m_max_layer_z;
    snap.last_width = m_last_width;
    snap.last_region_area = m_last_region_area;
    snap.last_interlocking_flow = m_last_interlocking_flow_multiplier;
    snap.feature_index = m_feature_index;
    snap.feature_temp_locked = m_feature_temp_locked;
    snap.feature_commanded_temp = m_feature_commanded_temp;
    snap.feature_temp_baseline = m_feature_temp_baseline;
    snap.last_processor_role = m_last_processor_extrusion_role;
    snap.last_extrusion_role = m_last_extrusion_role;
    snap.manual_fan = m_current_manual_fan_speed;
    snap.moved_to_first = m_moved_to_first_layer_point;
    snap.current_instance = m_current_instance;
    snap.wipe_path = m_wipe.path();
    snap.pending_gcode = m_pending_pre_extrusion_gcode;
    snap.region = m_conical_region;
    return snap;
}

void GCodeGenerator::restore_conical_emission_state(const ConicalEmissionSnapshot &snapshot)
{
    m_writer.restore_axis_state(snapshot.axis);
    m_config = snapshot.config;
    last_position = snapshot.last_position;
    m_origin = snapshot.origin;
    m_layer = snapshot.layer;
    m_object_layer_over_raft = snapshot.object_layer_over_raft;
    m_last_height = snapshot.last_height;
    m_last_layer_z = snapshot.last_layer_z;
    m_max_layer_z = snapshot.max_layer_z;
    m_last_width = snapshot.last_width;
    m_last_region_area = snapshot.last_region_area;
    m_last_interlocking_flow_multiplier = snapshot.last_interlocking_flow;
    m_feature_index = snapshot.feature_index;
    m_feature_temp_locked = snapshot.feature_temp_locked;
    m_feature_commanded_temp = snapshot.feature_commanded_temp;
    m_feature_temp_baseline = snapshot.feature_temp_baseline;
    m_last_processor_extrusion_role = snapshot.last_processor_role;
    m_last_extrusion_role = snapshot.last_extrusion_role;
    m_current_manual_fan_speed = snapshot.manual_fan;
    m_moved_to_first_layer_point = snapshot.moved_to_first;
    m_current_instance = snapshot.current_instance;
    if (snapshot.wipe_path.size() > 1)
        m_wipe.set_path(snapshot.wipe_path);
    else
        m_wipe.reset_path();
    m_pending_pre_extrusion_gcode = snapshot.pending_gcode;
    m_conical_region = snapshot.region;
}

} // namespace Slic3r
