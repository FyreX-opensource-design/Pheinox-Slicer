///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
#include "../GCode.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "../ClipperUtils.hpp"
#include "../Layer.hpp"
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

} // namespace Slic3r
