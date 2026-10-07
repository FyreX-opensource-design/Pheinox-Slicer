///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas
///|/ Copyright (c) SuperSlicer 2023 Remi Durand @supermerill
///|/ Copyright (c) 2016 Sakari Kapanen @Flannelhead
///|/ Copyright (c) Slic3r 2011 - 2015 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2013 Mark Hindess
///|/ Copyright (c) 2011 Michael Moon
///|/
///|/ preFlight is based on PrusaSlicer and released under AGPLv3 or higher
///|/
#include <oneapi/tbb/scalable_allocator.h>
#include <boost/container/vector.hpp>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <vector>
#include <functional>
#include <cassert>
#include <cinttypes>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

#include "../ClipperUtils.hpp"
#include "../Geometry.hpp"
#include "../Layer.hpp"

#include "../Print.hpp"
#include "../PreciseWalls.hpp"
#include "../PrintConfig.hpp"
#include "../Surface.hpp"
// for Arachne based infills
#include "../PerimeterGenerator.hpp"
#include "../Athena/WallToolPaths.hpp"
#include "../Athena/utils/ExtrusionLine.hpp"
#include "../Athena/PerimeterOrder.hpp"
#include "FillBase.hpp"
#include "FillRectilinear.hpp"
#include "FillLightning.hpp"
#include "FillEnsuring.hpp"
#include "FillCustom.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/ShortestPath.hpp"
#include "libslic3r/PerfTiming.hpp"

namespace Slic3r
{
PerfAccumTimer g_mf_group_fills;
PerfAccumTimer g_mf_intersection;
PerfAccumTimer g_mf_fill_surface;
std::atomic<int> g_mf_layer_count{0};
} // namespace Slic3r

namespace Slic3r
{
namespace FillAdaptive
{
struct Octree;
} // namespace FillAdaptive
namespace FillLightning
{
class Generator;
} // namespace FillLightning

//static constexpr const float NarrowInfillAreaThresholdMM = 3.f;

struct SurfaceFillParams
{
    // 1-based extruder ID, the raw region config value from PrintRegion::extruder().
    unsigned int extruder = 0;
    // Infill pattern, adjusted for the density etc.
    InfillPattern pattern = InfillPattern(0);

    // FillBase
    // in unscaled coordinates
    coordf_t spacing = 0.;
    // infill / perimeter overlap, in unscaled coordinates
    coordf_t overlap = 0.;
    // Angle as provided by the region config, in radians.
    float angle = 0.f;
    // Is bridging used for this fill? Bridging parameters may be used even if this->flow.bridge() is not set.
    bool bridge;
    // Non-negative for a bridge.
    float bridge_angle = 0.f;

    // FillParams
    float density = 0.f;
    // Don't adjust spacing to fill the space evenly.
    //    bool        	dont_adjust = false;
    // Length of the infill anchor along the perimeter line.
    // 1000mm is roughly the maximum length line that fits into a 32bit coord_t.
    float anchor_length = 1000.f;
    float anchor_length_max = 1000.f;

    // width, height of extrusion, nozzle diameter, is bridge
    // For the output, for fill generator.
    Flow flow;

    // For the output
    ExtrusionRole extrusion_role{ExtrusionRole::None};

    // Various print settings?

    // Index of this entry in a linear vector.
    size_t idx = 0;

    // Regions that print on different cones must not share one fill. The fill is stored on the
    // first region, and the other region's island then skips it.
    int conical_mode = 0;
    float conical_angle = 45.f;
    float conical_slice_height = 0.f;
    // A different infill wave must not share one fill, or the other region is extruded with the first wave.
    float infill_wave_amplitude = 0.f;
    float infill_wave_frequency = 0.f;
    float infill_wave_phase = 0.f;

    bool operator<(const SurfaceFillParams &rhs) const
    {
#define RETURN_COMPARE_NON_EQUAL(KEY) \
    if (this->KEY < rhs.KEY)          \
        return true;                  \
    if (this->KEY > rhs.KEY)          \
        return false;
#define RETURN_COMPARE_NON_EQUAL_TYPED(TYPE, KEY) \
    if (TYPE(this->KEY) < TYPE(rhs.KEY))          \
        return true;                              \
    if (TYPE(this->KEY) > TYPE(rhs.KEY))          \
        return false;

        // Sort first by decreasing bridging angle, so that the bridges are processed with priority when trimming one layer by the other.
        if (this->bridge_angle > rhs.bridge_angle)
            return true;
        if (this->bridge_angle < rhs.bridge_angle)
            return false;

        // TopSolidInfill must be processed first so it claims its area, then all other surfaces
        // (SolidInfill, sparse infill, etc.) get trimmed to avoid overlap.
        if (this->extrusion_role == ExtrusionRole::TopSolidInfill &&
            rhs.extrusion_role != ExtrusionRole::TopSolidInfill)
            return true; // TopSolidInfill goes first before everything
        if (this->extrusion_role != ExtrusionRole::TopSolidInfill &&
            rhs.extrusion_role == ExtrusionRole::TopSolidInfill)
            return false; // Everything else goes after TopSolidInfill

        RETURN_COMPARE_NON_EQUAL(extruder);
        RETURN_COMPARE_NON_EQUAL(conical_mode);
        RETURN_COMPARE_NON_EQUAL(conical_angle);
        RETURN_COMPARE_NON_EQUAL(conical_slice_height);
        RETURN_COMPARE_NON_EQUAL(infill_wave_amplitude);
        RETURN_COMPARE_NON_EQUAL(infill_wave_frequency);
        RETURN_COMPARE_NON_EQUAL(infill_wave_phase);
        RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, pattern);
        RETURN_COMPARE_NON_EQUAL(spacing);
        RETURN_COMPARE_NON_EQUAL(overlap);
        RETURN_COMPARE_NON_EQUAL(angle);
        RETURN_COMPARE_NON_EQUAL(density);
        //		RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, dont_adjust);
        RETURN_COMPARE_NON_EQUAL(anchor_length);
        RETURN_COMPARE_NON_EQUAL(anchor_length_max);
        RETURN_COMPARE_NON_EQUAL(flow.width());
        RETURN_COMPARE_NON_EQUAL(flow.height());
        RETURN_COMPARE_NON_EQUAL(flow.nozzle_diameter());
        RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, bridge);
        return this->extrusion_role.lower(rhs.extrusion_role);
    }

    bool operator==(const SurfaceFillParams &rhs) const
    {
        return this->extruder == rhs.extruder && this->conical_mode == rhs.conical_mode &&
               this->conical_angle == rhs.conical_angle && this->conical_slice_height == rhs.conical_slice_height &&
               this->infill_wave_amplitude == rhs.infill_wave_amplitude &&
               this->infill_wave_frequency == rhs.infill_wave_frequency &&
               this->infill_wave_phase == rhs.infill_wave_phase &&
               this->pattern == rhs.pattern && this->spacing == rhs.spacing &&
               this->overlap == rhs.overlap && this->angle == rhs.angle && this->bridge == rhs.bridge &&
               //				this->bridge_angle 		== rhs.bridge_angle		&&
               this->density == rhs.density &&
               //				this->dont_adjust   	== rhs.dont_adjust 		&&
               this->anchor_length == rhs.anchor_length && this->anchor_length_max == rhs.anchor_length_max &&
               this->flow == rhs.flow && this->extrusion_role == rhs.extrusion_role;
    }
};

struct SurfaceFill
{
    SurfaceFill(const SurfaceFillParams &params) : region_id(size_t(-1)), surface(stCount, ExPolygon()), params(params)
    {
    }

    size_t region_id;
    Surface surface;
    ExPolygons expolygons;
    SurfaceFillParams params;
};

static inline bool fill_type_monotonic(InfillPattern pattern)
{
    return pattern == ipMonotonic || pattern == ipMonotonicLines;
}

// Debug output (Slic3r::dbg_log) is in DebugOutput.hpp, via FillBase.hpp

static const char *dbg_pattern(InfillPattern p)
{
    switch (p)
    {
    case ipRectilinear:
        return "Rectilinear";
    case ipMonotonic:
        return "Monotonic";
    case ipMonotonicLines:
        return "MonotonicLines";
    case ipAlignedRectilinear:
        return "AlignedRectilinear";
    case ipAlignedMonotonic:
        return "AlignedMonotonic";
    case ipGrid:
        return "Grid";
    case ipTriangles:
        return "Triangles";
    case ipStars:
        return "Stars";
    case ipCubic:
        return "Cubic";
    case ipLine:
        return "Line";
    case ipConcentric:
        return "Concentric";
    case ipHoneycomb:
        return "Honeycomb";
    case ip3DHoneycomb:
        return "3DHoneycomb";
    case ipGyroid:
        return "Gyroid";
    case ipHilbertCurve:
        return "HilbertCurve";
    case ipArchimedeanChords:
        return "ArchimedeanChords";
    case ipOctagramSpiral:
        return "OctagramSpiral";
    case ipAdaptiveCubic:
        return "AdaptiveCubic";
    case ipSupportCubic:
        return "SupportCubic";
    case ipSupportBase:
        return "SupportBase";
    case ipLightning:
        return "Lightning";
    case ipEnsuring:
        return "Ensuring";
    case ipZigZag:
        return "ZigZag";
    case ipCustom:
        return "Custom";
    case ipCat:
        return "Cat";
    case ipCatMirrored:
        return "CatMirrored";
    case ipCatTiled:
        return "CatTiled";
    case ipShark:
        return "Sharkfill";
    case ipPuppy:
        return "Puppyfill";
    default:
        return "UNKNOWN";
    }
}

static const char *dbg_stype(SurfaceType t)
{
    switch (t)
    {
    case stTop:
        return "stTop";
    case stBottom:
        return "stBottom";
    case stBottomBridge:
        return "stBottomBridge";
    case stInternal:
        return "stInternal";
    case stInternalSolid:
        return "stInternalSolid";
    case stInternalBridge:
        return "stInternalBridge";
    case stInternalVoid:
        return "stInternalVoid";
    case stPerimeter:
        return "stPerimeter";
    case stSolidOverBridge:
        return "stSolidOverBridge";
    case stBridgeAnchor:
        return "stBridgeAnchor";
    case stCount:
        return "stCount";
    default:
        return "UNKNOWN";
    }
}

static void dbg_fill_input(const Layer &layer)
{
    if (!Slic3r::debug_enabled(Slic3r::DBG_FILL))
        return;
    double z = layer.print_z;
    int lid = (int) layer.id();
    dbg_log(Slic3r::DBG_FILL, z, "FILL", "========== INPUT SURFACES (layer %d, height=%.3f) ==========", lid,
            layer.height);
    for (size_t region_id = 0; region_id < layer.regions().size(); ++region_id)
    {
        const LayerRegion &layerm = *layer.regions()[region_id];
        int idx = 0;
        double region_total = 0;
        for (const Surface &surface : layerm.fill_surfaces())
        {
            double a = std::abs(surface.expolygon.area()) * 1e-12;
            region_total += a;
            BoundingBox bb = get_extents(surface.expolygon);
            dbg_log(Slic3r::DBG_FILL, z, "FILL",
                    "INPUT r=%zu i=%d type=%-18s area=%8.4fmm2 holes=%zu pts=%zu "
                    "bbox=(%.2f,%.2f)-(%.2f,%.2f) bridge_ang=%.1f",
                    region_id, idx, dbg_stype(surface.surface_type), a, surface.expolygon.holes.size(),
                    surface.expolygon.contour.points.size(), unscaled<double>(bb.min.x()), unscaled<double>(bb.min.y()),
                    unscaled<double>(bb.max.x()), unscaled<double>(bb.max.y()), surface.bridge_angle);
            idx++;
        }
        dbg_log(Slic3r::DBG_FILL, z, "FILL", "INPUT r=%zu TOTAL: %d surfaces, %.4fmm2", region_id, idx, region_total);
    }
}

static void dbg_fill_phase(const char *phase, const Layer &layer, const std::vector<SurfaceFill> &fills)
{
    if (!Slic3r::debug_enabled(Slic3r::DBG_FILL))
        return;
    double z = layer.print_z;
    int lid = (int) layer.id();
    int total_ep = 0;
    double total_area = 0;
    dbg_log(Slic3r::DBG_FILL, z, "FILL", "========== %s (layer %d) ==========", phase, lid);
    for (size_t i = 0; i < fills.size(); i++)
    {
        const SurfaceFill &sf = fills[i];
        if (sf.expolygons.empty())
            continue;
        double sf_area = 0;
        for (const ExPolygon &ep : sf.expolygons)
            sf_area += std::abs(ep.area());
        double sf_area_mm2 = sf_area * 1e-12;
        total_area += sf_area_mm2;
        total_ep += (int) sf.expolygons.size();
        BoundingBox bb = get_extents(sf.expolygons);
        dbg_log(Slic3r::DBG_FILL, z, "FILL",
                "%s [%zu] type=%-18s r=%zu dens=%.1f%% ep=%zu area=%8.4fmm2 "
                "bbox=(%.2f,%.2f)-(%.2f,%.2f)",
                phase, i, dbg_stype(sf.surface.surface_type), sf.region_id, sf.params.density, sf.expolygons.size(),
                sf_area_mm2, unscaled<double>(bb.min.x()), unscaled<double>(bb.min.y()), unscaled<double>(bb.max.x()),
                unscaled<double>(bb.max.y()));
        for (size_t j = 0; j < sf.expolygons.size(); j++)
        {
            const ExPolygon &ep = sf.expolygons[j];
            double ep_area = std::abs(ep.area()) * 1e-12;
            BoundingBox epbb = get_extents(ep);
            dbg_log(Slic3r::DBG_FILL, z, "FILL",
                    "  %s [%zu][%zu] area=%8.4fmm2 holes=%zu pts=%zu "
                    "bbox=(%.2f,%.2f)-(%.2f,%.2f)",
                    phase, i, j, ep_area, ep.holes.size(), ep.contour.points.size(), unscaled<double>(epbb.min.x()),
                    unscaled<double>(epbb.min.y()), unscaled<double>(epbb.max.x()), unscaled<double>(epbb.max.y()));
        }
    }
    dbg_log(Slic3r::DBG_FILL, z, "FILL", "%s TOTAL: %d expolygons, %.4fmm2", phase, total_ep, total_area);
}
// ===================== END FILL DEBUG HELPERS =====================

std::vector<SurfaceFill> group_fills(const Layer &layer)
{
    std::vector<SurfaceFill> surface_fills;

    dbg_fill_input(layer);

    // First pass: Check if merge is enabled in config and collect top solid surface polygons per region.
    // We always collect top solid polygons (needed for both merge and concentric fallback).
    bool config_allows_merge = false;
    std::vector<Polygons> region_top_solid_polygons(layer.regions().size());
    std::vector<BoundingBox> region_top_solid_bboxes(layer.regions().size());

    for (size_t region_id = 0; region_id < layer.regions().size(); ++region_id)
    {
        const LayerRegion &layerm = *layer.regions()[region_id];

        // Check config (only need to check once, assume all regions have same setting)
        if (region_id == 0)
            config_allows_merge = layerm.region().config().merge_top_solid_infills;

        // Always collect top solid surface polygons for this region
        // (needed for merge check and for concentric fallback when merge is disabled)
        for (const Surface &surface : layerm.fill_surfaces())
        {
            if (surface.is_top())
            {
                Polygons polys = to_polygons(surface.expolygon);
                append(region_top_solid_polygons[region_id], polys);
            }
        }
        if (!region_top_solid_polygons[region_id].empty())
            region_top_solid_bboxes[region_id] = get_extents(region_top_solid_polygons[region_id]);
    }

    // Check if an internal solid surface spatially intersects any top solid surface in the same region.
    // Uses bbox pre-filter to skip expensive Clipper2 intersection when surfaces are clearly disjoint.
    auto internal_solid_touches_top = [&](const Surface &surface, size_t region_id) -> bool
    {
        if (surface.surface_type != stInternalSolid)
            return false;

        const Polygons &top_solid_polys = region_top_solid_polygons[region_id];
        if (top_solid_polys.empty())
            return false;

        BoundingBox surface_bb = surface.expolygon.contour.bounding_box();
        if (!surface_bb.overlap(region_top_solid_bboxes[region_id]))
            return false;

        Polygons internal_polys = to_polygons(surface.expolygon);
        return !intersection(internal_polys, top_solid_polys).empty();
    };

    // Fill in a map of a region & surface to SurfaceFillParams.
    std::set<SurfaceFillParams> set_surface_params;
    std::vector<std::vector<const SurfaceFillParams *>> region_to_surface_params(
        layer.regions().size(), std::vector<const SurfaceFillParams *>());
    SurfaceFillParams params;
    bool has_internal_voids = false;
    for (size_t region_id = 0; region_id < layer.regions().size(); ++region_id)
    {
        const LayerRegion &layerm = *layer.regions()[region_id];
        region_to_surface_params[region_id].assign(layerm.fill_surfaces().size(), nullptr);
        for (const Surface &surface : layerm.fill_surfaces())
            if (surface.surface_type == stInternalVoid)
                has_internal_voids = true;
            else
            {
                const PrintRegionConfig &region_config = layerm.region().config();
                params.conical_mode = int(region_config.conical_slicing.value);
                params.conical_angle = float(region_config.conical_angle.value);
                params.conical_slice_height = float(region_config.conical_slice_height.value);
                params.infill_wave_amplitude = float(region_config.infill_wave_amplitude.value);
                params.infill_wave_frequency = float(region_config.infill_wave_frequency.value);
                params.infill_wave_phase = float(region_config.infill_wave_phase.value);
                FlowRole extrusion_role = surface.is_top() ? frTopSolidInfill
                                                           : (surface.is_solid() ? frSolidInfill : frInfill);
                bool is_bridge = layer.id() > 0 && surface.is_bridge();
                params.extruder = layerm.region().extruder(extrusion_role);
                params.pattern = region_config.fill_pattern.value;
                params.density = float(region_config.fill_density);

                const bool touches_top = internal_solid_touches_top(surface, region_id);
                if (surface.is_solid())
                {
                    params.density = 100.f;
                    if (is_bridge)
                    {
                        params.pattern = ipMonotonic;
                    }
                    else if (surface.is_top() || (config_allows_merge && touches_top))
                    {
                        // Top surface, or internal solid adjacent to top when merge is enabled
                        params.pattern = region_config.top_fill_pattern.value;
                    }
                    else if (!config_allows_merge && touches_top)
                    {
                        // When merge is disabled but internal solid is touching top solid,
                        // use concentric pattern for visual distinction
                        params.pattern = ipConcentric;
                    }
                    else if (surface.is_external())
                    {
                        // External bottom surface
                        params.pattern = surface.is_top() ? region_config.top_fill_pattern.value
                                                          : region_config.bottom_fill_pattern.value;
                    }
                    else if (surface.surface_type == stBridgeAnchor)
                    {
                        params.pattern = ipEnsuring;
                    }
                    else
                    {
                        // Internal solid: use user-selected solid fill pattern
                        params.pattern = region_config.solid_fill_pattern.value;
                    }

                    // Narrow surface detection via offset erosion. A surface is narrow
                    // if it completely collapses when inset by the threshold.
                    const Flow solid_flow = layerm.flow(frSolidInfill);
                    auto is_narrow_at = [&](float inset) -> bool
                    {
                        return offset_ex(surface.expolygon, -inset).empty();
                    };
                    const bool is_narrow_1_5x = !is_bridge && surface.is_solid() &&
                                                is_narrow_at(solid_flow.scaled_width() * 1.5f / 2.f);

                    // When Athena perimeters converge, a narrow sliver may be created that should be covered
                    // by perimeters but gets classified as a fill surface. Skip these very narrow surfaces
                    // for stTop and stBottom only (internal solid may legitimately need filling).
                    if (is_narrow_1_5x && (surface.surface_type == stTop || surface.surface_type == stBottom))
                    {
                        const double area_threshold = sqr(solid_flow.scaled_width() * 1.5f) * 4.0;
                        if (std::abs(surface.expolygon.area()) < area_threshold)
                        {
                            dbg_log(Slic3r::DBG_FILL, layer.print_z, "FILL", "SKIP_NARROW type=%-18s area=%8.4fmm2",
                                    dbg_stype(surface.surface_type), std::abs(surface.expolygon.area()) * 1e-12);
                            continue;
                        }
                    }

                    // Narrow-to-Athena does NOT decide the pattern here. The configured solid
                    // pattern is kept; thin sub-regions are split off into a concentric bead by
                    // the width-split pass after group_fills geometry settles (see below).
                }
                else if (params.density <= 0)
                    continue;

                if (is_bridge)
                {
                    params.extrusion_role = ExtrusionRole::BridgeInfill;
                }
                else
                {
                    if (surface.is_solid())
                    {
                        if (surface.is_top())
                        {
                            params.extrusion_role = ExtrusionRole::TopSolidInfill;
                        }
                        else if (surface.surface_type == stSolidOverBridge)
                        {
                            params.extrusion_role = ExtrusionRole::InfillOverBridge;
                        }
                        else
                        {
                            // Only use TopSolidInfill role for internal solid surfaces that are
                            // spatially adjacent to (touching) actual top solid surfaces
                            if (config_allows_merge && touches_top)
                            {
                                params.extrusion_role = ExtrusionRole::TopSolidInfill;
                            }
                            else
                            {
                                params.extrusion_role = ExtrusionRole::SolidInfill;
                            }
                        }
                    }
                    else
                    {
                        params.extrusion_role = ExtrusionRole::InternalInfill;
                    }
                }
                params.bridge_angle = float(surface.bridge_angle);
                params.angle = float(Geometry::deg2rad(region_config.fill_angle.value));
                // Angle alternation for all surfaces (including solid) is handled by
                // _layer_angle() in FillBase.cpp during fill generation. No manipulation here.

                // Calculate the actual flow we'll be using for this infill.
                params.bridge = is_bridge || Fill::use_bridge_flow(params.pattern);
                params.flow = params.bridge ? layerm.bridging_flow(extrusion_role)
                                            : layerm.flow(extrusion_role,
                                                          (surface.thickness == -1) ? layer.height : surface.thickness);

                // Calculate flow spacing for infill pattern generation.
                // Treat near-solid density (>= 99.9999%) like solid for spacing purposes to avoid
                // underextrusion when fill surface thickness differs from layer height.
                if (surface.is_solid() || is_bridge || params.density >= 99.9999f)
                {
                    if (is_bridge)
                    {
                        float bridge_diameter = params.flow.width(); // For bridges, width == height == diameter

                        // Line-to-line spacing (bridge_infill_overlap setting)
                        float line_overlap_percent;
                        if (region_config.bridge_infill_overlap.percent)
                        {
                            line_overlap_percent = float(region_config.bridge_infill_overlap.value);
                        }
                        else
                        {
                            line_overlap_percent = float(region_config.bridge_infill_overlap.value) / bridge_diameter *
                                                   100.0f;
                        }
                        line_overlap_percent = std::clamp(line_overlap_percent, -100.0f, 80.0f);
                        params.spacing = bridge_diameter * (1.0f - line_overlap_percent / 100.0f);
                    }
                    else
                    {
                        params.spacing = params.flow.spacing();
                    }
                    // Only apply overlap and anchor settings for actual solid/bridge, not high-density sparse
                    if (surface.is_solid() || is_bridge)
                    {
                        // Overlap = 0 because bridge surface geometry is already adjusted in LayerRegion.cpp
                        // by expand_bridges_for_overlap() which runs AFTER the merge logic completes.
                        // This ensures: merge first, then expand for overlap on final geometry.
                        params.overlap = 0.0f;
                        // Don't limit anchor length for solid or bridging infill.
                        params.anchor_length = 1000.f;
                        params.anchor_length_max = 1000.f;
                    }
                }
                else
                {
                    // Internal infill. Calculating infill line spacing independent of the current layer height and 1st layer status,
                    // so that internall infill will be aligned over all layers of the current region.
                    params.spacing = layerm.region()
                                         .flow(*layer.object(), frInfill, layer.object()->config().layer_height, false)
                                         .spacing();
                    // A bead cannot be narrower than it is tall. On a tall variable-height layer with a
                    // fixed extrusion width, the region-wide spacing can demand exactly that (width for
                    // spacing s at height h is s + h*(1-pi/4), which drops below h once s < h*pi/4).
                    // Clamp to the round-bead minimum: this layer prints width == height beads at
                    // slightly wider spacing instead of aborting the slice.
                    const double min_round_bead_spacing = params.flow.height() * (0.25 * M_PI) + EPSILON;
                    if (params.spacing < min_round_bead_spacing)
                    {
                        // Clamping breaks the layer-height-independent pitch on this surface, so
                        // its lines will not register with neighboring layers - log the trade.
                        dbg_log(Slic3r::DBG_FILL, layer.print_z, "FILL",
                                "SPACING_CLAMP surface_h=%.3f requested=%.3f clamped=%.3f", params.flow.height(),
                                params.spacing, min_round_bead_spacing);
                        params.spacing = min_round_bead_spacing;
                    }
                    // When fill surface thickness differs from layer height, rescale width to maintain
                    // requested density with the rounded rectangle extrusion model.
                    params.flow = params.flow.with_spacing(params.spacing);

                    // When interlocking perimeters are enabled, infill anchors create overlaps and conflicts.
                    // Interlocking perimeters provide their own bonding to real perimeters via P/P overlap.
                    // Override anchor_length to 0 when interlocking is active (user's settings preserved in UI).
                    // IMPORTANT: We only set anchor_length to 0, NOT anchor_length_max. Setting anchor_length_max
                    // to 0 causes dont_connect() to return true, which disables zigzag patterns entirely.
                    // We want to disable perimeter anchoring but still allow infill-to-infill zigzag connections.
                    const bool has_interlocking = region_config.interlock_perimeters_enabled &&
                                                  layerm.num_interlocking_shells() > 0;

                    // Anchor a sparse infill to inner perimeters with the following anchor length:
                    params.anchor_length = has_interlocking ? 0.0f : float(region_config.infill_anchor);
                    if (!has_interlocking && region_config.infill_anchor.percent)
                        params.anchor_length = float(params.anchor_length * 0.01 * params.spacing);
                    params.anchor_length_max = float(region_config.infill_anchor_max);
                    if (region_config.infill_anchor_max.percent)
                        params.anchor_length_max = float(params.anchor_length_max * 0.01 * params.spacing);
                    params.anchor_length = std::min(params.anchor_length, params.anchor_length_max);
                }

                auto it_params = set_surface_params.find(params);
                if (it_params == set_surface_params.end())
                    it_params = set_surface_params.insert(it_params, params);
                region_to_surface_params[region_id][&surface - &layerm.fill_surfaces().surfaces.front()] = &(
                    *it_params);
            }
    }

    surface_fills.reserve(set_surface_params.size());
    for (const SurfaceFillParams &params : set_surface_params)
    {
        const_cast<SurfaceFillParams &>(params).idx = surface_fills.size();
        surface_fills.emplace_back(params);
    }

    for (size_t region_id = 0; region_id < layer.regions().size(); ++region_id)
    {
        const LayerRegion &layerm = *layer.regions()[region_id];
        for (const Surface &surface : layerm.fill_surfaces())
            if (surface.surface_type != stInternalVoid)
            {
                const SurfaceFillParams *params =
                    region_to_surface_params[region_id][&surface - &layerm.fill_surfaces().surfaces.front()];
                if (params != nullptr)
                {
                    SurfaceFill &fill = surface_fills[params->idx];

                    if (fill.region_id == size_t(-1))
                    {
                        fill.region_id = region_id;
                        fill.surface = surface;
                        fill.expolygons.emplace_back(std::move(fill.surface.expolygon));
                    }
                    else
                        fill.expolygons.emplace_back(surface.expolygon);
                }
            }
    }

    dbg_fill_phase("GROUPED", layer, surface_fills);

    {
        Polygons all_polygons;
        // preFlight: Track TopSolidInfill polygons separately so we can apply
        // clearance only between TopSolid and SolidInfill (not between bridge and solid).
        Polygons top_solid_polygons;
        for (SurfaceFill &fill : surface_fills)
            if (!fill.expolygons.empty())
            {
                if (fill.expolygons.size() > 1 || !all_polygons.empty())
                {
                    Polygons polys = to_polygons(std::move(fill.expolygons));
                    // Make a union of polygons, use a safety offset, subtract the preceding polygons.
                    // Bridges are processed first (see SurfaceFill::operator<())

                    // When trimming SolidInfill, add clearance only against TopSolidInfill regions
                    // to prevent overlap where both expand during fill generation. Don't apply
                    // clearance against bridge/InfillOverBridge - those should seamlessly abut
                    // with internal solid to avoid leaving unfilled holes.
                    Polygons trim_polygons = all_polygons;
                    if (!all_polygons.empty() && fill.params.extrusion_role == ExtrusionRole::SolidInfill &&
                        fill.params.density > 0.99f && !top_solid_polygons.empty())
                    {
                        const float clearance = float(fill.params.flow.width() * 0.25);
                        Polygons top_expanded = offset(top_solid_polygons, scale_(clearance));
                        // Combine: non-top fills at original size + top fills with clearance
                        Polygons non_top = diff(all_polygons, top_solid_polygons);
                        trim_polygons = union_(non_top, top_expanded);
                    }

                    fill.expolygons = all_polygons.empty() ? union_safety_offset_ex(polys)
                                                           : diff_ex(polys, trim_polygons, ApplySafetyOffset::Yes);
                    append(all_polygons, std::move(polys));
                }
                else if (&fill != &surface_fills.back())
                    append(all_polygons, to_polygons(fill.expolygons));

                // Track TopSolidInfill polygons for targeted clearance
                if (fill.params.extrusion_role == ExtrusionRole::TopSolidInfill)
                    append(top_solid_polygons, to_polygons(fill.expolygons));
            }
    }

    dbg_fill_phase("TRIMMED", layer, surface_fills);

    // preFlight: Compute the total fill boundary from the layer's fill_expolygons - the area
    // inside the innermost perimeters. This is the true boundary for all fills, unaffected by
    // inter-fill trimming (which introduces safety-offset micro-gaps). Grow/union/shrink can
    // push geometry beyond fill boundaries into perimeter territory, so we clip results back
    // to this boundary after each merge operation.
    Polygons total_fill_boundary;
    for (size_t region_id = 0; region_id < layer.regions().size(); ++region_id)
        append(total_fill_boundary, to_polygons(layer.regions()[region_id]->fill_expolygons()));
    total_fill_boundary = union_(total_fill_boundary);

    // preFlight: Compute the sparse fill threshold once - used for hole removal
    // and sparse absorption below. Based on the sparse fill's actual line spacing so it
    // adapts to different infill densities and nozzle sizes.
    double sparse_min_area = 0;
    float sparse_erode_radius = 0;
    for (const SurfaceFill &sf : surface_fills)
        if (sf.surface.surface_type == stInternal && !sf.expolygons.empty() && sf.params.density < 99.f)
        {
            const float line_spacing = float(scale_(sf.params.spacing)) / (sf.params.density / 100.f);
            sparse_min_area = double(line_spacing) * double(line_spacing) * 4.0;
            sparse_erode_radius = line_spacing * 0.75f;
            break;
        }

    // preFlight: Consolidate all stSolidOverBridge SurfaceFill entries into one.
    // mark_as_infill_above_bridge() assigns different bridge_angles to fragments,
    // causing group_fills to place them in separate SurfaceFill entries. Merge them
    // so all subsequent processing (hole removal, absorption, grow/union/shrink) operates
    // on a single unified stSolidOverBridge region.
    {
        SurfaceFill *primary_sob = nullptr;
        double primary_area = 0;
        for (SurfaceFill &sf : surface_fills)
        {
            if (sf.expolygons.empty() || sf.surface.surface_type != stSolidOverBridge)
                continue;
            double total_area = 0;
            for (const ExPolygon &ep : sf.expolygons)
                total_area += std::abs(ep.area());
            if (!primary_sob || total_area > primary_area)
            {
                primary_sob = &sf;
                primary_area = total_area;
            }
        }
        if (primary_sob)
        {
            for (SurfaceFill &sf : surface_fills)
            {
                if (&sf == primary_sob || sf.expolygons.empty() || sf.surface.surface_type != stSolidOverBridge)
                    continue;
                append(primary_sob->expolygons, std::move(sf.expolygons));
                sf.expolygons.clear();
            }
            primary_sob->expolygons = union_ex(primary_sob->expolygons);
        }
    }

    dbg_fill_phase("SOB_CONSOLIDATED", layer, surface_fills);

    // preFlight: Remove small holes from stInternalSolid ExPolygons.
    // These holes come from trimming against bridge/top fills but are too small for
    // those fills to generate meaningful lines, leaving dark unfilled gaps.
    // Only remove holes that aren't occupied by another fill (stTop, bridge, etc.).
    if (sparse_min_area > 0)
    {
        Polygons other_fill_polys;
        for (const SurfaceFill &sf : surface_fills)
            if (sf.surface.surface_type != stInternalSolid && sf.surface.surface_type != stBridgeAnchor &&
                sf.surface.surface_type != stInternal && !sf.expolygons.empty())
                append(other_fill_polys, to_polygons(sf.expolygons));

        for (SurfaceFill &fill : surface_fills)
        {
            if (fill.expolygons.empty() ||
                (fill.surface.surface_type != stInternalSolid && fill.surface.surface_type != stBridgeAnchor))
                continue;
            for (ExPolygon &ep : fill.expolygons)
                ep.holes.erase(
                    std::remove_if(ep.holes.begin(), ep.holes.end(),
                                   [sparse_min_area, sparse_erode_radius, &other_fill_polys,
                                    &total_fill_boundary](const Polygon &hole)
                                   {
                                       Polygon contour = hole;
                                       contour.reverse();
                                       // Keep large holes unless they're too thin for sparse fill
                                       if (std::abs(hole.area()) >= sparse_min_area)
                                       {
                                           if (sparse_erode_radius <= 0)
                                               return false;
                                           // Large area but possibly too thin - erosion test
                                           ExPolygons eroded = opening_ex(ExPolygons{ExPolygon(contour)},
                                                                          sparse_erode_radius);
                                           if (!eroded.empty())
                                               return false; // Thick enough for sparse fill
                                           // Falls through: large area but too thin, evaluate further
                                       }
                                       // Keep hole if another fill occupies it
                                       if (!intersection_ex(ExPolygons{ExPolygon(contour)}, other_fill_polys).empty())
                                           return false;
                                       // Keep hole if it extends outside the fill boundary (real model feature
                                       // like a through-hole, not a trimming artifact)
                                       ExPolygons outside = diff_ex(ExPolygons{ExPolygon(contour)},
                                                                    total_fill_boundary);
                                       if (!outside.empty())
                                       {
                                           double outside_area = 0;
                                           for (const ExPolygon &o : outside)
                                               outside_area += std::abs(o.area());
                                           // If significant portion is outside fill boundary, it's a real feature
                                           if (outside_area > std::abs(contour.area()) * 0.1)
                                               return false;
                                       }
                                       return true;
                                   }),
                    ep.holes.end());
        }
    }

    dbg_fill_phase("HOLES_RM_SOLID", layer, surface_fills);

    // preFlight: Remove thin holes from stSolidOverBridge ExPolygons.
    // The surface classification creates stSolidOverBridge with holes for model features
    // (arcs, crescents, through-holes). Thin features like arcs and crescents are too
    // narrow for any fill to produce lines, leaving dark gaps. Remove holes that vanish
    // under erosion (too thin) while keeping thick ones (through-holes that bridge fills).
    if (sparse_erode_radius > 0)
        for (SurfaceFill &fill : surface_fills)
        {
            if (fill.surface.surface_type != stSolidOverBridge || fill.expolygons.empty())
                continue;
            for (ExPolygon &ep : fill.expolygons)
            {
                Polygons kept_holes;
                for (const Polygon &hole : ep.holes)
                {
                    // Reverse hole orientation (CW -> CCW) to create a testable ExPolygon.
                    Polygon contour = hole;
                    contour.reverse();
                    // Erosion test: if the hole shape vanishes, it's too thin to keep.
                    ExPolygons eroded = opening_ex(ExPolygons{ExPolygon(contour)}, sparse_erode_radius);
                    if (!eroded.empty())
                        kept_holes.push_back(hole);
                }
                ep.holes = std::move(kept_holes);
            }
        }

    dbg_fill_phase("HOLES_RM_SOB", layer, surface_fills);

    // preFlight: Transfer stInternalSolid that physically touches stSolidOverBridge into SOB.
    // Surface classification splits solid areas into SOB (above bridge) and InternalSolid (other).
    // When these are adjacent, filling them separately leaves thin gaps (e.g. arc-shaped voids
    // around hole features) that are too narrow for sparse fill. Merging adjacent pieces into
    // SOB lets the subsequent grow/union/shrink heal these gaps.
    // Only transfer InternalSolid that geometrically touches SOB - never reclassify
    // distant pieces that happen to share the same layer.
    {
        SurfaceFill *sob_fill = nullptr;
        for (SurfaceFill &sf : surface_fills)
            if (sf.surface.surface_type == stSolidOverBridge && !sf.expolygons.empty())
            {
                sob_fill = &sf;
                break;
            }
        if (sob_fill)
        {
            // Grow SOB slightly to detect touching/near-touching InternalSolid.
            // 0.1mm bridges classification micro-gaps without reaching distant pieces.
            Polygons sob_grown = offset(to_polygons(sob_fill->expolygons), scale_(0.1));

            bool merged = false;
            for (SurfaceFill &sf : surface_fills)
            {
                if (&sf == sob_fill || sf.surface.surface_type != stInternalSolid || sf.expolygons.empty())
                    continue;
                ExPolygons to_transfer;
                ExPolygons to_keep;
                for (const ExPolygon &ep : sf.expolygons)
                {
                    if (!intersection_ex(ExPolygons{ep}, sob_grown).empty())
                        to_transfer.push_back(ep);
                    else
                        to_keep.push_back(ep);
                }
                if (!to_transfer.empty())
                {
                    sf.expolygons = std::move(to_keep);
                    append(sob_fill->expolygons, std::move(to_transfer));
                    merged = true;
                }
            }
            if (merged)
                sob_fill->expolygons = union_ex(sob_fill->expolygons);
        }
    }

    dbg_fill_phase("ADJACENCY_XFER", layer, surface_fills);

    // preFlight: After stSolidOverBridge modifications (hole removal + thin region merge),
    // re-trim fills that were trimmed against the original stSolidOverBridge. The expanded
    // coverage now overlaps with remaining stInternalSolid and sparse fills.
    {
        Polygons sob_polys;
        for (const SurfaceFill &sf : surface_fills)
            if (sf.surface.surface_type == stSolidOverBridge && !sf.expolygons.empty())
                append(sob_polys, to_polygons(sf.expolygons));
        if (!sob_polys.empty())
            for (SurfaceFill &sf : surface_fills)
            {
                if (sf.expolygons.empty())
                    continue;
                if (sf.surface.surface_type == stInternalSolid || sf.surface.surface_type == stBridgeAnchor ||
                    sf.surface.surface_type == stInternal)
                    sf.expolygons = diff_ex(sf.expolygons, sob_polys);
            }
    }

    dbg_fill_phase("SOB_RETRIM", layer, surface_fills);

    // preFlight: Absorb sparse infill regions that are enclosed by solid or bridge fills
    // and too small or too thin for meaningful sparse fill lines. Two criteria identify
    // candidates: (1) total area below sparse_min_area, or (2) effective gap
    // (area/max_dimension) below sparse line spacing - catches thin annular rings that
    // have large total area but are too narrow for even a single fill line.
    for (SurfaceFill &absorber : surface_fills)
    {
        if (absorber.expolygons.empty())
            continue;
        if (absorber.surface.surface_type != stInternalSolid && absorber.surface.surface_type != stBridgeAnchor &&
            absorber.surface.surface_type != stSolidOverBridge && !absorber.surface.is_bridge())
            continue;

        // Build the "filled" boundary from contours only (no holes).
        // For stInternalSolid: contours already encompass the sparse pockets (one big region
        // with holes carved out), so stripping holes and unioning is sufficient.
        // For stSolidOverBridge: mark_as_infill_above_bridge() fragments the solid into
        // disjoint pieces covering only areas above bridge extrusions. Sparse pockets sit
        // in gaps between fragments. Morphological closing (dilate + erode) bridges these
        // inter-fragment gaps to reconstruct the encompassing boundary.
        // For bridge fills: contours with holes (counterbore holes etc.) - strip holes so
        // the boundary covers thin sparse rings between bridge and perimeters.
        ExPolygons absorber_filled;
        if (absorber.surface.surface_type == stSolidOverBridge && sparse_erode_radius > 0)
        {
            Polygons sob_contours;
            for (const ExPolygon &ep : absorber.expolygons)
                sob_contours.push_back(ep.contour);
            absorber_filled = closing_ex(sob_contours, sparse_erode_radius);
        }
        else
        {
            absorber_filled.reserve(absorber.expolygons.size());
            for (const ExPolygon &ep : absorber.expolygons)
                absorber_filled.emplace_back(ep.contour);
            absorber_filled = union_ex(absorber_filled);
        }

        for (SurfaceFill &sparse_fill : surface_fills)
        {
            if (sparse_fill.surface.surface_type != stInternal || sparse_fill.expolygons.empty())
                continue;
            if (sparse_fill.params.density >= 99.f)
                continue;

            // Threshold: area that can't fit meaningful sparse fill.
            // line_spacing is the actual distance between sparse fill lines.
            // A region needs at least a 2x2 grid of lines to be useful.
            const float line_spacing = float(scale_(sparse_fill.params.spacing)) / (sparse_fill.params.density / 100.f);
            const double min_area = double(line_spacing) * double(line_spacing) * 4.0;

            ExPolygons to_absorb;
            ExPolygons to_keep;

            for (const ExPolygon &ep : sparse_fill.expolygons)
            {
                double area = std::abs(ep.area());

                // Two ways a region can be too small/thin for sparse fill:
                // 1. Total area below threshold (small pockets)
                bool too_small = area < min_area;
                // 2. Effective gap below line spacing (thin rings/strips with large area)
                bool too_thin = false;
                if (!too_small && !ep.holes.empty())
                {
                    BoundingBox bb = get_extents(ep.contour);
                    double max_dim = double(std::max(bb.size().x(), bb.size().y()));
                    double effective_gap = (max_dim > 0) ? (area / max_dim) : 0;
                    too_thin = effective_gap < double(line_spacing);
                }

                if (!too_small && !too_thin)
                {
                    to_keep.push_back(ep);
                    continue;
                }

                // Check if this sparse region is enclosed by the absorber fill.
                // If the intersection with the absorber contours covers >= 90% of the
                // sparse region's area, it's an internal pocket that should be absorbed.
                double contained_area = 0;
                for (const ExPolygon &c : intersection_ex(ExPolygons{ep}, absorber_filled))
                    contained_area += std::abs(c.area());

                if (contained_area >= area * 0.9)
                    to_absorb.push_back(ep);
                else
                    to_keep.push_back(ep);
            }

            if (!to_absorb.empty())
            {
                sparse_fill.expolygons = std::move(to_keep);
                append(absorber.expolygons, std::move(to_absorb));
                absorber.expolygons = union_ex(absorber.expolygons);
            }
        }

        // preFlight: Merge nearby ExPolygons into a unified region. Grow/union/shrink
        // bridges micro-gaps between fragments that plain union can't bridge.
        // stSolidOverBridge uses sparse_erode_radius (gaps proportional to sparse spacing).
        // Others use 1x extrusion width (small classification gaps).
        if (absorber.expolygons.size() > 1)
        {
            const float merge_delta = (absorber.surface.surface_type == stSolidOverBridge && sparse_erode_radius > 0)
                                          ? sparse_erode_radius
                                          : float(scale_(absorber.params.flow.width()));
            Polygons grown;
            for (const ExPolygon &ep : absorber.expolygons)
                append(grown, offset(ep, merge_delta));
            absorber.expolygons = intersection_ex(offset_ex(union_(grown), -merge_delta), total_fill_boundary);
        }
    }

    dbg_fill_phase("ABSORBED_GROWN", layer, surface_fills);

    // preFlight: After grow/union/shrink, solid fills may have expanded into adjacent fills.
    // Re-trim to prevent overlaps: solid fills against each other (priority to earlier entries),
    // then sparse fills against all expanded solid fills.
    {
        Polygons processed_solid;
        for (SurfaceFill &sf : surface_fills)
        {
            if (sf.expolygons.empty())
                continue;
            if (sf.surface.surface_type != stInternalSolid && sf.surface.surface_type != stBridgeAnchor &&
                sf.surface.surface_type != stSolidOverBridge)
                continue;
            if (!processed_solid.empty())
                sf.expolygons = diff_ex(sf.expolygons, processed_solid);
            append(processed_solid, to_polygons(sf.expolygons));
        }
        if (!processed_solid.empty())
            for (SurfaceFill &sf : surface_fills)
                if (sf.surface.surface_type == stInternal && !sf.expolygons.empty())
                {
                    sf.expolygons = diff_ex(sf.expolygons, processed_solid);
                    // Remove thin slivers from the diff at solid/sparse boundaries
                    if (!sf.expolygons.empty())
                    {
                        float min_half_w = float(scale_(sf.params.flow.width() * 0.25));
                        sf.expolygons = opening_ex(sf.expolygons, min_half_w);
                    }
                }
    }

    dbg_fill_phase("RETRIMMED", layer, surface_fills);

    // preFlight: Remove tiny stSolidOverBridge expolygons that are too small for meaningful
    // fill lines. The grow/union/shrink merge can leave behind small fragments near tight
    // features (screw holes, pegs) that overlap perimeters when filled.
    // Use solid fill spacing (not sparse) for the threshold - stSolidOverBridge is 100% density
    // so even small areas produce valid fill. The sparse_min_area threshold (~127mm2 at 16%
    // density) was wildly too large and deleted legitimate SOB regions.
    {
        double sob_min_area = 0;
        for (const SurfaceFill &sf : surface_fills)
            if (sf.surface.surface_type == stSolidOverBridge && !sf.expolygons.empty())
            {
                // Solid fill at 100% density: minimum useful area is a few line widths squared.
                // Use scale_() so area threshold is in nm^2 like ep.area().
                double solid_spacing = scale_(sf.params.spacing);
                sob_min_area = solid_spacing * solid_spacing * 4.0; // 2x2 line grid
                break;
            }
        if (sob_min_area > 0)
            for (SurfaceFill &sf : surface_fills)
                if (sf.surface.surface_type == stSolidOverBridge && !sf.expolygons.empty())
                    sf.expolygons.erase(std::remove_if(sf.expolygons.begin(), sf.expolygons.end(),
                                                       [sob_min_area](const ExPolygon &ep)
                                                       { return std::abs(ep.area()) < sob_min_area; }),
                                        sf.expolygons.end());
    }

    dbg_fill_phase("TINY_SOB_RM", layer, surface_fills);

    // preFlight: Merge fragmented bridge infill into unified regions per angle.
    // Bridge detection creates separate ExPolygons for bridge-over-open-space (stBottomBridge)
    // and bridge-over-sparse (stInternalBridge). Merge fragments that share the same bridge angle
    // into one SurfaceFill each. Bridges with different angles (e.g. counterbore corridors)
    // remain separate to preserve their per-corridor fill direction.
    if (sparse_erode_radius > 0)
    {
        // Group bridge SurfaceFills by angle (within 5 degrees tolerance).
        // For each angle group, pick a primary (prefer stBottomBridge, then largest area)
        // and merge other same-angle bridges into it.
        static constexpr double angle_merge_tolerance = 5.0 * M_PI / 180.0;

        // Collect bridge indices
        std::vector<size_t> bridge_indices;
        for (size_t i = 0; i < surface_fills.size(); ++i)
            if (!surface_fills[i].expolygons.empty() && surface_fills[i].surface.is_bridge())
                bridge_indices.push_back(i);

        // Align bridge angles to counterbore corridor angles so that stInternalBridge
        // (from bridge_over_infill) and stBottomBridge (from BridgeDetector) that overlap
        // the same counterbore corridor end up in the same angle group.
        if (!layer.counterbore_bridge_regions.empty())
        {
            for (size_t bi : bridge_indices)
            {
                SurfaceFill &sf = surface_fills[bi];
                double sf_area = 0;
                for (const ExPolygon &ep : sf.expolygons)
                    sf_area += std::abs(ep.area());
                if (sf_area <= 0)
                    continue;
                BoundingBox sf_bb = get_extents(sf.expolygons);
                for (const auto &[cb_region, cb_angle] : layer.counterbore_bridge_regions)
                {
                    if (!sf_bb.overlap(get_extents(cb_region)))
                        continue;
                    double overlap_area = 0;
                    for (const ExPolygon &ov : intersection_ex(sf.expolygons, cb_region))
                        overlap_area += std::abs(ov.area());
                    if (overlap_area / sf_area > 0.01)
                    {
                        sf.surface.bridge_angle = float(cb_angle);
                        break;
                    }
                }
            }
        }

        // Group by angle: each entry is (primary_idx, list of same-angle indices)
        std::vector<std::pair<size_t, std::vector<size_t>>> angle_groups;
        std::vector<bool> assigned(surface_fills.size(), false);
        for (size_t bi : bridge_indices)
        {
            if (assigned[bi])
                continue;
            double ref_angle = fmod(surface_fills[bi].surface.bridge_angle + 2 * M_PI, M_PI);
            std::vector<size_t> group = {bi};
            assigned[bi] = true;
            for (size_t bj : bridge_indices)
            {
                if (assigned[bj])
                    continue;
                double a = fmod(surface_fills[bj].surface.bridge_angle + 2 * M_PI, M_PI);
                double diff = std::abs(a - ref_angle);
                if (diff < angle_merge_tolerance || diff > M_PI - angle_merge_tolerance)
                {
                    group.push_back(bj);
                    assigned[bj] = true;
                }
            }

            // Pick primary within this angle group (prefer stBottomBridge, then largest area)
            size_t primary_idx = group[0];
            double primary_area = 0;
            for (size_t gi : group)
            {
                double area = 0;
                for (const ExPolygon &ep : surface_fills[gi].expolygons)
                    area += std::abs(ep.area());
                bool better = (surface_fills[gi].surface.surface_type == stBottomBridge &&
                               surface_fills[primary_idx].surface.surface_type != stBottomBridge) ||
                              (surface_fills[gi].surface.surface_type ==
                                   surface_fills[primary_idx].surface.surface_type &&
                               area > primary_area);
                if (gi == group[0] || better)
                {
                    primary_idx = gi;
                    primary_area = area;
                }
            }
            angle_groups.push_back({primary_idx, group});
        }

        // Merge each angle group
        Polygons all_bridge_polys;
        for (auto &[primary_idx, group] : angle_groups)
        {
            SurfaceFill &primary = surface_fills[primary_idx];
            bool merged = false;
            for (size_t gi : group)
            {
                if (gi == primary_idx)
                    continue;
                append(primary.expolygons, std::move(surface_fills[gi].expolygons));
                surface_fills[gi].expolygons.clear();
                merged = true;
            }

            // Grow/union/shrink to bridge micro-gaps between fragments
            if (primary.expolygons.size() > 1)
            {
                const float merge_delta = float(scale_(primary.params.flow.width()));
                Polygons grown;
                for (const ExPolygon &ep : primary.expolygons)
                    append(grown, offset(ep, merge_delta));
                primary.expolygons = intersection_ex(offset_ex(union_(grown), -merge_delta), total_fill_boundary);
            }

            // Clip against previously processed bridge groups to prevent polygon overlap.
            // Without this, grow/union/shrink can extend a group beyond its trimmed boundary
            // into another group's territory, causing fill lines to cross.
            if (!all_bridge_polys.empty())
                primary.expolygons = diff_ex(primary.expolygons, all_bridge_polys);

            append(all_bridge_polys, to_polygons(primary.expolygons));
        }

        // Re-trim non-bridge fills against expanded bridge regions. Grow bridge
        // boundary by half the sparse flow width so thin borders between bridge
        // and perimeters get consumed - these can't fit a single sparse extrusion.
        // Also expand bridge fills to cover the consumed area.
        if (!all_bridge_polys.empty())
        {
            // Find sparse flow width for the growth amount
            float sparse_half_flow = 0;
            for (const SurfaceFill &sf : surface_fills)
                if (sf.surface.surface_type == stInternal && !sf.expolygons.empty() && sf.params.density < 99.f)
                {
                    sparse_half_flow = float(scale_(sf.params.flow.width() * 0.5));
                    break;
                }

            Polygons bridge_trim = sparse_half_flow > 0 ? offset(all_bridge_polys, sparse_half_flow) : all_bridge_polys;

            // Expand bridge fills to cover the grown boundary
            if (sparse_half_flow > 0)
                for (SurfaceFill &sf : surface_fills)
                    if (!sf.expolygons.empty() && sf.surface.is_bridge())
                    {
                        sf.expolygons = intersection_ex(offset_ex(sf.expolygons, sparse_half_flow),
                                                        total_fill_boundary);
                        // Re-clip against other bridge groups to prevent overlap
                        Polygons other_bridges;
                        for (const SurfaceFill &other : surface_fills)
                            if (&other != &sf && !other.expolygons.empty() && other.surface.is_bridge())
                                append(other_bridges, to_polygons(other.expolygons));
                        if (!other_bridges.empty())
                            sf.expolygons = diff_ex(sf.expolygons, other_bridges);
                    }

            // Trim non-bridge fills: sparse against grown boundary (consumes
            // thin slivers), all others against original boundary only.
            for (SurfaceFill &sf : surface_fills)
            {
                if (sf.expolygons.empty() || sf.surface.is_bridge())
                    continue;
                if (sf.surface.surface_type == stInternal)
                    sf.expolygons = diff_ex(sf.expolygons, bridge_trim);
                else
                    sf.expolygons = diff_ex(sf.expolygons, all_bridge_polys);
            }
        }
    }

    dbg_fill_phase("BRIDGE_MERGED", layer, surface_fills);

    // we need to detect any narrow surfaces that might collapse
    // when adding spacing below
    // such narrow surfaces are often generated in sloping walls
    // by bridge_over_infill() and combine_infill() as a result of the
    // subtraction of the combinable area from the layer infill area,
    // which leaves small areas near the perimeters
    // we are going to grow such regions by overlapping them with the void (if any)
    // TODO: detect and investigate whether there could be narrow regions without
    // any void neighbors
    if (has_internal_voids)
    {
        // Internal voids are generated only if "infill_only_where_needed" or "infill_every_layers" are active.
        coord_t distance_between_surfaces = 0;
        Polygons surfaces_polygons;
        Polygons voids;
        int region_internal_infill = -1;
        int region_solid_infill = -1;
        int region_some_infill = -1;
        for (SurfaceFill &surface_fill : surface_fills)
            if (!surface_fill.expolygons.empty())
            {
                distance_between_surfaces = std::max(distance_between_surfaces,
                                                     surface_fill.params.flow.scaled_spacing());
                append((surface_fill.surface.surface_type == stInternalVoid) ? voids : surfaces_polygons,
                       to_polygons(surface_fill.expolygons));
                if (surface_fill.surface.surface_type == stInternalSolid)
                    region_internal_infill = (int) surface_fill.region_id;
                if (surface_fill.surface.is_solid())
                    region_solid_infill = (int) surface_fill.region_id;
                if (surface_fill.surface.surface_type != stInternalVoid)
                    region_some_infill = (int) surface_fill.region_id;
            }
        if (!voids.empty() && !surfaces_polygons.empty())
        {
            // First clip voids by the printing polygons, as the voids were ignored by the loop above during mutual clipping.
            voids = diff(voids, surfaces_polygons);
            // Corners of infill regions, which would not be filled with an extrusion path with a radius of distance_between_surfaces/2
            Polygons collapsed = diff(surfaces_polygons,
                                      opening(surfaces_polygons, float(distance_between_surfaces / 2),
                                              float(distance_between_surfaces / 2 + ClipperSafetyOffset)));
            //FIXME why the voids are added to collapsed here? First it is expensive, second the result may lead to some unwanted regions being
            // added if two offsetted void regions merge.
            // polygons_append(voids, collapsed);
            ExPolygons extensions = intersection_ex(expand(collapsed, float(distance_between_surfaces)), voids,
                                                    ApplySafetyOffset::Yes);
            // Now find an internal infill SurfaceFill to add these extrusions to.
            SurfaceFill *internal_solid_fill = nullptr;
            unsigned int region_id = 0;
            if (region_internal_infill != -1)
                region_id = region_internal_infill;
            else if (region_solid_infill != -1)
                region_id = region_solid_infill;
            else if (region_some_infill != -1)
                region_id = region_some_infill;
            const LayerRegion &layerm = *layer.regions()[region_id];
            for (SurfaceFill &surface_fill : surface_fills)
                if (surface_fill.surface.surface_type == stInternalSolid &&
                    std::abs(layer.height - surface_fill.params.flow.height()) < EPSILON)
                {
                    internal_solid_fill = &surface_fill;
                    break;
                }
            if (internal_solid_fill == nullptr)
            {
                // Produce another solid fill.
                params.extruder = layerm.region().extruder(frSolidInfill);
                params.pattern = layerm.region().config().solid_fill_pattern.value;
                params.density = 100.f;
                params.extrusion_role = ExtrusionRole::InternalInfill;
                params.angle = float(Geometry::deg2rad(layerm.region().config().fill_angle.value));
                // calculate the actual flow we'll be using for this infill
                params.flow = layerm.flow(frSolidInfill);
                params.spacing = params.flow.spacing();
                surface_fills.emplace_back(params);
                surface_fills.back().surface.surface_type = stInternalSolid;
                surface_fills.back().surface.thickness = layer.height;
                surface_fills.back().expolygons = std::move(extensions);
            }
            else
            {
                append(extensions, std::move(internal_solid_fill->expolygons));
                internal_solid_fill->expolygons = union_ex(extensions);
            }
        }
    }

    // This was forcing ALL stInternalSolid surfaces to use ipEnsuring (Athena-style fill),
    // overriding the user's top_fill_pattern selection (Monotonic, Rectilinear, etc.).
    // ipEnsuring uses WallToolPaths which doesn't adjust spacing like FillRectilinear does,
    // causing gaps when fill runs parallel to the area.
    // Comment out this forced override to respect the user's pattern selection.
    /*
    // Use ipEnsuring pattern for all internal Solids.
    {
        for (size_t surface_fill_id = 0; surface_fill_id < surface_fills.size(); ++surface_fill_id)
            if (SurfaceFill &fill = surface_fills[surface_fill_id];
                    fill.surface.surface_type == stInternalSolid
                    || fill.surface.surface_type == stSolidOverBridge) {
                fill.params.pattern = ipEnsuring;
            }
    }
    */

    // preFlight: Narrow-to-Athena width split. Runs on settled geometry so only genuinely
    // thin sub-regions (thin frames around pockets, necks, spikes) are converted to a smooth
    // concentric bead - the wide bulk keeps its configured solid pattern. A morphological
    // opening at the threshold width isolates the bulk; the thin remainder becomes its own
    // concentric SurfaceFill. This replaces the old all-or-nothing per-surface pattern flip.
    {
        std::vector<SurfaceFill> narrow_fills;
        for (size_t fi = 0; fi < surface_fills.size(); ++fi)
        {
            SurfaceFill &sf = surface_fills[fi];
            if (sf.expolygons.empty() || sf.region_id == size_t(-1))
                continue;
            const SurfaceType st = sf.surface.surface_type;
            if (st != stInternalSolid && st != stTop && st != stBottom)
                continue;
            if (sf.surface.is_bridge() || sf.params.pattern == ipConcentric || sf.params.pattern == ipEnsuring)
                continue;
            const PrintRegionConfig &rc = layer.regions()[sf.region_id]->region().config();
            if (!rc.narrow_to_athena.value)
                continue;
            // Top/bottom are visible surfaces; only split them when the user opts in. Internal
            // solid always splits when Narrow-to-Athena is on.
            if ((st == stTop || st == stBottom) && !rc.narrow_to_athena_top_bottom.value)
                continue;

            // Opening radius: features narrower than threshold extrusion widths collapse.
            const float open_r = float(sf.params.flow.scaled_width()) * float(rc.narrow_to_athena_threshold.value) /
                                 2.f;
            if (open_r <= 0.f)
                continue;
            // Tiny narrow scraps are folded back into the wide pass (perimeter-covered, too
            // small for a clean bead) so no area is ever left unfilled.
            const double min_frag = sqr(double(sf.params.flow.scaled_width()) * 2.0);

            ExPolygons wide_all, narrow_all;
            auto ex_area = [](const ExPolygons &v)
            {
                double a = 0;
                for (const ExPolygon &e : v)
                    a += std::abs(e.area());
                return a;
            };
            // Self-heal tolerance: a real boolean failure drops a whole sub-region (>= a couple
            // fragments). Safety-offset slivers are ~1e-3mm2, far below this, so the coverage
            // revert never false-triggers on normal geometry.
            const double coverage_tol = 2.0 * min_frag;

            for (const ExPolygon &ep : sf.expolygons)
            {
                ExPolygons ep_in{ep};
                // Morphological opening with explicit hole handling. The offset/opening helpers
                // use perimeter-band hole semantics (negative delta SHRINKS holes), so they cannot
                // erode through a thin frame around a pocket. Erode here = shrink the contour and
                // grow each hole (as a positive region) then subtract; dilate the result back.
                ExPolygons eroded = offset_ex(Polygons{ep.contour}, -open_r);
                if (!ep.holes.empty())
                {
                    Polygons holes_grown;
                    for (const Polygon &h : ep.holes)
                    {
                        Polygon hc = h;
                        hc.make_counter_clockwise();
                        append(holes_grown, offset(hc, open_r));
                    }
                    eroded = diff_ex(eroded, holes_grown);
                }
                // Safety offset on these booleans: wide_core/narrow share a boundary coincident
                // with ep, so a plain diff/intersection can leave zero-width slivers. Matches the
                // ApplySafetyOffset::Yes convention used by the inter-fill trim pass above.
                ExPolygons wide_core = intersection_ex(ep_in, offset_ex(eroded, open_r), ApplySafetyOffset::Yes);
                ExPolygons narrow_raw = diff_ex(ep_in, wide_core, ApplySafetyOffset::Yes);

                // Classify this ExPolygon's outcome. For a real split, self-heal against coverage
                // loss: wide + narrow must reconstitute ep. If a boolean ever drops a region (which
                // would ship a gap in solid infill), revert this ep to un-split - the safe
                // pre-feature behavior - and flag it. The check runs in release too; only the
                // logging below is gated by --debug fill.
                const char *result;
                bool reverted = false;
                double lost = 0.0;
                ExPolygons ep_wide, ep_narrow;
                if (wide_core.empty())
                {
                    result = "whole_narrow"; // entire surface is narrower than threshold
                    ep_narrow = ep_in;
                }
                else
                {
                    ExPolygons narrow_kept;
                    for (ExPolygon &nf : narrow_raw)
                        if (std::abs(nf.area()) >= min_frag)
                            narrow_kept.emplace_back(std::move(nf));
                    if (narrow_kept.empty())
                    {
                        result = "kept_wide"; // nothing thin enough survived the area floor
                        ep_wide = ep_in;
                    }
                    else
                    {
                        // wide and narrow tile ep. The seam is bonded by the concentric filler
                        // itself: FillConcentric expands its region by half a spacing before
                        // generating beads, so the outermost Athena bead overspills into wide the
                        // same way concentric bonds against perimeters. No artificial grow.
                        ExPolygons w = diff_ex(ep_in, narrow_kept, ApplySafetyOffset::Yes);
                        lost = ex_area(ep_in) - (ex_area(w) + ex_area(narrow_kept));
                        if (lost > coverage_tol)
                        {
                            result = "reverted"; // coverage loss - do not ship a gap
                            reverted = true;
                            ep_wide = ep_in;
                        }
                        else
                        {
                            result = "split";
                            ep_wide = std::move(w);
                            ep_narrow = std::move(narrow_kept);
                        }
                    }
                }

                if (Slic3r::debug_enabled(Slic3r::DBG_FILL))
                {
                    auto sum = [](const ExPolygons &v)
                    {
                        double a = 0;
                        for (const ExPolygon &e : v)
                            a += std::abs(e.area());
                        return a * SCALING_FACTOR * SCALING_FACTOR;
                    };
                    // NARROW_DIAG: per-ExPolygon metrics + outcome (split / whole_narrow / kept_wide
                    // / reverted). The first look when a Narrow-to-Athena issue is reported.
                    dbg_log(Slic3r::DBG_FILL, layer.print_z, "FILL",
                            "NARROW_DIAG type=%-18s result=%-12s w=%.4fmm thr=%.2f open_r=%.4fmm "
                            "minfrag=%.4fmm2 holes=%zu ep=%.3f eroded=%.3f wide=%.3f narrow=%.3f mm2",
                            dbg_stype(st), result, sf.params.flow.width(), double(rc.narrow_to_athena_threshold.value),
                            double(open_r) * SCALING_FACTOR, min_frag * SCALING_FACTOR * SCALING_FACTOR,
                            ep.holes.size(), sum(ep_in), sum(eroded), sum(wide_core), sum(narrow_raw));
                    // NARROW_REJECT: the red flag. Only emitted on a self-heal revert - grep this
                    // first when a Narrow-to-Athena issue is reported; if absent, no coverage was lost.
                    if (reverted)
                    {
                        BoundingBox rb = get_extents(ep_in);
                        dbg_log(Slic3r::DBG_FILL, layer.print_z, "FILL",
                                "NARROW_REJECT reason=coverage_loss type=%-18s ep=%.4fmm2 "
                                "lost=%.4fmm2 bbox=(%.2f,%.2f)-(%.2f,%.2f)",
                                dbg_stype(st), sum(ep_in), lost * SCALING_FACTOR * SCALING_FACTOR,
                                unscaled<double>(rb.min.x()), unscaled<double>(rb.min.y()),
                                unscaled<double>(rb.max.x()), unscaled<double>(rb.max.y()));
                    }
                    // NARROW_PTS: verbose forensic - exact contour/hole points in mm so the true
                    // shape can be rendered offline when a bbox is misleading (the usual trap here).
                    auto dump_ring = [&](const char *tag, const Polygon &poly)
                    {
                        std::string s;
                        char buf[64];
                        for (const Point &pt : poly.points)
                        {
                            snprintf(buf, sizeof(buf), "%.3f,%.3f ", unscaled<double>(pt.x()),
                                     unscaled<double>(pt.y()));
                            s += buf;
                        }
                        dbg_log(Slic3r::DBG_FILL, layer.print_z, "FILL", "NARROW_PTS %s n=%zu %s", tag,
                                poly.points.size(), s.c_str());
                    };
                    dump_ring("C", ep.contour);
                    for (const Polygon &h : ep.holes)
                        dump_ring("H", h);
                }

                append(wide_all, std::move(ep_wide));
                append(narrow_all, std::move(ep_narrow));
            }

            if (narrow_all.empty())
                continue; // no split for this fill

            const size_t narrow_ep = narrow_all.size();
            sf.expolygons = std::move(wide_all);

            SurfaceFillParams np = sf.params;
            np.pattern = ipConcentric;
            SurfaceFill nf(np);
            nf.region_id = sf.region_id;
            nf.surface = sf.surface;
            nf.expolygons = std::move(narrow_all);
            narrow_fills.emplace_back(std::move(nf));

            dbg_log(Slic3r::DBG_FILL, layer.print_z, "FILL", "NARROW_SPLIT type=%-18s wide_ep=%zu narrow_ep=%zu",
                    dbg_stype(st), sf.expolygons.size(), narrow_ep);
        }
        for (SurfaceFill &nf : narrow_fills)
        {
            nf.params.idx = surface_fills.size();
            surface_fills.emplace_back(std::move(nf));
        }
    }

    dbg_fill_phase("FINAL", layer, surface_fills);

    return surface_fills;
}

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
void export_group_fills_to_svg(const char *path, const std::vector<SurfaceFill> &fills)
{
    BoundingBox bbox;
    for (const auto &fill : fills)
        for (const auto &expoly : fill.expolygons)
            bbox.merge(get_extents(expoly));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const auto &fill : fills)
        for (const auto &expoly : fill.expolygons)
            svg.draw(expoly, surface_type_to_color_name(fill.surface.surface_type), transparency);
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}
#endif

// Regular infill is assigned directly to islands in make_fills().
// This function is used by make_ironing() which runs after make_fills() and needs to
// assign ironing fills to islands so the G-code exporter can find them.
static void insert_fills_into_islands(Layer &layer, uint32_t fill_region_id, uint32_t fill_begin, uint32_t fill_end)
{
    if (fill_begin >= fill_end)
        return;

    const LayerRegion &layerm = *layer.get_region(fill_region_id);

    for (uint32_t fill_idx = fill_begin; fill_idx < fill_end; ++fill_idx)
    {
        const ExtrusionEntity *ee = layerm.fills().entities[fill_idx];
        Point rep_point = ee->first_point();

        // Find the island whose boundary contains this fill's representative point
        for (LayerSlice &lslice : layer.lslices_ex)
            for (LayerIsland &island : lslice.islands)
                if (island.boundary.contains(rep_point))
                {
                    island.add_fill_range(LayerExtrusionRange{fill_region_id, {fill_idx, fill_idx + 1}});
                    goto next_fill;
                }

        // Fallback: if no island contains the point (e.g. due to trimming), assign to the nearest island
        {
            double best_dist_sq = std::numeric_limits<double>::max();
            LayerIsland *best_island = nullptr;
            for (LayerSlice &lslice : layer.lslices_ex)
                for (LayerIsland &island : lslice.islands)
                {
                    Point centroid = island.boundary.contour.centroid();
                    double d = (centroid - rep_point).cast<double>().squaredNorm();
                    if (d < best_dist_sq)
                    {
                        best_dist_sq = d;
                        best_island = &island;
                    }
                }
            if (best_island)
                best_island->add_fill_range(LayerExtrusionRange{fill_region_id, {fill_idx, fill_idx + 1}});
        }

    next_fill:;
    }
}

static int region_for_filament(const Layer &layer, int filament)
{
    if (filament < 0)
        return -1;
    const int want = filament + 1;
    for (size_t i = 0; i < layer.regions().size(); ++i)
        if (layer.regions()[i]->region().config().solid_infill_extruder.value == want)
            return (int) i;
    return -1;
}

static std::vector<int> unique_in_order(const std::vector<int> &pattern)
{
    std::vector<int> uniq;
    for (int filament : pattern)
        if (std::find(uniq.begin(), uniq.end(), filament) == uniq.end())
            uniq.push_back(filament);
    return uniq;
}

// Split a polyline wherever the travel direction turns. Monotonic infill is one
// zigzag; each straight run is one scan line or the short link between two lines.
static std::vector<Polyline> split_straight_runs(const Polyline &pl)
{
    std::vector<Polyline> runs;
    if (pl.points.size() < 2)
        return runs;
    const double min_dot = std::cos(20.0 * M_PI / 180.0);
    Polyline cur;
    cur.points.push_back(pl.points.front());
    Vec2d dir{0., 0.};
    bool has_dir = false;
    for (size_t i = 1; i < pl.points.size(); ++i)
    {
        const Point &p = pl.points[i];
        if (p == cur.points.back())
            continue;
        Vec2d step = (p - cur.points.back()).cast<double>();
        const double len = step.norm();
        if (len < 1.0)
            continue;
        step /= len;
        if (has_dir && dir.dot(step) < min_dot)
        {
            const Point corner = cur.points.back();
            if (cur.size() >= 2)
                runs.emplace_back(std::move(cur));
            cur = Polyline();
            cur.points.push_back(corner);
            has_dir = false;
        }
        cur.points.push_back(p);
        dir = step;
        has_dir = true;
    }
    if (cur.size() >= 2)
        runs.emplace_back(std::move(cur));
    return runs;
}

static Point point_at(const Point &a, const Point &b, double t)
{
    return Point{coord_t(std::llround(double(a.x()) + t * (double(b.x()) - double(a.x())))),
                 coord_t(std::llround(double(a.y()) + t * (double(b.y()) - double(a.y()))))};
}

// Cut a scan line into steps about one bead long. The mix pattern walks these steps, and
// neighboring lines are shifted so the minority filament is scattered instead of striped.
static std::vector<Polyline> split_by_length(const Polyline &pl, double step)
{
    std::vector<Polyline> out;
    if (pl.points.size() < 2 || step < 1.0)
        return out;
    Polyline cur;
    cur.points.push_back(pl.points.front());
    double acc = 0;
    for (size_t i = 1; i < pl.points.size(); ++i)
    {
        Point target = pl.points[i];
        while (cur.points.back() != target)
        {
            const Point from = cur.points.back();
            const double seg = (target - from).cast<double>().norm();
            if (seg < 1.0)
                break;
            if (acc + seg < step)
            {
                cur.points.push_back(target);
                acc += seg;
                break;
            }
            const double t = std::min(1.0, (step - acc) / seg);
            const Point cut = point_at(from, target, t);
            if (cut == from)
            {
                cur.points.push_back(target);
                acc += seg;
                break;
            }
            cur.points.push_back(cut);
            if (cur.size() >= 2)
                out.emplace_back(std::move(cur));
            cur = Polyline();
            cur.points.push_back(cut);
            acc = 0;
        }
    }
    if (cur.size() >= 2)
    {
        if (!out.empty() && cur.length() < step * 0.5 && out.back().points.back() == cur.points.front())
            out.back().points.insert(out.back().points.end(), cur.points.begin() + 1, cur.points.end());
        else
            out.emplace_back(std::move(cur));
    }
    return out;
}

struct ColorMixWallPiece
{
    Polyline pl;
    ExtrusionAttributes attrs;
};

static void collect_wall_pieces(const ExtrusionEntity *entity, std::vector<ColorMixWallPiece> &out)
{
    auto take = [&](const ExtrusionPath &path) {
        if (path.polyline.size() >= 2 && path.role().has(ExtrusionRoleModifier::Perimeter))
            out.push_back(ColorMixWallPiece{path.polyline, path.attributes()});
    };
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
        take(*path);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
        for (const ExtrusionPath &path : loop->paths)
            take(path);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
        for (const ExtrusionPath &path : multi->paths)
            take(path);
}

// Walk the mix along a wall. Consecutive steps of the same filament stay one path.
// Returns how many steps the pattern advanced, so the next piece can continue it.
static int dither_wall_polyline(const Polyline &pl, const ExtrusionAttributes &attrs, const std::vector<int> &pattern,
                                int phase, double step, std::vector<ColorMixWallPiece> &out, std::vector<int> &filaments)
{
    const std::vector<Polyline> steps = split_by_length(pl, step);
    const int n = (int) pattern.size();
    if (n <= 0 || steps.empty())
        return 0;
    int run_filament = -1;
    Polyline run;
    auto flush = [&]() {
        if (run.size() >= 2 && run_filament >= 0)
        {
            out.push_back(ColorMixWallPiece{std::move(run), attrs});
            filaments.push_back(run_filament);
        }
        run.clear();
    };
    for (size_t s = 0; s < steps.size(); ++s)
    {
        const int slot = ((int(s) + phase) % n + n) % n;
        const int filament = pattern[(size_t) slot];
        if (filament != run_filament)
        {
            flush();
            run_filament = filament;
        }
        if (run.empty())
            run = steps[s];
        else if (run.points.back() == steps[s].points.front())
            run.points.insert(run.points.end(), steps[s].points.begin() + 1, steps[s].points.end());
        else
        {
            flush();
            run = steps[s];
            run_filament = filament;
        }
    }
    flush();
    return (int) steps.size();
}

// How many pattern slots one filament holds before the mix changes. A 50/50
// cycle of A A A B B B shifts by that whole run, so the other filament lands
// on top. An alternating A B cycle shifts by one slot.
static int pattern_run_slots(const std::vector<int> &pattern)
{
    const int n = (int) pattern.size();
    if (n <= 1)
        return 1;
    int run = 1;
    while (run < n && pattern[(size_t) run] == pattern[0])
        ++run;
    return run >= n ? 1 : run;
}

static double polyline_area2(const Polyline &pl)
{
    double area = 0.;
    const size_t n = pl.points.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++)
        area += double(pl.points[j].x()) * double(pl.points[i].y()) -
                double(pl.points[i].x()) * double(pl.points[j].y());
    return area;
}

// Cut `len` off the front of `pl`. The remainder starts at the cut.
static Polyline consume_prefix(Polyline &pl, double len)
{
    Polyline head;
    if (pl.points.size() < 2 || len <= 1.)
        return head;
    head.points.push_back(pl.points.front());
    double left = len;
    size_t i = 1;
    for (; i < pl.points.size(); ++i)
    {
        const Point target = pl.points[i];
        const Point from = head.points.back();
        const double seg = (target - from).cast<double>().norm();
        if (seg < 1.)
            continue;
        if (left >= seg - 1.)
        {
            head.points.push_back(target);
            left -= seg;
            if (left <= 1.)
            {
                ++i;
                break;
            }
        }
        else
        {
            const Point cut = point_at(from, target, std::min(1.0, left / seg));
            head.points.push_back(cut);
            pl.points.erase(pl.points.begin(), pl.points.begin() + (long) i);
            pl.points.insert(pl.points.begin(), cut);
            return head;
        }
    }
    pl.points.erase(pl.points.begin(), pl.points.begin() + (long) std::min(i, pl.points.size()));
    if (!head.points.empty() && (pl.points.empty() || pl.points.front() != head.points.back()))
        pl.points.insert(pl.points.begin(), head.points.back());
    return head;
}

// Brick the mix from the leftmost point of the wall. Each layer the joint moves
// by half a color run, so the other filament sits on top of the previous one.
// Counting from the loop start does not do this: that start walks with the seam
// and the joints stay stacked.
static void dither_anchored_wall(Polyline pl, const ExtrusionAttributes &attrs, const std::vector<int> &pattern,
                                 int layer_id, double step, std::vector<ColorMixWallPiece> &out,
                                 std::vector<int> &filaments)
{
    const int n = (int) pattern.size();
    if (n <= 0 || pl.points.size() < 2 || step < 1.)
        return;
    if (pl.points.size() >= 3 && polyline_area2(pl) < -1.)
        std::reverse(pl.points.begin(), pl.points.end());

    size_t anchor = 0;
    for (size_t i = 1; i < pl.points.size(); ++i)
        if (pl.points[i].x() < pl.points[anchor].x() ||
            (pl.points[i].x() == pl.points[anchor].x() && pl.points[i].y() < pl.points[anchor].y()))
            anchor = i;
    double dist_to_anchor = 0.;
    for (size_t i = 0; i < anchor; ++i)
        dist_to_anchor += (pl.points[i + 1] - pl.points[i]).cast<double>().norm();

    const int run = pattern_run_slots(pattern);
    const double period = step * double(n);
    const double brick = 0.5 * step * double(run);
    double coord = -dist_to_anchor + double(layer_id) * brick;
    coord = std::fmod(coord, period);
    if (coord < 0.)
        coord += period;

    int phase = (int) std::floor(coord / step + 1e-9) % n;
    const double into = coord - double(phase) * step;
    double first_len = step - into;
    if (first_len < step * 0.15)
    {
        phase = (phase + 1) % n;
        first_len = step;
    }
    if (first_len < step - 1.)
    {
        Polyline head = consume_prefix(pl, first_len);
        if (head.points.size() >= 2)
        {
            out.push_back(ColorMixWallPiece{std::move(head), attrs});
            filaments.push_back(pattern[(size_t) phase]);
        }
        phase = (phase + 1) % n;
    }
    if (pl.points.size() >= 2)
        dither_wall_polyline(pl, attrs, pattern, phase, step, out, filaments);
}

std::vector<Layer::ColorMixTopStripe> Layer::clip_color_mix_tops()
{
    std::vector<ColorMixTopStripe> jobs;
    if (color_mix_top_stripes.empty())
        return jobs;

    ExPolygons tops;
    for (LayerRegion *lr : m_regions)
        for (const Surface &s : lr->m_fill_surfaces.surfaces)
            if (s.surface_type == stTop)
                tops.emplace_back(s.expolygon);

    for (ColorMixTopStripe &stripe : color_mix_top_stripes)
    {
        if (stripe.pattern.empty() || stripe.area.empty() || stripe.coex_count >= 2)
            continue;
        const std::vector<int> uniq = unique_in_order(stripe.pattern);
        if (uniq.size() != 2 && uniq.size() != 3)
            continue;
        bool have_regions = true;
        for (int filament : uniq)
            if (region_for_filament(*this, filament) < 0)
                have_regions = false;
        if (!have_regions)
            continue;
        // The top surface shell dithers along its solid infill. Sparse infill does
        // not. The outer-wall mix is a separate copy and is not clipped here.
        const bool on_top = !tops.empty() && !intersection_ex(tops, stripe.area).empty();
        if (!on_top && !stripe.dither_infill)
            continue;
        jobs.push_back({std::move(stripe.area), std::move(stripe.pattern), stripe.coex_count,
                        stripe.coex_rotation_deg, stripe.dither_infill});
    }
    color_mix_top_stripes.clear();
    return jobs;
}

void Layer::emit_color_mix_top_lines(const std::vector<ColorMixTopStripe> &jobs)
{
    if (jobs.empty())
        return;

    std::vector<std::vector<int>> uniq_by_job;
    uniq_by_job.reserve(jobs.size());
    ExPolygons mask;
    for (const ColorMixTopStripe &job : jobs)
    {
        uniq_by_job.push_back(unique_in_order(job.pattern));
        append(mask, job.area);
    }
    if (mask.empty())
        return;
    const BoundingBox mask_bb = get_extents(mask);
    bool dither_under = false;
    for (const ColorMixTopStripe &job : jobs)
        if (job.dither_infill)
            dither_under = true;
    const double align_min = std::cos(35.0 * M_PI / 180.0);
    const double join_gap = double(scale_(0.05));

    struct Moved
    {
        double proj;
        double pitch;
        int job;
        int angle_key;
        ExtrusionAttributes attrs;
        bool oriented;
        Polyline pl;
    };
    struct Kept
    {
        Polyline pl;
        ExtrusionAttributes attrs;
        bool oriented;
    };

    for (LayerSlice &lslice : this->lslices_ex)
        for (LayerIsland &island : lslice.islands)
        {
            std::vector<Moved> moved;
            const size_t n_ranges = island.fills.size();
            for (size_t ri = 0; ri < n_ranges; ++ri)
            {
                const LayerExtrusionRange range = island.fills[ri];
                if (range.region() >= m_regions.size())
                    continue;
                LayerRegion *lr = this->get_region((int) range.region());
                const float fill_angle = float(Geometry::deg2rad(lr->region().config().fill_angle.value));
                const Vec2d fill_axis{std::cos(double(fill_angle)), std::sin(double(fill_angle))};
                // Rectilinear / monotonic lines run along the fill angle. Spacing is measured on the
                // perpendicular, which is the direction _infill_direction rotates the polygon by.
                const float spacing_axis = fill_angle + float(M_PI / 2.);
                const double axis_x = std::cos(double(spacing_axis));
                const double axis_y = std::sin(double(spacing_axis));
                const int angle_key = (int) std::lround(double(fill_angle) / (5.0 * M_PI / 180.0));
                auto spacing_for = [&](const ExtrusionRole &role) {
                    FlowRole flow_role = frTopSolidInfill;
                    if (role == ExtrusionRole::InternalInfill)
                        flow_role = frInfill;
                    else if (role.is_solid_infill() && role != ExtrusionRole::TopSolidInfill)
                        flow_role = frSolidInfill;
                    return std::max(double(scale_(0.05)), double(lr->flow(flow_role).scaled_spacing()));
                };

                for (uint32_t ei = *range.begin(); ei < *range.end(); ++ei)
                {
                    auto *eec = dynamic_cast<ExtrusionEntityCollection *>(lr->m_fills.entities[ei]);
                    if (eec == nullptr)
                        continue;

                    ExtrusionEntitiesPtr rebuilt;
                    rebuilt.reserve(eec->entities.size());
                    bool changed = false;
                    for (ExtrusionEntity *child : eec->entities)
                    {
                        auto *path = dynamic_cast<ExtrusionPath *>(child);
                        if (path == nullptr || !mask_bb.overlap(path->polyline.bounding_box()))
                        {
                            rebuilt.push_back(child);
                            continue;
                        }
                        const ExtrusionRole role = path->role();
                        const bool top_solid = role == ExtrusionRole::TopSolidInfill;
                        // The solid layers of the top shell dither too. Sparse infill, bridges,
                        // and solid that is not the top shell stay one filament.
                        const bool top_shell = dither_under && role == ExtrusionRole::SolidInfill;
                        if (!top_solid && !top_shell)
                        {
                            rebuilt.push_back(child);
                            continue;
                        }
                        const double pitch = spacing_for(role);

                        const ExtrusionAttributes attrs = path->attributes();
                        const bool oriented = !path->can_reverse();
                        const std::vector<Polyline> runs = split_straight_runs(path->polyline);
                        Vec2d line_axis = fill_axis;
                        bool saw_fill_line = false;
                        double longest_other = 0;
                        Vec2d other_axis = fill_axis;
                        for (const Polyline &run : runs)
                        {
                            if (run.length() < pitch)
                                continue;
                            Vec2d step = (run.points.back() - run.points.front()).cast<double>();
                            const double len = step.norm();
                            if (len < 1.0)
                                continue;
                            step /= len;
                            if (std::abs(step.dot(fill_axis)) >= align_min)
                                saw_fill_line = true;
                            else if (len > longest_other)
                            {
                                longest_other = len;
                                other_axis = step;
                            }
                        }
                        if (!saw_fill_line)
                            line_axis = other_axis;

                        std::vector<Kept> kept;
                        bool touched = false;
                        auto append_kept = [&](Polyline pl) {
                            if (pl.size() < 2)
                                return;
                            if (!kept.empty() &&
                                (kept.back().pl.points.back() - pl.points.front()).cast<double>().norm() <= join_gap)
                            {
                                kept.back().pl.points.insert(kept.back().pl.points.end(), pl.points.begin() +
                                                                                              (kept.back().pl.points.back() == pl.points.front() ? 1 : 0),
                                                             pl.points.end());
                                return;
                            }
                            kept.push_back(Kept{std::move(pl), attrs, oriented});
                        };

                        for (const Polyline &run : runs)
                        {
                            if (run.size() < 2)
                                continue;
                            Vec2d run_dir = (run.points.back() - run.points.front()).cast<double>();
                            const double run_len = run_dir.norm();
                            if (run_len > 1.0)
                                run_dir /= run_len;
                            const Point origin = run.points.front();

                            struct Bit
                            {
                                double key;
                                bool inside;
                                Polyline pl;
                            };
                            std::vector<Bit> bits;
                            auto take = [&](Polylines pls, bool inside) {
                                for (Polyline &pl : pls)
                                {
                                    if (pl.size() < 2)
                                        continue;
                                    const double key = (pl.points.front() - origin).cast<double>().dot(run_dir);
                                    bits.push_back(Bit{key, inside, std::move(pl)});
                                }
                            };
                            take(diff_pl(Polylines{run}, mask), false);
                            take(intersection_pl(Polylines{run}, mask), true);
                            std::sort(bits.begin(), bits.end(), [](const Bit &a, const Bit &b) { return a.key < b.key; });

                            for (Bit &bit : bits)
                            {
                                if (!bit.inside)
                                {
                                    append_kept(std::move(bit.pl));
                                    continue;
                                }
                                Vec2d step = (bit.pl.points.back() - bit.pl.points.front()).cast<double>();
                                const double len = step.norm();
                                if (len < 1.0 || bit.pl.length() < pitch * 0.5)
                                {
                                    touched = true;
                                    continue;
                                }
                                step /= len;
                                // Monotonic top lines drop the short connectors between them.
                                // Infill under the top follows the path it was given.
                                if (top_solid && std::abs(step.dot(line_axis)) < align_min)
                                {
                                    touched = true;
                                    continue;
                                }
                                const Point mid{(bit.pl.points.front().x() + bit.pl.points.back().x()) / 2,
                                                (bit.pl.points.front().y() + bit.pl.points.back().y()) / 2};
                                int job_i = -1;
                                for (size_t j = 0; j < jobs.size() && job_i < 0; ++j)
                                    for (const ExPolygon &ep : jobs[j].area)
                                        if (ep.contains(mid))
                                        {
                                            job_i = (int) j;
                                            break;
                                        }
                                if (job_i < 0 || uniq_by_job[job_i].size() < 2)
                                {
                                    append_kept(std::move(bit.pl));
                                    continue;
                                }
                                const double proj = double(mid.x()) * axis_x + double(mid.y()) * axis_y;
                                moved.push_back(Moved{proj, pitch, job_i, angle_key, attrs, oriented, std::move(bit.pl)});
                                touched = true;
                            }
                        }

                        if (!touched)
                        {
                            rebuilt.push_back(child);
                            continue;
                        }
                        changed = true;
                        for (Kept &piece : kept)
                        {
                            if (piece.oriented)
                                rebuilt.push_back(new ExtrusionPathOriented(std::move(piece.pl), piece.attrs));
                            else
                                rebuilt.push_back(new ExtrusionPath(std::move(piece.pl), piece.attrs));
                        }
                        delete child;
                    }
                    if (changed)
                        eec->entities.swap(rebuilt);
                }
            }

            if (moved.empty())
                continue;

            std::sort(moved.begin(), moved.end(), [](const Moved &a, const Moved &b) {
                if (a.job != b.job)
                    return a.job < b.job;
                if (a.angle_key != b.angle_key)
                    return a.angle_key < b.angle_key;
                return a.proj < b.proj;
            });

            std::vector<int> line_of(moved.size(), 0);
            int line = 0;
            double line_proj = moved.front().proj;
            int line_job = moved.front().job;
            int line_angle = moved.front().angle_key;
            double line_pitch = moved.front().pitch;
            for (size_t i = 0; i < moved.size(); ++i)
            {
                const Moved &m = moved[i];
                if (m.job != line_job || m.angle_key != line_angle || m.proj - line_proj > 0.45 * line_pitch)
                {
                    if (i > 0)
                        ++line;
                    line_job = m.job;
                    line_angle = m.angle_key;
                    line_proj = m.proj;
                    line_pitch = m.pitch;
                }
                line_of[i] = line;
            }

            std::vector<std::pair<int, ExtrusionEntityCollection *>> groups;
            auto group_for = [&](int region_id) -> ExtrusionEntityCollection * {
                for (auto &g : groups)
                    if (g.first == region_id)
                        return g.second;
                auto *eec = new ExtrusionEntityCollection();
                eec->no_sort = true;
                groups.emplace_back(region_id, eec);
                return eec;
            };
            for (size_t i = 0; i < moved.size(); ++i)
            {
                Moved &m = moved[i];
                const std::vector<int> &pattern = jobs[m.job].pattern;
                const int n = (int) pattern.size();
                if (n <= 0 || m.pl.size() < 2)
                    continue;
                // One bead per pattern step. Consecutive steps of the same filament stay one path.
                const std::vector<Polyline> steps = split_by_length(m.pl, m.pitch);
                const int phase = line_of[i];
                int run_filament = -1;
                Polyline run;
                auto flush_run = [&]() {
                    if (run.size() < 2 || run_filament < 0)
                    {
                        run.clear();
                        return;
                    }
                    const int region_id = region_for_filament(*this, run_filament);
                    if (region_id >= 0)
                    {
                        ExtrusionEntityCollection *eec = group_for(region_id);
                        if (m.oriented)
                            eec->entities.push_back(new ExtrusionPathOriented(std::move(run), m.attrs));
                        else
                            eec->entities.push_back(new ExtrusionPath(std::move(run), m.attrs));
                    }
                    run.clear();
                };
                for (size_t s = 0; s < steps.size(); ++s)
                {
                    const int slot = ((int(s) + phase) % n + n) % n;
                    const int filament = pattern[(size_t) slot];
                    if (filament != run_filament)
                    {
                        flush_run();
                        run_filament = filament;
                    }
                    if (run.empty())
                        run = steps[s];
                    else if (run.points.back() == steps[s].points.front())
                        run.points.insert(run.points.end(), steps[s].points.begin() + 1, steps[s].points.end());
                    else
                    {
                        flush_run();
                        run = steps[s];
                        run_filament = filament;
                    }
                }
                flush_run();
            }
            for (auto &g : groups)
            {
                if (g.second->entities.empty())
                {
                    delete g.second;
                    continue;
                }
                LayerRegion *lr = this->get_region(g.first);
                const uint32_t begin = (uint32_t) lr->m_fills.entities.size();
                lr->m_fills.entities.push_back(g.second);
                island.add_fill_range(LayerExtrusionRange{(uint32_t) g.first, {begin, begin + 1}});
            }

            // Wall loops around the painted top. The home extruder's stretches stay with the
            // perimeters; the other filaments are parked on their own regions and printed with
            // the infill of that color, still tagged as walls.
            if (island.perimeters.empty() || island.perimeters.region() >= m_regions.size())
                continue;
            LayerRegion *wall_region = this->get_region((int) island.perimeters.region());
            const int home_region = (int) island.perimeters.region();
            std::vector<std::pair<int, ExtrusionEntityCollection *>> wall_groups;
            auto wall_group_for = [&](int region_id) -> ExtrusionEntityCollection * {
                for (auto &g : wall_groups)
                    if (g.first == region_id)
                        return g.second;
                auto *collection = new ExtrusionEntityCollection();
                collection->no_sort = true;
                wall_groups.emplace_back(region_id, collection);
                return collection;
            };
            int wall_phase = 0;
            for (uint32_t ei = *island.perimeters.begin(); ei < *island.perimeters.end(); ++ei)
            {
                auto *eec = dynamic_cast<ExtrusionEntityCollection *>(wall_region->m_perimeters.entities[ei]);
                if (eec == nullptr)
                    continue;
                ExtrusionEntitiesPtr rebuilt;
                rebuilt.reserve(eec->entities.size());
                bool changed = false;
                for (ExtrusionEntity *child : eec->entities)
                {
                    std::vector<ColorMixWallPiece> pieces;
                    collect_wall_pieces(child, pieces);
                    if (pieces.empty())
                    {
                        rebuilt.push_back(child);
                        continue;
                    }
                    bool touched = false;
                    ExtrusionEntitiesPtr replacement;
                    for (ColorMixWallPiece &piece : pieces)
                    {
                        // The outer-wall mix walks this loop itself, one bead tall, and shifts
                        // with the layer. Leave that path whole so it is not split twice.
                        if (this->object()->config().color_mixing_wall_z_dither.value &&
                            piece.attrs.role.is_external_perimeter() && !piece.attrs.role.is_bridge())
                        {
                            replacement.push_back(new ExtrusionPath(std::move(piece.pl), piece.attrs));
                            continue;
                        }
                        if (!mask_bb.overlap(piece.pl.bounding_box()))
                        {
                            replacement.push_back(new ExtrusionPath(std::move(piece.pl), piece.attrs));
                            continue;
                        }
                        Polylines inside = intersection_pl(Polylines{piece.pl}, mask);
                        if (inside.empty())
                        {
                            replacement.push_back(new ExtrusionPath(std::move(piece.pl), piece.attrs));
                            continue;
                        }
                        touched = true;
                        for (Polyline &out : diff_pl(Polylines{piece.pl}, mask))
                            if (out.size() >= 2)
                                replacement.push_back(new ExtrusionPath(std::move(out), piece.attrs));
                        const double step = std::max(double(scale_(0.05)), double(scale_(std::max(0.05f, piece.attrs.width))));
                        for (Polyline &in : inside)
                        {
                            if (in.size() < 2 || in.length() < step * 0.5)
                            {
                                if (in.size() >= 2)
                                    replacement.push_back(new ExtrusionPath(std::move(in), piece.attrs));
                                continue;
                            }
                            const Point mid{coord_t((int64_t(in.points.front().x()) + in.points.back().x()) / 2),
                                            coord_t((int64_t(in.points.front().y()) + in.points.back().y()) / 2)};
                            int job_i = -1;
                            for (size_t j = 0; j < jobs.size() && job_i < 0; ++j)
                                for (const ExPolygon &ep : jobs[j].area)
                                    if (ep.contains(mid))
                                    {
                                        job_i = (int) j;
                                        break;
                                    }
                            if (job_i < 0)
                            {
                                replacement.push_back(new ExtrusionPath(std::move(in), piece.attrs));
                                continue;
                            }
                            std::vector<ColorMixWallPiece> spans;
                            std::vector<int> filaments;
                            dither_wall_polyline(in, piece.attrs, jobs[job_i].pattern, wall_phase, step, spans, filaments);
                            ++wall_phase;
                            for (size_t s = 0; s < spans.size(); ++s)
                            {
                                const int dest = region_for_filament(*this, filaments[s]);
                                ExtrusionPath *span_path = new ExtrusionPath(std::move(spans[s].pl), spans[s].attrs);
                                if (dest < 0 || dest == home_region)
                                    replacement.push_back(span_path);
                                else
                                    wall_group_for(dest)->entities.push_back(span_path);
                            }
                        }
                    }
                    if (!touched)
                    {
                        for (ExtrusionEntity *made : replacement)
                            delete made;
                        rebuilt.push_back(child);
                        continue;
                    }
                    changed = true;
                    for (ExtrusionEntity *made : replacement)
                        rebuilt.push_back(made);
                    delete child;
                }
                if (changed)
                    eec->entities.swap(rebuilt);
            }
            for (auto &g : wall_groups)
            {
                if (g.second->entities.empty())
                {
                    delete g.second;
                    continue;
                }
                LayerRegion *lr = this->get_region(g.first);
                const uint32_t begin = (uint32_t) lr->m_fills.entities.size();
                lr->m_fills.entities.push_back(g.second);
                island.add_fill_range(LayerExtrusionRange{(uint32_t) g.first, {begin, begin + 1}});
            }
        }
}

void Layer::dither_color_mix_outer_walls()
{
    if (color_mix_wall_z.empty() || !this->object()->config().color_mixing_wall_z_dither.value ||
        this->object()->print()->config().spiral_vase.value)
        return;

    ExPolygons mask;
    for (const ColorMixWallZ &stripe : color_mix_wall_z)
        append(mask, stripe.area);
    if (mask.empty())
        return;
    const BoundingBox mask_bb = get_extents(mask);
    const int layer_phase = (int) this->id();
    const double step = scale_(std::max(0.05, this->object()->config().color_mixing_wall_z_length.value));

    for (LayerSlice &lslice : this->lslices_ex)
        for (LayerIsland &island : lslice.islands)
        {
            if (island.perimeters.empty() || island.perimeters.region() >= m_regions.size())
                continue;
            LayerRegion *wall_region = this->get_region((int) island.perimeters.region());
            const int home_region = (int) island.perimeters.region();
            std::vector<std::pair<int, ExtrusionEntityCollection *>> wall_groups;
            auto wall_group_for = [&](int region_id) -> ExtrusionEntityCollection *
            {
                for (auto &g : wall_groups)
                    if (g.first == region_id)
                        return g.second;
                auto *collection = new ExtrusionEntityCollection();
                collection->no_sort = true;
                wall_groups.emplace_back(region_id, collection);
                return collection;
            };
            for (uint32_t ei = *island.perimeters.begin(); ei < *island.perimeters.end(); ++ei)
            {
                auto *eec = dynamic_cast<ExtrusionEntityCollection *>(wall_region->m_perimeters.entities[ei]);
                if (eec == nullptr)
                    continue;
                ExtrusionEntitiesPtr rebuilt;
                rebuilt.reserve(eec->entities.size());
                bool changed = false;
                for (ExtrusionEntity *child : eec->entities)
                {
                    std::vector<ColorMixWallPiece> pieces;
                    collect_wall_pieces(child, pieces);
                    if (pieces.empty())
                    {
                        rebuilt.push_back(child);
                        continue;
                    }
                    bool touched = false;
                    ExtrusionEntitiesPtr replacement;
                    for (ColorMixWallPiece &piece : pieces)
                    {
                        const bool outer = piece.attrs.role.is_external_perimeter() && !piece.attrs.role.is_bridge();
                        if (!outer || !mask_bb.overlap(piece.pl.bounding_box()))
                        {
                            replacement.push_back(new ExtrusionPath(std::move(piece.pl), piece.attrs));
                            continue;
                        }
                        Polylines inside = intersection_pl(Polylines{piece.pl}, mask);
                        if (inside.empty())
                        {
                            replacement.push_back(new ExtrusionPath(std::move(piece.pl), piece.attrs));
                            continue;
                        }
                        touched = true;
                        for (Polyline &out : diff_pl(Polylines{piece.pl}, mask))
                            if (out.size() >= 2)
                                replacement.push_back(new ExtrusionPath(std::move(out), piece.attrs));
                        for (Polyline &in : inside)
                        {
                            if (in.size() < 2 || in.length() < step * 0.5)
                            {
                                if (in.size() >= 2)
                                    replacement.push_back(new ExtrusionPath(std::move(in), piece.attrs));
                                continue;
                            }
                            const Point mid{coord_t((int64_t(in.points.front().x()) + in.points.back().x()) / 2),
                                            coord_t((int64_t(in.points.front().y()) + in.points.back().y()) / 2)};
                            int stripe_i = -1;
                            for (size_t j = 0; j < color_mix_wall_z.size() && stripe_i < 0; ++j)
                            {
                                bool contains = false;
                                for (const ExPolygon &ep : color_mix_wall_z[j].area)
                                    if (ep.contains(mid))
                                    {
                                        contains = true;
                                        break;
                                    }
                                if (!contains)
                                    continue;
                                bool have_regions = true;
                                for (int filament : unique_in_order(color_mix_wall_z[j].pattern))
                                    if (region_for_filament(*this, filament) < 0)
                                        have_regions = false;
                                if (have_regions)
                                    stripe_i = (int) j;
                            }
                            if (stripe_i < 0)
                            {
                                replacement.push_back(new ExtrusionPath(std::move(in), piece.attrs));
                                continue;
                            }
                            std::vector<ColorMixWallPiece> spans;
                            std::vector<int> filaments;
                            dither_anchored_wall(in, piece.attrs, color_mix_wall_z[(size_t) stripe_i].pattern,
                                                 layer_phase, step, spans, filaments);
                            for (size_t s = 0; s < spans.size(); ++s)
                            {
                                const int dest = region_for_filament(*this, filaments[s]);
                                ExtrusionPath *span_path = new ExtrusionPath(std::move(spans[s].pl), spans[s].attrs);
                                if (dest < 0 || dest == home_region)
                                    replacement.push_back(span_path);
                                else
                                    wall_group_for(dest)->entities.push_back(span_path);
                            }
                        }
                    }
                    if (!touched)
                    {
                        for (ExtrusionEntity *made : replacement)
                            delete made;
                        rebuilt.push_back(child);
                        continue;
                    }
                    changed = true;
                    for (ExtrusionEntity *made : replacement)
                        rebuilt.push_back(made);
                    delete child;
                }
                if (changed)
                    eec->entities.swap(rebuilt);
            }
            for (auto &g : wall_groups)
            {
                if (g.second->entities.empty())
                {
                    delete g.second;
                    continue;
                }
                LayerRegion *lr = this->get_region(g.first);
                const uint32_t begin = (uint32_t) lr->m_fills.entities.size();
                lr->m_fills.entities.push_back(g.second);
                island.add_fill_range(LayerExtrusionRange{(uint32_t) g.first, {begin, begin + 1}});
            }
        }
}

void Layer::clear_fills()
{
    for (LayerRegion *layerm : m_regions)
        layerm->m_fills.clear();
    for (LayerSlice &lslice : lslices_ex)
        for (LayerIsland &island : lslice.islands)
            island.fills.clear();
}

void Layer::make_fills(FillAdaptive::Octree *adaptive_fill_octree, FillAdaptive::Octree *support_fill_octree,
                       FillLightning::Generator *lightning_generator)
{
    auto t0 = std::chrono::steady_clock::now();

    this->clear_fills();

    // Second sliver removal pass: the first runs in PrintObject after
    // slices_to_fill_surfaces_clipped(), but discover_horizontal_shells(),
    // process_external_surfaces(), and bridge_over_infill() can create
    // new narrow stInternal fragments. Catch those before fill generation.
    for (size_t region_id = 0; region_id < this->regions().size(); ++region_id)
        this->regions()[region_id]->remove_narrow_fill_surfaces();

    // Remember painted 2-way and 3-way tops. The monotonic fill is generated first, then recolored per line.
    const std::vector<ColorMixTopStripe> color_mix_jobs = this->clip_color_mix_tops();

    std::vector<SurfaceFill> surface_fills = group_fills(*this);
    {
        extern Slic3r::PerfAccumTimer g_mf_group_fills;
        g_mf_group_fills.add(t0, std::chrono::steady_clock::now());
    }
    const Slic3r::BoundingBox bbox = this->object()->bounding_box();
    const auto resolution = this->object()->print()->config().gcode_resolution.value;
    const auto perimeter_generator = this->object()->config().perimeter_generator;

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    {
        static int iRun = 0;
        export_group_fills_to_svg(debug_out_path("Layer-fill_surfaces-10_fill-final-%d.svg", iRun++).c_str(),
                                  surface_fills);
    }
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */

    // Debug: dump surface_fills before fill generation
    dbg_fill_phase("PRE_FILL", *this, surface_fills);

    size_t first_object_layer_id = this->object()->get_layer(0)->id();
    // Each island's infill is generated and filled completely before moving to the next island,
    // eliminating chaotic back-and-forth travel caused by global generation + spatial division.

    for (LayerSlice &lslice : this->lslices_ex)
    {
        for (LayerIsland &island : lslice.islands)
        {
            // The island's perimeters may have been merged into a single (highest-density) region
            // by make_perimeters() while each part's fill still belongs to its own region. The
            // perimeter-owning region is used only to anchor the infill start point.
            const uint32_t perim_region_id = island.perimeters.region();
            LayerRegion *perim_layerm = this->get_region(perim_region_id);

            // Process each surface type for THIS island
            for (SurfaceFill &surface_fill : surface_fills)
            {
                // Match the fill to this island by its fill-owning region, or by geometry alone
                // when the island's fill is composite (split across regions). Keying on the
                // perimeter region instead would drop the lower-density part's infill wherever it
                // shares layers with a merged, higher-density part.
                if (!island.fill_expolygons_composite() && surface_fill.region_id != island.fill_region_id)
                    continue;

                // The fill is generated, stored, and attributed to its own region.
                const uint32_t region_id = surface_fill.region_id;
                LayerRegion *layerm = this->get_region(region_id);

                // Intersect surface fill with island boundary
                auto ti0 = std::chrono::steady_clock::now();
                ExPolygons island_expolygons = intersection_ex(surface_fill.expolygons, ExPolygons{island.boundary});
                {
                    extern Slic3r::PerfAccumTimer g_mf_intersection;
                    g_mf_intersection.add(ti0, std::chrono::steady_clock::now());
                }

                if (island_expolygons.empty())
                    continue;

                // Preprocessing API: fill region area and pattern for this island+surface combo.
                // Summed area is the fallback for the batch (non-monotonic) path where
                // per-ExPolygon attribution is lost after chaining. The per-ExPolygon path
                // overrides this with individual ExPolygon areas inside the loop.
                float fill_region_area_mm2 = 0.0f;
                for (const ExPolygon &ep : island_expolygons)
                    fill_region_area_mm2 += static_cast<float>(std::abs(ep.area()) * SCALING_FACTOR * SCALING_FACTOR);
                const int fill_pattern_id = static_cast<int>(surface_fill.params.pattern);

                if (Slic3r::debug_enabled(Slic3r::DBG_FILL))
                {
                    double isl_area = 0;
                    for (const ExPolygon &ep : island_expolygons)
                        isl_area += std::abs(ep.area());
                    BoundingBox ibb = get_extents(island_expolygons);
                    dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL",
                            "ISLAND_FILL type=%-18s pattern=%-16s ep=%zu area=%8.4fmm2 "
                            "bbox=(%.2f,%.2f)-(%.2f,%.2f)",
                            dbg_stype(surface_fill.surface.surface_type), dbg_pattern(surface_fill.params.pattern),
                            island_expolygons.size(), isl_area * 1e-12, unscaled<double>(ibb.min.x()),
                            unscaled<double>(ibb.min.y()), unscaled<double>(ibb.max.x()),
                            unscaled<double>(ibb.max.y()));
                    for (size_t i = 0; i < island_expolygons.size(); i++)
                    {
                        const ExPolygon &ep = island_expolygons[i];
                        BoundingBox ebb = ep.contour.bounding_box();
                        dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL",
                                "  ISLAND_FILL [%zu] area=%8.4fmm2 holes=%zu pts=%zu "
                                "bbox=(%.2f,%.2f)-(%.2f,%.2f)",
                                i, std::abs(ep.area()) * 1e-12, ep.holes.size(), ep.contour.points.size(),
                                unscaled<double>(ebb.min.x()), unscaled<double>(ebb.min.y()),
                                unscaled<double>(ebb.max.x()), unscaled<double>(ebb.max.y()));
                        for (size_t h = 0; h < ep.holes.size(); h++)
                        {
                            BoundingBox hbb = ep.holes[h].bounding_box();
                            dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL",
                                    "    ISLAND_HOLE [%zu][%zu] pts=%zu "
                                    "bbox=(%.2f,%.2f)-(%.2f,%.2f)",
                                    i, h, ep.holes[h].points.size(), unscaled<double>(hbb.min.x()),
                                    unscaled<double>(hbb.min.y()), unscaled<double>(hbb.max.x()),
                                    unscaled<double>(hbb.max.y()));
                        }
                    }
                }

                // Create the filler object for this surface type
                std::unique_ptr<Fill> f = std::unique_ptr<Fill>(Fill::new_from_type(surface_fill.params.pattern));
                f->set_bounding_box(bbox);
                // Layer ID is used for orienting the infill in alternating directions.
                // Layer::id() returns layer ID including raft layers, subtract them to make the infill direction independent
                // from raft.
                f->layer_id = this->id() - first_object_layer_id;
                f->z = this->print_z;
                f->angle = surface_fill.params.angle;
                f->overlap = surface_fill.params.overlap;
                f->perimeter_width = m_regions[surface_fill.region_id]->flow(frPerimeter).width();
                f->adapt_fill_octree = (surface_fill.params.pattern == ipSupportCubic) ? support_fill_octree
                                                                                       : adaptive_fill_octree;
                f->print_config = &this->object()->print()->config();
                f->print_object_config = &this->object()->config();

                if (surface_fill.params.pattern == ipLightning)
                    dynamic_cast<FillLightning::Filler *>(f.get())->generator = lightning_generator;

                if (surface_fill.params.pattern == ipEnsuring)
                {
                    auto *fill_ensuring = dynamic_cast<FillEnsuring *>(f.get());
                    assert(fill_ensuring != nullptr);
                    fill_ensuring->print_region_config = &m_regions[surface_fill.region_id]->region().config();
                }

                if (surface_fill.params.pattern == ipCustom)
                {
                    auto *fill_custom = dynamic_cast<FillCustom *>(f.get());
                    assert(fill_custom != nullptr);
                    fill_custom->print_region_config = &m_regions[surface_fill.region_id]->region().config();
                }

                // calculate flow spacing for infill pattern generation
                bool using_internal_flow = !surface_fill.surface.is_solid() && !surface_fill.params.bridge;
                double link_max_length = 0.;
                if (!surface_fill.params.bridge)
                {
#if 0
                    link_max_length = layerm->region().config().get_abs_value(surface.is_external() ? "external_fill_link_max_length" : "fill_link_max_length", flow.spacing());
//                    printf("flow spacing: %f,  is_external: %d, link_max_length: %lf\n", flow.spacing(), int(surface.is_external()), link_max_length);
#else
                    if (surface_fill.params.density > 80.) // 80%
                        link_max_length = 3. * f->spacing;
#endif
                }

                // Maximum length of the perimeter segment linking two infill lines.
                f->link_max_length = (coord_t) scale_(link_max_length);
                // Used by the concentric infill pattern to clip the loops to create extrusion paths.
                f->loop_clipping = coord_t(scale_(surface_fill.params.flow.nozzle_diameter()) *
                                           LOOP_CLIPPING_LENGTH_OVER_NOZZLE_DIAMETER);

                // apply half spacing using this flow's own spacing and generate infill
                FillParams params;
                params.density = float(0.01 * surface_fill.params.density);

                // At exactly 50% density, distance = min_spacing / 0.5 = min_spacing * 2.0 (exact integer multiple)
                // This creates perfectly aligned coordinates that trigger geometric degeneracies in Clipper2.
                // Treat 50.0% as 49.9% to avoid the exact 2x multiplier.
                // Applies to: ipConcentric (9) and ipEnsuring/Athena (20) which use heavy Clipper2 operations.
                if ((surface_fill.params.pattern == ipConcentric || surface_fill.params.pattern == ipEnsuring) &&
                    std::abs(params.density - 0.5f) < 0.0001f)
                {
                    params.density = 0.499f;
                }

                params.dont_adjust = false; //  surface_fill.params.dont_adjust;
                params.bridge = surface_fill.params.bridge;
                params.anchor_length = surface_fill.params.anchor_length;
                params.anchor_length_max = surface_fill.params.anchor_length_max;
                params.resolution = resolution;
                params.use_advanced_perimeters = ((perimeter_generator == PerimeterGeneratorType::Arachne ||
                                                   perimeter_generator == PerimeterGeneratorType::Athena) &&
                                                  surface_fill.params.pattern == ipConcentric) ||
                                                 surface_fill.params.pattern == ipEnsuring;
                // Concentric fill always uses Athena for clean variable-width beads,
                // regardless of the perimeter generator setting.
                params.perimeter_generator = (surface_fill.params.pattern == ipConcentric)
                                                 ? PerimeterGeneratorType::Athena
                                                 : perimeter_generator;
                params.layer_height = layerm->layer()->height;
                params.prefer_clockwise_movements = this->object()->print()->config().prefer_clockwise_movements;
                // Bead width ceiling for Athena fills from max_perimeter_width, resolved against
                // the nozzle of the extruder printing this fill, the same basis as the
                // perimeter-side ceiling and the per-extruder generated-width warning.
                if (const double mpw_pct = layerm->region().config().max_perimeter_width.value; mpw_pct > 0)
                {
                    const int fill_extruder = surface_fill.params.extruder;
                    const double nozzle = this->object()->print()->config().nozzle_diameter.get_at(
                        fill_extruder > 0 ? size_t(fill_extruder - 1) : size_t(0));
                    params.max_bead_width = scaled<coord_t>(nozzle * mpw_pct * 0.01);
                }

                // Track fill range for this island and surface type
                uint32_t fill_begin = uint32_t(layerm->m_fills.entities.size());

                // An island may have multiple disconnected sparse regions (e.g., separated by interlocking).
                // Fill each ExPolygon completely before moving to the next to prevent chaotic jumping.

                // Create ONE collection for all fills in this island
                ExtrusionEntityCollection *eec = new ExtrusionEntityCollection();

                // Initialize to perimeter endpoint - that's where the nozzle is when infill starts
                Point last_fill_pos = Point(0, 0);
                bool have_last_pos = false;

                // Get the last perimeter's endpoint as the initial starting position
                // For closed perimeter loops, first_point == last_point, which is where the nozzle
                // finishes after printing the perimeter (the seam position)
                if (!island.perimeters.empty())
                {
                    uint32_t last_perim_idx = *island.perimeters.end() - 1;
                    if (last_perim_idx < perim_layerm->m_perimeters.entities.size())
                    {
                        const ExtrusionEntity *last_perim = perim_layerm->m_perimeters.entities[last_perim_idx];
                        if (last_perim != nullptr)
                        {
                            last_fill_pos = last_perim->last_point();
                            have_last_pos = true;
                        }
                    }
                }

                // Monotonic fills must preserve their ant-colony sweep ordering within
                // each ExPolygon. Non-monotonic fills can be freely reordered across
                // ExPolygon boundaries for better travel optimization.
                const bool is_monotonic_fill = fill_type_monotonic(surface_fill.params.pattern);
                // Solid fills produce long connected zigzag polylines that must stay intact
                // per-ExPolygon; cross-fragment chaining corrupts traverse graph connections.
                const bool use_per_expolygon_path = is_monotonic_fill || surface_fill.params.density > 99.f;

                if (use_per_expolygon_path || params.use_advanced_perimeters)
                {
                    // MONOTONIC / ADVANCED PATH: per-ExPolygon entity creation preserves ordering
                    for (ExPolygon &expoly : island_expolygons)
                    {
                        float ep_area_mm2 = static_cast<float>(std::abs(expoly.area()) * SCALING_FACTOR *
                                                               SCALING_FACTOR);
                        f->spacing = surface_fill.params.spacing;
                        // For bridges: use original flow width so boundary offset is independent of line spacing
                        f->bounding_width = surface_fill.params.bridge ? surface_fill.params.flow.width()
                                                                       : surface_fill.params.spacing;
                        params.start_near = have_last_pos ? last_fill_pos : expoly.contour.centroid();

                        // Override fill direction for counterbore bridges to match corridor angle.
                        f->counterbore_fill_angle = -1.f;
                        if (surface_fill.surface.is_bridge() && !this->counterbore_bridge_regions.empty())
                        {
                            double ep_area = std::abs(expoly.area());
                            for (const auto &[cb_region, cb_angle] : this->counterbore_bridge_regions)
                            {
                                double overlap_area = 0;
                                for (const ExPolygon &ov : intersection_ex(ExPolygons{expoly}, cb_region))
                                    overlap_area += std::abs(ov.area());
                                if (ep_area > 0 && overlap_area / ep_area > 0.01)
                                {
                                    f->counterbore_fill_angle = float(cb_angle);
                                    break;
                                }
                            }
                        }

                        surface_fill.surface.expolygon = std::move(expoly);
                        Polylines polylines;
                        ThickPolylines thick_polylines;
                        auto tf0 = std::chrono::steady_clock::now();
                        try
                        {
                            if (params.use_advanced_perimeters)
                                thick_polylines = f->fill_surface_advanced(&surface_fill.surface, params);
                            else
                                polylines = f->fill_surface(&surface_fill.surface, params);
                        }
                        catch (InfillFailedException &)
                        {
                            dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL",
                                    "FILL_EXCEPTION type=%-18s InfillFailedException!",
                                    dbg_stype(surface_fill.surface.surface_type));
                        }
                        {
                            extern Slic3r::PerfAccumTimer g_mf_fill_surface;
                            g_mf_fill_surface.add(tf0, std::chrono::steady_clock::now());
                        }

                        // Bridge gap fill: detect uncovered regions between bridge lines
                        // and the fill boundary, generate variable-width beads to fill them
                        ThickPolylines bridge_gap_fills;
                        if (surface_fill.params.bridge && !polylines.empty())
                        {
                            const float bridge_width = surface_fill.params.flow.width();
                            const float half_width_scaled = float(scale_(bridge_width / 2.0));
                            const float nozzle_dia = surface_fill.params.flow.nozzle_diameter();
                            const double min_w = double(nozzle_dia) / 3.0;
                            const double max_w = double(bridge_width);

                            // 10.73% overlap per side so the bead fuses with neighbors
                            const float layer_h = surface_fill.params.flow.height();
                            const float overlap = float(scale_(layer_h * (1.0 - M_PI / 4.0) / 2.0));

                            // If overlap >= half the bridge width, lines are already
                            // fused and there are no gaps to fill
                            if (overlap < half_width_scaled)
                            {
                                // Coverage shrunk by overlap, boundary grown by overlap
                                Polygons covered;
                                for (const Polyline &pl : polylines)
                                    append(covered, offset(pl, half_width_scaled - overlap));
                                covered = union_(covered);

                                ExPolygons boundary = offset_ex(ExPolygons{surface_fill.surface.expolygon}, overlap);

                                ExPolygons raw_gaps = diff_ex(boundary, covered);

                                // Clean up geometry to prevent Voronoi hangs, then
                                // extract medial axis (same as PerimeterGenerator)
                                ExPolygons gaps_clean = diff_ex(opening_ex(raw_gaps, float(scale_(min_w / 2.))),
                                                                offset2_ex(raw_gaps, -float(scale_(max_w / 2.)),
                                                                           float(scale_(max_w / 2.) +
                                                                                 ClipperSafetyOffset)));

                                const double min_area = scale_(min_w) * scale_(min_w);
                                for (const ExPolygon &gap : gaps_clean)
                                {
                                    if (std::abs(gap.area()) < min_area)
                                        continue;
                                    // Skip overly complex polygons that could stall
                                    // the Voronoi computation
                                    if (gap.contour.points.size() > 5000)
                                        continue;
                                    gap.medial_axis(scale_(min_w), scale_(max_w), &bridge_gap_fills);
                                }

                                // Drop short fragments
                                const double min_len = scale_(bridge_width * 3.0);
                                bridge_gap_fills.erase(std::remove_if(bridge_gap_fills.begin(), bridge_gap_fills.end(),
                                                                      [min_len](const ThickPolyline &tp)
                                                                      { return tp.length() < min_len; }),
                                                       bridge_gap_fills.end());
                            }
                        }

                        if (!polylines.empty())
                        {
                            last_fill_pos = polylines.back().last_point();
                            have_last_pos = true;
                        }
                        else if (!thick_polylines.empty())
                        {
                            last_fill_pos = thick_polylines.back().last_point();
                            have_last_pos = true;
                        }

                        if (!polylines.empty() || !thick_polylines.empty())
                        {
                            double flow_mm3_per_mm = surface_fill.params.flow.mm3_per_mm();
                            double flow_width = surface_fill.params.flow.width();
                            if (using_internal_flow || surface_fill.params.bridge)
                            {
                            }
                            else
                            {
                                Flow new_flow = surface_fill.params.flow.with_spacing(float(f->spacing));
                                flow_mm3_per_mm = new_flow.mm3_per_mm();
                                flow_width = new_flow.width();
                            }

                            if (params.use_advanced_perimeters)
                            {
                                for (const ThickPolyline &thick_polyline : thick_polylines)
                                {
                                    Flow new_flow = surface_fill.params.bridge
                                                        ? surface_fill.params.flow
                                                        : surface_fill.params.flow.with_spacing(float(f->spacing));
                                    // The flow-hold baseline is the configured feature flow, not
                                    // the solver-adjusted spacing flow, so a solver-widened bead
                                    // still registers its volumetric excess.
                                    ExtrusionMultiPath multi_path = PerimeterGenerator::thick_polyline_to_multi_path(
                                        thick_polyline, surface_fill.params.extrusion_role, new_flow,
                                        scaled<float>(0.05), float(SCALED_EPSILON), std::nullopt,
                                        surface_fill.params.flow.mm3_per_mm());
                                    if (!multi_path.empty())
                                    {
                                        for (auto &p : multi_path.paths)
                                        {
                                            p.attributes().region_area_mm2 = ep_area_mm2;
                                            p.attributes().fill_pattern = fill_pattern_id;
                                        }
                                        if (multi_path.paths.front().first_point() ==
                                            multi_path.paths.back().last_point())
                                            eec->entities.emplace_back(new ExtrusionLoop(std::move(multi_path.paths)));
                                        else
                                            eec->entities.emplace_back(new ExtrusionMultiPath(std::move(multi_path)));
                                    }
                                }
                            }
                            else
                            {
                                // Bridge lines are always reversible - either direction
                                // is equivalent over air, and this lets nearest-neighbor
                                // pick the closest endpoint
                                const bool can_reverse = surface_fill.params.bridge
                                                             ? true
                                                             : !params.prefer_clockwise_movements;
                                {
                                    ExtrusionAttributes fill_attrs{surface_fill.params.extrusion_role,
                                                                   ExtrusionFlow{flow_mm3_per_mm, float(flow_width),
                                                                                 surface_fill.params.flow.height()},
                                                                   f->is_self_crossing()};
                                    // Constant-width lines at solver-widened spacing carry their
                                    // volumetric excess over the configured feature flow, same as
                                    // the variable-width path above.
                                    if (const double nominal = surface_fill.params.flow.mm3_per_mm();
                                        nominal > 0. && !surface_fill.params.extrusion_role.is_bridge() &&
                                        !surface_fill.params.extrusion_role.has(ExtrusionRoleModifier::Interlocking))
                                        fill_attrs.flow_ratio = float(std::max(1., flow_mm3_per_mm / nominal));
                                    fill_attrs.region_area_mm2 = ep_area_mm2;
                                    fill_attrs.fill_pattern = fill_pattern_id;
                                    extrusion_entities_append_paths(eec->entities, std::move(polylines), fill_attrs,
                                                                    can_reverse);
                                }
                            }
                        }

                        // Append bridge gap fill beads as variable-width extrusions
                        if (!bridge_gap_fills.empty())
                        {
                            // Use bridge height so the bead matches the bridge lines
                            // it bonds with, but non-bridge type so variable width works
                            Flow gap_flow(surface_fill.params.flow.width(), surface_fill.params.flow.height(),
                                          surface_fill.params.flow.nozzle_diameter());
                            for (const ThickPolyline &tp : bridge_gap_fills)
                            {
                                ExtrusionMultiPath multi_path = PerimeterGenerator::thick_polyline_to_multi_path(
                                    tp, ExtrusionRole::BridgeInfill, gap_flow, scaled<float>(0.05),
                                    float(SCALED_EPSILON));
                                if (!multi_path.empty())
                                {
                                    for (auto &p : multi_path.paths)
                                    {
                                        p.attributes().region_area_mm2 = ep_area_mm2;
                                        p.attributes().fill_pattern = fill_pattern_id;
                                    }
                                    eec->entities.emplace_back(new ExtrusionMultiPath(std::move(multi_path)));
                                }
                            }
                        }
                    }
                    if (is_monotonic_fill)
                    {
                        // The ant-colony sweep order is already correct - same
                        // order as Monotonic connected fill, just without the
                        // connections. Prevent reordering here and in G-code export.
                        eec->no_sort = true;
                    }
                    else if (eec->entities.size() > 1)
                    {
                        const Point *start = have_last_pos ? &last_fill_pos : nullptr;
                        chain_and_reorder_extrusion_entities(eec->entities, start);
                    }
                }
                else
                {
                    // NON-MONOTONIC PATH: batch polylines across fragments for cross-fragment chaining.
                    // Tag each polyline with its originating ExPolygon's flow for correct extrusion.
                    struct PolylineFlowTag
                    {
                        double mm3_per_mm;
                        float width;
                        bool self_crossing;
                    };
                    Polylines all_polylines;
                    std::vector<PolylineFlowTag> flow_tags;

                    for (ExPolygon &expoly : island_expolygons)
                    {
                        f->spacing = surface_fill.params.spacing;
                        // For bridges: use original flow width so boundary offset is independent of line spacing
                        f->bounding_width = surface_fill.params.bridge ? surface_fill.params.flow.width()
                                                                       : surface_fill.params.spacing;
                        params.start_near = have_last_pos ? last_fill_pos : expoly.contour.centroid();

                        surface_fill.surface.expolygon = std::move(expoly);
                        Polylines polylines;
                        try
                        {
                            polylines = f->fill_surface(&surface_fill.surface, params);
                        }
                        catch (InfillFailedException &)
                        {
                            dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL",
                                    "FILL_EXCEPTION type=%-18s InfillFailedException!",
                                    dbg_stype(surface_fill.surface.surface_type));
                        }

                        if (!polylines.empty())
                        {
                            last_fill_pos = polylines.back().last_point();
                            have_last_pos = true;

                            // Compute this ExPolygon's adjusted flow
                            double ep_mm3 = surface_fill.params.flow.mm3_per_mm();
                            float ep_width = float(surface_fill.params.flow.width());
                            if (!using_internal_flow && !surface_fill.params.bridge)
                            {
                                Flow adj = surface_fill.params.flow.with_spacing(float(f->spacing));
                                ep_mm3 = adj.mm3_per_mm();
                                ep_width = float(adj.width());
                            }
                            bool ep_self_crossing = f->is_self_crossing();

                            // Tag each polyline with its origin ExPolygon's flow
                            for (size_t i = 0; i < polylines.size(); ++i)
                                flow_tags.push_back({ep_mm3, ep_width, ep_self_crossing});
                            append(all_polylines, std::move(polylines));
                        }
                    }

                    // Chain all polylines across ExPolygon fragment boundaries with 2-opt,
                    // using index tracking to preserve per-polyline flow tags.
                    if (!all_polylines.empty())
                    {
                        const Point *chain_start = have_last_pos ? &last_fill_pos : nullptr;
                        dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL", "PRE_CHAIN all_polylines=%zu flow_tags=%zu",
                                all_polylines.size(), flow_tags.size());
                        auto [chained, index_map] = chain_polylines_with_indices(std::move(all_polylines), chain_start);
                        dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL", "POST_CHAIN chained=%zu index_map=%zu",
                                chained.size(), index_map.size());

                        // Reorder flow tags to match the chained polyline order
                        std::vector<PolylineFlowTag> reordered_tags;
                        reordered_tags.reserve(index_map.size());
                        for (size_t orig_idx : index_map)
                            reordered_tags.push_back(flow_tags[orig_idx]);

                        // Convert to extrusion entities with correct per-polyline flow
                        if (Slic3r::debug_enabled(Slic3r::DBG_FILL))
                        {
                            size_t degen = 0;
                            double degen_len = 0, good_len = 0;
                            for (size_t i = 0; i < chained.size(); ++i)
                                if (chained[i].size() < 2)
                                    ++degen;
                                else
                                    good_len += unscale<double>(chained[i].length());
                            if (degen > 0)
                                dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL",
                                        "DEGENERATE chained=%zu degen=%zu good=%zu good_len=%.1fmm", chained.size(),
                                        degen, chained.size() - degen, good_len);
                        }
                        for (size_t i = 0; i < chained.size(); ++i)
                        {
                            if (chained[i].size() < 2)
                                continue;
                            const auto &tag = reordered_tags[i];
                            ExtrusionAttributes attrs{surface_fill.params.extrusion_role,
                                                      ExtrusionFlow{tag.mm3_per_mm, tag.width,
                                                                    surface_fill.params.flow.height()},
                                                      tag.self_crossing};
                            attrs.region_area_mm2 = fill_region_area_mm2;
                            attrs.fill_pattern = fill_pattern_id;
                            // !prefer_clockwise_movements -> can_reverse=true -> ExtrusionPath
                            // prefer_clockwise_movements -> can_reverse=false -> ExtrusionPathOriented
                            if (!params.prefer_clockwise_movements)
                                eec->entities.emplace_back(new ExtrusionPath(std::move(chained[i]), attrs));
                            else
                                eec->entities.emplace_back(new ExtrusionPathOriented(std::move(chained[i]), attrs));
                        }
                    }
                }

                // Add the collection to the layer (if it has any fills)
                if (!eec->empty())
                {
                    dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL", "FILL_OK type=%-18s entities=%zu",
                            dbg_stype(surface_fill.surface.surface_type), eec->entities.size());
                    layerm->m_fills.entities.push_back(eec);
                }
                else
                {
                    dbg_log(Slic3r::DBG_FILL, this->print_z, "FILL", "FILL_EMPTY type=%-18s (no extrusions generated)",
                            dbg_stype(surface_fill.surface.surface_type));
                    delete eec;
                }

                uint32_t fill_end = uint32_t(layerm->m_fills.entities.size());

                // Direct assignment to THIS island (no spatial search needed)
                if (fill_end > fill_begin)
                {
                    island.add_fill_range(LayerExtrusionRange{region_id, {fill_begin, fill_end}});
                }
            }
        }
    }

    this->emit_color_mix_top_lines(color_mix_jobs);
    this->dither_color_mix_outer_walls();
    this->emit_coex_partner_walls();

    for (LayerSlice &lslice : this->lslices_ex)
        for (LayerIsland &island : lslice.islands)
        {
            if (!island.thin_fills.empty())
            {
                // Copy thin fills into fills packed as a collection.
                // Fills are always stored as collections, the rest of the pipeline (wipe into infill, G-code generator) relies on it.
                LayerRegion &layerm = *this->get_region(island.perimeters.region());
                ExtrusionEntityCollection &collection = *(new ExtrusionEntityCollection());
                layerm.m_fills.entities.push_back(&collection);
                collection.entities.reserve(island.thin_fills.size());
                for (uint32_t fill_id : island.thin_fills)
                    collection.entities.push_back(layerm.thin_fills().entities[fill_id]->clone());
                island.add_fill_range(
                    {island.perimeters.region(),
                     {uint32_t(layerm.m_fills.entities.size() - 1), uint32_t(layerm.m_fills.entities.size())}});
            }
            // Sort the fills by region ID.
            std::sort(island.fills.begin(), island.fills.end(), [](auto &l, auto &r)
                      { return l.region() < r.region() || (l.region() == r.region() && *l.begin() < *r.begin()); });
            // Compress continuous fill ranges of the same region.
            {
                size_t k = 0;
                for (size_t i = 0; i < island.fills.size();)
                {
                    uint32_t region_id = island.fills[i].region();
                    uint32_t begin = *island.fills[i].begin();
                    uint32_t end = *island.fills[i].end();
                    size_t j = i + 1;
                    for (; j < island.fills.size() && island.fills[j].region() == region_id &&
                           *island.fills[j].begin() == end;
                         ++j)
                        end = *island.fills[j].end();
                    island.fills[k++] = {region_id, {begin, end}};
                    i = j;
                }
                island.fills.erase(island.fills.begin() + k, island.fills.end());
            }
        }

    g_mf_layer_count++;

#ifndef NDEBUG
    for (LayerRegion *layerm : m_regions)
        for (const ExtrusionEntity *e : layerm->fills())
            assert(dynamic_cast<const ExtrusionEntityCollection *>(e) != nullptr);
#endif

    // Perimeters were already checked when they were generated
    this->check_generated_widths_against_warning(false);
}

Polylines Layer::generate_sparse_infill_polylines_for_anchoring(FillAdaptive::Octree *adaptive_fill_octree,
                                                                FillAdaptive::Octree *support_fill_octree,
                                                                FillLightning::Generator *lightning_generator) const
{
    std::vector<SurfaceFill> surface_fills = group_fills(*this);
    const Slic3r::BoundingBox bbox = this->object()->bounding_box();
    const auto resolution = this->object()->print()->config().gcode_resolution.value;

    Polylines sparse_infill_polylines{};

    for (SurfaceFill &surface_fill : surface_fills)
    {
        if (surface_fill.surface.surface_type != stInternal)
        {
            continue;
        }

        switch (surface_fill.params.pattern)
        {
        case ipCount:
            continue;
            break;
        case ipSupportBase:
            continue;
            break;
        case ipEnsuring:
            continue;
            break;
        case ipLightning:
        case ipAdaptiveCubic:
        case ipSupportCubic:
        case ipRectilinear:
        case ipMonotonic:
        case ipMonotonicLines:
        case ipAlignedRectilinear:
        case ipGrid:
        case ipTriangles:
        case ipStars:
        case ipCubic:
        case ipLine:
        case ipConcentric:
        case ipHoneycomb:
        case ip3DHoneycomb:
        case ipGyroid:
        case ipHilbertCurve:
        case ipArchimedeanChords:
        case ipOctagramSpiral:
        case ipZigZag:
        case ipCustom:
        case ipCat:
        case ipCatMirrored:
        case ipCatTiled:
        case ipShark:
        case ipPuppy:
            break;
        }

        // Create the filler object.
        std::unique_ptr<Fill> f = std::unique_ptr<Fill>(Fill::new_from_type(surface_fill.params.pattern));
        f->set_bounding_box(bbox);
        f->layer_id = this->id() - this->object()->get_layer(0)->id(); // We need to subtract raft layers.
        f->z = this->print_z;
        f->angle = surface_fill.params.angle;
        f->overlap = surface_fill.params.overlap;
        f->perimeter_width = m_regions[surface_fill.region_id]->flow(frPerimeter).width();
        f->adapt_fill_octree = (surface_fill.params.pattern == ipSupportCubic) ? support_fill_octree
                                                                               : adaptive_fill_octree;
        f->print_config = &this->object()->print()->config();
        f->print_object_config = &this->object()->config();

        if (surface_fill.params.pattern == ipLightning)
            dynamic_cast<FillLightning::Filler *>(f.get())->generator = lightning_generator;

        if (surface_fill.params.pattern == ipCustom) {
            auto *fill_custom = dynamic_cast<FillCustom *>(f.get());
            assert(fill_custom != nullptr);
            fill_custom->print_region_config = &m_regions[surface_fill.region_id]->region().config();
        }

        // calculate flow spacing for infill pattern generation
        double link_max_length = 0.;
        if (!surface_fill.params.bridge)
        {
#if 0
            link_max_length = layerm.region()->config().get_abs_value(surface.is_external() ? "external_fill_link_max_length" : "fill_link_max_length", flow.spacing());
//            printf("flow spacing: %f,  is_external: %d, link_max_length: %lf\n", flow.spacing(), int(surface.is_external()), link_max_length);
#else
            if (surface_fill.params.density > 80.) // 80%
                link_max_length = 3. * f->spacing;
#endif
        }

        // Maximum length of the perimeter segment linking two infill lines.
        f->link_max_length = (coord_t) scale_(link_max_length);
        // Used by the concentric infill pattern to clip the loops to create extrusion paths.
        f->loop_clipping = coord_t(scale_(surface_fill.params.flow.nozzle_diameter()) *
                                   LOOP_CLIPPING_LENGTH_OVER_NOZZLE_DIAMETER);

        LayerRegion &layerm = *m_regions[surface_fill.region_id];

        // apply half spacing using this flow's own spacing and generate infill
        FillParams params;
        params.density = float(0.01 * surface_fill.params.density);

        // At exactly 50% density, distance = min_spacing / 0.5 = min_spacing * 2.0 (exact integer multiple)
        // This creates perfectly aligned coordinates that trigger geometric degeneracies in Clipper2.
        // Treat 50.0% as 49.9% to avoid the exact 2x multiplier.
        // Applies to: ipConcentric (9) and ipEnsuring/Athena (20) which use heavy Clipper2 operations.
        if ((surface_fill.params.pattern == ipConcentric || surface_fill.params.pattern == ipEnsuring) &&
            std::abs(params.density - 0.5f) < 0.0001f)
        {
            params.density = 0.499f;
        }

        params.dont_adjust = false; //  surface_fill.params.dont_adjust;
        params.bridge = surface_fill.params.bridge;
        params.anchor_length = surface_fill.params.anchor_length;
        params.anchor_length_max = surface_fill.params.anchor_length_max;
        params.resolution = resolution;
        params.use_advanced_perimeters = false;
        params.layer_height = layerm.layer()->height;

        Point last_fill_pos = Point(0, 0);
        bool have_last_pos = false;

        for (ExPolygon &expoly : surface_fill.expolygons)
        {
            // Spacing is modified by the filler to indicate adjustments. Reset it for each expolygon.
            f->spacing = surface_fill.params.spacing;
            // For bridges: use original flow width so boundary offset is independent of line spacing
            f->bounding_width = surface_fill.params.bridge ? surface_fill.params.flow.width()
                                                           : surface_fill.params.spacing;

            if (have_last_pos)
            {
                params.start_near = last_fill_pos;
            }
            else
            {
                params.start_near = expoly.contour.centroid();
            }

            surface_fill.surface.expolygon = std::move(expoly);
            try
            {
                Polylines polylines = f->fill_surface(&surface_fill.surface, params);
                if (!polylines.empty())
                {
                    last_fill_pos = polylines.back().last_point();
                    have_last_pos = true;
                }
                sparse_infill_polylines.insert(sparse_infill_polylines.end(), polylines.begin(), polylines.end());
            }
            catch (InfillFailedException &)
            {
            }
        }
    }

    return sparse_infill_polylines;
}

// Create ironing extrusions over top surfaces.
void Layer::make_ironing()
{
    // LayerRegion::slices contains surfaces marked with SurfaceType.
    // Here we want to collect top surfaces extruded with the same extruder.
    // A surface will be ironed with the same extruder to not contaminate the print with another material leaking from the nozzle.

    // First classify regions based on the extruder used.
    struct IroningParams
    {
        int extruder = -1;
        bool just_infill = false;
        // Spacing of the ironing lines, also to calculate the extrusion flow from.
        double line_spacing;
        // Height of the extrusion, to calculate the extrusion flow from.
        double height;
        double speed;
        double angle;

        bool operator<(const IroningParams &rhs) const
        {
            if (this->extruder < rhs.extruder)
                return true;
            if (this->extruder > rhs.extruder)
                return false;
            if (int(this->just_infill) < int(rhs.just_infill))
                return true;
            if (int(this->just_infill) > int(rhs.just_infill))
                return false;
            if (this->line_spacing < rhs.line_spacing)
                return true;
            if (this->line_spacing > rhs.line_spacing)
                return false;
            if (this->height < rhs.height)
                return true;
            if (this->height > rhs.height)
                return false;
            if (this->speed < rhs.speed)
                return true;
            if (this->speed > rhs.speed)
                return false;
            if (this->angle < rhs.angle)
                return true;
            if (this->angle > rhs.angle)
                return false;
            return false;
        }

        bool operator==(const IroningParams &rhs) const
        {
            return this->extruder == rhs.extruder && this->just_infill == rhs.just_infill &&
                   this->line_spacing == rhs.line_spacing && this->height == rhs.height && this->speed == rhs.speed &&
                   this->angle == rhs.angle;
        }

        LayerRegion *layerm;
        uint32_t region_id;

        // IdeaMaker: ironing
        // ironing flowrate (5% percent)
        // ironing speed (10 mm/sec)

        // Kisslicer:
        // iron off, Sweep, Group
        // ironing speed: 15 mm/sec

        // Cura:
        // Pattern (zig-zag / concentric)
        // line spacing (0.1mm)
        // flow: from normal layer height. 10%
        // speed: 20 mm/sec
    };

    std::vector<IroningParams> by_extruder;
    double default_layer_height = this->object()->config().layer_height;

    for (uint32_t region_id = 0; region_id < uint32_t(this->regions().size()); ++region_id)
        if (LayerRegion *layerm = this->get_region(region_id); !layerm->slices().empty())
        {
            IroningParams ironing_params;
            const PrintRegionConfig &config = layerm->region().config();
            if (config.ironing && (config.ironing_type == IroningType::AllSolid ||
                                   (config.top_solid_layers > 0 && (config.ironing_type == IroningType::TopSurfaces ||
                                                                    (config.ironing_type == IroningType::TopmostOnly &&
                                                                     layerm->layer()->upper_layer == nullptr)))))
            {
                if (config.perimeter_extruder == config.solid_infill_extruder || config.perimeters == 0)
                {
                    // Iron the whole face.
                    ironing_params.extruder = config.solid_infill_extruder;
                }
                else
                {
                    // Iron just the infill.
                    ironing_params.extruder = config.solid_infill_extruder;
                }
            }
            if (ironing_params.extruder != -1)
            {
                //TODO just_infill is currently not used.
                ironing_params.just_infill = false;
                ironing_params.line_spacing = config.ironing_spacing;
                ironing_params.height = default_layer_height * 0.01 * config.ironing_flowrate;
                ironing_params.speed = config.ironing_speed;
                ironing_params.angle = config.fill_angle * M_PI / 180.;
                ironing_params.layerm = layerm;
                ironing_params.region_id = region_id;
                by_extruder.emplace_back(ironing_params);
            }
        }
    std::sort(by_extruder.begin(), by_extruder.end());

    FillRectilinear fill;
    FillParams fill_params;
    fill.set_bounding_box(this->object()->bounding_box());
    // Layer ID is used for orienting the infill in alternating directions.
    // Layer::id() returns layer ID including raft layers, subtract them to make the infill direction independent
    // from raft.
    //FIXME ironing does not take fill angle into account. Shall it? Does it matter?
    fill.layer_id = this->id() - this->object()->get_layer(0)->id();
    fill.z = this->print_z;
    fill.overlap = 0;
    fill_params.density = 1.;
    fill_params.monotonic = true;

    for (size_t i = 0; i < by_extruder.size();)
    {
        // Find span of regions equivalent to the ironing operation.
        IroningParams &ironing_params = by_extruder[i];
        size_t j = i;
        for (++j; j < by_extruder.size() && ironing_params == by_extruder[j]; ++j)
            ;
        // Create the ironing extrusions for regions <i, j)
        ExPolygons ironing_areas;
        double nozzle_dmr = this->object()->print()->config().nozzle_diameter.values[ironing_params.extruder - 1];
        if (ironing_params.just_infill)
        {
            //TODO just_infill is currently not used.
            // Just infill.
        }
        else
        {
            // Infill and perimeter.
            // Merge top surfaces with the same ironing parameters.
            Polygons polys;
            Polygons infills;
            for (size_t k = i; k < j; ++k)
            {
                const IroningParams &ironing_params = by_extruder[k];
                const PrintRegionConfig &region_config = ironing_params.layerm->region().config();
                bool iron_everything = region_config.ironing_type == IroningType::AllSolid;
                bool iron_completely = iron_everything;
                if (iron_everything)
                {
                    // Check whether there is any non-solid hole in the regions.
                    bool internal_infill_solid = region_config.fill_density.value > 95.;
                    for (const Surface &surface : ironing_params.layerm->fill_surfaces())
                        if ((!internal_infill_solid && surface.surface_type == stInternal) ||
                            surface.surface_type == stInternalBridge || surface.surface_type == stInternalVoid)
                        {
                            // Some fill region is not quite solid. Don't iron over the whole surface.
                            iron_completely = false;
                            break;
                        }
                }
                if (iron_completely)
                {
                    // Iron everything. This is likely only good for solid transparent objects.
                    for (const Surface &surface : ironing_params.layerm->slices())
                        polygons_append(polys, surface.expolygon);
                }
                else
                {
                    for (const Surface &surface : ironing_params.layerm->slices())
                        if (surface.surface_type == stTop || (iron_everything && surface.surface_type == stBottom))
                            // stBottomBridge is not being ironed on purpose, as it would likely destroy the bridges.
                            polygons_append(polys, surface.expolygon);
                }
                if (iron_everything && !iron_completely)
                {
                    // Add solid fill surfaces. This may not be ideal, as one will not iron perimeters touching these
                    // solid fill surfaces, but it is likely better than nothing.
                    for (const Surface &surface : ironing_params.layerm->fill_surfaces())
                        if (surface.surface_type == stInternalSolid)
                            polygons_append(infills, surface.expolygon);
                }
            }

            if (!infills.empty() || j > i + 1)
            {
                // Ironing over more than a single region or over solid internal infill.
                if (!infills.empty())
                    // For IroningType::AllSolid only:
                    // Add solid infill areas for layers, that contain some non-ironable infil (sparse infill, bridge infill).
                    append(polys, std::move(infills));
                polys = union_safety_offset(polys);
            }
            // Trim the top surfaces with half the nozzle diameter.
            ironing_areas = intersection_ex(polys, offset(this->lslices, -float(scale_(0.5 * nozzle_dmr))));
        }

        // Create the filler object.
        fill.spacing = ironing_params.line_spacing;
        fill.bounding_width = ironing_params.line_spacing;
        fill.angle = float(ironing_params.angle + 0.25 * M_PI);
        fill.link_max_length = (coord_t) scale_(3. * fill.spacing);
        double extrusion_height = ironing_params.height * fill.spacing / nozzle_dmr;
        float extrusion_width = Flow::rounded_rectangle_extrusion_width_from_spacing(float(nozzle_dmr),
                                                                                     float(extrusion_height));
        double flow_mm3_per_mm = nozzle_dmr * extrusion_height;
        Surface surface_fill(stTop, ExPolygon());
        for (ExPolygon &expoly : ironing_areas)
        {
            surface_fill.expolygon = std::move(expoly);
            Polylines polylines;
            try
            {
                assert(!fill_params.use_advanced_perimeters);
                polylines = fill.fill_surface(&surface_fill, fill_params);
            }
            catch (InfillFailedException &)
            {
            }
            if (!polylines.empty())
            {
                // Save into layer.
                auto fill_begin = uint32_t(ironing_params.layerm->fills().size());
                ExtrusionEntityCollection *eec = nullptr;
                ironing_params.layerm->m_fills.entities.push_back(eec = new ExtrusionEntityCollection());
                // Don't sort the ironing infill lines as they are monotonicly ordered.
                eec->no_sort = true;
                extrusion_entities_append_paths(eec->entities, std::move(polylines),
                                                ExtrusionAttributes{ExtrusionRole::Ironing,
                                                                    ExtrusionFlow{flow_mm3_per_mm, extrusion_width,
                                                                                  float(extrusion_height)}});
                insert_fills_into_islands(*this, ironing_params.region_id, fill_begin,
                                          uint32_t(ironing_params.layerm->fills().size()));
            }
        }

        // Regions up to j were processed.
        i = j;
    }
}

} // namespace Slic3r
