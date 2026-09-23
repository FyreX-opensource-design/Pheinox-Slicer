///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#include "FillCat.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "../ClipperUtils.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

namespace Slic3r
{
namespace
{

// Tightest parallel gap in the tile, as a fraction of the tile size.
// The spiral's radial pitch is (0.15 - 0.04) / 1.35.
constexpr double kCatGapFraction = 0.08;

// One cat, plus the same cat rotated 180 degrees, in a unit square.
// Ears sit on a horizontal line. The tail is a spiral that starts on that line
// and curls away from it. A vertical line ties the rows together.
std::vector<std::vector<Vec2d>> cat_unit()
{
    std::vector<std::vector<Vec2d>> paths;
    const double spine_y = 0.73;

    std::vector<Vec2d> spine;
    spine.emplace_back(0.0, spine_y);
    constexpr int kEarSamples = 32;
    for (int i = 0; i <= kEarSamples; ++i)
    {
        const double t = double(i) / double(kEarSamples);
        const double x = 0.02 + 0.38 * t;
        const double y = spine_y + 0.22 * std::abs(std::sin(2.0 * M_PI * t));
        spine.emplace_back(x, y);
    }
    spine.emplace_back(1.0, spine_y);
    paths.push_back(std::move(spine));

    std::vector<Vec2d> spiral;
    const double sx = 0.70;
    const double sy = spine_y;
    const double cx = 0.70;
    const double cy = 0.58;
    const double r0 = std::hypot(sx - cx, sy - cy);
    const double r1 = 0.04;
    const double turns = 1.35;
    const double a0 = std::atan2(sy - cy, sx - cx);
    constexpr int kSpiralSamples = 40;
    spiral.reserve(kSpiralSamples + 1);
    for (int i = 0; i <= kSpiralSamples; ++i)
    {
        const double t = double(i) / double(kSpiralSamples);
        const double a = a0 - turns * 2.0 * M_PI * t;
        const double r = r0 + (r1 - r0) * t;
        spiral.emplace_back(cx + r * std::cos(a), cy + r * std::sin(a));
    }
    paths.push_back(std::move(spiral));

    const size_t motif_count = paths.size();
    for (size_t i = 0; i < motif_count; ++i)
    {
        std::vector<Vec2d> turned;
        turned.reserve(paths[i].size());
        for (const Vec2d &p : paths[i])
            turned.emplace_back(1.0 - p.x(), 1.0 - p.y());
        paths.push_back(std::move(turned));
    }

    paths.push_back({Vec2d(0.50, 0.0), Vec2d(0.50, 1.0)});
    return paths;
}

const std::vector<std::vector<Vec2d>> &cat_unit_paths()
{
    static const std::vector<std::vector<Vec2d>> paths = cat_unit();
    return paths;
}

} // namespace

void FillCat::_fill_surface_single(const FillParams &params, unsigned int, const std::pair<float, Point> &direction,
                                   ExPolygon expolygon, Polylines &polylines_out)
{
    if (params.density < 1e-4f || this->spacing < 1e-6)
        return;

    // _infill_direction adds a quarter turn. Take it back so a fill angle of 0
    // keeps the ears upright, and so layers stay aligned (layer angle is 0).
    const double user_angle = double(direction.first) - M_PI / 2.0;
    const double tile_mm = double(this->spacing) / (double(params.density) * kCatGapFraction);
    const coord_t tile = std::max(coord_t(scale_(tile_mm)), coord_t(1));

    Polygon bounds = expolygon.contour;
    bounds.rotate(user_angle, Point(0, 0));
    BoundingBox bb = bounds.bounding_box();
    bb.merge(Point(bb.max.x() + tile, bb.max.y() + tile));
    const Point origin = align_to_grid(bb.min, Point(tile, tile));

    Polylines tiled;
    const auto &unit = cat_unit_paths();
    for (coord_t y = origin.y(); y <= bb.max.y(); y += tile)
    {
        for (coord_t x = origin.x(); x <= bb.max.x(); x += tile)
        {
            for (const std::vector<Vec2d> &path : unit)
            {
                Polyline pl;
                pl.points.reserve(path.size());
                for (const Vec2d &p : path)
                {
                    pl.points.emplace_back(x + coord_t(std::lround(p.x() * double(tile))),
                                           y + coord_t(std::lround(p.y() * double(tile))));
                }
                pl.rotate(-user_angle, Point(0, 0));
                tiled.push_back(std::move(pl));
            }
        }
    }

    tiled = intersection_pl(std::move(tiled), expolygon);
    if (tiled.empty())
        return;

    append(polylines_out,
           chain_polylines_by_region(std::move(tiled), coord_t(scale_(this->spacing)),
                                     params.start_near ? &(*params.start_near) : nullptr));
}

} // namespace Slic3r
