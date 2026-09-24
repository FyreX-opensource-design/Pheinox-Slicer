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

// Centerline of the original Cat Fill tile, in millimeters. The tile is
// 21 mm wide and 15 mm tall. The first stroke is the tail; the second is the
// head, the body, and the vertical that runs into the next row.
// Spiral turns sit about 2.4 mm apart, which density scales against.
constexpr double kCatTileW = 21.0;
constexpr double kCatTileH = 15.0;
constexpr double kCatGapMm = 2.4;

const std::vector<std::vector<Vec2d>> &cat_paths()
{
    static const std::vector<std::vector<Vec2d>> paths = {
        {Vec2d(6.42, 0.92),  Vec2d(16.33, 1.01), Vec2d(17.53, 1.33), Vec2d(18.71, 2.18), Vec2d(19.54, 3.61),
         Vec2d(19.68, 5.11), Vec2d(19.19, 6.58), Vec2d(18.10, 7.75), Vec2d(17.05, 8.25), Vec2d(15.93, 8.40),
         Vec2d(15.36, 8.22), Vec2d(14.89, 7.66), Vec2d(14.83, 6.93), Vec2d(15.15, 6.35), Vec2d(16.64, 5.82),
         Vec2d(17.11, 5.26), Vec2d(17.20, 4.73), Vec2d(17.02, 4.16), Vec2d(16.27, 3.63), Vec2d(15.09, 3.71),
         Vec2d(13.90, 4.25), Vec2d(13.00, 5.13), Vec2d(12.46, 6.21), Vec2d(12.32, 7.71), Vec2d(12.81, 9.18),
         Vec2d(13.90, 10.35), Vec2d(15.60, 11.08)},
        {Vec2d(0.92, 0.00), Vec2d(1.00, 9.08), Vec2d(1.18, 9.60), Vec2d(3.30, 8.97),  Vec2d(4.31, 9.06),
         Vec2d(5.81, 9.60), Vec2d(6.00, 9.00), Vec2d(5.99, 6.00), Vec2d(6.18, 5.40),  Vec2d(8.30, 6.03),
         Vec2d(9.27, 5.95), Vec2d(10.81, 5.40), Vec2d(10.99, 5.92), Vec2d(10.91, 15.00)},
    };
    return paths;
}

// Tightest parallel gap in the drawn tile, as a fraction of the tile size.
// The spiral's radial pitch is (0.15 - 0.04) / 1.35.
constexpr double kCatMirroredGapFraction = 0.08;

// One cat, plus the same cat rotated 180 degrees, in a unit square.
// Ears sit on a horizontal line. The tail is a spiral that starts on that line
// and curls away from it. A vertical line ties the rows together.
std::vector<std::vector<Vec2d>> cat_mirrored_unit()
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

const std::vector<std::vector<Vec2d>> &cat_mirrored_paths()
{
    static const std::vector<std::vector<Vec2d>> paths = cat_mirrored_unit();
    return paths;
}

void stamp_tiles(const std::vector<std::vector<Vec2d>> &paths, double tile_w_mm, double tile_h_mm,
                 double user_angle, const ExPolygon &expolygon, double chain_mm, bool start_near_valid,
                 const Point *start_near, Polylines &polylines_out)
{
    const coord_t tile_w = std::max(coord_t(std::lround(scale_(tile_w_mm))), coord_t(1));
    const coord_t tile_h = std::max(coord_t(std::lround(scale_(tile_h_mm))), coord_t(1));

    Polygon bounds = expolygon.contour;
    bounds.rotate(user_angle, Point(0, 0));
    BoundingBox bb = bounds.bounding_box();
    bb.merge(Point(bb.max.x() + tile_w, bb.max.y() + tile_h));
    const Point origin = align_to_grid(bb.min, Point(tile_w, tile_h));

    Polylines tiled;
    for (coord_t y = origin.y(); y <= bb.max.y(); y += tile_h)
    {
        for (coord_t x = origin.x(); x <= bb.max.x(); x += tile_w)
        {
            for (const std::vector<Vec2d> &path : paths)
            {
                Polyline pl;
                pl.points.reserve(path.size());
                for (const Vec2d &p : path)
                {
                    pl.points.emplace_back(x + coord_t(std::lround(p.x() * double(tile_w))),
                                           y + coord_t(std::lround(p.y() * double(tile_h))));
                }
                pl.rotate(-user_angle, Point(0, 0));
                tiled.push_back(std::move(pl));
            }
        }
    }

    tiled = intersection_pl(std::move(tiled), expolygon);
    if (tiled.empty())
        return;

    append(polylines_out, chain_polylines_by_region(std::move(tiled), coord_t(scale_(chain_mm)),
                                                    start_near_valid ? start_near : nullptr));
}

// One repeat of the cat field, as fractions of a 342 by 234 cell.
// The closest lines in that cell are about 14.4 px apart.
constexpr double kCatTiledWpx = 342.0;
constexpr double kCatTiledHpx = 234.0;
constexpr double kCatTiledGapPx = 14.4;

const std::vector<std::vector<Vec2d>> &cat_tiled_paths()
{
    static const std::vector<std::vector<Vec2d>> paths = {
        {Vec2d(0.3158, 0.0000), Vec2d(0.3158, 0.2949), Vec2d(0.3275, 0.2949), Vec2d(0.3713, 0.2479),
         Vec2d(0.4298, 0.2479), Vec2d(0.4854, 0.2949), Vec2d(0.4883, 0.1026), Vec2d(0.5468, 0.1624),
         Vec2d(0.5877, 0.1624), Vec2d(0.6462, 0.1111), Vec2d(0.6462, 0.6282), Vec2d(0.6608, 0.6282),
         Vec2d(0.7047, 0.5812), Vec2d(0.7632, 0.5812), Vec2d(0.8158, 0.6325), Vec2d(0.8187, 0.4402),
         Vec2d(0.8801, 0.4957), Vec2d(0.9211, 0.4957), Vec2d(0.9766, 0.4402), Vec2d(0.9795, 0.9615),
         Vec2d(0.9883, 0.9658), Vec2d(1.0000, 0.9573)},
        {Vec2d(0.9240, 1.0000), Vec2d(0.9298, 0.9915), Vec2d(0.9298, 0.9103), Vec2d(0.8947, 0.8632),
         Vec2d(0.8713, 0.8291), Vec2d(0.4327, 0.8291), Vec2d(0.4064, 0.7906), Vec2d(0.3743, 0.7393),
         Vec2d(0.3743, 0.6624), Vec2d(0.4035, 0.6111), Vec2d(0.4327, 0.5769), Vec2d(0.4883, 0.5855),
         Vec2d(0.5029, 0.6111), Vec2d(0.5029, 0.6282), Vec2d(0.4678, 0.7051), Vec2d(0.4942, 0.7479),
         Vec2d(0.5351, 0.7479), Vec2d(0.5965, 0.6624), Vec2d(0.5848, 0.5598), Vec2d(0.5380, 0.4957),
         Vec2d(0.0994, 0.4957), Vec2d(0.0819, 0.4701), Vec2d(0.0439, 0.4060), Vec2d(0.0409, 0.3291),
         Vec2d(0.0556, 0.3034), Vec2d(0.0994, 0.2436), Vec2d(0.1550, 0.2479), Vec2d(0.1696, 0.2735),
         Vec2d(0.1725, 0.2906), Vec2d(0.1374, 0.3632), Vec2d(0.1345, 0.3718), Vec2d(0.1404, 0.3803),
         Vec2d(0.1608, 0.4145), Vec2d(0.2018, 0.4145), Vec2d(0.2310, 0.3718), Vec2d(0.2632, 0.3291),
         Vec2d(0.2632, 0.2393), Vec2d(0.2544, 0.2265), Vec2d(0.2105, 0.1709), Vec2d(0.2047, 0.1624),
         Vec2d(0.0000, 0.1624)},
        {Vec2d(0.0000, 0.9573), Vec2d(0.0380, 0.9145), Vec2d(0.0965, 0.9145), Vec2d(0.1520, 0.9615),
         Vec2d(0.1550, 0.7692), Vec2d(0.2135, 0.8291), Vec2d(0.2573, 0.8291), Vec2d(0.2895, 0.7906),
         Vec2d(0.3129, 0.7735), Vec2d(0.3158, 1.0000)},
        {Vec2d(1.0000, 0.1624), Vec2d(0.7632, 0.1581), Vec2d(0.7251, 0.0983), Vec2d(0.7047, 0.0684),
         Vec2d(0.7047, 0.0000)},
        {Vec2d(0.7047, 1.0000), Vec2d(0.7281, 0.9573), Vec2d(0.7661, 0.9103), Vec2d(0.8158, 0.9103),
         Vec2d(0.8275, 0.9274), Vec2d(0.8363, 0.9573), Vec2d(0.8158, 1.0000)},
        {Vec2d(0.8158, 0.0000), Vec2d(0.8012, 0.0427), Vec2d(0.8275, 0.0812), Vec2d(0.8684, 0.0812),
         Vec2d(0.8772, 0.0684), Vec2d(0.9211, 0.0000)},
    };
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
    const double scale_mm = double(this->spacing) / (double(params.density) * kCatGapMm);
    // Paths are stored in millimeters of the native tile. stamp_tiles treats
    // coordinates as fractions of the tile, so divide the native size out.
    std::vector<std::vector<Vec2d>> unit;
    unit.reserve(cat_paths().size());
    for (const std::vector<Vec2d> &path : cat_paths())
    {
        std::vector<Vec2d> scaled;
        scaled.reserve(path.size());
        for (const Vec2d &p : path)
            scaled.emplace_back(p.x() / kCatTileW, p.y() / kCatTileH);
        unit.push_back(std::move(scaled));
    }

    stamp_tiles(unit, kCatTileW * scale_mm, kCatTileH * scale_mm, user_angle, expolygon, this->spacing,
                params.start_near.has_value(), params.start_near ? &(*params.start_near) : nullptr, polylines_out);
}

void FillCatMirrored::_fill_surface_single(const FillParams &params, unsigned int,
                                           const std::pair<float, Point> &direction, ExPolygon expolygon,
                                           Polylines &polylines_out)
{
    if (params.density < 1e-4f || this->spacing < 1e-6)
        return;

    const double user_angle = double(direction.first) - M_PI / 2.0;
    const double tile_mm = double(this->spacing) / (double(params.density) * kCatMirroredGapFraction);
    stamp_tiles(cat_mirrored_paths(), tile_mm, tile_mm, user_angle, expolygon, this->spacing,
                params.start_near.has_value(), params.start_near ? &(*params.start_near) : nullptr, polylines_out);
}

void FillCatTiled::_fill_surface_single(const FillParams &params, unsigned int,
                                        const std::pair<float, Point> &direction, ExPolygon expolygon,
                                        Polylines &polylines_out)
{
    if (params.density < 1e-4f || this->spacing < 1e-6)
        return;

    const double user_angle = double(direction.first) - M_PI / 2.0;
    const double px_to_mm = double(this->spacing) / (double(params.density) * kCatTiledGapPx);
    stamp_tiles(cat_tiled_paths(), kCatTiledWpx * px_to_mm, kCatTiledHpx * px_to_mm, user_angle, expolygon,
                this->spacing, params.start_near.has_value(),
                params.start_near ? &(*params.start_near) : nullptr, polylines_out);
}

} // namespace Slic3r
