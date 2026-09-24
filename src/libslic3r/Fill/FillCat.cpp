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

// Shark outlines. The SVG tile is 500 by 420. The gap is the spacing between
// sharks, so a normal density draws a field of them rather than one huge shark.
constexpr double kSharkW = 500.0;
constexpr double kSharkH = 420.0;
constexpr double kSharkGap = 28.0;

// Puppy faces. The SVG tile is 60 by 108, one face wide and two rows tall.
constexpr double kPuppyW = 60.0;
constexpr double kPuppyH = 108.0;
constexpr double kPuppyGap = 6.5;

const std::vector<std::vector<Vec2d>> &shark_paths()
{
    static const std::vector<std::vector<Vec2d>> paths = {
        {Vec2d(0.0119, 1.0000), Vec2d(0.0000, 0.9857)},
        {Vec2d(0.5510, 0.9917), Vec2d(0.5540, 1.0000)},
        {Vec2d(1.0000, 0.9857), Vec2d(0.9754, 0.9560), Vec2d(0.9715, 1.0000)},
        {Vec2d(0.5593, 1.0000), Vec2d(0.5510, 0.9917), Vec2d(0.5593, 1.0000)},
        {Vec2d(0.0510, 0.4917), Vec2d(0.1234, 0.6898), Vec2d(0.0694, 0.8879), Vec2d(0.1744, 0.7721),
         Vec2d(0.3600, 0.8007), Vec2d(0.4832, 0.9238), Vec2d(0.5680, 0.7971), Vec2d(0.9586, 0.7960),
         Vec2d(0.8892, 0.7017), Vec2d(0.8690, 0.7207), Vec2d(0.8474, 0.7017), Vec2d(0.8244, 0.7207),
         Vec2d(0.8042, 0.7017), Vec2d(0.7826, 0.7207), Vec2d(0.7626, 0.7017), Vec2d(0.7424, 0.7207),
         Vec2d(0.7208, 0.7017), Vec2d(0.6992, 0.7207), Vec2d(0.6684, 0.6755), Vec2d(0.6992, 0.6276),
         Vec2d(0.7208, 0.6443), Vec2d(0.7424, 0.6252), Vec2d(0.7626, 0.6443), Vec2d(0.7826, 0.6252),
         Vec2d(0.8042, 0.6443), Vec2d(0.8244, 0.6252), Vec2d(0.8474, 0.6443), Vec2d(0.8690, 0.6252),
         Vec2d(0.8860, 0.6443), Vec2d(0.9000, 0.6229), Vec2d(0.8892, 0.5990), Vec2d(0.5958, 0.6014),
         Vec2d(0.4754, 0.4560), Vec2d(0.4646, 0.5788), Vec2d(0.1614, 0.6021), Vec2d(0.0510, 0.4917),
         Vec2d(0.1614, 0.6021), Vec2d(0.3600, 0.8007), Vec2d(0.4832, 0.9238), Vec2d(0.5510, 0.9917)},
        {Vec2d(0.0000, 0.3987), Vec2d(0.0680, 0.2971), Vec2d(0.4586, 0.2960), Vec2d(0.3892, 0.2017),
         Vec2d(0.3690, 0.2207), Vec2d(0.3474, 0.2017), Vec2d(0.3244, 0.2207), Vec2d(0.3042, 0.2017),
         Vec2d(0.2826, 0.2207), Vec2d(0.2626, 0.2017), Vec2d(0.2424, 0.2207), Vec2d(0.2208, 0.2017),
         Vec2d(0.1992, 0.2207), Vec2d(0.1684, 0.1755), Vec2d(0.1992, 0.1276), Vec2d(0.2208, 0.1443),
         Vec2d(0.2424, 0.1252), Vec2d(0.2626, 0.1443), Vec2d(0.2826, 0.1252), Vec2d(0.3042, 0.1443),
         Vec2d(0.3244, 0.1252), Vec2d(0.3474, 0.1443), Vec2d(0.3690, 0.1252), Vec2d(0.3860, 0.1443),
         Vec2d(0.4000, 0.1229), Vec2d(0.3892, 0.0990), Vec2d(0.0958, 0.1014), Vec2d(0.0119, 0.0000)},
        {Vec2d(0.0000, 0.4406), Vec2d(0.0510, 0.4917)},
        {Vec2d(0.5540, 0.0000), Vec2d(0.6234, 0.1898), Vec2d(0.5694, 0.3879), Vec2d(0.6744, 0.2721),
         Vec2d(0.8600, 0.3007), Vec2d(0.9832, 0.4238), Vec2d(1.0000, 0.3987)},
        {Vec2d(0.9715, 0.0000), Vec2d(0.9646, 0.0788), Vec2d(0.6614, 0.1021), Vec2d(0.5593, 0.0000),
         Vec2d(0.6614, 0.1021), Vec2d(0.8600, 0.3007), Vec2d(0.9832, 0.4238), Vec2d(1.0000, 0.4406)},
    };
    return paths;
}

const std::vector<std::vector<Vec2d>> &puppy_paths()
{
    static const std::vector<std::vector<Vec2d>> paths = {
        {Vec2d(0.0000, 1.0000), Vec2d(0.0264, 0.9909), Vec2d(0.0552, 0.9844), Vec2d(0.1167, 0.9792),
         Vec2d(0.1781, 0.9844), Vec2d(0.2069, 0.9909), Vec2d(0.2333, 1.0000), Vec2d(0.2333, 1.0000)},
        {Vec2d(0.0000, 0.9028), Vec2d(0.0000, 0.9028), Vec2d(0.0141, 0.8646), Vec2d(0.0123, 0.8481),
         Vec2d(0.0000, 0.8333), Vec2d(0.0617, 0.8024), Vec2d(0.1187, 0.7662), Vec2d(0.1664, 0.7266),
         Vec2d(0.2000, 0.6852), Vec2d(0.1000, 0.7384), Vec2d(0.0500, 0.7607), Vec2d(0.0000, 0.7778),
         Vec2d(0.0000, 0.7778)},
        {Vec2d(0.0000, 0.6944), Vec2d(0.0000, 0.6944), Vec2d(0.0141, 0.6528), Vec2d(0.0123, 0.6319),
         Vec2d(0.0000, 0.6111)},
        {Vec2d(0.2583, 0.9861), Vec2d(0.2364, 0.9806), Vec2d(0.2173, 0.9729), Vec2d(0.1876, 0.9522),
         Vec2d(0.1691, 0.9266), Vec2d(0.1615, 0.8987), Vec2d(0.1643, 0.8710), Vec2d(0.1775, 0.8461),
         Vec2d(0.2006, 0.8265), Vec2d(0.2333, 0.8148), Vec2d(0.2567, 0.8155), Vec2d(0.2768, 0.8238),
         Vec2d(0.2934, 0.8386), Vec2d(0.3063, 0.8588), Vec2d(0.3201, 0.9112), Vec2d(0.3167, 0.9722)},
        {Vec2d(0.7417, 0.9861), Vec2d(0.7636, 0.9806), Vec2d(0.7827, 0.9729), Vec2d(0.8124, 0.9522),
         Vec2d(0.8309, 0.9266), Vec2d(0.8385, 0.8987), Vec2d(0.8357, 0.8710), Vec2d(0.8225, 0.8461),
         Vec2d(0.7994, 0.8265), Vec2d(0.7667, 0.8148), Vec2d(0.7433, 0.8155), Vec2d(0.7232, 0.8238),
         Vec2d(0.7066, 0.8386), Vec2d(0.6937, 0.8588), Vec2d(0.6799, 0.9112), Vec2d(0.6833, 0.9722)},
        {Vec2d(0.5000, 0.8796), Vec2d(0.5000, 0.8611)},
        {Vec2d(0.4167, 0.8611), Vec2d(0.4375, 0.8524), Vec2d(0.4583, 0.8495), Vec2d(0.4792, 0.8524),
         Vec2d(0.5000, 0.8611), Vec2d(0.5208, 0.8524), Vec2d(0.5417, 0.8495), Vec2d(0.5625, 0.8524),
         Vec2d(0.5833, 0.8611)},
        {Vec2d(0.4667, 0.8472), Vec2d(0.4688, 0.8290), Vec2d(0.4750, 0.8160), Vec2d(0.4854, 0.8082),
         Vec2d(0.5000, 0.8056), Vec2d(0.5146, 0.8082), Vec2d(0.5250, 0.8160), Vec2d(0.5312, 0.8290),
         Vec2d(0.5333, 0.8472)},
        {Vec2d(0.4383, 0.9537), Vec2d(0.4354, 0.9477), Vec2d(0.4275, 0.9433), Vec2d(0.4167, 0.9417),
         Vec2d(0.4058, 0.9433), Vec2d(0.3979, 0.9477), Vec2d(0.3950, 0.9537), Vec2d(0.3979, 0.9597),
         Vec2d(0.4058, 0.9641), Vec2d(0.4167, 0.9657), Vec2d(0.4275, 0.9641), Vec2d(0.4354, 0.9597),
         Vec2d(0.4383, 0.9537)},
        {Vec2d(0.6050, 0.9537), Vec2d(0.6021, 0.9477), Vec2d(0.5942, 0.9433), Vec2d(0.5833, 0.9417),
         Vec2d(0.5725, 0.9433), Vec2d(0.5646, 0.9477), Vec2d(0.5617, 0.9537), Vec2d(0.5646, 0.9597),
         Vec2d(0.5725, 0.9641), Vec2d(0.5833, 0.9657), Vec2d(0.5942, 0.9641), Vec2d(0.6021, 0.9597),
         Vec2d(0.6050, 0.9537)},
        {Vec2d(0.5500, 0.8981), Vec2d(0.5462, 0.8911), Vec2d(0.5354, 0.8851), Vec2d(0.5191, 0.8810),
         Vec2d(0.5000, 0.8796), Vec2d(0.4809, 0.8810), Vec2d(0.4646, 0.8851), Vec2d(0.4538, 0.8911),
         Vec2d(0.4500, 0.8981), Vec2d(0.4538, 0.9052), Vec2d(0.4646, 0.9112), Vec2d(0.4809, 0.9153),
         Vec2d(0.5000, 0.9167), Vec2d(0.5191, 0.9153), Vec2d(0.5354, 0.9112), Vec2d(0.5462, 0.9052),
         Vec2d(0.5500, 0.8981)},
        {Vec2d(1.0000, 1.0000), Vec2d(0.9877, 0.9731), Vec2d(0.9859, 0.9479), Vec2d(0.9912, 0.9245),
         Vec2d(1.0000, 0.9028), Vec2d(1.0000, 0.9028)},
        {Vec2d(1.0000, 0.7778), Vec2d(1.0000, 0.7778), Vec2d(0.9877, 0.7569), Vec2d(0.9859, 0.7361),
         Vec2d(1.0000, 0.6944), Vec2d(1.0000, 0.6944)},
        {Vec2d(0.2333, 0.0000), Vec2d(0.2333, 0.0000), Vec2d(0.2380, 0.0287), Vec2d(0.2516, 0.0525),
         Vec2d(0.2738, 0.0718), Vec2d(0.3042, 0.0868), Vec2d(0.3424, 0.0979), Vec2d(0.3880, 0.1055),
         Vec2d(0.4407, 0.1098), Vec2d(0.5000, 0.1111), Vec2d(0.5593, 0.1098), Vec2d(0.6120, 0.1055),
         Vec2d(0.6576, 0.0979), Vec2d(0.6958, 0.0868), Vec2d(0.7262, 0.0718), Vec2d(0.7484, 0.0525),
         Vec2d(0.7620, 0.0287), Vec2d(0.7667, 0.0000), Vec2d(0.7931, 0.0091), Vec2d(0.8219, 0.0156),
         Vec2d(0.8833, 0.0208), Vec2d(0.9448, 0.0156), Vec2d(0.9736, 0.0091), Vec2d(1.0000, 0.0000)},
        {Vec2d(0.5000, 0.5000), Vec2d(0.5264, 0.4909), Vec2d(0.5552, 0.4844), Vec2d(0.6167, 0.4792),
         Vec2d(0.6781, 0.4844), Vec2d(0.7069, 0.4909), Vec2d(0.7333, 0.5000), Vec2d(0.7380, 0.5287),
         Vec2d(0.7516, 0.5525), Vec2d(0.7738, 0.5718), Vec2d(0.8042, 0.5868), Vec2d(0.8424, 0.5979),
         Vec2d(0.8880, 0.6055), Vec2d(0.9407, 0.6098), Vec2d(1.0000, 0.6111), Vec2d(1.0000, 0.6111)},
        {Vec2d(0.5000, 0.5000), Vec2d(0.4877, 0.4731), Vec2d(0.4859, 0.4479), Vec2d(0.4912, 0.4245),
         Vec2d(0.5000, 0.4028), Vec2d(0.5141, 0.3646), Vec2d(0.5123, 0.3481), Vec2d(0.5000, 0.3333),
         Vec2d(0.5617, 0.3024), Vec2d(0.6188, 0.2662), Vec2d(0.6664, 0.2266), Vec2d(0.7000, 0.1852),
         Vec2d(0.6000, 0.2384), Vec2d(0.5500, 0.2607), Vec2d(0.5000, 0.2778), Vec2d(0.4877, 0.2569),
         Vec2d(0.4859, 0.2361), Vec2d(0.5000, 0.1944), Vec2d(0.5141, 0.1528), Vec2d(0.5123, 0.1319),
         Vec2d(0.5000, 0.1111)},
        {Vec2d(0.7583, 0.4861), Vec2d(0.7364, 0.4806), Vec2d(0.7173, 0.4729), Vec2d(0.6876, 0.4522),
         Vec2d(0.6691, 0.4266), Vec2d(0.6615, 0.3987), Vec2d(0.6643, 0.3710), Vec2d(0.6775, 0.3461),
         Vec2d(0.7006, 0.3265), Vec2d(0.7333, 0.3148), Vec2d(0.7567, 0.3155), Vec2d(0.7768, 0.3238),
         Vec2d(0.7934, 0.3386), Vec2d(0.8063, 0.3588), Vec2d(0.8201, 0.4112), Vec2d(0.8167, 0.4722)},
        {Vec2d(1.0000, 0.3796), Vec2d(1.0000, 0.3611)},
        {Vec2d(0.9167, 0.3611), Vec2d(0.9375, 0.3524), Vec2d(0.9583, 0.3495), Vec2d(0.9792, 0.3524),
         Vec2d(1.0000, 0.3611), Vec2d(1.0000, 0.3611)},
        {Vec2d(0.9667, 0.3472), Vec2d(0.9688, 0.3290), Vec2d(0.9750, 0.3160), Vec2d(0.9854, 0.3082),
         Vec2d(1.0000, 0.3056), Vec2d(1.0000, 0.3056)},
        {Vec2d(0.9383, 0.4537), Vec2d(0.9354, 0.4477), Vec2d(0.9275, 0.4433), Vec2d(0.9167, 0.4417),
         Vec2d(0.9058, 0.4433), Vec2d(0.8979, 0.4477), Vec2d(0.8950, 0.4537), Vec2d(0.8979, 0.4597),
         Vec2d(0.9058, 0.4641), Vec2d(0.9167, 0.4657), Vec2d(0.9275, 0.4641), Vec2d(0.9354, 0.4597),
         Vec2d(0.9383, 0.4537)},
        {Vec2d(1.0000, 0.3796), Vec2d(1.0000, 0.3796), Vec2d(0.9809, 0.3810), Vec2d(0.9646, 0.3851),
         Vec2d(0.9538, 0.3911), Vec2d(0.9500, 0.3981), Vec2d(0.9538, 0.4052), Vec2d(0.9646, 0.4112),
         Vec2d(0.9809, 0.4153), Vec2d(1.0000, 0.4167), Vec2d(1.0000, 0.4167)},
        {Vec2d(0.0000, 0.6111), Vec2d(0.0000, 0.6111), Vec2d(0.0593, 0.6098), Vec2d(0.1120, 0.6055),
         Vec2d(0.1576, 0.5979), Vec2d(0.1958, 0.5868), Vec2d(0.2262, 0.5718), Vec2d(0.2484, 0.5525),
         Vec2d(0.2620, 0.5287), Vec2d(0.2667, 0.5000), Vec2d(0.2931, 0.5091), Vec2d(0.3219, 0.5156),
         Vec2d(0.3833, 0.5208), Vec2d(0.4448, 0.5156), Vec2d(0.4736, 0.5091), Vec2d(0.5000, 0.5000)},
        {Vec2d(0.2417, 0.4861), Vec2d(0.2636, 0.4806), Vec2d(0.2827, 0.4729), Vec2d(0.3124, 0.4522),
         Vec2d(0.3309, 0.4266), Vec2d(0.3385, 0.3987), Vec2d(0.3357, 0.3710), Vec2d(0.3225, 0.3461),
         Vec2d(0.2994, 0.3265), Vec2d(0.2667, 0.3148), Vec2d(0.2433, 0.3155), Vec2d(0.2232, 0.3238),
         Vec2d(0.2066, 0.3386), Vec2d(0.1938, 0.3588), Vec2d(0.1799, 0.4112), Vec2d(0.1833, 0.4722)},
        {Vec2d(0.0000, 0.3796), Vec2d(0.0000, 0.3611)},
        {Vec2d(0.0000, 0.3611), Vec2d(0.0000, 0.3611), Vec2d(0.0208, 0.3524), Vec2d(0.0417, 0.3495),
         Vec2d(0.0625, 0.3524), Vec2d(0.0833, 0.3611)},
        {Vec2d(0.0000, 0.3056), Vec2d(0.0000, 0.3056), Vec2d(0.0146, 0.3082), Vec2d(0.0250, 0.3160),
         Vec2d(0.0312, 0.3290), Vec2d(0.0333, 0.3472)},
        {Vec2d(0.1050, 0.4537), Vec2d(0.1021, 0.4477), Vec2d(0.0942, 0.4433), Vec2d(0.0833, 0.4417),
         Vec2d(0.0725, 0.4433), Vec2d(0.0646, 0.4477), Vec2d(0.0617, 0.4537), Vec2d(0.0646, 0.4597),
         Vec2d(0.0725, 0.4641), Vec2d(0.0833, 0.4657), Vec2d(0.0942, 0.4641), Vec2d(0.1021, 0.4597),
         Vec2d(0.1050, 0.4537)},
        {Vec2d(0.0500, 0.3981), Vec2d(0.0462, 0.3911), Vec2d(0.0354, 0.3851), Vec2d(0.0191, 0.3810),
         Vec2d(0.0000, 0.3796), Vec2d(0.0000, 0.3796)},
        {Vec2d(0.0000, 0.4167), Vec2d(0.0000, 0.4167), Vec2d(0.0191, 0.4153), Vec2d(0.0354, 0.4112),
         Vec2d(0.0462, 0.4052), Vec2d(0.0500, 0.3981)},
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

void FillShark::_fill_surface_single(const FillParams &params, unsigned int,
                                     const std::pair<float, Point> &direction, ExPolygon expolygon,
                                     Polylines &polylines_out)
{
    if (params.density < 1e-4f || this->spacing < 1e-6)
        return;

    const double user_angle = double(direction.first) - M_PI / 2.0;
    const double unit_to_mm = double(this->spacing) / (double(params.density) * kSharkGap);
    stamp_tiles(shark_paths(), kSharkW * unit_to_mm, kSharkH * unit_to_mm, user_angle, expolygon, this->spacing,
                params.start_near.has_value(), params.start_near ? &(*params.start_near) : nullptr, polylines_out);
}

void FillPuppy::_fill_surface_single(const FillParams &params, unsigned int,
                                     const std::pair<float, Point> &direction, ExPolygon expolygon,
                                     Polylines &polylines_out)
{
    if (params.density < 1e-4f || this->spacing < 1e-6)
        return;

    const double user_angle = double(direction.first) - M_PI / 2.0;
    const double unit_to_mm = double(this->spacing) / (double(params.density) * kPuppyGap);
    stamp_tiles(puppy_paths(), kPuppyW * unit_to_mm, kPuppyH * unit_to_mm, user_angle, expolygon, this->spacing,
                params.start_near.has_value(), params.start_near ? &(*params.start_near) : nullptr, polylines_out);
}

} // namespace Slic3r
