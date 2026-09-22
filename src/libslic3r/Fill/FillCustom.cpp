///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#include "FillCustom.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/log/trivial.hpp>

#include "../ClipperUtils.hpp"
#include "../FileReader.hpp"
#include "../Geometry.hpp"
#include "../Line.hpp"
#include "../NSVGUtils.hpp"
#include "../PNGReadWrite.hpp"
#include "../ShortestPath.hpp"
#include "../TriangleMesh.hpp"
#include "../TriangleMeshSlicer.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace Slic3r
{
namespace
{

// ---------------------------------------------------------------------------
// Tiny expression evaluator: numbers, + - * / ^, (), sin/cos/.../min/max, x y z pi
// ---------------------------------------------------------------------------
class Expr
{
public:
    explicit Expr(std::string src) : m_src(std::move(src)) {}

    bool ok() const { return m_error.empty(); }
    const std::string &error() const { return m_error; }

    double eval(double x, double y, double z) const
    {
        if (! ok())
            return 0.;
        size_t i = 0;
        try {
            const double v = parse_expr(i, x, y, z);
            skip_ws(i);
            if (i != m_src.size())
                throw std::runtime_error("trailing characters");
            return v;
        } catch (...) {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    static std::optional<Expr> compile(const std::string &src)
    {
        Expr e(src);
        // Dry-run parse to catch syntax errors early.
        size_t i = 0;
        try {
            (void) e.parse_expr(i, 0., 0., 0.);
            e.skip_ws(i);
            if (i != e.m_src.size())
                e.m_error = "trailing characters in expression";
        } catch (const std::exception &ex) {
            e.m_error = ex.what();
        }
        if (! e.ok())
            return std::nullopt;
        return e;
    }

private:
    std::string m_src;
    mutable std::string m_error;

    void skip_ws(size_t &i) const
    {
        while (i < m_src.size() && std::isspace(static_cast<unsigned char>(m_src[i])))
            ++i;
    }

    bool match(size_t &i, char c) const
    {
        skip_ws(i);
        if (i < m_src.size() && m_src[i] == c) {
            ++i;
            return true;
        }
        return false;
    }

    double parse_expr(size_t &i, double x, double y, double z) const
    {
        double v = parse_term(i, x, y, z);
        for (;;) {
            if (match(i, '+'))
                v += parse_term(i, x, y, z);
            else if (match(i, '-'))
                v -= parse_term(i, x, y, z);
            else
                break;
        }
        return v;
    }

    double parse_term(size_t &i, double x, double y, double z) const
    {
        double v = parse_power(i, x, y, z);
        for (;;) {
            if (match(i, '*'))
                v *= parse_power(i, x, y, z);
            else if (match(i, '/')) {
                const double d = parse_power(i, x, y, z);
                v = (std::abs(d) < 1e-18) ? 0. : (v / d);
            } else
                break;
        }
        return v;
    }

    double parse_power(size_t &i, double x, double y, double z) const
    {
        double v = parse_unary(i, x, y, z);
        if (match(i, '^'))
            v = std::pow(v, parse_unary(i, x, y, z));
        return v;
    }

    double parse_unary(size_t &i, double x, double y, double z) const
    {
        if (match(i, '+'))
            return parse_unary(i, x, y, z);
        if (match(i, '-'))
            return -parse_unary(i, x, y, z);
        return parse_primary(i, x, y, z);
    }

    double parse_primary(size_t &i, double x, double y, double z) const
    {
        skip_ws(i);
        if (i >= m_src.size())
            throw std::runtime_error("unexpected end of expression");

        if (std::isdigit(static_cast<unsigned char>(m_src[i])) || m_src[i] == '.') {
            size_t j = i;
            while (j < m_src.size() &&
                   (std::isdigit(static_cast<unsigned char>(m_src[j])) || m_src[j] == '.' || m_src[j] == 'e' ||
                    m_src[j] == 'E' ||
                    ((m_src[j] == '+' || m_src[j] == '-') && j > i && (m_src[j - 1] == 'e' || m_src[j - 1] == 'E'))))
                ++j;
            const double v = std::stod(m_src.substr(i, j - i));
            i = j;
            return v;
        }

        if (match(i, '(')) {
            const double v = parse_expr(i, x, y, z);
            if (! match(i, ')'))
                throw std::runtime_error("missing ')'");
            return v;
        }

        // identifier
        size_t j = i;
        if (! std::isalpha(static_cast<unsigned char>(m_src[j])) && m_src[j] != '_')
            throw std::runtime_error("unexpected character");
        while (j < m_src.size() &&
               (std::isalnum(static_cast<unsigned char>(m_src[j])) || m_src[j] == '_'))
            ++j;
        const std::string id = m_src.substr(i, j - i);
        i = j;

        auto arg1 = [&]() {
            if (! match(i, '('))
                throw std::runtime_error("expected '(' after function");
            const double a = parse_expr(i, x, y, z);
            if (! match(i, ')'))
                throw std::runtime_error("missing ')'");
            return a;
        };
        auto arg2 = [&]() {
            if (! match(i, '('))
                throw std::runtime_error("expected '(' after function");
            const double a = parse_expr(i, x, y, z);
            if (! match(i, ','))
                throw std::runtime_error("expected ','");
            const double b = parse_expr(i, x, y, z);
            if (! match(i, ')'))
                throw std::runtime_error("missing ')'");
            return std::pair<double, double>{a, b};
        };

        if (id == "x")
            return x;
        if (id == "y")
            return y;
        if (id == "z")
            return z;
        if (id == "pi" || id == "PI")
            return M_PI;
        if (id == "sin")
            return std::sin(arg1());
        if (id == "cos")
            return std::cos(arg1());
        if (id == "tan")
            return std::tan(arg1());
        if (id == "abs")
            return std::abs(arg1());
        if (id == "sqrt")
            return std::sqrt(std::max(0., arg1()));
        if (id == "floor")
            return std::floor(arg1());
        if (id == "ceil")
            return std::ceil(arg1());
        if (id == "min") {
            const auto p = arg2();
            return std::min(p.first, p.second);
        }
        if (id == "max") {
            const auto p = arg2();
            return std::max(p.first, p.second);
        }
        throw std::runtime_error("unknown identifier: " + id);
    }
};

struct SampleGrid
{
    int cols = 0;
    int rows = 0;
    double origin_x = 0.; // mm
    double origin_y = 0.;
    double step = 0.; // mm
    std::vector<float> values; // row-major

    float at(int c, int r) const { return values[size_t(r) * size_t(cols) + size_t(c)]; }
};

static void lerp_edge(double x0, double y0, float v0, double x1, double y1, float v1, float isolevel, double &ox,
                      double &oy)
{
    if (std::abs(v1 - v0) < 1e-12f) {
        ox = 0.5 * (x0 + x1);
        oy = 0.5 * (y0 + y1);
        return;
    }
    const double t = double(isolevel - v0) / double(v1 - v0);
    ox = x0 + t * (x1 - x0);
    oy = y0 + t * (y1 - y0);
}

// Marching-squares segments in mm.
static Lines marching_squares_mm(const SampleGrid &grid, float isolevel)
{
    // edge id: 0 bottom, 1 right, 2 top, 3 left
    static const int edge_table[16][4] = {
        {-1, -1, -1, -1}, {0, 3, -1, -1}, {0, 1, -1, -1}, {1, 3, -1, -1}, {1, 2, -1, -1}, {0, 1, 2, 3},
        {0, 2, -1, -1},   {2, 3, -1, -1}, {2, 3, -1, -1}, {0, 2, -1, -1}, {0, 1, 2, 3}, {1, 2, -1, -1},
        {1, 3, -1, -1},   {0, 1, -1, -1}, {0, 3, -1, -1}, {-1, -1, -1, -1},
    };

    Lines out;
    out.reserve(size_t(grid.cols * grid.rows));
    for (int r = 0; r < grid.rows - 1; ++r) {
        for (int c = 0; c < grid.cols - 1; ++c) {
            const float v00 = grid.at(c, r);
            const float v10 = grid.at(c + 1, r);
            const float v11 = grid.at(c + 1, r + 1);
            const float v01 = grid.at(c, r + 1);
            int idx = 0;
            if (v00 >= isolevel)
                idx |= 1;
            if (v10 >= isolevel)
                idx |= 2;
            if (v11 >= isolevel)
                idx |= 4;
            if (v01 >= isolevel)
                idx |= 8;
            if (idx == 0 || idx == 15)
                continue;

            const double x0 = grid.origin_x + c * grid.step;
            const double y0 = grid.origin_y + r * grid.step;
            const double x1 = x0 + grid.step;
            const double y1 = y0 + grid.step;

            auto edge_pt = [&](int e, double &ox, double &oy) {
                switch (e) {
                case 0:
                    lerp_edge(x0, y0, v00, x1, y0, v10, isolevel, ox, oy);
                    break;
                case 1:
                    lerp_edge(x1, y0, v10, x1, y1, v11, isolevel, ox, oy);
                    break;
                case 2:
                    lerp_edge(x0, y1, v01, x1, y1, v11, isolevel, ox, oy);
                    break;
                default:
                    lerp_edge(x0, y0, v00, x0, y1, v01, isolevel, ox, oy);
                    break;
                }
            };

            const int *edges = edge_table[idx];
            // Ambiguous cases 5 and 10 produce two segments.
            for (int k = 0; k < 4 && edges[k] >= 0; k += 2) {
                if (edges[k + 1] < 0)
                    break;
                double ax, ay, bx, by;
                edge_pt(edges[k], ax, ay);
                edge_pt(edges[k + 1], bx, by);
                out.emplace_back(Point::new_scale(ax, ay), Point::new_scale(bx, by));
            }
        }
    }
    return out;
}

static Polylines lines_to_polylines(Lines &&lines)
{
    // Chain nearby segments into polylines.
    Polylines polylines;
    const double join_eps2 = sqr(scaled<double>(0.05));
    while (! lines.empty()) {
        Polyline pl;
        pl.points.push_back(lines.back().a);
        pl.points.push_back(lines.back().b);
        lines.pop_back();
        bool extended = true;
        while (extended) {
            extended = false;
            for (size_t i = 0; i < lines.size(); ++i) {
                const Point &tail = pl.points.back();
                const Point &head = pl.points.front();
                Line &ln = lines[i];
                if ((ln.a - tail).cast<double>().squaredNorm() < join_eps2) {
                    pl.points.push_back(ln.b);
                } else if ((ln.b - tail).cast<double>().squaredNorm() < join_eps2) {
                    pl.points.push_back(ln.a);
                } else if ((ln.a - head).cast<double>().squaredNorm() < join_eps2) {
                    pl.points.insert(pl.points.begin(), ln.b);
                } else if ((ln.b - head).cast<double>().squaredNorm() < join_eps2) {
                    pl.points.insert(pl.points.begin(), ln.a);
                } else
                    continue;
                lines[i] = lines.back();
                lines.pop_back();
                extended = true;
                break;
            }
        }
        if (pl.size() >= 2)
            polylines.emplace_back(std::move(pl));
    }
    return polylines;
}

static Polylines clip_to_expolygon(const Polylines &in, const ExPolygon &expoly)
{
    Polylines out;
    for (const Polyline &pl : in) {
        Polylines clipped = intersection_pl(pl, ExPolygons{expoly});
        append(out, std::move(clipped));
    }
    return out;
}

static void rotate_polylines_mm(Polylines &pls, double angle_rad, const Point &center)
{
    if (std::abs(angle_rad) < 1e-12)
        return;
    const double ca = std::cos(angle_rad);
    const double sa = std::sin(angle_rad);
    for (Polyline &pl : pls) {
        for (Point &pt : pl.points) {
            const double dx = unscaled<double>(pt.x() - center.x());
            const double dy = unscaled<double>(pt.y() - center.y());
            pt = Point::new_scale(unscaled<double>(center.x()) + ca * dx - sa * dy,
                                  unscaled<double>(center.y()) + sa * dx + ca * dy);
        }
    }
}

static std::vector<std::string> split_equations(const std::string &text)
{
    std::vector<std::string> lines;
    boost::split(lines, text, boost::is_any_of("\n\r"));
    std::vector<std::string> out;
    for (std::string &ln : lines) {
        boost::trim(ln);
        if (ln.empty() || ln[0] == '#')
            continue;
        out.emplace_back(std::move(ln));
    }
    return out;
}

static Polylines generate_equation_infill(const ExPolygon &expoly, const std::vector<Expr> &exprs, double z_mm,
                                          double tile_mm, float threshold, double spacing_mm, double angle_rad)
{
    BoundingBox bb = get_extents(expoly);
    bb.offset(scale_(spacing_mm));
    const double min_x = unscaled<double>(bb.min.x());
    const double min_y = unscaled<double>(bb.min.y());
    const double max_x = unscaled<double>(bb.max.x());
    const double max_y = unscaled<double>(bb.max.y());

    // Sample denser than extrusion spacing for smooth contours.
    const double step = std::clamp(std::min(spacing_mm, tile_mm) * 0.35, 0.15, 1.0);
    const double scale_xy = (tile_mm > EPSILON) ? (2. * M_PI / tile_mm) : 1.;

    SampleGrid grid;
    grid.step = step;
    grid.origin_x = min_x;
    grid.origin_y = min_y;
    grid.cols = std::max(2, int(std::ceil((max_x - min_x) / step)) + 1);
    grid.rows = std::max(2, int(std::ceil((max_y - min_y) / step)) + 1);
    grid.values.assign(size_t(grid.cols) * size_t(grid.rows), 0.f);

    // Combine equations with a soft-min so multiple surfaces contribute.
    for (int r = 0; r < grid.rows; ++r) {
        for (int c = 0; c < grid.cols; ++c) {
            const double x = min_x + c * step;
            const double y = min_y + r * step;
            const double sx = x * scale_xy;
            const double sy = y * scale_xy;
            const double sz = z_mm * scale_xy;
            double best = std::numeric_limits<double>::infinity();
            for (const Expr &e : exprs) {
                const double v = e.eval(sx, sy, sz);
                if (std::isfinite(v))
                    best = std::min(best, std::abs(v - threshold));
            }
            // Convert distance-to-iso into a signed field around 0 for marching squares.
            // Using -distance so the zero contour tracks the iso-level.
            grid.values[size_t(r) * size_t(grid.cols) + size_t(c)] =
                best < std::numeric_limits<double>::infinity() ? float(-best) : 0.f;
        }
    }

    // Zero contour of -|f-threshold| is exactly the iso-contour. Sample a thin band with a high isolevel
    // close to 0 from below would be empty; use isolevel just below 0 and keep all segments near zero.
    // Better approach: evaluate signed (f - threshold) and take isolevel 0 for each equation separately.
    Polylines all;
    for (const Expr &e : exprs) {
        SampleGrid g = grid;
        for (int r = 0; r < g.rows; ++r) {
            for (int c = 0; c < g.cols; ++c) {
                const double x = min_x + c * step;
                const double y = min_y + r * step;
                const double v = e.eval(x * scale_xy, y * scale_xy, z_mm * scale_xy);
                g.values[size_t(r) * size_t(g.cols) + size_t(c)] = float(std::isfinite(v) ? (v - threshold) : 0.);
            }
        }
        Lines segs = marching_squares_mm(g, 0.f);
        Polylines pls = lines_to_polylines(std::move(segs));
        append(all, std::move(pls));
    }

    const Point center = bb.center();
    rotate_polylines_mm(all, angle_rad, center);
    return clip_to_expolygon(all, expoly);
}

struct ImageTile
{
    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels; // grayscale 0-255
};

static bool load_png_grayscale(const std::string &path, ImageTile &out)
{
    std::ifstream ifs(path, std::ios::binary);
    if (! ifs)
        return false;
    std::string data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    png::ReadBuf rb{data.data(), data.size()};
    png::ImageGreyscale img;
    if (! png::decode_png(rb, img)) {
        // Try decoding as RGB via decode_png string API then average channels if available.
        std::vector<unsigned char> rgba;
        unsigned w = 0, h = 0;
        if (! png::decode_png(data, rgba, w, h) || w == 0 || h == 0)
            return false;
        out.width = int(w);
        out.height = int(h);
        out.pixels.resize(size_t(w) * size_t(h));
        const size_t channels = rgba.size() / (size_t(w) * size_t(h));
        for (size_t i = 0; i < out.pixels.size(); ++i) {
            if (channels >= 3) {
                const size_t o = i * channels;
                out.pixels[i] = uint8_t((int(rgba[o]) + int(rgba[o + 1]) + int(rgba[o + 2])) / 3);
            } else
                out.pixels[i] = rgba[i * channels];
        }
        return true;
    }
    out.width = int(img.cols);
    out.height = int(img.rows);
    out.pixels = std::move(img.buf);
    return out.width > 0 && out.height > 0;
}

static Polylines generate_image_infill(const ExPolygon &expoly, const ImageTile &img, double tile_mm, float threshold01,
                                       double angle_rad)
{
    if (img.width < 2 || img.height < 2 || tile_mm < EPSILON)
        return {};

    BoundingBox bb = get_extents(expoly);
    const double min_x = unscaled<double>(bb.min.x());
    const double min_y = unscaled<double>(bb.min.y());
    const double max_x = unscaled<double>(bb.max.x());
    const double max_y = unscaled<double>(bb.max.y());

    const double aspect = double(img.height) / double(img.width);
    const double tile_w = tile_mm;
    const double tile_h = tile_mm * aspect;
    const double step = std::clamp(tile_mm / std::max(img.width, img.height), 0.1, 1.0);

    SampleGrid grid;
    grid.step = step;
    grid.origin_x = min_x;
    grid.origin_y = min_y;
    grid.cols = std::max(2, int(std::ceil((max_x - min_x) / step)) + 1);
    grid.rows = std::max(2, int(std::ceil((max_y - min_y) / step)) + 1);
    grid.values.assign(size_t(grid.cols) * size_t(grid.rows), 0.f);

    auto sample = [&](double x, double y) -> float {
        double u = std::fmod(x, tile_w);
        double v = std::fmod(y, tile_h);
        if (u < 0)
            u += tile_w;
        if (v < 0)
            v += tile_h;
        const double px = u / tile_w * (img.width - 1);
        const double py = (1. - v / tile_h) * (img.height - 1); // image Y down
        const int x0 = std::clamp(int(std::floor(px)), 0, img.width - 1);
        const int y0 = std::clamp(int(std::floor(py)), 0, img.height - 1);
        return float(img.pixels[size_t(y0) * size_t(img.width) + size_t(x0)]) / 255.f;
    };

    for (int r = 0; r < grid.rows; ++r)
        for (int c = 0; c < grid.cols; ++c)
            grid.values[size_t(r) * size_t(grid.cols) + size_t(c)] =
                sample(min_x + c * step, min_y + r * step) - threshold01;

    Polylines pls = lines_to_polylines(marching_squares_mm(grid, 0.f));
    rotate_polylines_mm(pls, angle_rad, bb.center());
    return clip_to_expolygon(pls, expoly);
}

static Polylines generate_svg_infill(const ExPolygon &expoly, const std::string &path, double tile_mm, double angle_rad)
{
    NSVGimage_ptr image = nsvgParseFromFile(path, "px", 96.f);
    if (! image || image->width <= 0.f || image->height <= 0.f)
        return {};

    NSVGLineParams params(0.5);
    params.is_y_negative = false;
    // Convert SVG units to mm such that image width maps to tile_mm.
    const double svg_to_mm = tile_mm / double(image->width);
    params.scale = svg_to_mm / SCALING_FACTOR;

    Polygons polys = to_polygons(*image, params);
    if (polys.empty())
        return {};

    BoundingBox tile_bb = get_extents(polys);
    if (tile_bb.size().x() < 1 || tile_bb.size().y() < 1)
        return {};

    // Shift tile so its min corner is at origin in scaled coords.
    for (Polygon &p : polys)
        p.translate(-tile_bb.min);

    BoundingBox bb = get_extents(expoly);
    const coord_t tile_sx = tile_bb.size().x();
    const coord_t tile_sy = tile_bb.size().y();
    if (tile_sx < 1 || tile_sy < 1)
        return {};

    Polylines all;
    const coord_t x0 = bb.min.x() - (bb.min.x() % tile_sx + tile_sx) % tile_sx;
    const coord_t y0 = bb.min.y() - (bb.min.y() % tile_sy + tile_sy) % tile_sy;
    for (coord_t y = y0; y <= bb.max.y(); y += tile_sy) {
        for (coord_t x = x0; x <= bb.max.x(); x += tile_sx) {
            for (const Polygon &src : polys) {
                Polygon p = src;
                p.translate(Point(x, y));
                Polyline pl(p.points);
                if (! pl.points.empty() && pl.points.front() != pl.points.back())
                    pl.points.push_back(pl.points.front());
                append(all, clip_to_expolygon(Polylines{pl}, expoly));
            }
        }
    }
    rotate_polylines_mm(all, angle_rad, bb.center());
    return all;
}

struct MeshCacheEntry
{
    TriangleMesh mesh;
    BoundingBoxf3 bbox;
};

static std::mutex s_mesh_cache_mutex;
static std::unordered_map<std::string, MeshCacheEntry> s_mesh_cache;

static const MeshCacheEntry *get_cached_mesh(const std::string &path)
{
    std::lock_guard<std::mutex> lock(s_mesh_cache_mutex);
    auto it = s_mesh_cache.find(path);
    if (it != s_mesh_cache.end())
        return &it->second;
    try {
        MeshCacheEntry entry;
        entry.mesh = FileReader::load_mesh(path);
        if (entry.mesh.empty())
            return nullptr;
        entry.bbox = entry.mesh.bounding_box();
        it = s_mesh_cache.emplace(path, std::move(entry)).first;
        return &it->second;
    } catch (const std::exception &ex) {
        BOOST_LOG_TRIVIAL(error) << "Custom infill mesh load failed: " << ex.what();
        return nullptr;
    }
}

static Polylines generate_mesh_infill(const ExPolygon &expoly, const std::string &path, double tile_mm, double z_mm,
                                      double angle_rad)
{
    const MeshCacheEntry *entry = get_cached_mesh(path);
    if (! entry)
        return {};

    const BoundingBoxf3 &mb = entry->bbox;
    const double mesh_w = std::max(mb.size().x(), 1e-3);
    const double mesh_h = std::max(mb.size().y(), 1e-3);
    const double mesh_d = std::max(mb.size().z(), 1e-3);
    const double scale_xy = tile_mm / mesh_w;
    const double tile_y = mesh_h * scale_xy;
    const double tile_z = mesh_d * scale_xy;

    BoundingBox bb = get_extents(expoly);
    const double min_x = unscaled<double>(bb.min.x());
    const double min_y = unscaled<double>(bb.min.y());
    const double max_x = unscaled<double>(bb.max.x());
    const double max_y = unscaled<double>(bb.max.y());

    Polylines all;
    // Tile in XY; wrap Z periodically through the mesh height.
    const double z_local = std::fmod(z_mm, tile_z);
    const double z_in_mesh = mb.min.z() + ((z_local < 0. ? z_local + tile_z : z_local) / tile_z) * mesh_d;

    MeshSlicingParams slice_params;
    Polygons slice = slice_mesh(entry->mesh.its, float(z_in_mesh), slice_params);
    if (slice.empty())
        return {};

    // Scale mesh slice from mesh units to tile mm, then origin at (0,0).
    for (Polygon &p : slice) {
        for (Point &pt : p.points) {
            const double x = (unscaled<double>(pt.x()) - mb.min.x()) * scale_xy;
            const double y = (unscaled<double>(pt.y()) - mb.min.y()) * scale_xy;
            pt = Point::new_scale(x, y);
        }
    }

    BoundingBox tile_bb = get_extents(slice);
    const coord_t tile_sx = std::max(coord_t(1), tile_bb.size().x());
    const coord_t tile_sy = std::max(coord_t(1), coord_t(scale_(tile_y)));

    const coord_t x0 = scale_(min_x) - (coord_t(scale_(min_x)) % tile_sx + tile_sx) % tile_sx;
    const coord_t y0 = scale_(min_y) - (coord_t(scale_(min_y)) % tile_sy + tile_sy) % tile_sy;
    for (coord_t y = y0; y <= scale_(max_y); y += tile_sy) {
        for (coord_t x = x0; x <= scale_(max_x); x += tile_sx) {
            for (const Polygon &src : slice) {
                Polygon p = src;
                p.translate(Point(x, y) - tile_bb.min);
                Polyline pl(p.points);
                if (! pl.points.empty() && pl.points.front() != pl.points.back())
                    pl.points.push_back(pl.points.front());
                append(all, clip_to_expolygon(Polylines{pl}, expoly));
            }
        }
    }
    rotate_polylines_mm(all, angle_rad, bb.center());
    return all;
}

static std::string extension_of(const std::string &path)
{
    boost::filesystem::path p(path);
    std::string ext = p.extension().string();
    boost::algorithm::to_lower(ext);
    return ext;
}

} // namespace

void FillCustom::_fill_surface_single(const FillParams &params, unsigned int /*thickness_layers*/,
                                      const std::pair<float, Point> & /*direction*/, ExPolygon expolygon,
                                      Polylines &polylines_out)
{
    if (! print_region_config) {
        BOOST_LOG_TRIVIAL(warning) << "FillCustom: missing print_region_config";
        return;
    }

    const CustomInfillSource source = print_region_config->custom_infill_source.value;
    const double tile_mm = std::max(0.1, print_region_config->custom_infill_tile_size.value);
    const float threshold = float(print_region_config->custom_infill_threshold.value);
    const double angle_rad = Geometry::deg2rad(print_region_config->custom_infill_angle.value);
    const double spacing_mm = std::max(0.1, this->spacing / std::max(0.05, double(params.density)));

    Polylines generated;
    switch (source) {
    case CustomInfillSource::Equation: {
        const auto lines = split_equations(print_region_config->custom_infill_equations.value);
        std::vector<Expr> exprs;
        for (const std::string &ln : lines) {
            if (auto e = Expr::compile(ln))
                exprs.emplace_back(std::move(*e));
            else
                BOOST_LOG_TRIVIAL(warning) << "FillCustom: bad equation '" << ln << "'";
        }
        if (exprs.empty())
            return;
        generated = generate_equation_infill(expolygon, exprs, this->z, tile_mm, threshold, spacing_mm, angle_rad);
        break;
    }
    case CustomInfillSource::Image: {
        const std::string &path = print_region_config->custom_infill_file.value;
        if (path.empty())
            return;
        const std::string ext = extension_of(path);
        if (ext == ".svg") {
            generated = generate_svg_infill(expolygon, path, tile_mm, angle_rad);
        } else {
            ImageTile img;
            if (! load_png_grayscale(path, img)) {
                BOOST_LOG_TRIVIAL(warning) << "FillCustom: failed to load image " << path;
                return;
            }
            // Map threshold from typical -10..10 UI range: if outside 0..1 treat as 0..1 via abs clamp.
            float th = threshold;
            if (th < 0.f || th > 1.f)
                th = std::clamp((threshold + 1.f) * 0.5f, 0.f, 1.f);
            generated = generate_image_infill(expolygon, img, tile_mm, th, angle_rad);
        }
        break;
    }
    case CustomInfillSource::Mesh: {
        const std::string &path = print_region_config->custom_infill_file.value;
        if (path.empty())
            return;
        generated = generate_mesh_infill(expolygon, path, tile_mm, this->z, angle_rad);
        break;
    }
    }

    // Connect paths greedily to reduce travel.
    if (! generated.empty()) {
        if (params.dont_connect())
            append(polylines_out, chain_polylines(std::move(generated), params.start_near ? &*params.start_near : nullptr));
        else
            this->connect_infill(std::move(generated), expolygon, polylines_out, this->spacing, params);
    }
}

} // namespace Slic3r
