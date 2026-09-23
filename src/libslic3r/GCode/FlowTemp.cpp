///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
#include "FlowTemp.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <boost/log/trivial.hpp>

#include "../PrintConfig.hpp"

namespace Slic3r
{
namespace
{

struct ToolCtl
{
    bool active = false;
    double area = 0.;
    double max_vol = 0.;
    double min_feed = 0.; // mm/min
    double temp_low = 0.;
    double temp_high = 0.;
    double first_layer_temp = 0.;
    double sec_heat = 6.;
    double sec_cool = 4.;
    double window = 1.;
    double smoothed = 0.;
    int last_cmd = std::numeric_limits<int>::min();
};

struct ExSeg
{
    double t_end = 0.;
    double dt = 0.;
    double flow = 0.;
};

struct Motion
{
    size_t line = 0;
    int tool = 0;
    double z = 0.;
    double f = 0.;
    double dt = 0.;
    double flow = 0.;
    double t_end = 0.;
    bool extruding = false;
};

double opt_float(const ConfigOptionFloats &opt, size_t idx, double fallback)
{
    if (opt.values.empty())
        return fallback;
    return opt.values[std::min(idx, opt.values.size() - 1)];
}

int opt_int(const ConfigOptionInts &opt, size_t idx, int fallback)
{
    if (opt.values.empty())
        return fallback;
    return opt.values[std::min(idx, opt.values.size() - 1)];
}

bool opt_bool(const ConfigOptionBools &opt, size_t idx)
{
    if (opt.values.empty())
        return false;
    return opt.values[std::min(idx, opt.values.size() - 1)];
}

struct Words
{
    int g = -1;
    int m = -1;
    int t = -1;
    bool has_x = false, has_y = false, has_z = false, has_e = false, has_f = false;
    bool has_i = false, has_j = false;
    double x = 0, y = 0, z = 0, e = 0, f = 0, i = 0, j = 0;
};

bool parse_words(const std::string &line, Words &w)
{
    const char *s = line.c_str();
    const char *end = s + line.size();
    bool any = false;
    while (s < end)
    {
        if (*s == ';')
            break;
        if (std::isspace(static_cast<unsigned char>(*s)))
        {
            ++s;
            continue;
        }
        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(*s)));
        if (c < 'A' || c > 'Z')
        {
            ++s;
            continue;
        }
        ++s;
        char *num_end = nullptr;
        const double v = std::strtod(s, &num_end);
        if (num_end == s)
            continue;
        s = num_end;
        any = true;
        switch (c)
        {
        case 'G':
            w.g = static_cast<int>(std::lround(v));
            break;
        case 'M':
            w.m = static_cast<int>(std::lround(v));
            break;
        case 'T':
            w.t = static_cast<int>(std::lround(v));
            break;
        case 'X':
            w.has_x = true;
            w.x = v;
            break;
        case 'Y':
            w.has_y = true;
            w.y = v;
            break;
        case 'Z':
            w.has_z = true;
            w.z = v;
            break;
        case 'E':
            w.has_e = true;
            w.e = v;
            break;
        case 'F':
            w.has_f = true;
            w.f = v;
            break;
        case 'I':
            w.has_i = true;
            w.i = v;
            break;
        case 'J':
            w.has_j = true;
            w.j = v;
            break;
        default:
            break;
        }
    }
    return any;
}

double arc_length(double x0, double y0, double x1, double y1, double i, double j, bool ccw, double dz)
{
    const double r = std::hypot(i, j);
    if (r < 1e-9)
        return std::hypot(std::hypot(x1 - x0, y1 - y0), dz);
    const double a0 = std::atan2(-j, -i);
    const double a1 = std::atan2(y1 - (y0 + j), x1 - (x0 + i));
    double da = a1 - a0;
    if (ccw)
    {
        if (da <= 1e-9)
            da += 2. * M_PI;
    }
    else if (da >= -1e-9)
        da -= 2. * M_PI;
    return std::hypot(r * std::abs(da), dz);
}

void set_feedrate_word(std::string &line, int feed)
{
    const std::string repl = "F" + std::to_string(feed);
    const size_t comment = line.find(';');
    const size_t limit = comment == std::string::npos ? line.size() : comment;
    size_t fpos = std::string::npos;
    for (size_t i = 0; i < limit; ++i)
    {
        if ((line[i] == 'F' || line[i] == 'f') &&
            (i == 0 || !std::isalnum(static_cast<unsigned char>(line[i - 1]))))
        {
            fpos = i;
            break;
        }
    }
    if (fpos != std::string::npos)
    {
        size_t k = fpos + 1;
        while (k < limit && (std::isdigit(static_cast<unsigned char>(line[k])) || line[k] == '.' || line[k] == '+' ||
                             line[k] == '-'))
            ++k;
        line.replace(fpos, k - fpos, repl);
        return;
    }
    size_t ins = limit;
    while (ins > 0 && (line[ins - 1] == '\n' || line[ins - 1] == '\r' || line[ins - 1] == ' '))
        --ins;
    line.insert(ins, " " + repl);
}

} // namespace

std::string apply_flow_temp(const std::string &gcode, const PrintConfig &config)
{
    const size_t ntools = std::max<size_t>(1, config.nozzle_diameter.values.size());
    std::vector<ToolCtl> tools(ntools);
    bool any = false;
    const double first_h = config.first_layer_height.get_abs_value(
        config.nozzle_diameter.values.empty() ? 0.4 : config.nozzle_diameter.values.front());
    for (size_t i = 0; i < ntools; ++i)
    {
        ToolCtl &t = tools[i];
        if (!opt_bool(config.flow_temp_enabled, i))
            continue;
        t.temp_low = opt_int(config.flow_temp_low, i, 0);
        t.temp_high = opt_int(config.flow_temp_high, i, 0);
        t.max_vol = opt_float(config.filament_max_volumetric_speed, i, 0.);
        if (t.max_vol <= 0.)
            t.max_vol = opt_float(config.filament_max_volumetric_flow, i, 0.);
        const double dia = opt_float(config.filament_diameter, i, 1.75);
        // A max volumetric flow of 0 means "no cap" in this slicer. The tool still runs;
        // the top of the temperature range is filled in from the fastest extrusion later.
        if (!(t.temp_high > t.temp_low) || dia <= 0.)
            continue;
        t.active = true;
        any = true;
        t.area = M_PI * (dia * 0.5) * (dia * 0.5);
        t.min_feed = std::max(0., opt_float(config.min_print_speed, i, 0.)) * 60.;
        t.first_layer_temp = opt_int(config.first_layer_temperature, i, static_cast<int>(t.temp_low));
        t.sec_heat = std::max(0.1, opt_float(config.flow_temp_sec_per_c_heating, i, 6.));
        t.sec_cool = std::max(0.1, opt_float(config.flow_temp_sec_per_c_cooling, i, 4.));
        t.window = std::max(1., (t.temp_high - t.temp_low) * std::max(t.sec_heat, t.sec_cool));
        t.smoothed = t.temp_low;
    }
    if (!any)
        return {};

    std::vector<std::string> lines;
    lines.reserve(gcode.size() / 32 + 1);
    for (size_t i = 0; i < gcode.size();)
    {
        const size_t j = gcode.find('\n', i);
        if (j == std::string::npos)
        {
            lines.emplace_back(gcode.substr(i));
            break;
        }
        lines.emplace_back(gcode.substr(i, j - i + 1));
        i = j + 1;
    }

    const bool volumetric = config.use_volumetric_e.value;
    bool rel_e = config.use_relative_e_distances.value;
    bool rel_xyz = false;
    int tool = 0;
    double x = 0, y = 0, z = 0, e = 0, f = 0;
    std::vector<Motion> motions;
    std::vector<int> temp_line_tool(lines.size(), std::numeric_limits<int>::min());
    size_t first_ex = std::string::npos;
    size_t last_ex = std::string::npos;

    for (size_t li = 0; li < lines.size(); ++li)
    {
        Words w;
        if (!parse_words(lines[li], w))
            continue;
        if (w.t >= 0 && w.g < 0 && w.m < 0)
            tool = w.t;
        if (w.m == 82)
            rel_e = false;
        else if (w.m == 83)
            rel_e = true;
        else if (w.m == 104 || w.m == 109)
            temp_line_tool[li] = w.t >= 0 ? w.t : tool;
        if (w.g == 90)
            rel_xyz = false;
        else if (w.g == 91)
            rel_xyz = true;
        else if (w.g == 92)
        {
            if (w.has_x)
                x = w.x;
            if (w.has_y)
                y = w.y;
            if (w.has_z)
                z = w.z;
            if (w.has_e)
                e = w.e;
        }
        else if (w.g == 0 || w.g == 1 || w.g == 2 || w.g == 3)
        {
            if (w.has_f)
                f = w.f;
            const double x1 = w.has_x ? (rel_xyz ? x + w.x : w.x) : x;
            const double y1 = w.has_y ? (rel_xyz ? y + w.y : w.y) : y;
            const double z1 = w.has_z ? (rel_xyz ? z + w.z : w.z) : z;
            double e1 = e;
            double de = 0.;
            if (w.has_e)
            {
                if (rel_e)
                {
                    de = w.e;
                    e1 = e + w.e;
                }
                else
                {
                    de = w.e - e;
                    e1 = w.e;
                }
            }
            double dist = 0.;
            if ((w.g == 2 || w.g == 3) && w.has_i && w.has_j)
                dist = arc_length(x, y, x1, y1, w.i, w.j, w.g == 3, z1 - z);
            else
                dist = std::hypot(std::hypot(x1 - x, y1 - y), z1 - z);
            const double dt = (f > 1e-6 && dist > 0.) ? dist / (f / 60.) : 0.;
            const bool extruding = de > 1e-8 && dt > 0.;
            double flow = 0.;
            if (extruding)
                flow = volumetric ? (de / dt) : (de * (tool >= 0 && static_cast<size_t>(tool) < tools.size() &&
                                                               tools[static_cast<size_t>(tool)].active
                                                           ? tools[static_cast<size_t>(tool)].area
                                                           : 0.) /
                                                      dt);
            Motion mv;
            mv.line = li;
            mv.tool = tool;
            mv.z = z1;
            mv.f = f;
            mv.dt = dt;
            mv.flow = flow;
            mv.extruding = extruding && flow > 0.;
            motions.push_back(mv);
            if (mv.extruding)
            {
                if (first_ex == std::string::npos)
                    first_ex = li;
                last_ex = li;
            }
            x = x1;
            y = y1;
            z = z1;
            e = e1;
        }
    }

    if (first_ex == std::string::npos)
        return {};

    std::vector<Motion> region;
    region.reserve(motions.size());
    double clock = 0.;
    for (Motion &mv : motions)
    {
        if (mv.line < first_ex || mv.line > last_ex)
            continue;
        clock += mv.dt;
        mv.t_end = clock;
        region.push_back(mv);
    }
    if (region.empty())
        return {};

    std::vector<std::vector<ExSeg>> segs(ntools);
    for (const Motion &mv : region)
    {
        if (!mv.extruding || mv.tool < 0 || static_cast<size_t>(mv.tool) >= ntools || !tools[mv.tool].active)
            continue;
        segs[mv.tool].push_back(ExSeg{mv.t_end, mv.dt, mv.flow});
    }

    for (size_t ti = 0; ti < ntools; ++ti)
    {
        ToolCtl &ctl = tools[ti];
        if (!ctl.active || ctl.max_vol > 0.)
            continue;
        // No volumetric cap is configured. Scale the high temperature to a high flow that this
        // print actually spends time at, so one purge line does not pin every other move at the
        // low temperature.
        std::vector<double> flows;
        flows.reserve(segs[ti].size());
        for (const ExSeg &seg : segs[ti])
        {
            if (seg.dt >= 0.01 && seg.flow > 0.)
                flows.push_back(seg.flow);
        }
        if (flows.empty())
        {
            ctl.active = false;
            continue;
        }
        const size_t idx = static_cast<size_t>(0.95 * static_cast<double>(flows.size() - 1));
        std::nth_element(flows.begin(), flows.begin() + static_cast<std::ptrdiff_t>(idx), flows.end());
        ctl.max_vol = std::max(flows[idx], 1e-6);
        BOOST_LOG_TRIVIAL(info) << "Flow temperature control for extruder " << ti
                                << " has no max volumetric flow; scaling the high temperature to " << ctl.max_vol
                                << " mm^3/s";
    }
    any = false;
    for (const ToolCtl &ctl : tools)
        any = any || ctl.active;
    if (!any)
        return {};

    std::vector<size_t> left(ntools, 0), right(ntools, 0);
    std::vector<double> sum_ft(ntools, 0.), sum_t(ntools, 0.);
    std::vector<int> new_feed(lines.size(), -1);
    std::vector<std::string> prepend(lines.size());
    const bool multi = ntools > 1;

    for (const Motion &mv : region)
    {
        const double t0 = mv.t_end - mv.dt;
        // This tool's own first-layer moves stay at the first layer temperature. Other tools keep
        // ramping so a parked hotend can preheat for the flow it will actually print.
        const bool move_on_first_layer = std::abs(mv.z - first_h) <= 0.05;
        for (size_t ti = 0; ti < ntools; ++ti)
        {
            ToolCtl &ctl = tools[ti];
            if (!ctl.active)
                continue;
            const std::vector<ExSeg> &ex = segs[ti];
            while (right[ti] < ex.size() && ex[right[ti]].t_end <= t0 + ctl.window)
            {
                sum_ft[ti] += ex[right[ti]].flow * ex[right[ti]].dt;
                sum_t[ti] += ex[right[ti]].dt;
                ++right[ti];
            }
            while (left[ti] < right[ti] && ex[left[ti]].t_end <= t0)
            {
                sum_ft[ti] -= ex[left[ti]].flow * ex[left[ti]].dt;
                sum_t[ti] -= ex[left[ti]].dt;
                ++left[ti];
            }
            double target = ctl.temp_low;
            double max_allowed = ctl.max_vol;
            if (move_on_first_layer && mv.tool == static_cast<int>(ti))
            {
                ctl.smoothed = ctl.first_layer_temp;
            }
            else
            {
                const double look = sum_t[ti] > 1e-9 ? sum_ft[ti] / sum_t[ti] : 0.;
                const double capped = std::min(look, ctl.max_vol);
                target = ctl.temp_low + (ctl.temp_high - ctl.temp_low) * (capped / ctl.max_vol);
                const double elapsed = mv.dt;
                const double sec = target > ctl.smoothed ? ctl.sec_heat : ctl.sec_cool;
                const double max_change = elapsed / sec;
                const double diff = target - ctl.smoothed;
                if (std::abs(diff) > max_change)
                    ctl.smoothed += std::copysign(max_change, diff);
                else
                    ctl.smoothed = target;
                ctl.smoothed = std::clamp(ctl.smoothed, ctl.temp_low, ctl.temp_high);
                const double ratio = (ctl.smoothed - ctl.temp_low) / (ctl.temp_high - ctl.temp_low);
                max_allowed = ctl.max_vol * std::clamp(ratio, 0., 1.);
            }
            const int cmd = static_cast<int>(std::lround(ctl.smoothed));
            // A parked tool with nothing coming up in its lookahead stays untouched, so a
            // start-gcode preheat is not immediately overwritten with the low temperature.
            const bool engaged = mv.tool == static_cast<int>(ti) || sum_t[ti] > 1e-9 ||
                                 ctl.last_cmd != std::numeric_limits<int>::min();
            if (engaged && cmd != ctl.last_cmd)
            {
                ctl.last_cmd = cmd;
                if (multi)
                    prepend[mv.line] += "M104 T" + std::to_string(ti) + " S" + std::to_string(cmd) + "\n";
                else
                    prepend[mv.line] += "M104 S" + std::to_string(cmd) + "\n";
            }
            if (mv.extruding && mv.tool == static_cast<int>(ti) && mv.flow > max_allowed && mv.f > 0.)
            {
                double feed = (max_allowed / mv.flow) * mv.f;
                feed = std::max(feed, ctl.min_feed);
                if (feed < mv.f - 0.5)
                    new_feed[mv.line] = static_cast<int>(std::lround(feed));
            }
        }
    }

    std::string out;
    out.reserve(gcode.size() + gcode.size() / 20);
    for (size_t li = 0; li < lines.size(); ++li)
    {
        if (!prepend[li].empty())
            out += prepend[li];
        const int temp_tool = temp_line_tool[li];
        if (li >= first_ex && li <= last_ex && temp_tool != std::numeric_limits<int>::min() && temp_tool >= 0 &&
            static_cast<size_t>(temp_tool) < ntools && tools[temp_tool].active)
            continue;
        std::string line = lines[li];
        if (new_feed[li] >= 0)
            set_feedrate_word(line, new_feed[li]);
        out += line;
    }
    BOOST_LOG_TRIVIAL(info) << "Flow temperature control adjusted " << region.size() << " moves";
    return out;
}

} // namespace Slic3r
