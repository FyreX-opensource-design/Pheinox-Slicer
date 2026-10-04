///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#include "PaintBrushLibrary.hpp"

#include "libslic3r/miniz_extension.hpp"

#include <gdk-pixbuf/gdk-pixbuf.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>

namespace Slic3r::GUI
{
namespace
{

namespace fs = std::filesystem;

uint32_t read_be32(const uint8_t *p)
{
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

float eval_curve(const std::vector<std::pair<float, float>> &curve, float x)
{
    if (curve.empty())
        return 1.f;
    if (x <= curve.front().first)
        return curve.front().second;
    for (size_t i = 1; i < curve.size(); ++i)
    {
        if (x <= curve[i].first)
        {
            const float span = curve[i].first - curve[i - 1].first;
            const float t = span > 1e-6f ? (x - curve[i - 1].first) / span : 0.f;
            return curve[i - 1].second + t * (curve[i].second - curve[i - 1].second);
        }
    }
    return curve.back().second;
}

bool read_file(const std::string &path, std::vector<uint8_t> &out)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
        return false;
    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    if (size < 0)
        return false;
    in.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(size));
    if (!out.empty())
        in.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(out.size()));
    return bool(in);
}

bool read_zip_entry(const std::string &bundle, const std::string &inner, std::vector<uint8_t> &out)
{
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!open_zip_reader(&zip, bundle))
        return false;
    size_t size = 0;
    void *data = mz_zip_reader_extract_file_to_heap(&zip, inner.c_str(), &size, 0);
    close_zip_reader(&zip);
    if (data == nullptr)
        return false;
    out.assign(static_cast<uint8_t *>(data), static_cast<uint8_t *>(data) + size);
    mz_free(data);
    return true;
}

void downsample_mask(std::vector<uint8_t> &mask, int &width, int &height, int limit)
{
    if (width <= limit && height <= limit)
        return;
    const int nw = std::max(1, width * limit / std::max(width, height));
    const int nh = std::max(1, height * limit / std::max(width, height));
    std::vector<uint8_t> next(static_cast<size_t>(nw * nh));
    for (int y = 0; y < nh; ++y)
    {
        const int y0 = y * height / nh;
        const int y1 = std::max(y0 + 1, (y + 1) * height / nh);
        for (int x = 0; x < nw; ++x)
        {
            const int x0 = x * width / nw;
            const int x1 = std::max(x0 + 1, (x + 1) * width / nw);
            unsigned sum = 0;
            unsigned count = 0;
            for (int yy = y0; yy < y1; ++yy)
                for (int xx = x0; xx < x1; ++xx)
                {
                    sum += mask[static_cast<size_t>(yy * width + xx)];
                    ++count;
                }
            next[static_cast<size_t>(y * nw + x)] = static_cast<uint8_t>(count ? sum / count : 0);
        }
    }
    mask.swap(next);
    width = nw;
    height = nh;
}

bool mask_from_gbr(const uint8_t *data, size_t size, PaintBrushTip &out)
{
    if (size < 28)
        return false;
    const uint32_t header_size = read_be32(data);
    const uint32_t version = read_be32(data + 4);
    const uint32_t width = read_be32(data + 8);
    const uint32_t height = read_be32(data + 12);
    const uint32_t bytes = read_be32(data + 16);
    if (version < 1 || version > 2 || header_size < 28 || header_size >= size)
        return false;
    if (std::memcmp(data + 20, "GIMP", 4) != 0)
        return false;
    if (width == 0 || height == 0 || width > 4096 || height > 4096)
        return false;
    if (bytes != 1 && bytes != 4)
        return false;
    const size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (header_size + pixels * bytes > size)
        return false;

    const uint8_t *src = data + header_size;
    out.width = static_cast<int>(width);
    out.height = static_cast<int>(height);
    out.mask.assign(pixels, 0);
    if (bytes == 1)
    {
        std::copy(src, src + pixels, out.mask.begin());
    }
    else
    {
        bool alpha_varies = false;
        for (size_t i = 0; i < pixels; ++i)
        {
            if (src[i * 4 + 3] != 255)
            {
                alpha_varies = true;
                break;
            }
        }
        for (size_t i = 0; i < pixels; ++i)
        {
            const uint8_t *px = src + i * 4;
            if (alpha_varies)
                out.mask[i] = px[3];
            else
                out.mask[i] = static_cast<uint8_t>((unsigned(px[0]) * 30 + unsigned(px[1]) * 59 + unsigned(px[2]) * 11) /
                                                   100);
        }
    }
    downsample_mask(out.mask, out.width, out.height, 256);
    return true;
}

bool mask_from_gih(const uint8_t *data, size_t size, PaintBrushTip &out)
{
    // An image hose is a short text header followed by one or more GIMP brushes. The first cell is enough.
    const uint8_t *end = data + size;
    for (size_t i = 20; i + 8 < size; ++i)
    {
        if (std::memcmp(data + i, "GIMP", 4) != 0)
            continue;
        const uint8_t *start = data + i - 20;
        if (start < data)
            continue;
        if (mask_from_gbr(start, static_cast<size_t>(end - start), out))
            return true;
    }
    return false;
}

bool mask_from_pixbuf(GdkPixbuf *pix, PaintBrushTip &out)
{
    if (pix == nullptr || gdk_pixbuf_get_bits_per_sample(pix) != 8)
        return false;
    const int width = gdk_pixbuf_get_width(pix);
    const int height = gdk_pixbuf_get_height(pix);
    const int channels = gdk_pixbuf_get_n_channels(pix);
    const int stride = gdk_pixbuf_get_rowstride(pix);
    if (width <= 0 || height <= 0 || width > 4096 || height > 4096 || (channels != 3 && channels != 4))
        return false;
    const uint8_t *src = gdk_pixbuf_get_pixels(pix);
    out.width = width;
    out.height = height;
    out.mask.assign(static_cast<size_t>(width * height), 0);
    const bool has_alpha = channels == 4 && gdk_pixbuf_get_has_alpha(pix);
    // Krita tips are usually black ink on white with a flat alpha. A varying alpha is the mask
    // itself (GIMP and some Krita pngs). Otherwise black is opaque and white is empty.
    bool alpha_varies = false;
    if (has_alpha)
    {
        const uint8_t first = src[3];
        for (int y = 0; y < height && !alpha_varies; ++y)
        {
            const uint8_t *row = src + y * stride;
            for (int x = 0; x < width; ++x)
            {
                if (row[x * channels + 3] != first)
                {
                    alpha_varies = true;
                    break;
                }
            }
        }
    }
    for (int y = 0; y < height; ++y)
    {
        const uint8_t *row = src + y * stride;
        for (int x = 0; x < width; ++x)
        {
            const uint8_t *px = row + x * channels;
            uint8_t value;
            if (alpha_varies)
                value = px[3];
            else
            {
                const unsigned luma = (unsigned(px[0]) * 30 + unsigned(px[1]) * 59 + unsigned(px[2]) * 11) / 100;
                value = static_cast<uint8_t>(255 - std::min(255u, luma));
            }
            out.mask[static_cast<size_t>(y * width + x)] = value;
        }
    }
    downsample_mask(out.mask, out.width, out.height, 256);
    return std::any_of(out.mask.begin(), out.mask.end(), [](uint8_t v) { return v != 0; });
}

bool mask_from_png(const uint8_t *data, size_t size, PaintBrushTip &out)
{
    GdkPixbufLoader *loader = gdk_pixbuf_loader_new();
    if (loader == nullptr)
        return false;
    GError *error = nullptr;
    const bool wrote = gdk_pixbuf_loader_write(loader, data, size, &error);
    if (error != nullptr)
        g_error_free(error);
    error = nullptr;
    gdk_pixbuf_loader_close(loader, &error);
    if (error != nullptr)
        g_error_free(error);
    bool ok = false;
    if (wrote)
        ok = mask_from_pixbuf(gdk_pixbuf_loader_get_pixbuf(loader), out);
    g_object_unref(loader);
    return ok;
}

bool mask_from_png_file(const std::string &path, PaintBrushTip &out)
{
    GError *error = nullptr;
    GdkPixbuf *pix = gdk_pixbuf_new_from_file(path.c_str(), &error);
    if (error != nullptr)
        g_error_free(error);
    if (pix == nullptr)
        return false;
    const bool ok = mask_from_pixbuf(pix, out);
    g_object_unref(pix);
    return ok;
}

void make_auto_circle(PaintBrushTip &out, float hfade, float vfade, float ratio)
{
    constexpr int kSize = 64;
    out.width = kSize;
    out.height = kSize;
    out.mask.assign(kSize * kSize, 0);
    const float fade = std::clamp(std::max(hfade, vfade), 0.05f, 1.f);
    const float rx = std::max(ratio, 0.05f);
    for (int y = 0; y < kSize; ++y)
    {
        const float ny = (static_cast<float>(y) + 0.5f) / kSize * 2.f - 1.f;
        for (int x = 0; x < kSize; ++x)
        {
            const float nx = (static_cast<float>(x) + 0.5f) / kSize * 2.f - 1.f;
            const float r = std::sqrt((nx / rx) * (nx / rx) + ny * ny);
            if (r >= 1.f)
                continue;
            const float coverage = std::pow(1.f - r, fade);
            out.mask[static_cast<size_t>(y * kSize + x)] =
                static_cast<uint8_t>(std::clamp(coverage, 0.f, 1.f) * 255.f);
        }
    }
}

std::string param_cdata(const std::string &xml, const std::string &name)
{
    const std::string key = "name=\"" + name + "\"";
    const size_t pos = xml.find(key);
    if (pos == std::string::npos)
        return {};
    const size_t next_param = xml.find("<param ", pos + key.size());
    const size_t cdata = xml.find("<![CDATA[", pos);
    if (cdata == std::string::npos || (next_param != std::string::npos && cdata > next_param))
        return {};
    const size_t begin = cdata + 9;
    const size_t end = xml.find("]]>", begin);
    if (end == std::string::npos)
        return {};
    return xml.substr(begin, end - begin);
}

std::string xml_attr(const std::string &tag, const std::string &name)
{
    const std::string key = name + "=\"";
    const size_t pos = tag.find(key);
    if (pos == std::string::npos)
        return {};
    const size_t begin = pos + key.size();
    const size_t end = tag.find('"', begin);
    if (end == std::string::npos)
        return {};
    return tag.substr(begin, end - begin);
}

std::vector<std::pair<float, float>> parse_pressure_curve(const std::string &sensor_xml)
{
    std::string region = sensor_xml;
    const size_t pressure = sensor_xml.find("id=\"pressure\"");
    if (pressure == std::string::npos)
    {
        if (sensor_xml.find("sensorslist") != std::string::npos)
            return {};
    }
    else
    {
        region = sensor_xml.substr(pressure);
        const size_t next = region.find("<ChildSensor", 8);
        if (next != std::string::npos)
            region.resize(next);
    }

    const size_t curve = region.find("<curve>");
    if (curve == std::string::npos)
        return {};
    const size_t begin = curve + 7;
    const size_t end = region.find("</curve>", begin);
    if (end == std::string::npos)
        return {};

    std::vector<std::pair<float, float>> points;
    const std::string body = region.substr(begin, end - begin);
    size_t at = 0;
    while (at < body.size())
    {
        const size_t comma = body.find(',', at);
        const size_t semi = body.find(';', at);
        if (comma == std::string::npos)
            break;
        const size_t stop = semi == std::string::npos ? body.size() : semi;
        if (comma >= stop)
            break;
        try
        {
            const float x = std::stof(body.substr(at, comma - at));
            const float y = std::stof(body.substr(comma + 1, stop - comma - 1));
            points.emplace_back(x, y);
        }
        catch (...)
        {
        }
        if (semi == std::string::npos)
            break;
        at = semi + 1;
    }
    return points;
}

std::string preset_xml_from_png(const uint8_t *data, size_t size)
{
    if (size < 8 || std::memcmp(data, "\x89PNG\r\n\x1a\n", 8) != 0)
        return {};
    size_t i = 8;
    while (i + 8 <= size)
    {
        const uint32_t length = read_be32(data + i);
        if (i + 12 + length > size)
            break;
        const uint8_t *type = data + i + 4;
        const uint8_t *chunk = data + i + 8;
        if (std::memcmp(type, "zTXt", 4) == 0 && length > 8)
        {
            const uint8_t *nul = static_cast<const uint8_t *>(std::memchr(chunk, 0, length));
            if (nul != nullptr && static_cast<size_t>(nul - chunk) == 6 && std::memcmp(chunk, "preset", 6) == 0 &&
                nul + 2 < chunk + length)
            {
                const uint8_t *z = nul + 2;
                const size_t zlen = static_cast<size_t>((chunk + length) - z);
                uLongf dest = static_cast<uLongf>(zlen * 8 + 64);
                std::string xml;
                for (int attempt = 0; attempt < 6; ++attempt)
                {
                    xml.resize(dest);
                    mz_ulong got = dest;
                    const int rc = mz_uncompress(reinterpret_cast<unsigned char *>(xml.data()), &got, z,
                                                 static_cast<mz_ulong>(zlen));
                    if (rc == Z_OK)
                    {
                        xml.resize(got);
                        return xml;
                    }
                    if (rc != Z_BUF_ERROR)
                        break;
                    dest *= 2;
                }
            }
        }
        if (std::memcmp(type, "IEND", 4) == 0)
            break;
        i += 12 + length;
    }
    return {};
}

std::string preset_xml_from_bytes(const std::vector<uint8_t> &bytes)
{
    if (bytes.size() >= 8 && std::memcmp(bytes.data(), "\x89PNG\r\n\x1a\n", 8) == 0)
        return preset_xml_from_png(bytes.data(), bytes.size());
    return {};
}

struct BrushFile
{
    std::string bundle;
    std::string path;
};

std::map<std::string, BrushFile> &brush_files()
{
    static std::map<std::string, BrushFile> files;
    return files;
}

std::string filename_of(const std::string &path)
{
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::string stem_of(const std::string &filename)
{
    const size_t dot = filename.find_last_of('.');
    return dot == std::string::npos ? filename : filename.substr(0, dot);
}

std::string pretty_name(const std::string &filename)
{
    std::string name = stem_of(filename);
    for (char &ch : name)
        if (ch == '_')
            ch = ' ';
    // Krita's default packs prefix names with a sort key such as "b)_".
    const size_t marker = name.find(") ");
    if (marker != std::string::npos && marker <= 3)
        name.erase(0, marker + 2);
    return name;
}

bool ends_with(const std::string &value, const char *suffix)
{
    const size_t n = std::strlen(suffix);
    return value.size() >= n && value.compare(value.size() - n, n, suffix) == 0;
}

void remember_brush_file(const std::string &filename, const BrushFile &loc)
{
    if (filename.empty())
        return;
    brush_files().emplace(filename, loc);
}

bool load_mask_at(const BrushFile &loc, PaintBrushTip &out)
{
    const std::string &name = loc.bundle.empty() ? filename_of(loc.path) : filename_of(loc.path);
    std::vector<uint8_t> bytes;
    const bool in_zip = !loc.bundle.empty();
    if (in_zip)
    {
        if (!read_zip_entry(loc.bundle, loc.path, bytes))
            return false;
    }
    if (ends_with(name, ".png"))
        return in_zip ? mask_from_png(bytes.data(), bytes.size(), out) : mask_from_png_file(loc.path, out);
    if (!in_zip && !read_file(loc.path, bytes))
        return false;
    if (ends_with(name, ".gbr"))
        return mask_from_gbr(bytes.data(), bytes.size(), out);
    if (ends_with(name, ".gih"))
        return mask_from_gih(bytes.data(), bytes.size(), out);
    return false;
}

void apply_preset_xml(const std::string &xml, PaintBrushTip &out)
{
    const std::string opacity_use = param_cdata(xml, "OpacityUseCurve");
    const std::string flow_use = param_cdata(xml, "FlowUseCurve");
    const std::string size_use = param_cdata(xml, "SizeUseCurve");
    out.opacity_curve = parse_pressure_curve(param_cdata(xml, "OpacitySensor"));
    out.flow_curve = parse_pressure_curve(param_cdata(xml, "FlowSensor"));
    out.size_curve = parse_pressure_curve(param_cdata(xml, "SizeSensor"));
    out.opacity_uses_pressure = opacity_use == "true" && !out.opacity_curve.empty();
    out.flow_uses_pressure = flow_use == "true" && !out.flow_curve.empty();
    out.size_uses_pressure = size_use == "true" && !out.size_curve.empty();
    try
    {
        const std::string opacity = param_cdata(xml, "OpacityValue");
        const std::string flow = param_cdata(xml, "FlowValue");
        if (!opacity.empty())
            out.opacity_value = std::clamp(std::stof(opacity), 0.f, 1.f);
        if (!flow.empty())
            out.flow_value = std::clamp(std::stof(flow), 0.f, 1.f);
    }
    catch (...)
    {
    }

    const std::string definition = param_cdata(xml, "brush_definition");
    const std::string spacing = xml_attr(definition, "spacing");
    if (!spacing.empty())
    {
        try
        {
            out.spacing = std::clamp(std::stof(spacing), 0.05f, 1.f);
        }
        catch (...)
        {
        }
    }
}

bool load_kpp_bytes(const std::vector<uint8_t> &bytes, PaintBrushTip &out)
{
    const std::string xml = preset_xml_from_bytes(bytes);
    if (xml.empty())
        return false;
    apply_preset_xml(xml, out);
    const std::string definition = param_cdata(xml, "brush_definition");
    const std::string type = xml_attr(definition, "type");
    if (type == "auto_brush")
    {
        float hfade = 1.f;
        float vfade = 1.f;
        float ratio = 1.f;
        try
        {
            const std::string h = xml_attr(definition, "hfade");
            const std::string v = xml_attr(definition, "vfade");
            const std::string r = xml_attr(definition, "ratio");
            if (!h.empty())
                hfade = std::stof(h);
            if (!v.empty())
                vfade = std::stof(v);
            if (!r.empty())
                ratio = std::stof(r);
        }
        catch (...)
        {
        }
        make_auto_circle(out, hfade, vfade, ratio);
        return true;
    }

    std::string filename = param_cdata(xml, "requiredBrushFile");
    if (filename.empty())
        filename = xml_attr(definition, "filename");
    if (filename.empty())
        return false;
    const auto found = brush_files().find(filename_of(filename));
    if (found == brush_files().end())
        return false;
    PaintBrushTip mask;
    if (!load_mask_at(found->second, mask))
        return false;
    out.width = mask.width;
    out.height = mask.height;
    out.mask = std::move(mask.mask);
    return true;
}

std::vector<std::string> install_roots()
{
    std::vector<std::string> roots;
    const auto add = [&](const fs::path &path)
    {
        std::error_code error;
        if (!path.empty() && fs::is_directory(path, error))
            roots.push_back(path.string());
    };
    const char *home = std::getenv("HOME");
    const char *profile = std::getenv("USERPROFILE");
    const std::string base = home != nullptr ? home : (profile != nullptr ? profile : "");
    if (!base.empty())
    {
        add(fs::path(base) / ".local/share/krita");
        add(fs::path(base) / ".var/app/org.kde.krita/data/krita");
        add(fs::path(base) / ".config/GIMP");
        add(fs::path(base) / ".var/app/org.gimp.GIMP/config/GIMP");
    }
    add("/usr/share/krita");
    add("/usr/share/gimp");
    return roots;
}

void scan_directory(const fs::path &dir, std::vector<PaintBrushEntry> &out, std::set<std::string> &seen, int depth)
{
    if (depth > 3)
        return;
    std::error_code error;
    if (!fs::is_directory(dir, error))
        return;
    for (fs::directory_iterator it(dir, error), end; it != end && !error; it.increment(error))
    {
        const fs::path path = it->path();
        if (it->is_directory(error))
        {
            const std::string folder = path.filename().string();
            const bool named = folder == "paintoppresets" || folder == "brushes" || folder == "bundles";
            // GIMP keeps brushes under a version directory (2.10, 3.0, …).
            const bool version = !folder.empty() && std::isdigit(static_cast<unsigned char>(folder[0]));
            if (depth == 0 || named || version)
                scan_directory(path, out, seen, depth + 1);
            continue;
        }
        const std::string name = path.filename().string();
        const std::string ext_path = path.string();
        if (ends_with(name, ".bundle"))
        {
            mz_zip_archive zip;
            mz_zip_zero_struct(&zip);
            if (!open_zip_reader(&zip, ext_path))
                continue;
            const mz_uint count = mz_zip_reader_get_num_files(&zip);
            for (mz_uint i = 0; i < count; ++i)
            {
                mz_zip_archive_file_stat stat;
                if (!mz_zip_reader_file_stat(&zip, i, &stat))
                    continue;
                const std::string inner = stat.m_filename;
                const std::string file = filename_of(inner);
                if (inner.find("brushes/") != std::string::npos &&
                    (ends_with(file, ".gbr") || ends_with(file, ".png") || ends_with(file, ".gih")))
                {
                    remember_brush_file(file, BrushFile{ext_path, inner});
                    if (seen.insert("tip:" + file).second)
                    {
                        PaintBrushEntry tip;
                        tip.label = "Krita tip: " + pretty_name(file);
                        tip.key = ext_path + "!" + inner;
                        tip.path = ext_path;
                        tip.inner = inner;
                        out.push_back(std::move(tip));
                    }
                }
                if (inner.find("paintoppresets/") == std::string::npos || !ends_with(file, ".kpp"))
                    continue;
                if (!seen.insert("kpp:" + file).second)
                    continue;
                PaintBrushEntry entry;
                entry.label = "Krita: " + pretty_name(file);
                entry.key = ext_path + "!" + inner;
                entry.path = ext_path;
                entry.inner = inner;
                out.push_back(std::move(entry));
            }
            close_zip_reader(&zip);
            continue;
        }

        const fs::path parent = path.parent_path().filename();
        const std::string folder = parent.string();
        if (folder == "brushes" && (ends_with(name, ".gbr") || ends_with(name, ".png") || ends_with(name, ".gih")))
        {
            remember_brush_file(name, BrushFile{{}, ext_path});
            if (!seen.insert("tip:" + name).second)
                continue;
            const bool gimp = ext_path.find("GIMP") != std::string::npos || ext_path.find("gimp") != std::string::npos;
            PaintBrushEntry entry;
            entry.label = (gimp ? "GIMP: " : "Krita tip: ") + pretty_name(name);
            entry.key = ext_path;
            entry.path = ext_path;
            out.push_back(std::move(entry));
        }
        else if (folder == "paintoppresets" && ends_with(name, ".kpp"))
        {
            if (!seen.insert("kpp:" + name).second)
                continue;
            PaintBrushEntry entry;
            entry.label = "Krita: " + pretty_name(name);
            entry.key = ext_path;
            entry.path = ext_path;
            out.push_back(std::move(entry));
        }
    }
}

} // namespace

float PaintBrushTip::mask_at(float u, float v) const
{
    if (width <= 0 || height <= 0 || mask.empty())
        return 0.f;
    u = std::clamp(u, 0.f, 1.f);
    v = std::clamp(v, 0.f, 1.f);
    const float x = u * static_cast<float>(width - 1);
    const float y = v * static_cast<float>(height - 1);
    const int x0 = std::clamp(static_cast<int>(std::floor(x)), 0, width - 1);
    const int y0 = std::clamp(static_cast<int>(std::floor(y)), 0, height - 1);
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float tx = x - static_cast<float>(x0);
    const float ty = y - static_cast<float>(y0);
    const auto at = [&](int px, int py) { return float(mask[static_cast<size_t>(py * width + px)]) / 255.f; };
    const float top = at(x0, y0) * (1.f - tx) + at(x1, y0) * tx;
    const float bottom = at(x0, y1) * (1.f - tx) + at(x1, y1) * tx;
    return top * (1.f - ty) + bottom * ty;
}

float PaintBrushTip::coverage(float u, float v, float pressure) const
{
    const float p = std::clamp(pressure, 0.f, 1.f);
    float opacity = opacity_value;
    float flow = flow_value;
    if (opacity_uses_pressure)
        opacity *= eval_curve(opacity_curve, p);
    if (flow_uses_pressure)
        flow *= eval_curve(flow_curve, p);
    return std::clamp(mask_at(u, v) * opacity * flow, 0.f, 1.f);
}

float PaintBrushTip::size_factor(float pressure) const
{
    if (!size_uses_pressure)
        return 1.f;
    return std::clamp(eval_curve(size_curve, std::clamp(pressure, 0.f, 1.f)), 0.02f, 1.f);
}

std::vector<PaintBrushEntry> scan_installed_paint_brushes()
{
    brush_files().clear();
    std::vector<PaintBrushEntry> entries;
    std::set<std::string> seen;
    for (const std::string &root : install_roots())
        scan_directory(fs::path(root), entries, seen, 0);
    std::sort(entries.begin(), entries.end(),
              [](const PaintBrushEntry &a, const PaintBrushEntry &b) { return a.label < b.label; });
    return entries;
}

bool load_paint_brush(const PaintBrushEntry &entry, PaintBrushTip &out)
{
    out = PaintBrushTip{};
    out.name = entry.label;
    if (!entry.inner.empty())
    {
        std::vector<uint8_t> bytes;
        if (!read_zip_entry(entry.path, entry.inner, bytes))
            return false;
        if (ends_with(entry.inner, ".kpp"))
            return load_kpp_bytes(bytes, out);
        BrushFile loc{entry.path, entry.inner};
        return load_mask_at(loc, out);
    }
    if (ends_with(entry.path, ".kpp"))
    {
        std::vector<uint8_t> bytes;
        if (!read_file(entry.path, bytes))
            return false;
        return load_kpp_bytes(bytes, out);
    }
    BrushFile loc{{}, entry.path};
    return load_mask_at(loc, out);
}

} // namespace Slic3r::GUI
