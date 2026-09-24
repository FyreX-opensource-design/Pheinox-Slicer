///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#ifndef slic3r_FillCat_hpp_
#define slic3r_FillCat_hpp_

#include "FillBase.hpp"

namespace Slic3r
{

// MakerBot Cat Fill. Two strokes traced from the original tile: the head and
// body, and the spiral tail. A vertical line on the tile edge meets the next row.
class FillCat : public Fill
{
public:
    Fill *clone() const override { return new FillCat(*this); }
    ~FillCat() override = default;
    bool is_self_crossing() override { return true; }

protected:
    float _layer_angle(size_t) const override { return 0.f; }
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

// The earlier drawn cat: ears on a line, a spiral, and that shape turned
// halfway around. Kept as its own pattern once Cat took the original tile.
class FillCatMirrored : public Fill
{
public:
    Fill *clone() const override { return new FillCatMirrored(*this); }
    ~FillCatMirrored() override = default;
    bool is_self_crossing() override { return true; }

protected:
    float _layer_angle(size_t) const override { return 0.f; }
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

// The cat field: several cats in one rectangle, repeating in both directions.
class FillCatTiled : public Fill
{
public:
    Fill *clone() const override { return new FillCatTiled(*this); }
    ~FillCatTiled() override = default;
    bool is_self_crossing() override { return true; }

protected:
    float _layer_angle(size_t) const override { return 0.f; }
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

// Continuous-line sharks. Each stroke runs from one tail tip to the next.
class FillShark : public Fill
{
public:
    Fill *clone() const override { return new FillShark(*this); }
    ~FillShark() override = default;
    bool is_self_crossing() override { return true; }

protected:
    float _layer_angle(size_t) const override { return 0.f; }
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

// Interlocking puppy faces, two offset rows to a tile.
class FillPuppy : public Fill
{
public:
    Fill *clone() const override { return new FillPuppy(*this); }
    ~FillPuppy() override = default;
    bool is_self_crossing() override { return true; }

protected:
    float _layer_angle(size_t) const override { return 0.f; }
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

} // namespace Slic3r

#endif
