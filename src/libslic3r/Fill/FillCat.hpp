///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#ifndef slic3r_FillCat_hpp_
#define slic3r_FillCat_hpp_

#include "FillBase.hpp"

namespace Slic3r
{

// Tiled cat: two rounded ears on a line, a spiral tail, and the same shape
// turned halfway around. Traced from the MakerBot Cat Fill print.
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

} // namespace Slic3r

#endif
