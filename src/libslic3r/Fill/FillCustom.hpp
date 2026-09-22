///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
///|/
#ifndef slic3r_FillCustom_hpp_
#define slic3r_FillCustom_hpp_

#include "FillBase.hpp"

namespace Slic3r
{

class PrintRegionConfig;

// Sparse infill driven by user equations, PNG/SVG images, or a tiled mesh.
class FillCustom : public Fill
{
public:
    FillCustom() = default;
    Fill *clone() const override { return new FillCustom(*this); }

    bool use_bridge_flow() const override { return false; }
    bool is_self_crossing() override { return true; }

    const PrintRegionConfig *print_region_config = nullptr;

protected:
    void _fill_surface_single(const FillParams &params, unsigned int thickness_layers,
                              const std::pair<float, Point> &direction, ExPolygon expolygon,
                              Polylines &polylines_out) override;
};

} // namespace Slic3r

#endif // slic3r_FillCustom_hpp_
