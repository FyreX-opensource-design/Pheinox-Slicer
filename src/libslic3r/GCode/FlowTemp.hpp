///|/ Copyright (c) preFlight 2025+ oozeBot, LLC
///|/
///|/ Released under AGPLv3 or higher
#ifndef slic3r_FlowTemp_hpp_
#define slic3r_FlowTemp_hpp_

#include <string>

namespace Slic3r
{

class PrintConfig;

// Rewrite gcode so each enabled toolhead's nozzle temperature follows upcoming
// volumetric flow, and extrusion slows down when that hotend is still too cold.
// Returns an empty string when no tool has the feature configured.
std::string apply_flow_temp(const std::string &gcode, const PrintConfig &config);

} // namespace Slic3r

#endif
