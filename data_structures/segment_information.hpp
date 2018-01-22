/*

Copyright (c) 2014, Project OSRM, Dennis Luxen, others
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef SEGMENT_INFORMATION_H
#define SEGMENT_INFORMATION_H

#include "turn_instructions.hpp"

#include "../data_structures/travel_mode.hpp"
#include "../data_structures/facility.hpp"
#include "../typedefs.h"

#include <osrm/Coordinate.h>

// Struct fits everything in one cache line
struct SegmentInformation
{
    FixedPointCoordinate location;
    NodeID name_id;
    NodeID towns_id;
    NodeID bike_routes_id;
    EdgeWeight duration;
    float length;
    short bearing; // more than enough [0..3600] fits into 12 bits
    TurnInstruction turn_instruction;
    TravelMode travel_mode;
    Facility facility;
    bool necessary;
    bool is_via_location;

    explicit SegmentInformation(const FixedPointCoordinate &location,
                                const NodeID name_id,
                                const NodeID towns_id,
                                const NodeID bike_routes_id,
                                const EdgeWeight duration,
                                const float length,
                                const TurnInstruction turn_instruction,
                                const bool necessary,
                                const bool is_via_location,
                                const TravelMode travel_mode,
                                const Facility facility)
        : location(location), name_id(name_id), towns_id(towns_id), bike_routes_id(bike_routes_id), duration(duration), length(length), bearing(0),
          turn_instruction(turn_instruction), travel_mode(travel_mode), facility(facility),
          necessary(necessary), is_via_location(is_via_location)
    {
    }

    explicit SegmentInformation(const FixedPointCoordinate &location,
                                const NodeID name_id,
                                const NodeID towns_id,
                                const NodeID bike_routes_id,
                                const EdgeWeight duration,
                                const float length,
                                const TurnInstruction turn_instruction,
                                const TravelMode travel_mode,
                                const Facility facility)
        : location(location), name_id(name_id), towns_id(towns_id), bike_routes_id(bike_routes_id), duration(duration), length(length), bearing(0),
          turn_instruction(turn_instruction), travel_mode(travel_mode), facility(facility),
          necessary(turn_instruction != TurnInstruction::NoTurn), is_via_location(false)
    {
    }
};

#endif /* SEGMENT_INFORMATION_H */
