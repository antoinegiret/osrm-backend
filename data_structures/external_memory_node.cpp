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

#include "external_memory_node.hpp"

#include <limits>

ExternalMemoryNode::ExternalMemoryNode(
    int lat, int lon, double ele, unsigned int node_id, bool barrier, bool traffic_lights, bool crossing, bool elevator)
    : QueryNode(lat, lon, ele, node_id), barrier(barrier), traffic_lights(traffic_lights), crossing(crossing), elevator(elevator)
{
}

ExternalMemoryNode::ExternalMemoryNode() : barrier(false), traffic_lights(false), crossing(false), elevator(false) {}

ExternalMemoryNode ExternalMemoryNode::min_value()
{
    return ExternalMemoryNode(0, 0, 0.0, 0, false, false, false, false);
}

ExternalMemoryNode ExternalMemoryNode::max_value()
{
    return ExternalMemoryNode(std::numeric_limits<int>::max(),
                              std::numeric_limits<int>::max(),
                              std::numeric_limits<double>::max(),
                              std::numeric_limits<unsigned>::max(),
                              false,
                              false,
                              false,
                              false);
}

bool ExternalMemoryNodeSTXXLCompare::operator()(const ExternalMemoryNode &left,
                                                const ExternalMemoryNode &right) const
{
    return left.node_id < right.node_id;
}

ExternalMemoryNodeSTXXLCompare::value_type ExternalMemoryNodeSTXXLCompare::max_value()
{
    return ExternalMemoryNode::max_value();
}

ExternalMemoryNodeSTXXLCompare::value_type ExternalMemoryNodeSTXXLCompare::min_value()
{
    return ExternalMemoryNode::min_value();
}
