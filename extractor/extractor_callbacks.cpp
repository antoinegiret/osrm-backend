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

#include "extractor_callbacks.hpp"
#include "extraction_containers.hpp"
#include "extraction_node.hpp"
#include "extraction_way.hpp"

#include "../data_structures/external_memory_node.hpp"
#include "../data_structures/restriction.hpp"
#include "../Util/container.hpp"
#include "../Util/simple_logger.hpp"

#include <osrm/Coordinate.h>

#include <limits>
#include <string>
#include <vector>

ExtractorCallbacks::ExtractorCallbacks(ExtractionContainers &extraction_containers,
                                       std::unordered_map<std::string, NodeID> &string_map,
                                       std::unordered_map<std::string, NodeID> &towns_map,
                                       std::unordered_map<std::string, NodeID> &bike_routes_map,
                                       std::unordered_map<NodeID, FixedPointCoordinate> &coordinates_map) :
        string_map(string_map),
        towns_map(towns_map),
        bike_routes_map(bike_routes_map),
        coordinates_map(coordinates_map),
        external_memory(extraction_containers)
{
}

/** warning: caller needs to take care of synchronization! */
void ExtractorCallbacks::ProcessNode(const osmium::Node &input_node,
                                     const ExtractionNode &result_node)
{
	const int lat = static_cast<int>(input_node.location().lat() * COORDINATE_PRECISION);
	const int lon = static_cast<int>(input_node.location().lon() * COORDINATE_PRECISION);
	const double ele = atof(input_node.tags().get_value_by_key("height"));
	const FixedPointCoordinate coordinates(lat, lon, ele);
	coordinates_map.insert(std::make_pair(static_cast<NodeID>(input_node.id()), coordinates));
	
    external_memory.all_nodes_list.push_back({
        lat,
        lon,
        ele,
        static_cast<NodeID>(input_node.id()),
        result_node.barrier,
        result_node.traffic_lights,
        result_node.crossing,
        result_node.elevator
    });
}

void ExtractorCallbacks::ProcessRestriction(
    const mapbox::util::optional<InputRestrictionContainer> &restriction)
{
    if (restriction)
    {
        external_memory.restrictions_list.push_back(restriction.get());
        // SimpleLogger().Write() << "from: " << restriction.get().restriction.from.node <<
        //                           ",via: " << restriction.get().restriction.via.node <<
        //                           ", to: " << restriction.get().restriction.to.node <<
        //                           ", only: " << (restriction.get().restriction.flags.is_only ? "y" : "n");
    }
}
/** warning: caller needs to take care of synchronization! */
void ExtractorCallbacks::ProcessWay(const osmium::Way &input_way, const ExtractionWay &parsed_way)
{
    if (((0 >= parsed_way.forward_speed) ||
         (TRAVEL_MODE_INACCESSIBLE == parsed_way.forward_travel_mode)) &&
        ((0 >= parsed_way.backward_speed) ||
         (TRAVEL_MODE_INACCESSIBLE == parsed_way.backward_travel_mode)) &&
        (0 >= parsed_way.duration))
    { // Only true if the way is specified by the speed profile
        return;
    }

    if (input_way.nodes().size() <= 1)
    { // safe-guard against broken data
        return;
    }

    if (std::numeric_limits<decltype(input_way.id())>::max() == input_way.id())
    {
        SimpleLogger().Write(logDEBUG) << "found bogus way with id: " << input_way.id()
                                       << " of size " << input_way.nodes().size();
        return;
    }
    if (0 < parsed_way.duration)
    {
        // TODO: iterate all way segments and set duration corresponding to the length of each
        // segment
        const_cast<ExtractionWay&>(parsed_way).forward_speed = parsed_way.duration / (input_way.nodes().size() - 1);
        const_cast<ExtractionWay&>(parsed_way).backward_speed = parsed_way.duration / (input_way.nodes().size() - 1);
    }

    if (std::numeric_limits<double>::epsilon() >= std::abs(-1. - parsed_way.forward_speed))
    {
        SimpleLogger().Write(logDEBUG) << "found way with bogus speed, id: " << input_way.id();
        return;
    }

    // Get the unique identifier for the street name
    const auto &string_map_iterator = string_map.find(parsed_way.name);
    unsigned name_id = external_memory.name_list.size();
    if (string_map.end() == string_map_iterator)
    {
        external_memory.name_list.push_back(parsed_way.name);
        string_map.insert(std::make_pair(parsed_way.name, name_id));
    }
    else
    {
        name_id = string_map_iterator->second;
    }

    // Get the unique identifier for the street towns
    const auto &towns_map_iterator = towns_map.find(parsed_way.towns);
    unsigned towns_id = external_memory.towns_list.size();
    if (towns_map.end() == towns_map_iterator)
    {
        external_memory.towns_list.push_back(parsed_way.towns);
        towns_map.insert(std::make_pair(parsed_way.towns, towns_id));
    }
    else
    {
        towns_id = towns_map_iterator->second;
    }

    // Get the unique identifier for the bike routes
    const auto &bike_routes_map_iterator = bike_routes_map.find(parsed_way.bike_routes);
    unsigned bike_routes_id = external_memory.bike_routes_list.size();
    if (bike_routes_map.end() == bike_routes_map_iterator)
    {
        external_memory.bike_routes_list.push_back(parsed_way.bike_routes);
        bike_routes_map.insert(std::make_pair(parsed_way.bike_routes, bike_routes_id));
    }
    else
    {
        bike_routes_id = bike_routes_map_iterator->second;
    }

    const bool split_edge = (parsed_way.forward_speed > 0) &&
                            (TRAVEL_MODE_INACCESSIBLE != parsed_way.forward_travel_mode) &&
                            (parsed_way.backward_speed > 0) &&
                            (TRAVEL_MODE_INACCESSIBLE != parsed_way.backward_travel_mode)/* &&
                            ((parsed_way.forward_speed != parsed_way.backward_speed) ||
                             (parsed_way.forward_travel_mode != parsed_way.backward_travel_mode) ||
                             (parsed_way.forward_facility != parsed_way.backward_facility))*/;

    auto pair_wise_segment_split = [&](const osmium::NodeRef &first_node,
                                       const osmium::NodeRef &last_node)
    {
		const double slope_ratio = GetSlopeRatio(first_node, last_node);
		
        external_memory.all_edges_list.push_back(InternalExtractorEdge(
            first_node.ref(),
            last_node.ref(),
            ((split_edge || TRAVEL_MODE_INACCESSIBLE == parsed_way.backward_travel_mode)
                 ? ExtractionWay::oneway
                 : ExtractionWay::bidirectional),
            parsed_way.forward_speed * slope_ratio,
            name_id,
            towns_id,
            bike_routes_id,
            parsed_way.roundabout,
            parsed_way.ignore_in_grid,
            (0 < parsed_way.duration),
            parsed_way.is_access_restricted,
            parsed_way.forward_travel_mode,
            split_edge,
            parsed_way.forward_facility));
        external_memory.used_node_id_list.push_back(first_node.ref());
    };

    const bool is_opposite_way = TRAVEL_MODE_INACCESSIBLE == parsed_way.forward_travel_mode;
    if (is_opposite_way)
    {
        const_cast<ExtractionWay&>(parsed_way).forward_travel_mode = parsed_way.backward_travel_mode;
        const_cast<ExtractionWay&>(parsed_way).backward_travel_mode = TRAVEL_MODE_INACCESSIBLE;
        
        const_cast<ExtractionWay&>(parsed_way).forward_speed = parsed_way.backward_speed;
        const_cast<ExtractionWay&>(parsed_way).backward_speed = 0.0;
        
        const_cast<ExtractionWay&>(parsed_way).forward_facility = parsed_way.backward_facility;
        const_cast<ExtractionWay&>(parsed_way).backward_facility = FACILITY_FORBIDDEN;
        
        osrm::for_each_pair(
            input_way.nodes().crbegin(), input_way.nodes().crend(), pair_wise_segment_split);
        external_memory.used_node_id_list.push_back(input_way.nodes().front().ref());
    }
    else
    {
        osrm::for_each_pair(
            input_way.nodes().cbegin(), input_way.nodes().cend(), pair_wise_segment_split);
        external_memory.used_node_id_list.push_back(input_way.nodes().back().ref());
    }

    // The following information is needed to identify start and end segments of restrictions
    external_memory.way_start_end_id_list.push_back(
        {(EdgeID)input_way.id(),
         (NodeID)input_way.nodes()[0].ref(),
         (NodeID)input_way.nodes()[1].ref(),
         (NodeID)input_way.nodes()[input_way.nodes().size() - 2].ref(),
         (NodeID)input_way.nodes().back().ref()});

    if (split_edge)
    { // Only true if the way should be split
        BOOST_ASSERT(parsed_way.backward_travel_mode>0);
        auto pair_wise_segment_split_2 = [&](const osmium::NodeRef &first_node,
                                             const osmium::NodeRef &last_node)
        {
            const double slope_ratio = GetSlopeRatio(last_node, first_node);
            
            external_memory.all_edges_list.push_back(
                InternalExtractorEdge(last_node.ref(),
                                      first_node.ref(),
                                      ExtractionWay::oneway,
                                      parsed_way.backward_speed * slope_ratio,
                                      name_id,
                                      towns_id,
                                      bike_routes_id,
                                      parsed_way.roundabout,
                                      parsed_way.ignore_in_grid,
                                      (0 < parsed_way.duration),
                                      parsed_way.is_access_restricted,
                                      parsed_way.backward_travel_mode,
                                      split_edge,
                                      parsed_way.backward_facility));
        };

        if (is_opposite_way)
        {
            osrm::for_each_pair(input_way.nodes().crbegin(),
                          input_way.nodes().crend(),
                          pair_wise_segment_split_2);
            external_memory.used_node_id_list.push_back(input_way.nodes().front().ref());
        }
        else
        {
            osrm::for_each_pair(input_way.nodes().cbegin(),
                          input_way.nodes().cend(),
                          pair_wise_segment_split_2);
            external_memory.used_node_id_list.push_back(input_way.nodes().back().ref());
        }

        external_memory.way_start_end_id_list.push_back(
            {(EdgeID)input_way.id(),
             (NodeID)input_way.nodes()[1].ref(),
             (NodeID)input_way.nodes()[0].ref(),
             (NodeID)input_way.nodes().back().ref(),
             (NodeID)input_way.nodes()[input_way.nodes().size() - 2].ref()});
    }
}

double ExtractorCallbacks::GetSlopeRatio(const osmium::NodeRef &first_node, const osmium::NodeRef &last_node) {
	const auto &first_node_coordinates_map_iterator = coordinates_map.find(first_node.ref());
	const auto &last_node_coordinates_map_iterator = coordinates_map.find(last_node.ref());
	if (coordinates_map.end() != first_node_coordinates_map_iterator && coordinates_map.end() != last_node_coordinates_map_iterator) {
		const FixedPointCoordinate first_node_coordinates = first_node_coordinates_map_iterator->second;
		const FixedPointCoordinate last_node_coordinates = last_node_coordinates_map_iterator->second;
		
		const double distance = FixedPointCoordinate::ApproximateEuclideanDistance(first_node_coordinates, last_node_coordinates);
		const double diff_ele = last_node_coordinates.ele - first_node_coordinates.ele;
		const double slope_percentage = diff_ele / distance;
		
		if(slope_percentage > 0.08) {
			return 0.25;
		}
		else if(slope_percentage > 0.04) {
			return 0.5;
		}
	}
	
	return 1.0;
}
