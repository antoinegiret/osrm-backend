/*

Copyright (c) 2015, Project OSRM, Dennis Luxen, others
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

#include "extractor.hpp"

#include "extraction_containers.hpp"
#include "extraction_node.hpp"
#include "extraction_way.hpp"
#include "extractor_callbacks.hpp"
#include "extractor_options.hpp"
#include "restriction_parser.hpp"
#include "scripting_environment.hpp"

#include "../Util/git_sha.hpp"
#include "../Util/IniFileUtil.h"
#include "../Util/simple_logger.hpp"
#include "../Util/timing_util.hpp"
#include "../Util/make_unique.hpp"

#include "../typedefs.h"

#include <luabind/luabind.hpp>

#include <osmium/io/any_input.hpp>

#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>

#include <variant/optional.hpp>

#include <cstdlib>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <vector>

int Extractor::Run(int argc, char *argv[])
{
    ExtractorConfig extractor_config;

    try
    {
        LogPolicy::GetInstance().Unmute();
        TIMER_START(extracting);

        if (!ExtractorOptions::ParseArguments(argc, argv, extractor_config))
        {
            return 0;
        }
        ExtractorOptions::GenerateOutputFilesNames(extractor_config);

        if (1 > extractor_config.requested_num_threads)
        {
            SimpleLogger().Write(logWARNING) << "Number of threads must be 1 or larger";
            return 1;
        }

        if (!boost::filesystem::is_regular_file(extractor_config.input_path))
        {
            SimpleLogger().Write(logWARNING)
                << "Input file " << extractor_config.input_path.string() << " not found!";
            return 1;
        }

        if (!boost::filesystem::is_regular_file(extractor_config.profile_path))
        {
            SimpleLogger().Write(logWARNING) << "Profile " << extractor_config.profile_path.string()
                                             << " not found!";
            return 1;
        }

        const unsigned recommended_num_threads = tbb::task_scheduler_init::default_num_threads();
        const auto number_of_threads = std::min(recommended_num_threads, extractor_config.requested_num_threads);
        tbb::task_scheduler_init init(number_of_threads);

        SimpleLogger().Write() << "Input file: " << extractor_config.input_path.filename().string();
        SimpleLogger().Write() << "Profile: " << extractor_config.profile_path.filename().string();
        SimpleLogger().Write() << "Threads: " << number_of_threads;

        // setup scripting environment
        ScriptingEnvironment scripting_environment(extractor_config.profile_path.string().c_str());

        std::unordered_map<std::string, NodeID> string_map;
        string_map[""] = 0;

        std::unordered_map<std::string, NodeID> towns_map;
        towns_map[""] = 0;
        
        std::unordered_map<NodeID, FixedPointCoordinate> coordinates_map;

        ExtractionContainers extraction_containers;
        auto extractor_callbacks =
            osrm::make_unique<ExtractorCallbacks>(extraction_containers, string_map, towns_map, coordinates_map);

        const osmium::io::File input_file(extractor_config.input_path.string());
        osmium::io::Reader reader(input_file);
        const osmium::io::Header header = reader.header();

        std::atomic<unsigned> number_of_nodes {0};
        std::atomic<unsigned> number_of_ways {0};
        std::atomic<unsigned> number_of_relations {0};
        std::atomic<unsigned> number_of_others {0};

        SimpleLogger().Write() << "Parsing in progress..";
        TIMER_START(parsing);

        std::string generator = header.get("generator");
        if (generator.empty())
        {
            generator = "unknown tool";
        }
        SimpleLogger().Write() << "input file generated by " << generator;

        // write .timestamp data file
        std::string timestamp = header.get("osmosis_replication_timestamp");
        if (timestamp.empty())
        {
            timestamp = "n/a";
        }
        SimpleLogger().Write() << "timestamp: " << timestamp;

        boost::filesystem::ofstream timestamp_out(extractor_config.timestamp_file_name);
        timestamp_out.write(timestamp.c_str(), timestamp.length());
        timestamp_out.close();

        // initialize vectors holding parsed objects
        tbb::concurrent_vector<std::pair<std::size_t, ExtractionNode>> resulting_nodes;
        tbb::concurrent_vector<std::pair<std::size_t, ExtractionWay>> resulting_ways;
        tbb::concurrent_vector<mapbox::util::optional<InputRestrictionContainer>>
            resulting_restrictions;

        // setup restriction parser
        const RestrictionParser restriction_parser(scripting_environment.get_lua_state());

        while (const osmium::memory::Buffer buffer = reader.read())
        {
            // create a vector of iterators into the buffer
            std::vector<osmium::memory::Buffer::const_iterator> osm_elements;
            for (auto iter = std::begin(buffer); iter != std::end(buffer); ++iter) {
                osm_elements.push_back(iter);
            }

            // clear resulting vectors
            resulting_nodes.clear();
            resulting_ways.clear();
            resulting_restrictions.clear();

            // parse OSM entities in parallel, store in resulting vectors
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, osm_elements.size()),
                              [&](const tbb::blocked_range<std::size_t> &range)
                              {
                for (auto x = range.begin(); x != range.end(); ++x)
                {
                    const auto entity = osm_elements[x];

                    ExtractionNode result_node;
                    ExtractionWay result_way;

                    lua_State * local_state = scripting_environment.get_lua_state();

                    switch (entity->type())
                    {
                    case osmium::item_type::node:
                        ++number_of_nodes;
                        luabind::call_function<void>(
                            local_state,
                            "node_function",
                            boost::cref(static_cast<const osmium::Node &>(*entity)),
                            boost::ref(result_node));
                        resulting_nodes.push_back(std::make_pair(x, result_node));
                        break;
                    case osmium::item_type::way:
                        ++number_of_ways;
                        luabind::call_function<void>(
                            local_state,
                            "way_function",
                            boost::cref(static_cast<const osmium::Way &>(*entity)),
                            boost::ref(result_way));
                        resulting_ways.push_back(std::make_pair(x, result_way));
                        break;
                    case osmium::item_type::relation:
                        ++number_of_relations;
                        resulting_restrictions.push_back(
                            restriction_parser.TryParse(static_cast<const osmium::Relation &>(*entity)));
                        break;
                    default:
                        ++number_of_others;
                        break;
                    }
                }
            });

            // put parsed objects thru extractor callbacks
            for (const auto &result : resulting_nodes)
            {
                extractor_callbacks->ProcessNode(
                    static_cast<const osmium::Node &>(*(osm_elements[result.first])), result.second);
            }
            for (const auto &result : resulting_ways)
            {
                extractor_callbacks->ProcessWay(
                    static_cast<const osmium::Way &>(*(osm_elements[result.first])), result.second);
            }
            for (const auto &result : resulting_restrictions)
            {
                extractor_callbacks->ProcessRestriction(result);
            }
        }
        TIMER_STOP(parsing);
        SimpleLogger().Write() << "Parsing finished after " << TIMER_SEC(parsing) << " seconds";

        unsigned nn = number_of_nodes;
        unsigned nw = number_of_ways;
        unsigned nr = number_of_relations;
        unsigned no = number_of_others;
        SimpleLogger().Write() << "Raw input contains "
                               << nn << " nodes, "
                               << nw << " ways, and "
                               << nr << " relations, and "
                               << no << " unknown entities";

        extractor_callbacks.reset();

        if (extraction_containers.all_edges_list.empty())
        {
            SimpleLogger().Write(logWARNING) << "The input data is empty, exiting.";
            return 1;
        }

        extraction_containers.PrepareData(extractor_config.output_file_name,
                                          extractor_config.restriction_file_name);
        TIMER_STOP(extracting);
        SimpleLogger().Write() << "extraction finished after " << TIMER_SEC(extracting) << "s";
        SimpleLogger().Write() << "To prepare the data for routing, run: "
                               << "./osrm-prepare " << extractor_config.output_file_name
                               << std::endl;
    }
    catch (std::exception &e)
    {
        SimpleLogger().Write(logWARNING) << e.what();
        return 1;
    }
    return 0;
}
