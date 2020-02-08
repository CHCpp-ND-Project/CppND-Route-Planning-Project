#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};                   // 2 options, binary therefore no conversion & ate = at the end to determine length
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();                                                     // determine size of input stream
    std::vector<std::byte> contents(size);                                      // create a vector of bytes the size of the input stream
    
    is.seekg(0);                                                                // seek back to beginning of input stream
    is.read((char*)contents.data(), size);                                      // read contents into that vector is.read ??syntax??

    if( contents.empty() )                                                      
        return std::nullopt;
    return std::move(contents);                                                 // return the contents vector using a move command, to be discussed later
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {                                                            // if more than 1 command line argument
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )               // -f is to specify the osm data file to be used, otherwise use the one provided
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;                                            // create vector of bytes for osm data
 
    if( osm_data.empty() && !osm_data_file.empty() ) {                          // read contents of data file into the vector
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    float start_x = 0.0f;
    float start_y = 0.0f;
    float end_x = 0.0f;
    float end_y = 0.0f;
    std::cout << "Please input start position: x, y: \n";
    std::cin >> start_x, start_y;

    std::cout << "Please input end position: x, y: \n";
    std::cin >> end_x, end_y;
    std::cout << "Calculating path between: " << start_x << "," << start_y << " and " << end_x << "," << end_y << "\n";

    // Build Model.
    RouteModel model{osm_data};                                                 // create RouteModel object called model, populated with osm_data vector

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};                          // 10,10 start point, 90, 90 end point
    route_planner.AStarSearch();                                                // call Astarsearch method

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};                                                       // renders from provided rendering code

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
