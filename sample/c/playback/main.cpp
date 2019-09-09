#include <iostream>
#include <sstream>

#include "kinect.hpp"

int main( int argc, char* argv[] )
{
    try{
        /*
        // Sensor
        const uint32_t index = K4A_DEVICE_DEFAULT;
        kinect kinect( index );
        */
        ///*
        // File
        const filesystem::path file = "../file.mkv";
        kinect kinect( file );
        //*/
        kinect.run();
    }
    catch( const std::runtime_error& error ){
        std::cout << error.what() << std::endl;
    }

    return 0;
}