#include <iostream>
#include <sstream>

#include "kinect.hpp"

int main( int argc, char* argv[] )
{
    try{
        kinect kinect;
        kinect.run();
    }
    catch( const k4a::error& error ){
        std::cout << error.what() << std::endl;
    }

    return 0;
}