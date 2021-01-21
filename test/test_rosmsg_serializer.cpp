#include "../include/boost_websocket_server/RosMessageJsonSerializer.hpp"

#include <geometry_msgs/Point.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    auto serializer = RosMessageJsonSerializer();

    auto message = geometry_msgs::Point();
    std::cout << serializer.Serialize<geometry_msgs::Point>(message);


    return 0;
}
