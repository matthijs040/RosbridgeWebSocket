#include "../include/boost_websocket_server/RosMessageJsonSerializer.hpp"

#include <geometry_msgs/Point.h>

int main(int argc, char const *argv[])
{
    auto serializer = RosMessageJsonSerializer();

    auto message = geometry_msgs::Point();
    message.x++;


    return 0;
}
