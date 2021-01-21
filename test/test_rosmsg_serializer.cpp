#include "../include/boost_websocket_server/RosMessageJsonSerializer.hpp"

#include <geometry_msgs/Point.h>
#include <iostream>

namespace test
{
    void pose_with_covariance_correctly_deSerializes()
    {
        auto serializer = RosMessageJsonSerializer();
        auto msg = geometry_msgs::PoseWithCovariance();
        
        msg.pose.position.x = 1;
        msg.pose.position.y = 2;
        msg.pose.position.z = 3;

        msg.pose.orientation.x = 4;
        msg.pose.orientation.y = 5;
        msg.pose.orientation.z = 6;
        msg.pose.orientation.w = 7;

        int set = 0;
        for(auto& val : msg.covariance)
        { val = set++; }

        std::cout << serializer.Serialize(msg) << "\n";

    }
};

int main(int argc, char const *argv[])
{



    test::pose_with_covariance_correctly_deSerializes();


    return 0;
}
