#include "../include/boost_websocket_server/RosMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/TestingUtils.hpp"

#include <iostream>

namespace test
{
    void point_correctly_serializes()
    {
        auto serializer = RosMessageJsonSerializer();
        auto msg = geometry_msgs::Point();  

        msg.x = 1;
        msg.y = 2;
        msg.z = 3;

        const auto data = serializer.Serialize(msg);

        assert(contains_keyvalue_pair(data, "x", msg.x));
        assert(contains_keyvalue_pair(data, "y", msg.y));
        assert(contains_keyvalue_pair(data, "z", msg.z));
    }

    void point_correctly_deserializes()
    {
        auto serializer = RosMessageJsonSerializer();
        auto msg = geometry_msgs::Point();  

        msg.x = 1;
        msg.y = 2;
        msg.z = 3;

        const auto data = serializer.Deserialize<geometry_msgs::Point>(serializer.Serialize(msg));
        assert(data == msg);
    }

    void header_correctly_serializes()
    {
        auto serializer = RosMessageJsonSerializer();
        auto msg = std_msgs::Header();

        msg.frame_id = "test_id";
        msg.seq = 1;
        msg.stamp = ros::Time(1, 2);

        const auto data = serializer.Serialize(msg);
        assert( contains_keyvalue_pair(data, "frame_id", msg.frame_id) );
        assert( contains_keyvalue_pair(data, "seq", msg.seq) );
        assert( contains_keyvalue_pair(data, "sec", msg.stamp.sec) );
        assert( contains_keyvalue_pair(data, "nsec", msg.stamp.nsec) );
    }

    void vector3_correctly_serializes()
    {
        auto serializer = RosMessageJsonSerializer();
        auto msg = geometry_msgs::Vector3();  

        msg.x = 1;
        msg.y = 2;
        msg.z = 3;

        const auto data = serializer.Serialize(msg);

        assert(contains_keyvalue_pair(data, "x", msg.x));
        assert(contains_keyvalue_pair(data, "y", msg.y));
        assert(contains_keyvalue_pair(data, "z", msg.z));
    }

    void vector3_correctly_deserializes()
    {
        auto serializer = RosMessageJsonSerializer();
        auto msg = geometry_msgs::Vector3();  

        msg.x = 1;
        msg.y = 2;
        msg.z = 3;

        const auto data = serializer.Deserialize<geometry_msgs::Vector3>(serializer.Serialize(msg));
        assert(data == msg);       
    }

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

        const auto data = serializer.Serialize(msg);

        auto ptr = msg.covariance.data();
        auto size =  msg.covariance.size();
        assert( contains_keyvalue_pair(data, "covariance", std::vector<double>(ptr, ptr + size) ) );

        auto des = serializer.Deserialize<geometry_msgs::PoseWithCovariance>(data);

        assert( msg ==  des);
    }
};

int main(int argc, char const *argv[])
{
    test::point_correctly_serializes();
    test::point_correctly_deserializes();
    test::header_correctly_serializes();
    test::vector3_correctly_serializes();
    test::pose_with_covariance_correctly_deSerializes();

    return 0;
}
