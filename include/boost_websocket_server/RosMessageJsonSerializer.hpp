#ifndef ROSMESSAGEJSONSERIALIZER_HPP
#define ROSMESSAGEJSONSERIALIZER_HPP

#include "json.hpp"
using nlohmann::json;

#include <ros/ros.h> // ros::Time

namespace ros
{
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Time, sec, nsec) 
}

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

namespace std_msgs
{
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Float32, data)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Float64, data)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Header, seq, stamp, frame_id)
}; 

// Example ripped from: https://github.com/nlohmann/json#how-do-i-convert-third-party-types
// Specialized for boost::array<double, size> used in the Covariance structure.
namespace nlohmann {
    template <typename T, std::size_t N>
    struct adl_serializer<boost::array<T, N>> {
        static void to_json(json& j, const boost::array<T, N>& data) {
            if (N == 0) {
                j = nullptr;
            } else {
              j = data;
            }
        }

        static void from_json(const json& j, boost::array<T, N>& data) 
        {
            if (!j.is_null() && j.is_array() && j.size() == ( sizeof(T) * N ) ) {
                int ind = 0;
                for(auto item : j.items())
                {
                    data.at(ind) = item.value();
                    ind++;
                }            
            } else {
                data = boost::array<T, N>();
            }
        }
    };
}

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace geometry_msgs
{
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point32, x, y, z)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, x, y, z)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector3, x, y, z)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Quaternion, x, y, z, w)

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Twist, angular, linear)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TwistWithCovariance, twist, covariance)

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose, position, orientation)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseWithCovariance, pose, covariance)
};



class RosMessageJsonSerializer
{
private:
   
public:
    RosMessageJsonSerializer(/* args */) {}
    ~RosMessageJsonSerializer() {}

    template<typename MessageType>
    MessageType Deserialize(const std::string& data)
    {
        return json::parse(data).get<MessageType>();
    }

    template<typename MessageType>
    std::string Serialize(const MessageType& message)
    {
        return json(message).dump();
    }
};


#endif // ROSMESSAGEJSONSERIALIZER_HPP