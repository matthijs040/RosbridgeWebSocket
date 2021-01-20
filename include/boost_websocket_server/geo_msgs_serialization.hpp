#ifndef GEO_MSGS_SERIALIZATION_HPP
#define GEO_MSGS_SERIALIZATION_HPP

#include "json.hpp"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

using nlohmann::json;

namespace geometry_msgs
{
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(::geometry_msgs::Point32, x, y, z)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(::geometry_msgs::Point, x, y, z)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(::geometry_msgs::Vector3, x, y, z)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(::geometry_msgs::Twist, angular, linear)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(::geometry_msgs::Quaternion, x, y, z, w)
};


#endif // GEO_MSGS_SERIALIZATION_HPP