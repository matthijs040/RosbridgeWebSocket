#ifndef ROSMESSAGEJSONSERIALIZER_HPP
#define ROSMESSAGEJSONSERIALIZER_HPP

#include "json.hpp"
using nlohmann::json;

#include <ros/ros.h>    // ros::Time
#include <variant>      // std::variant

namespace ros
{
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Time, sec, nsec) 
};

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
namespace nlohmann 
{
    template <typename T, std::size_t N>
    struct adl_serializer<boost::array<T, N>> {
        static void to_json(json& j, const boost::array<T, N>& data) {
            if (N == 0) {
                j = nullptr;
            } else {
                for(const auto& value : data )
                {
                    j.push_back(value);
                }
            }
        }

        static void from_json(const json& j, boost::array<T, N>& data) 
        {
            if (!j.is_null() && j.is_array() && j.size() == data.size() ) {
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
};

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
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TwistWithCovarianceStamped, header, twist)

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose, position, orientation)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseWithCovariance, pose, covariance)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseWithCovarianceStamped, header, pose)
};


#include <type_traits>

class RosMessageJsonSerializer
{
public:
    using supported_message = std::variant< ros::Time, std_msgs::Float32, std_msgs::Float64, std_msgs::Header
    , geometry_msgs::Point32, geometry_msgs::Point, geometry_msgs::Vector3, geometry_msgs::Quaternion
    , geometry_msgs::Twist, geometry_msgs::TwistWithCovariance, geometry_msgs::TwistWithCovarianceStamped
    , geometry_msgs::Pose, geometry_msgs::PoseWithCovariance, geometry_msgs::PoseWithCovarianceStamped >;

private:

    // shamelessly ripped from: http://coliru.stacked-crooked.com/a/91462340f495ad24
    // https://stackoverflow.com/questions/57642102/how-to-iterate-over-the-types-of-stdvariant

    // Compile-time error that is thrown if a specialization of the variant parse function is not found.
    struct ParseError { ParseError(const char* m) {} }; 

    // Resolver function for the subtypes of the supported message variant.
    template <std::size_t I = 0>                // Subtype count.
    supported_message parse(const json& j)
    {
        if constexpr (I < std::variant_size_v<supported_message>)   // While there are subtypes to evaluate.
        {
            auto result = j.get<std::optional<std::variant_alternative_t<I, supported_message>>>();    // Try the subtype at index I

            return result ? std::move(*result) : parse<I + 1>(j);   // Return the valid specialization result. Iterate otherwise.
        } 
        throw ParseError("Can't parse");
    }

    template <std::size_t I = 0>               
    json construct(const supported_message& msg)
    {
        if constexpr (I < std::variant_size_v<supported_message>)
        {
            auto result = std::get<std::optional<std::variant_alternative_t<I, supported_message>>>(msg);

            return result ? std::move(json(*result)) : construct<I + 1>(msg);
        } 
        throw ParseError("Can't construct");
    }


public:

    RosMessageJsonSerializer(/* args */) {}
    ~RosMessageJsonSerializer() {}

    supported_message Deserialize(const std::string& data)
    {
        const auto json = json::parse(data);
        return parse(json);
    }

    std::string Serialize(const supported_message& message)
    {
        return construct(message).dump();
    }
};


#endif // ROSMESSAGEJSONSERIALIZER_HPP