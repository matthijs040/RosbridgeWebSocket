#ifndef ROSMESSAGEJSONSERIALIZER_HPP
#define ROSMESSAGEJSONSERIALIZER_HPP

#include "json.hpp"
#include "geo_msgs_serialization.hpp"

using nlohmann::json;

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