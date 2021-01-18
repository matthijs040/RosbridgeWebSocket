#ifndef BRIDGEMESSAGEJSONSERIALIZER_HPP
#define BRIDGEMESSAGEJSONSERIALIZER_HPP

#include "BridgeMessageSerializer.hpp"  // Interface to implement.
#include "BridgeMessages.hpp"           // Messages to parse.
#include "json.hpp"

#include <iostream>
// https://www.boost.org/doc/libs/develop/libs/json/doc/html/json/quick_look.html#json.quick_look.value_conversion
// Function overloads for serializing the BridgeMessage types with boost::json.

using nlohmann::json;

namespace BridgeMessages
{

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SetStatusLevel, op, id, level)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Status, op, level, msg)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Authenticate, op, mac, client, dest, rand, t, level, end)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Advertise, op, id, topic, type)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Unadvertise, op, id, topic)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Publish, op, topic, msg)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Subscribe, op, id, topic, type, throttle_rate, queue_length, fragment_size, compression)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Unsubscribe, op, id, topic)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CallService, op, id, service, args, fragment_size, compression)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AdvertiseService, op, type, service)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(UnadvertiseService, op, service)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ServiceResponse, op, id, service, values, result)

    void to_json(json& j, const BridgeMessage& base )
    {
        // Call conversion for the correct sub-type.
        if(auto chld = dynamic_cast<const SetStatusLevel*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Status*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Authenticate*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Advertise*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Unadvertise*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Publish*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Subscribe*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const Unsubscribe*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const AdvertiseService*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const UnadvertiseService*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const CallService*>(&base))
        { to_json(j, *chld); }
        else if (auto chld = dynamic_cast<const ServiceResponse*>(&base))
        { to_json(j, *chld); }
        // else 
        // {
        //     j = {"unimplemented type"};         // Or just leave the "j" outparam unchanged?
        // }
    }
   
};

using namespace BridgeMessages;

class BridgeMessageJsonSerializer : public BridgeMessageSerializer
{
private:
    
public:
    BridgeMessageJsonSerializer(/* args */) {}
    ~BridgeMessageJsonSerializer() {}

    // virtual std::string Serialize(const BridgeMessage& data) = 0;
    // virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data) = 0;

    virtual std::string Serialize(const BridgeMessage& message)
    {
        return json(message).dump();
    }

    std::string Serialize(const Status& message)
    {
        return json(message).dump();
    }

    
    virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data)
    {
        auto parsed_json = json::parse(data);
        const auto operation = parsed_json.at("op").get<std::string>();
        if(operation == "set_level")
        { 
            return std::make_unique<SetStatusLevel>( parsed_json.get<SetStatusLevel>() );
        }
        else
        {
            return nullptr;
        }
        

    }
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP