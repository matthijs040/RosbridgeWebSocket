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
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Status, op, id, level, msg)
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
        try
        {
            auto parsed_json = json::parse(data);
            const auto operation = parsed_json.at("op").get<std::string>();

            if(operation == "set_level")
            { return std::make_unique<SetStatusLevel>( parsed_json.get<SetStatusLevel>() ); }
            else if(operation == "status" )
            { return std::make_unique<Status>( parsed_json.get<Status>() ); }
            else if(operation == "auth" )
            { return std::make_unique<Authenticate>( parsed_json.get<Authenticate>() ); }
            else if(operation == "advertise" )
            { return std::make_unique<Advertise>( parsed_json.get<Advertise>() ); }
            else if(operation == "unadvertise" )
            { return std::make_unique<Unadvertise>( parsed_json.get<Unadvertise>() ); }
            else if(operation == "publish" )
            { return std::make_unique<Publish>( parsed_json.get<Publish>() ); }
            else if(operation == "subscribe" )
            { return std::make_unique<Subscribe>( parsed_json.get<Subscribe>() ); }
            else if(operation == "unsubscribe" )
            { return std::make_unique<Unsubscribe>( parsed_json.get<Unsubscribe>() ); }
            else if(operation == "call_service" )
            { return std::make_unique<CallService>( parsed_json.get<CallService>() ); }
            else if(operation == "advertise_service" )
            { return std::make_unique<AdvertiseService>( parsed_json.get<AdvertiseService>() ); }
            else if(operation == "unadvertise_service" )
            { return std::make_unique<UnadvertiseService>( parsed_json.get<UnadvertiseService>() ); }
            else if(operation == "service_response" )
            { return std::make_unique<UnadvertiseService>( parsed_json.get<UnadvertiseService>() ); }
            else
            { return nullptr; }
        }
        catch(const nlohmann::detail::parse_error& e)
        {
            std::cerr << e.what() << '\n';
            std::cerr << "parsed data does not conform to json standard \n";
            return nullptr;
        }
        catch(const nlohmann::detail::out_of_range& e)
        {
            std::cerr << e.what() << '\n';
            std::cerr << "parsed json does not contain the required \"op\" field.\n";
            return nullptr;
        }
    }
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP