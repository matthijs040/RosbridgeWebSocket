#ifndef BRIDGEMESSAGEJSONSERIALIZER_HPP
#define BRIDGEMESSAGEJSONSERIALIZER_HPP

#include "BridgeMessageSerializer.hpp"  // Interface to implement.
#include "BridgeMessages.hpp"           // Messages to parse.
#include <boost/json.hpp>

// https://www.boost.org/doc/libs/develop/libs/json/doc/html/json/quick_look.html#json.quick_look.value_conversion
// Function overloads for serializing the BridgeMessage types with boost::json.

using namespace boost::json;


namespace BridgeMessages
{

SetStatusLevel tag_invoke( value_to_tag< SetStatusLevel >, const value& v )
{
    const object& obj = v.as_object();
    SetStatusLevel ret;
    ret.level =  value_to<std::string>(obj.at("level"));
    ret.id =     std::make_unique<std::string>( value_to<std::string>(obj.at("id")) );
    return ret;
}

void tag_invoke( value_from_tag, value& v, const SetStatusLevel& m )
{
    v = {
        { "op", m.op },
        { "id", m.id ? *(m.id) : "nullptr" },
        { "level", m.level }
    };  
}

void tag_invoke(value_from_tag, value& v, const BridgeMessage& m)
{
    
    if(auto message = dynamic_cast<const SetStatusLevel*>(&m))
    { 
        tag_invoke(value_from_tag(), v, *message); 
    }
    else
    {
        v = {"unimplemented type"};
    }
    
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
        return serialize( value_from(message ) );
    }

    
    virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data)
    {
        auto parsed_json = parse(data);
        // auto message = value_to<SetStatusLevel>( parsed_json ); 
        return std::make_unique<SetStatusLevel>( value_to<SetStatusLevel>( parsed_json ) );
    }
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP