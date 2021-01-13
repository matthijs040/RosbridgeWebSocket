#ifndef BRIDGEMESSAGEJSONSERIALIZER_HPP
#define BRIDGEMESSAGEJSONSERIALIZER_HPP

#include "BridgeMessageSerializer.hpp"  // Interface to implement.
#include "BridgeMessages.hpp"           // Messages to parse.
#include <boost/json.hpp>

#include <iostream>
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
        { "id", m.id ? *(m.id) : "" },
        { "level", m.level }
    };  
}

void tag_invoke( value_from_tag, value& v, const Status& m)
{
    v = {
        { "op", m.op },
        { "id", m.id ? *(m.id) : "" },
        { "level", m.level },
        { "msg", m.msg },
    };
}

void tag_invoke( value_from_tag, value& v, const Authenticate& m)
{
    v = {
        { "op", m.op },
        { "mac", m.mac},
        { "client", m.client},
        { "dest",  m.dest},
        { "rand", m.rand},
        { "t", m.t},
        { "level", m.level},
        { "end", m.end}
    };
}

void tag_invoke( value_from_tag, value& v, const Advertise& m)
{
    v = {
        { "op", m.op },
        { "id", m.id ? *(m.id) : "" },
        { "topic", m.topic},
        { "type", m.type}
    };
}

void tag_invoke( value_from_tag, value& v, const Unadvertise& m)
{
    v = {
        { "op", m.op },
        { "id", m.id ? *(m.id) : "" },
        { "topic", m.topic}
    };
}


void tag_invoke( value_from_tag, value& v, const Publish& m)
{
    v = {
        { "op", m.op },
        { "topic", m.topic},
        { "msg", m.msg ? *m.msg : "" } //placeholder
    };
}


void tag_invoke( value_from_tag, value& v, const Subscribe& m)
{
    v = {
        { "op", m.op },
        { "id", m.id ? *(m.id) : "" },
        { "topic", m.topic},
        { "msg", m.type ? *m.type : "" },
        { }
    };
}




void tag_invoke(value_from_tag t, value& v, const BridgeMessage& base)
{
    std::cout << "WARN: tag_invoke on base 'BridgeMessage' called!\n";
    if      (auto chld = dynamic_cast<const SetStatusLevel*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Status*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Authenticate*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Advertise*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Unadvertise*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Publish*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Subscribe*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const Unsubscribe*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const AdvertiseService*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const UnadvertiseService*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const CallService*>(&base))
    { tag_invoke(t, v, *chld); }
    else if (auto chld = dynamic_cast<const ServiceResponse*>(&base))
    { tag_invoke(t, v, *chld); }
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