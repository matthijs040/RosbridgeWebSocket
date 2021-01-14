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
    ret.level = value_to<std::string>(obj.at("level"));
    ret.id =    value_to<std::string>(obj.at("id"));
    return ret;
}

BridgeMessage tag_invoke( value_to_tag<BridgeMessage>, const value& v)
{
    const object& obj = v.as_object();
    const auto op = value_to<std::string>(obj.at("op"));

    if(op == "set_level")
    {
        auto ret = SetStatusLevel();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.level = value_to<std::string>(obj.at("level"));
        return ret;
    }
    else if(op == "status")
    {
        auto ret = Status();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.level = value_to<std::string>(obj.at("level"));
        ret.msg = value_to<std::string>(obj.at("msg"));
        return ret;
    }
    else if(op == "auth")
    {
        auto ret = Authenticate();
        ret.mac = value_to<std::string>(obj.at("mac"));
        ret.client = value_to<std::string>(obj.at("client"));
        ret.dest = value_to<std::string>(obj.at("dest"));
        ret.rand = value_to<std::string>(obj.at("rand"));
        ret.t = value_to<int>(obj.at("t"));
        ret.level = value_to<std::string>(obj.at("level"));
        ret.end = value_to<int>(obj.at("end"));                
        return ret;
    } 
    else if(op == "advertise")
    {
        auto ret = Advertise();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.topic = value_to<std::string>(obj.at("topic"));
        ret.type = value_to<std::string>(obj.at("type"));
        return ret;
    }
    else if(op == "unadvertise")
    {
        auto ret = Unadvertise();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.topic = value_to<std::string>(obj.at("topic"));
        return ret;
    }
    else if(op == "publish")
    {
        auto ret = Publish();
        ret.msg = value_to<std::string>(obj.at("msg"));
        ret.topic = value_to<std::string>(obj.at("topic"));
        return ret;
    }
    else if(op == "subscribe")
    {
        auto ret = Subscribe();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.topic = value_to<std::string>(obj.at("topic"));
        ret.type = value_to<std::string>(obj.at("type"));
        ret.throttle_rate = value_to<int>(obj.at("throttle_rate"));
        ret.queue_length = value_to<int>(obj.at("queue_length"));
        ret.fragment_size = value_to<int>(obj.at("fragment_size"));
        ret.compression = value_to<std::string>(obj.at("compression"));
        return ret;
    }
    else if(op == "unsubscribe")
    {
        auto ret = Unsubscribe();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.topic = value_to<std::string>(obj.at("topic"));
        return ret;
    }
    else if(op == "call_service")
    {
        auto ret = CallService();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.service = value_to<std::string>(obj.at("service"));
        ret.args = value_to<std::string>(obj.at("args"));
        ret.fragment_size = value_to<int>(obj.at("fragment_size"));
        ret.compression = value_to<std::string>(obj.at("compression"));
        return ret;
    }
    else if(op == "advertise_service")
    {
        auto ret = AdvertiseService();
        ret.type = value_to<std::string>(obj.at("type"));
        ret.service = value_to<std::string>(obj.at("service")); 
        return ret;
    }
    else if(op == "unadvertise_service")
    {
        auto ret = UnadvertiseService();
        ret.service = value_to<std::string>(obj.at("service")); 
        return ret;
    }
    else if(op == "service_response")
    {
        auto ret = ServiceResponse();
        ret.id = value_to<std::string>(obj.at("id"));
        ret.service = value_to<std::string>(obj.at("service"));
        ret.values = value_to<std::string>(obj.at("values"));
        ret.result = value_to<bool>(obj.at("result"));
        return ret;
    }
    else
    {
        throw std::logic_error("attempting to deserialize unknown bridgemessage type");
    }
}

void tag_invoke( value_from_tag, value& v, const SetStatusLevel& m )
{
    v = {
        { "op", m.op },
        { "id", m.id },
        { "level", m.level }
    };  
}

void tag_invoke( value_from_tag, value& v, const Status& m)
{
    v = {
        { "op", m.op },
        { "id", m.id },
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
        { "id", m.id },
        { "topic", m.topic},
        { "type", m.type}
    };
}

void tag_invoke( value_from_tag, value& v, const Unadvertise& m)
{
    v = {
        { "op", m.op },
        { "id", m.id },
        { "topic", m.topic}
    };
}


void tag_invoke( value_from_tag, value& v, const Publish& m)
{
    v = {
        { "op", m.op },
        { "topic", m.topic},
        { "msg", "" } //placeholder m.msg
    };
}


void tag_invoke( value_from_tag, value& v, const Subscribe& m)
{
    v = {
        { "op", m.op },
        { "id", m.id },
        { "topic", m.topic},
        { "msg", m.type },
        { "throttle_rate", m.throttle_rate},
        { "queue_length", m.queue_length},
        { "fragment_size", m.fragment_size},
        { "compression", m.compression}
    };
}

void tag_invoke( value_from_tag, value& v, const Unsubscribe& m)
{
    v = {
        { "op", m.op },
        { "id", m.id },
        { "topic", m.topic}
    };
}

void tag_invoke( value_from_tag, value& v, const CallService& m)
{
    v = {
        { "op", m.op },
        { "id", m.id },
        { "service", m.service},
        { "args", m.args },
        { "fragment_size", m.fragment_size},
        { "compression", m.compression}
    };
}

void tag_invoke( value_from_tag, value& v, const AdvertiseService& m)
{
    v = {
        { "op", m.op },
        { "type", m.type },
        { "service", m.service}
    };
}

void tag_invoke( value_from_tag, value& v, const UnadvertiseService& m)
{
    v = {
        { "op", m.op },
        { "service", m.service }
    };
}

void tag_invoke( value_from_tag, value& v, const ServiceResponse& m)
{
    v = {
        { "op", m.op },
        { "id", m.id },
        { "service", m.service},
        { "values", m.values },
        { "result", m.result}
    };
}


void tag_invoke(value_from_tag t, value& v, const BridgeMessage& base)
{
    //std::cout << "WARN: tag_invoke on base 'BridgeMessage' called!\n";
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

    std::string Serialize(const Status& message)
    {
        return serialize( value_from(message) );
    }

    
    virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data)
    {
        auto parsed_json = parse(data);
        // auto message = value_to<SetStatusLevel>( parsed_json ); 
        return std::make_unique<SetStatusLevel>( value_to<SetStatusLevel>( parsed_json ) );
    }
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP