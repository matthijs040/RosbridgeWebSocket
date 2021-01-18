#ifndef BRIDGEMESSAGES_HPP
#define BRIDGEMESSAGES_HPP

#include <string>                   // std::string
#include <memory>                   // std::unique_ptr.

namespace BridgeMessages {

struct BridgeMessage
{
    const std::string op;
    virtual ~BridgeMessage() {};
};

struct SetStatusLevel : public BridgeMessage
{
    const std::string op = "set_level";
    std::string id = std::string();
    std::string level = "error";

    bool operator==(const SetStatusLevel& p)
    { return ( op == p.op && id == p.id && level == p.level );}
};

struct Status : public BridgeMessage
{
    const std::string op = "status";
    
    std::string id = std::string();
    std::string level= "error";
    std::string msg = std::string();

    // Status(   std::string id    = std::string()
    //         , std::string level = std::string()
    //         , std::string msg   = std::string())
    // : id(id), level(level), msg(msg) {}

    bool operator==(const Status& p)
    { return ( op == p.op && id == p.id && level == p.level && msg == p.msg ); }
};

struct Authenticate : public BridgeMessage
{
    const std::string op = "auth";
    std::string mac = std::string();
    std::string client = std::string();
    std::string dest = std::string();
    std::string rand = std::string();
    int t = 0;
    std::string level = std::string();
    int end = 0;

    bool operator==(const Authenticate& p)
    { return ( op == p.op && mac == p.mac && client == p.client && dest == p.dest && rand == p.rand && t == p.t && level == p.level && end == p.end);}

};

struct Advertise : public BridgeMessage
{
    const std::string op = "advertise";
    std::string id = std::string();
    std::string topic = std::string();
    std::string type = std::string();

    bool operator==(const Advertise& p)
    { return ( op == p.op && id == p.id && topic == p.topic && type == p.type ); }
};

struct Unadvertise : public BridgeMessage
{
    const std::string op = "unadvertise";
    std::string id = std::string();
    std::string topic = std::string();

    bool operator==(const Unadvertise& p)
    { return ( op == p.op && id == p.id && topic == p.topic ); }
};

struct Publish : public BridgeMessage
{
    const std::string op = "publish";
    std::string topic = std::string();
    std::string msg = std::string();
};

struct Subscribe : public BridgeMessage
{
    const std::string  op = "subscribe";
    std::string id = std::string();
    std::string topic = std::string();
    std::string type = std::string();
    int throttle_rate = 0;
    int queue_length = 0;
    int fragment_size = 0;
    std::string compression = std::string();
};

struct Unsubscribe : public BridgeMessage
{
    const std::string op = "unsubscribe";
    std::string id = std::string();
    std::string topic = std::string();
};

struct CallService : public BridgeMessage
{
    const std::string op = "call_service";
    std::string id = std::string();
    std::string service = std::string();
    std::string args = std::string(); // Should be custom datatype? List<json>?
    int fragment_size = 0;
    std::string compression = std::string();

};

struct AdvertiseService : public BridgeMessage
{
    const std::string op = "advertise_service";
    std::string type = std::string();
    std::string service = std::string(); 
};

struct UnadvertiseService : public BridgeMessage
{
    const std::string op = "unadvertise_service";
    std::string service = std::string();
};

struct ServiceResponse : public BridgeMessage
{
    const std::string op = "service_response";
    std::string id = std::string();
    std::string service = std::string();
    std::string values = std::string(); // Should be custom datatype? List<json>?
    bool result = false;

};

}; // namespace BridgeMessages

#endif



