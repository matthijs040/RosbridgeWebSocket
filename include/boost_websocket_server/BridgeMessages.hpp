#ifndef BRIDGEMESSAGES_HPP
#define BRIDGEMESSAGES_HPP

#include <string>                   // std::string
#include <memory>                   // std::unique_ptr.
// #include "BridgeMessageHandler.hpp" // BridgeMessageHandler&
#include "RosMessages.hpp"          // Supported RosMessage payload. (Could be made templated / generic?)

namespace BridgeMessages {

struct BridgeMessage
{
    const std::string op;
    virtual ~BridgeMessage() {};
};

struct SetStatusLevel : public BridgeMessage
{
    const std::string op = "set_level";
    std::unique_ptr<std::string> id = nullptr;
    std::string level = "error";

    SetStatusLevel() {}; 
    SetStatusLevel(const SetStatusLevel& og) 
    : id( og.id.get() ? std::make_unique<std::string>( *(og.id.get()) ) : nullptr )
    , level(og.level)
    {}

    virtual ~SetStatusLevel() {};
};

struct Status : public BridgeMessage
{
    const std::string op = "status";
    std::string* id = nullptr;
    std::string level = std::string();
    std::string msg = std::string();

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

};

struct Advertise : public BridgeMessage
{
    const std::string op = "advertise";
    std::string* id = nullptr;
    std::string topic = std::string();
    std::string type = std::string();

};

struct Unadvertise : public BridgeMessage
{
    const std::string op = "unadvertise";
    std::unique_ptr<std::string> id = nullptr;
    std::string topic = std::string();
};

struct Publish : public BridgeMessage
{
    const std::string op = "publish";
    std::string topic = std::string();
    RosMessage* msg = nullptr;

};

struct Subscribe : public BridgeMessage
{
    std::string  op = "subscribe";
    std::string* id = nullptr;
    std::string topic = std::string();
    std::string* type = nullptr;
    int* throttle_rate = nullptr;
    int* queue_length = nullptr;
    int* fragment_size = nullptr;
    std::string* compression = nullptr;

};

struct Unsubscribe : public BridgeMessage
{
    const std::string op = "unsubscribe";
    std::string* id = nullptr;
    std::string topic = std::string();

};

struct CallService : public BridgeMessage
{
    const std::string op = "call_service";
    std::string* id = nullptr;
    std::string service = std::string();
    std::string* args = nullptr; // Should be custom datatype? List<json>?
    int* fragment_size = nullptr;
    std::string* compression = nullptr;

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
    std::string* id = nullptr;
    std::string service = std::string();
    std::string* values = nullptr; // Should be custom datatype? List<json>?
    bool result = false;

};

};

#endif



