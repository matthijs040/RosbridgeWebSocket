#ifndef BRIDGEMESSAGES_HPP
#define BRIDGEMESSAGES_HPP

#include <string>
#include <memory>

#include "RosMessages.hpp"

struct BridgeMessage
{
    const std::string op;
    virtual std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler) = 0;
};

struct SetStatusLevel : public BridgeMessage
{
    const std::string op = "set_level";
    std::string* id = nullptr;
    std::string level = "error";

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleSetStatusLevel(this); } 
};

struct Status : public BridgeMessage
{
    const std::string op = "status";
    std::string* id = nullptr;
    std::string level;
    std::string msg;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleStatus(this); } 
};

struct Authenticate : public BridgeMessage
{
    const std::string op = "auth";
    std::string mac;
    std::string client;
    std::string dest;
    std::string rand;
    int t;
    std::string level;
    int end;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAuthenticate(this); } 
};


struct Advertise : public BridgeMessage
{
    const std::string op = "advertise";
    std::string* id = nullptr;
    std::string topic;
    std::string type;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAdvertise(this); } 
};

struct Publish : public BridgeMessage
{
    const std::string op = "publish";
    std::string topic;
    RosMessage msg;

    
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandlePublish(this); } 
};

struct Subscribe : public BridgeMessage
{
    std::string  op = "subscribe";
    std::string* id = nullptr;
    std::string topic;
    std::string* type = nullptr;
    int* throttle_rate = nullptr;
    int* queue_length = nullptr;
    int* fragment_size = nullptr;
    std::string* compression = nullptr;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleSubscribe(this); } 
};

struct Unsubscribe : public BridgeMessage
{
    const std::string op = "unsubscribe";
    std::string* id = nullptr;
    std::string topic;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleUnsubscribe(this); } 
};

struct CallService : public BridgeMessage
{
    const std::string op = "call_service";
    std::string* id = nullptr;
    std::string service;
    std::string* args = nullptr; // Should be custom datatype? List<json>?
    int* fragment_size = nullptr;
    std::string* compression = nullptr;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleCallService(this); } 
};

struct AdvertiseService : public BridgeMessage
{
    const std::string op = "advertise_service";
    std::string type;
    std::string service;

    
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAdvertiseService(this); } 
};

struct UnadvertiseService : public BridgeMessage
{
    const std::string op = "unadvertise_service";
    std::string service;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleUnadvertiseService(this); } 
};

struct ServiceResponse : public BridgeMessage
{
    const std::string op = "service_response";
    std::string* id = nullptr;
    std::string service;
    std::string* values = nullptr; // Should be custom datatype? List<json>?
    bool result;

    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleServiceResponse(this); } 
};


#include "BridgeMessageHandler.hpp"


#endif