#include "../include/boost_websocket_server/BridgeMessages.hpp"

// Implementation of the 'visit' functions for the bridgeMessages.
// NOTE that this cannot be placed inside the header as using the required forward declaration is a compiler error. 

struct SetStatusLevel : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleSetStatusLevel(*this); } 
};

struct Status : public BridgeMessage
{    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleStatus(*this); } 
};

struct Authenticate : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAuthenticate(*this); } 
};


struct Advertise : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAdvertise(*this); } 
};

struct Publish : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandlePublish(*this); } 
};

struct Subscribe : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleSubscribe(*this); } 
};

struct Unsubscribe : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleUnsubscribe(*this); } 
};

struct CallService : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleCallService(*this); } 
};

struct AdvertiseService : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAdvertiseService(*this); } 
};

struct UnadvertiseService : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleUnadvertiseService(*this); } 
};

struct ServiceResponse : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleServiceResponse(*this); } 
};