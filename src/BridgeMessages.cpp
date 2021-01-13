#include "../include/boost_websocket_server/BridgeMessages.hpp"

// Implementation of the 'visit' functions for the bridgeMessages.
// NOTE that this cannot be placed inside the header as using the required forward declaration is a compiler error. 


struct SetStatusLevel::SetStatusLevel : public BridgeMessage
{    
    virtual std::unique_ptr<BridgeMessage> SetStatusLevel::getHandled(BridgeMessageHandler& handler)
    { return handler.HandleSetStatusLevel(*this); } 
};

struct Status::Status : public BridgeMessage
{    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleStatus(*this); } 
};

struct Authenticate::Authenticate : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAuthenticate(*this); } 
};


struct Advertise::Advertise : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAdvertise(*this); } 
};

struct Publish::Publish : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandlePublish(*this); } 
};

struct Subscribe::Subscribe : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleSubscribe(*this); } 
};

struct Unsubscribe::Unsubscribe : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleUnsubscribe(*this); } 
};

struct CallService::CallService : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleCallService(*this); } 
};

struct AdvertiseService::AdvertiseService : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleAdvertiseService(*this); } 
};

struct UnadvertiseService::UnadvertiseService : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleUnadvertiseService(*this); } 
};

struct ServiceResponse::ServiceResponse : public BridgeMessage
{
    std::unique_ptr<BridgeMessage> getHandled(BridgeMessageHandler& handler)
    { return handler.HandleServiceResponse(*this); } 
};
