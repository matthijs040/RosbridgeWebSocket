#ifndef BRIDGEMESSAGEHANDLER_HPP
#define BRIDGEMESSAGEHANDLER_HPP

class BridgeMessageHandler;

// Include each-other? Problem?
#include "BridgeMessages.hpp"
#include <memory>

class BridgeMessageHandler
{
private:
    /* data */
public:
    BridgeMessageHandler(/* args */) {}
    ~BridgeMessageHandler() {}

    std::unique_ptr<BridgeMessage> HandleSetStatusLevel(const SetStatusLevel* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleStatus(const Status* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleAuthenticate(const Authenticate* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleAdvertise(const Advertise* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandlePublish(const Publish* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleSubscribe(const Subscribe* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleUnsubscribe(const Unsubscribe* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleCallService(const CallService* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleAdvertiseService(const AdvertiseService* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleUnadvertiseService(const UnadvertiseService* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BridgeMessage> HandleServiceResponse(const ServiceResponse* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }
    
};

#endif // BRIDGEMESSAGEHANDLER_HPP