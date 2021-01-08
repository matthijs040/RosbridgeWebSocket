#ifndef BRIDGEMESSAGEHANDLER_HPP
#define BRIDGEMESSAGEHANDLER_HPP

// Include each-other? Problem?
#include "BridgeMessages.hpp"
#include <memory>

using BaseMessage = BridgeMessage;

class BridgeMessageHandler
{
private:
    /* data */
public:
    BridgeMessageHandler(/* args */) {}
    ~BridgeMessageHandler() {}

    std::unique_ptr<BaseMessage> HandleSetStatusLevel(const SetStatusLevel* message)
    {
        auto ret = SetStatusLevel();
        return std::make_unique<SetStatusLevel>(ret); 
    }

    std::unique_ptr<BaseMessage> HandleStatus(const Status* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleAuthenticate(const Authenticate* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleAdvertise(const Advertise* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandlePublish(const Publish* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleSubscribe(const Subscribe* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleUnsubscribe(const Unsubscribe* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleCallService(const CallService* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleAdvertiseService(const AdvertiseService* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleUnadvertiseService(const UnadvertiseService* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }

    std::unique_ptr<BaseMessage> HandleServiceResponse(const ServiceResponse* message)
    { return std::make_unique<SetStatusLevel>(SetStatusLevel()); }
    
};

#endif // BRIDGEMESSAGEHANDLER_HPP