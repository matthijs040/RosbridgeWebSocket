#ifndef BRIDGEMESSAGEHANDLER_HPP
#define BRIDGEMESSAGEHANDLER_HPP

#include <functional>           // std::function callback through which the handler can send responses.
#include <memory>               // std::unique_ptr
#include "BridgeMessages.hpp"   // The messages of the Bridge-Protocol

using namespace BridgeMessages;

/**
 * @brief Interface that specifies the functions required to handle the output of a BridgeMessageServer.
 */
class BridgeMessageHandler
{
public:
    virtual Status HandleSetStatusLevel     (const SetStatusLevel&      message) = 0;
    virtual Status HandleStatus             (const Status&              message) = 0;
    virtual Status HandleAuthenticate       (const Authenticate&        message) = 0;
    virtual Status HandleAdvertise          (const Advertise&           message) = 0; 
    virtual Status HandleUnadvertise        (const Unadvertise&         message) = 0; 
    virtual Status HandlePublish            (const Publish&             message) = 0;
    virtual Status HandleSubscribe          (const Subscribe&           message, const std::function<void(const Publish&)>& callback) = 0;
    virtual Status HandleUnsubscribe        (const Unsubscribe&         message) = 0;
    virtual Status HandleCallService        (const CallService&         message) = 0;
    virtual Status HandleAdvertiseService   (const AdvertiseService&    message, const std::function<void(const CallService&)>& callback) = 0;
    virtual Status HandleUnadvertiseService (const UnadvertiseService&  message) = 0;
    virtual Status HandleServiceResponse    (const ServiceResponse&     message) = 0;
    virtual std::unique_ptr<BridgeMessageHandler> copy() const = 0;
};

#endif // BRIDGEMESSAGEHANDLER_HPP