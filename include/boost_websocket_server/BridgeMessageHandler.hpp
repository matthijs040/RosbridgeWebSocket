#ifndef BRIDGEMESSAGEHANDLER_HPP
#define BRIDGEMESSAGEHANDLER_HPP

class BridgeMessageHandler;     // Declaration required by BridgeMessages.

#include "BridgeMessages.hpp"   // The messages of the Bridge-Protocol
#include <memory>               // std::unique_ptr

/**
 * @brief Interface that specifies the functions required to handle the output of a BridgeMessageServer.
 * This is an implementation of the Visitor pattern where this handler is the visitor.
 * looking at a message and acts according to its data.
 */
class BridgeMessageHandler
{
public:
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleSetStatusLevel     (const BridgeMessages::SetStatusLevel&      message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleStatus             (const BridgeMessages::Status&              message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleAuthenticate       (const BridgeMessages::Authenticate&        message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleAdvertise          (const BridgeMessages::Advertise&           message) = 0; 
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandlePublish            (const BridgeMessages::Publish&             message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleSubscribe          (const BridgeMessages::Subscribe&           message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleUnsubscribe        (const BridgeMessages::Unsubscribe&         message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleCallService        (const BridgeMessages::CallService&         message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleAdvertiseService   (const BridgeMessages::AdvertiseService&    message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleUnadvertiseService (const BridgeMessages::UnadvertiseService&  message) = 0;
    virtual std::unique_ptr< BridgeMessages::BridgeMessage > HandleServiceResponse    (const BridgeMessages::ServiceResponse&     message) = 0;
};

#endif // BRIDGEMESSAGEHANDLER_HPP