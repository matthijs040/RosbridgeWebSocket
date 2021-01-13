#ifndef BRIDGEMESSAGEHANDLER_HPP
#define BRIDGEMESSAGEHANDLER_HPP

class BridgeMessageHandler;     // Declaration required by BridgeMessages.

// Temporary declarations required by BridgeMessageHandler.
namespace BridgeMessages{
struct BridgeMessage; 
struct SetStatusLevel;
struct Status;
struct Authenticate;
struct Advertise;
struct Publish;
struct Subscribe;
struct Unsubscribe;
struct CallService;
struct AdvertiseService;
struct UnadvertiseService;
struct ServiceResponse;
};

#include <memory>               // std::unique_ptr
#include "BridgeMessages.hpp"   // The messages of the Bridge-Protocol

using namespace BridgeMessages;

/**
 * @brief Interface that specifies the functions required to handle the output of a BridgeMessageServer.
 * This is an implementation of the Visitor pattern where this handler is the visitor.
 * looking at a message and acts according to its data.
 */
class BridgeMessageHandler
{
public:
    virtual std::unique_ptr< BridgeMessage > HandleSetStatusLevel     (const SetStatusLevel&      message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleStatus             (const Status&              message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleAuthenticate       (const Authenticate&        message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleAdvertise          (const Advertise&           message) = 0; 
    virtual std::unique_ptr< BridgeMessage > HandlePublish            (const Publish&             message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleSubscribe          (const Subscribe&           message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleUnsubscribe        (const Unsubscribe&         message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleCallService        (const CallService&         message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleAdvertiseService   (const AdvertiseService&    message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleUnadvertiseService (const UnadvertiseService&  message) = 0;
    virtual std::unique_ptr< BridgeMessage > HandleServiceResponse    (const ServiceResponse&     message) = 0;
};

#endif // BRIDGEMESSAGEHANDLER_HPP