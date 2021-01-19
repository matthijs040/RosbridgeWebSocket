#ifndef BRIDGEMESSAGESERVER_HPP
#define BRIDGEMESSAGESERVER_HPP

#include "StringMessageServer.hpp"      // Interface
#include "BridgeMessageHandler.hpp"     // this::handler
#include "BridgeMessageSerializer.hpp"  // this::serializer

#include <functional>   // std::function
#include <iostream>     // std::cerr

class BridgeMessageServer : public StringMessageServer
{
    private:
    BridgeMessageSerializer& serializer;
    std::unique_ptr<BridgeMessageHandler> handler;

    public:

    BridgeMessageServer(BridgeMessageSerializer& serializer, BridgeMessageHandler& handler)
    : serializer(serializer)
    , handler(handler.copy() )
    {

    }

    virtual void handleRequest(const std::string& request, const std::function<void(const std::string&)>& serveResponse ) override
    {
        auto message = serializer.Deserialize(request);

        if( message )
        {
            if(auto msg = dynamic_cast<const SetStatusLevel*>(message.get()))
            { serveResponse(std::move(serializer.Serialize( handler->HandleSetStatusLevel(*msg) ) ) ); }
            else if (auto msg = dynamic_cast<const Status*>(message.get()))
            { serveResponse(std::move(serializer.Serialize( handler->HandleStatus(*msg) ) ) ); }
            else if (auto msg = dynamic_cast<const Authenticate*>(message.get()))
            { serveResponse(std::move(serializer.Serialize( handler->HandleAuthenticate(*msg) ) ) ); }
            else if (auto msg = dynamic_cast<const Advertise*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleAdvertise(*msg) ) ) );  }
            else if (auto msg = dynamic_cast<const Unadvertise*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleUnadvertise(*msg) ) ) );  }
            else if (auto msg = dynamic_cast<const Publish*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandlePublish(*msg) ) ) );  }
            else if (auto msg = dynamic_cast<const Subscribe*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleSubscribe(*msg,  
                [this, serveResponse](const Publish& reaction) { serveResponse( this->serializer.Serialize(reaction) ); } ) ) ) ); 
            }
            else if (auto msg = dynamic_cast<const Unsubscribe*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleUnsubscribe(*msg) ) ) );  }
            else if (auto msg = dynamic_cast<const AdvertiseService*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleAdvertiseService(*msg ,
                [this, serveResponse](const CallService& reaction) { serveResponse( this->serializer.Serialize(reaction) ); }) ) ) );  
            }
            else if (auto msg = dynamic_cast<const UnadvertiseService*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleUnadvertiseService(*msg) ) ) );  }
            else if (auto msg = dynamic_cast<const CallService*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleCallService(*msg) ) ) );  }
            else if (auto msg = dynamic_cast<const ServiceResponse*>(message.get()))
            {  serveResponse(std::move(serializer.Serialize( handler->HandleServiceResponse(*msg) ) ) );  }   
            else
            {
                std::cerr << "unimlemented child of bridgemessage parsed.\n";
            }        
        }
        // serveResponse(std::move(serializer.Serialize( *(message.get()) ) ) );
    }

    virtual std::unique_ptr<StringMessageServer> copy() const 
    {
        return std::make_unique<BridgeMessageServer>( BridgeMessageServer(serializer, *handler ));
    }
};

#endif