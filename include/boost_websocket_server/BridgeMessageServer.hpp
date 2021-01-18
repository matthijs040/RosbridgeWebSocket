#ifndef BRIDGEMESSAGESERVER_HPP
#define BRIDGEMESSAGESERVER_HPP

#include "StringMessageServer.hpp"
#include "BridgeMessageHandler.hpp"
#include "BridgeMessageSerializer.hpp"

#include <functional>


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
        
        serveResponse(std::move(serializer.Serialize( *(message.get()) ) ) );
    }

    virtual std::unique_ptr<StringMessageServer> copy() const 
    {
        return std::make_unique<BridgeMessageServer>( BridgeMessageServer(serializer, *handler ));
    }
};

#endif