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



    public:

    BridgeMessageServer(BridgeMessageSerializer& serializer)
    : serializer(serializer)
    {

    }

    void handleRequest(const std::string& request, const std::function<void(const std::string&&)>& serveResponse )
    {
        auto message = serializer.Deserialize(request);
        
        //// CHANGE TO VISITOR PATTERN
        //if(dynamic_cast<Advertise*>(&message))
        //{
        //
        //}

        serveResponse(std::move(serializer.Serialize( *(message.get()) ) ) );
    }
};

#endif