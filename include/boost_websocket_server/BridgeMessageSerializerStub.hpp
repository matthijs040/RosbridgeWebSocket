#ifndef BRIDGEMESSAGESERIALIZERSTUB_HPP
#define BRIDGEMESSAGESERIALIZERSTUB_HPP

#include "BridgeMessageSerializer.hpp"
#include "json.hpp"

using namespace BridgeMessages;


class BridgeMessageSerializerStub : public BridgeMessageSerializer
{
private:
    /* data */
public:
    BridgeMessageSerializerStub() = default;
    ~BridgeMessageSerializerStub() = default;

    virtual std::string Serialize(const BridgeMessage& data)
    {
        return std::string("using the stub.");
    }

    virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data)
    {
        return std::make_unique<BridgeMessage>(SetStatusLevel()); 
    }
};

#endif