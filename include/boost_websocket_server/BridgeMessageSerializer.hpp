#ifndef BRIDGEMESSAGESERIALIZER_HPP
#define BRIDGEMESSAGESERIALIZER_HPP

#include <string> 
#include "BridgeMessages.hpp"

class BridgeMessageSerializer
{
    public:
    virtual std::string Serialize(const BridgeMessage& data) = 0;
    virtual BridgeMessage Deserialize(const std::string& data) = 0;
};


#endif // BRIDGEMESSAGESERIALIZER_HPP