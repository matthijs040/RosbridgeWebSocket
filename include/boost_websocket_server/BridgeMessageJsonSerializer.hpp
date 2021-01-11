#ifndef BRIDGEMESSAGEJSONSERIALIZER_HPP
#define BRIDGEMESSAGEJSONSERIALIZER_HPP

#include "BridgeMessages.hpp"
#include "json.hpp"

using nlohmann::json;

// https://github.com/nlohmann/json#basic-usage

namespace BridgeMessages 
{
    void to_json( json& j, const SetStatusLevel& m) 
    {
        // Using ternary operator for conditional assignment of optionally initialized/used parameters.
        // https://www.tutorialspoint.com/c-cplusplus-ternary-operator
        j = json{{"op", m.op}, {"id", m.id ? *m.id : "NULL" }, {"level", m.level} };
    }

    void from_json(const json& j, SetStatusLevel& m)
    {
        j.at("id").get_to(*m.id); // What am i supposed to do here??
        j.at("level").get_to(m.level);
    }
};


class BridgeMessageJsonSerializer
{
private:
    /* data */
public:
    BridgeMessageJsonSerializer(/* args */) {}
    ~BridgeMessageJsonSerializer() {}
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP