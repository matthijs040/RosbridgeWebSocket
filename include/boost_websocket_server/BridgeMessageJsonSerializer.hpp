#ifndef BRIDGEMESSAGEJSONSERIALIZER_HPP
#define BRIDGEMESSAGEJSONSERIALIZER_HPP

#include "BridgeMessageSerializer.hpp"  // Interface to implement.
#include "BridgeMessages.hpp"           // Messages to parse.

#include <json/json.h>

#include "json.hpp"                     // json parser, single header release: https://github.com/nlohmann/json
using nlohmann::json;   

template<typename T>
T* tryAt_optional_value(const json& j, const std::string name)
{
    try
    {
        j.at(name);

        auto val = new T();
        j.at(name).get_to(val);
        return val;
    }
    catch(const std::out_of_range& e) 
    {
        return nullptr;
    }
    
}

// method used: https://github.com/nlohmann/json#basic-usage
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
        // Check if ID exists.
        m.id = tryAt_optional_value<std::string>(j, "id");
        j.at("level").get_to(m.level);
    }
};


class BridgeMessageJsonSerializer : public BridgeMessageSerializer
{
private:
    /* data */
public:
    BridgeMessageJsonSerializer(/* args */) {}
    ~BridgeMessageJsonSerializer() {}

    /**
     * @brief Serializes a given datatype that has to be of subset BridgeMessages
     * 
     * @tparam BridgeMessage 
     * @param message 
     * @return std::string 
     */
    template<typename BridgeMessage>
    std::string Serialize(BridgeMessage& message)
    {
        return json(message).dump();
    }

    
    std::unique_ptr<BridgeMessage> Deserialize(std::string data)
    {
        from_json(json(data), data);
    }
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP