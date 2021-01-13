#include <boost/json/src.hpp>

#include "../include/boost_websocket_server/BridgeMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/BridgeMessages.hpp"

#include <iostream>

/**
 * @brief Namespace containing all tests for the BridgeMessageJsonSerializer class.
 */
namespace SerializerTest
{

    BridgeMessageJsonSerializer serializer = BridgeMessageJsonSerializer();

    // test
    bool serialization_preserves_keyvalue_pair(const BridgeMessage& message, const std::string& key, const std::string& value)
    {
        auto data = serializer.Serialize(message);
        return data.find("\"" + key + "\":\"" + value +"\"") != std::string::npos;
    }

    void can_serialize_SetStatusLevel()
    {
        // Given:
        auto message = SetStatusLevel();
        message.level = "error";
        message.id = std::make_unique<std::string>("id:10");

        // Then:
        assert(serialization_preserves_keyvalue_pair(message, "op", message.op));
        assert(serialization_preserves_keyvalue_pair(message, "level", message.level));
        assert(serialization_preserves_keyvalue_pair(message, "id", *message.id));
    }

    void can_serialize_Status()
    {
        auto message = Status();
        message.level = "error";
        message.id = new std::string("id:1");
        message.msg = std::string("some message");
        
        assert(serialization_preserves_keyvalue_pair(message, "op", message.op) );
    }



};

int main(int argc, char const *argv[])
{
    SerializerTest::can_serialize_SetStatusLevel();
    SerializerTest::can_serialize_Status();


    return EXIT_SUCCESS;
}
