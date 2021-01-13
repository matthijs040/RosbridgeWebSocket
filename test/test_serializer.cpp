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

    /**
     * @brief Utility function that tells if a given JSON key-value-pair is contained within a given string. 
     * 
     * @param data as a json serialized string.
     * @param key of the json field.
     * @param value of the json field. 
     * @return true if the key-value are in the string in json format.
     * @return false otherwise
     */
    bool contains_keyvalue_pair(const std::string& data, const std::string& key, const std::string& value)
    {
        return data.find("\"" + key + "\":\"" + value +"\"") != std::string::npos;
    }

    bool contains(const std::string& data, const std::string& contains)
    {
        return data.find(contains) != std::string::npos;
    }

    void SetStatusLevel_serialized_preserving_all_fields()
    {
        // Given:
        auto message = SetStatusLevel();
        message.level = "error";
        message.id = std::make_unique<std::string>("id:10");

        // When:
        auto data = serializer.Serialize(message);

        // Then:
        assert(contains_keyvalue_pair(data, "op", message.op));
        assert(contains_keyvalue_pair(data, "level", message.level));
        assert(contains_keyvalue_pair(data, "id", *message.id));
    }

    void optional_field_serialized_as_empty()
    {
        auto message = Status();
        message.id = nullptr;

        auto data = serializer.Serialize(message);
        std::cout << data << '\n';
        assert(contains(data, "id"));
    }

    void Status_serialized_preserving_all_fields()
    {
        auto message = Status();
        message.level = "error";
        message.id = std::make_unique<std::string>("id:1");
        message.msg = std::string("some message");

        auto data = serializer.Serialize(message);
        
        assert(contains_keyvalue_pair(data, "op", message.op) );
        assert(contains_keyvalue_pair(data, "level", message.level) );
        assert(contains_keyvalue_pair(data, "id", *message.id) );
        assert(contains_keyvalue_pair(data, "msg", message.msg) );
    }



};

int main(int argc, char const *argv[])
{
    SerializerTest::SetStatusLevel_serialized_preserving_all_fields();
    SerializerTest::Status_serialized_preserving_all_fields();
    SerializerTest::optional_field_serialized_as_empty();


    return EXIT_SUCCESS;
}
