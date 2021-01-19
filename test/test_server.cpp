#include "../include/boost_websocket_server/BridgeMessageServer.hpp"
#include "../include/boost_websocket_server/BoostWebSocketServer.hpp"
#include "../include/boost_websocket_server/BridgeMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/RosMessageHandler.hpp"

#include "../include/boost_websocket_server/BoostWebSocketClient.hpp"

#include "../include/boost_websocket_server/TestingUtils.hpp"

namespace test
{
    auto serializer = BridgeMessageJsonSerializer();
    auto messageHandler = RosMessageHandler();
    auto messageServer = BridgeMessageServer(serializer, messageHandler);

    void valid_advertise_returns_correct_acknowledge()
    {
        messageServer.handleRequest(
            "{\"op\":\"advertise\",\"topic\":\"/android/phone/GPS\",\"id\":\"id_string\",\"type\":\"/sensor_msgs/NavSatFix\"}", 
            [](const std::string& response) {
                assert( test::contains_keyvalue_pair( response, "op", "status") );
                assert( test::contains_keyvalue_pair( response, "level","info"));
                assert( test::contains_keyvalue_pair( response, "msg","advertise_ack"));
                assert( test::contains_keyvalue_pair( response, "id", "id_string") );
            } );
    }

    void set_level_to_error_returns_correct_acknowledge()
    {
        messageServer.handleRequest(
            "{\"op\":\"set_level\",\"id\":\"id_string\",\"level\":\"error\"}", 
            [](const std::string& response) {
                assert( test::contains_keyvalue_pair( response, "op", "status") );
                assert( test::contains_keyvalue_pair( response, "level","error"));
                assert( test::contains_keyvalue_pair( response, "msg","set_level_ack"));
                assert( test::contains_keyvalue_pair( response, "id", "id_string") );
            } 
        );
    }

    void set_level_to_warning_returns_correct_acknowledge()
    {
        messageServer.handleRequest(
            "{\"op\":\"set_level\",\"id\":\"id_string\",\"level\":\"warning\"}", 
            [](const std::string& response) {
                assert( test::contains_keyvalue_pair( response, "op", "status") );
                assert( test::contains_keyvalue_pair( response, "msg","set_level_ack"));
                assert( test::contains_keyvalue_pair( response, "level","warning"));
                assert( test::contains_keyvalue_pair( response, "id", "id_string") );
            } 
        );        
    }

    void set_level_to_info_returns_correct_acknowledge()
    {
        messageServer.handleRequest(
            "{\"op\":\"set_level\",\"id\":\"id_string\",\"level\":\"info\"}", 
            [](const std::string& response) {
                assert( test::contains_keyvalue_pair( response, "op", "status") );
                assert( test::contains_keyvalue_pair( response, "msg","set_level_ack"));
                assert( test::contains_keyvalue_pair( response, "level","info"));
                assert( test::contains_keyvalue_pair( response, "id", "id_string") );
            } 
        );        
    }

    void set_level_to_invalid_level_returns_nack()
    {
        messageServer.handleRequest(
            "{\"op\":\"set_level\",\"id\":\"id_string\",\"level\":\"invalid_level\"}", 
            [](const std::string& response) {
                assert( test::contains_keyvalue_pair( response, "op", "status") );
                assert( test::contains_keyvalue_pair( response, "msg","set_level_nack"));
                assert( test::contains_keyvalue_pair( response, "level","error"));
                assert( test::contains_keyvalue_pair( response, "id", "id_string") );
            } 
        );        
    }


    
};

int main(int argc, char const *argv[])
{
    test::valid_advertise_returns_correct_acknowledge();
    test::set_level_to_error_returns_correct_acknowledge();
    test::set_level_to_warning_returns_correct_acknowledge();
    test::set_level_to_info_returns_correct_acknowledge();
    test::set_level_to_invalid_level_returns_nack();

    return EXIT_SUCCESS;
}
