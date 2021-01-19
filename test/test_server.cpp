#include "../include/boost_websocket_server/BridgeMessageServer.hpp"
#include "../include/boost_websocket_server/BoostWebSocketServer.hpp"
#include "../include/boost_websocket_server/BridgeMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/RosMessageHandler.hpp"

#include "../include/boost_websocket_server/BoostWebSocketClient.hpp"

#include "../include/boost_websocket_server/ChronoBenchmarker.hpp"

void function(const std::string& response)
{
    std::cout << "response served: " << response << "\n";
}

int main(int argc, char const *argv[])
{

    // const std::string addr ="172.30.57.55";
    // const int port = 10000;
    // const int threads = 3;

    auto serializer = BridgeMessageJsonSerializer();
    auto messageHandler = RosMessageHandler();
    auto messageServer = BridgeMessageServer(serializer, messageHandler);
    //auto server = BoostWebSocketServer(addr, port, threads, messageServer); Actual server not set up.

    benchmark::run(
        [&messageServer]()
        {
            messageServer.handleRequest("{\"op\":\"advertise\",\"topic\":\"/android/phone/GPS\",\"id\":\"\",\"type\":\"/sensor_msgs/NavSatFix\"}", function );
        },
        true
    );

    return EXIT_SUCCESS;
}
