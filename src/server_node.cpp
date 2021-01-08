#include <ros/ros.h>
#include "../include/boost_websocket_server/BoostWebSocketServer.hpp"
#include "../include/boost_websocket_server/EchoStringMessageServer.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "boost_websocket_rosbridge_server_node");

    if (argc != 4)
    {
        std::cerr <<
            "Usage: " << argv[0] << " <address> <port> <threads>\n" <<
            "Example:\n" <<
            "    " << argv[0] << " 0.0.0.0 8080 1\n";
        return EXIT_FAILURE;
    }

    auto echoReponse = EchoStringMessageServer();

    auto wsServer = WebSocketServer( std::string(argv[1]), std::stoi(argv[2]), std::stoi(argv[3]), echoReponse );


    return EXIT_SUCCESS;
}
