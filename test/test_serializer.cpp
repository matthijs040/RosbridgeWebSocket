#include <boost/json/src.hpp>

#include "../include/boost_websocket_server/BridgeMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/BridgeMessages.hpp"

#include <iostream>

int main(int argc, char const *argv[])
{
    auto serializer = BridgeMessageJsonSerializer();
    auto message = SetStatusLevel();

    std::cout << serializer.Serialize(message) << "\n";


    return EXIT_SUCCESS;
}
