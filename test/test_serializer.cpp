#include "../include/boost_websocket_server/BridgeMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/BridgeMessages.hpp"

#include <iostream>

using namespace BridgeMessages;

int main(int argc, char const *argv[])
{
    auto serializer = BridgeMessageJsonSerializer();
    auto message = SetStatusLevel();

    std::cout << serializer.Serialize(message) << "\n";


    return EXIT_SUCCESS;
}
