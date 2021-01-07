#include "../include/boost_websocket_server/BoostWebSocketClient.hpp"

// Sends a WebSocket message and prints the response
int main(int argc, char** argv)
{
    try
    {
        // Check command line arguments.
        if(argc != 4)
        {
            std::cerr <<
                "Usage: "<< argv[0] << " <host> <port> <text>\n" <<
                "Example:\n" <<
                "    "<< argv[0] << " echo.websocket.org 80 \"Hello, world!\"\n";
            return EXIT_FAILURE;
        }

        BoostWebSocketClient ws{};
        std::string host = argv[1];
        auto const  port = argv[2];
        auto const  text = argv[3];

        ws.connect(host, port);
        ws.write(text);
        auto response = ws.blocking_read();
        ws.disconnect();

        std::cout << response << std::endl;
    }
    catch(std::exception const& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}