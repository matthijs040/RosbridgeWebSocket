#ifndef BOOSTWEBSOCKETCLIENT_HPP
#define BOOSTWEBSOCKETCLIENT_HPP

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <string>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

class BoostWebSocketClient
{

    private:

    // The io_context is required for all I/O
    net::io_context ioc;

    // These objects perform our I/O
    websocket::stream<tcp::socket> connection = websocket::stream<tcp::socket>(ioc);
    tcp::resolver resolver = tcp::resolver(ioc);

    public:

    BoostWebSocketClient()
    { }

    BoostWebSocketClient(BoostWebSocketClient&) = delete;

    // Connect to the server specifications the server was constructed with. 
    void connect(const std::string& address,const std::string& port, const std::string& client_name = "websocket-client")
    {
        // Look up the domain name
        auto results = resolver.resolve(address, port);

        auto ep = net::connect(connection.next_layer(), results);

        std::string uri = address + ':' + std::to_string(ep.port());

        connection.set_option(websocket::stream_base::decorator(
            [client_name](websocket::request_type& req)
            {
                req.set(http::field::user_agent,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        + " " + client_name);
            }));

        // Perform the websocket handshake
        connection.handshake(uri, "/");
    }

    void disconnect()
    {
       connection.close(websocket::close_code::normal);
    }

    /**
     * @brief Writes the given string over the given connection. 
     *          Returns the number of bytes written.
     * @param connection To write the data over. 
     * @param data To write over the connection.
     * @return size_t containing the number of bytes written.
     */
    size_t write(const std::string& data)
    {
        return connection.write(net::buffer(data));  
    }

    std::string blocking_read()
    {
        beast::flat_buffer buffer;
        connection.read(buffer);
        return std::string( beast::buffers_to_string(buffer.data()) ); // https://stackoverflow.com/questions/45761865/how-to-convert-boost-beast-multi-buffer-to-string
    }

};

#endif // BOOSTWEBSOCKETCLIENT_HPP