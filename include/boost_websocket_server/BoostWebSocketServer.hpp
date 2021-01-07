//
// Copyright (c) 2016-2019 Vinnie Falco (vinnie dot falco at gmail dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// Official repository: https://github.com/boostorg/beast
//

//------------------------------------------------------------------------------
//
// Example: WebSocket server, asynchronous
//
//------------------------------------------------------------------------------

#ifndef WEBSOCKETSERVER_HPP
#define WEBSOCKETSERVER_HPP

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ConnectionListener.hpp"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

//------------------------------------------------------------------------------

class WebSocketServer
{
    //list of threads that contain the IOC runtimes.
    std::vector<std::thread> v;

    const net::ip::address address;
    const uint16_t port; // 32bit gives shortening compiler warning.
    const uint8_t threads;

    StringMessageServer* dataHandler;

    public:

    WebSocketServer( const std::string addr, const uint16_t port, const uint8_t threads,  StringMessageServer* dataHandler)
    : address( net::ip::make_address(addr) )
    , port(port)
    , threads(std::max<int>(1, threads))
    , dataHandler(dataHandler)
    {
        net::io_context ioc{threads};

        std::make_shared<listener>(ioc, tcp::endpoint{address, port}, dataHandler )->run();

        v.reserve(threads - 1);
        // Plant a continuously running IOC run function in every thread.
        for(auto i = threads - 1; i > 0; --i)
            v.emplace_back(
            [&ioc]
            {
                ioc.run();
            });
        ioc.run();
    }

    bool send(std::string data)
    {
        return false;
    }
};

#endif //WEBSOCKETSERVER_HPP