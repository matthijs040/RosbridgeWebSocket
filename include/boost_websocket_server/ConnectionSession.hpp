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

#ifndef CONNECTIONSESSION_HPP
#define CONNECTIONSESSION_HPP

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

#include "BridgeMessageServer.hpp"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
//------------------------------------------------------------------------------

class session : public std::enable_shared_from_this<session>
{

    websocket::stream<beast::tcp_stream> ws_;
    std::unique_ptr<StringMessageServer> dataHandler;
    std::function<void(const std::string&)> callback = std::bind(&session::do_write, this, std::placeholders::_1);
    beast::flat_buffer buffer_;

    void fail(beast::error_code ec, char const* what)
{
    std::cerr << what << ": " << ec.message() << "\n";
}


public:
    // Take ownership of the socket
    // Can optionally take a datahandler object to forward received payloads and callback function to.
    explicit session(tcp::socket&& socket, const StringMessageServer& dataHandler)
        : ws_(std::move(socket))
        , dataHandler( dataHandler.copy() )
    {
    }

    // Get on the correct executor
    void run()
    {
        // We need to be executing within a strand to perform async operations
        // on the I/O objects in this session. Although not strictly necessary
        // for single-threaded contexts, this example code is written to be
        // thread-safe by default.
        net::dispatch(ws_.get_executor(),
            beast::bind_front_handler(
                &session::on_run,
                shared_from_this()));
    }

    // Start the asynchronous operation
    void on_run()
    {
        // Set suggested timeout settings for the websocket
        ws_.set_option(
            websocket::stream_base::timeout::suggested(
                beast::role_type::server));

        // Set a decorator to change the Server of the handshake
        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-server-async");
            }));
        // Accept the websocket handshake
        ws_.async_accept(
            beast::bind_front_handler(
                &session::on_accept,
                shared_from_this()));
    }

    void on_accept(beast::error_code ec)
    {
        if(ec)
            return fail(ec, "accept");

        // Read a message
        do_read();
    }

    void do_read()
    {
        // Read a message into our buffer
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(
                &session::on_read,
                shared_from_this()));
    }

    void do_write(const std::string& data)
    {
        auto dataBuf = buffer_.prepare(data.size());            // Get the writable part of the transmission buffer.
        memcpy(dataBuf.data(), data.data(), dataBuf.size());    // Copy the payloiad into the writable buffer.
        buffer_.commit(dataBuf.size());                         // Commit the written data to the transmission buffer.

        ws_.async_write(
            buffer_.data(),                                     // Transmit the new written payload.
            beast::bind_front_handler(
                &session::on_write,
                shared_from_this()));
    }

    void on_read(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        // This indicates that the session was closed
        if(ec == websocket::error::closed)
            return;

        if(ec)
            fail(ec, "read");

        // Use the datahandler to send read bytes as string to.
        // Provide the write function as a callback that the handler can optionally send a response through.
        ws_.text(ws_.got_text());

        // https://stackoverflow.com/questions/7582546/using-generic-stdfunction-objects-with-member-functions-in-one-class
        // using namespace std::placeholders;

        dataHandler->handleRequest(beast::buffers_to_string(buffer_.data()), callback);
        // ws_.async_write(
        //     buffer_.data(),
        //     beast::bind_front_handler(
        //         &session::on_write,
        //         shared_from_this()));
        


    }

    void on_write(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if(ec)
            return fail(ec, "write");

        // Clear the buffer
        buffer_.consume(buffer_.size());

        // Do another read
        do_read();
    }
};

#endif // CONNECTIONSESSION_HPP 