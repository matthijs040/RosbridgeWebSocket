#ifndef ECHOSTRINGMESSAGESERVER_HPP
#define ECHOSTRINGMESSAGESERVER_HPP

#include "StringMessageServer.hpp"
#include <memory>

/**
 * @brief minimal implementation of the StringMessageServer interface.
 * Echoes the received data back through the response function.
 */
class EchoStringMessageServer : public StringMessageServer
{
private:
public:
    EchoStringMessageServer() = default;
    ~EchoStringMessageServer() = default;

    virtual void handleRequest(const std::string& request, const std::function<void(const std::string&)>& serveResponse )
    {
        serveResponse(std::move(request));
    }

    virtual std::unique_ptr<StringMessageServer> copy() const
    {
        auto ret = EchoStringMessageServer();
        return std::make_unique<EchoStringMessageServer>(ret);
    }
};

#endif