#ifndef ECHOWITHIDSERVER_HPP
#define ECHOWITHIDSERVER_HPP

#include "StringMessageServer.hpp"
#include <map>
#include <vector>
#include <iostream>

class Handler
{

};

/**
 * @brief implementation of messageserver to test handling multiple requestgivers through a lookup table.
 */
class EchoWithIDServer : public StringMessageServer
{
private:

    std::map<size_t, Handler> lookup;

public:
    EchoWithIDServer() = default;
    ~EchoWithIDServer() = default;

    virtual void handleRequest(std::string&& request, const std::function<void(std::string&&)>& serveResponse )
    {
        serveResponse(std::move(request));

        auto result = lookup.find( (size_t)&serveResponse );
        if(result != lookup.end())
        {
            
        }
    }
};

#endif