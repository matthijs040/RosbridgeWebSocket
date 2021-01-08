#ifndef STRINGMESSAGESERVER_HPP
#define STRINGMESSAGESERVER_HPP

#include <string>
#include <functional>

class StringMessageServer
{
    public: 
    /**
     * @brief function specification for submitting a request to the main server handling logic. 
     * With a callback through which a reaction can be sent back.
     * 
     * @param request for some behavior to occur or behavior to take place.
     * @param serveResponse function through which a possible response can be served.
     */
    virtual void handleRequest(std::string&& request, const std::function<void(std::string&&)>& serveResponse ) = 0;

    /**
     * @brief A copy function that returns a StringMessageServer with the same behaviors that the caller object had.
     * 
     * @return StringMessageServer&& An ownership transferred instance of a new StringMessageServer 
     */
    virtual std::unique_ptr<StringMessageServer> copy() const = 0;
};


#endif
