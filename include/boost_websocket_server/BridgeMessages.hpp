#ifndef BRIDGEMESSAGES_HPP
#define BRIDGEMESSAGES_HPP

#include <string>
#include "RosMessages.hpp"

struct BridgeMessage
{
    const std::string op;
};

struct SetStatusLevel : public BridgeMessage
{
    const std::string op = "set_level";
    const std::string* id = nullptr;
    const std::string level = "error";
};

struct Status : public BridgeMessage
{
    const std::string op = "status";
    const std::string* id = nullptr;
    const std::string level;
    const std::string msg;
};

struct Authenticate : public BridgeMessage
{
    const std::string op = "auth";
    const std::string mac;
    const std::string client;
    const std::string dest;
    const std::string rand;
    const int t;
    const std::string level;
    const int end;
};


struct Advertise : public BridgeMessage
{
    const std::string op = "advertise";
    const std::string* id = nullptr;
    const std::string topic;
    const std::string type;
};

struct Publish : public BridgeMessage
{
    const std::string op = "publish";
    const std::string topic;
    const RosMessage msg;
};

struct Subscribe : public BridgeMessage
{
    const std::string  op = "subscribe";
    const std::string* id = nullptr;
    const std::string topic;
    const std::string* type = nullptr;
    const int* throttle_rate = nullptr;
    const int* queue_length = nullptr;
    const int* fragment_size = nullptr;
    const std::string* compression = nullptr;
};

struct Unsubscribe : public BridgeMessage
{
    const std::string op = "unsubscribe";
    const std::string* id = nullptr;
    const std::string topic;
};

struct CallService : public BridgeMessage
{
    const std::string op = "call_service";
    const std::string* id = nullptr;
    const std::string service;
    const std::string* args = nullptr; // Should be custom datatype? List<json>?
    const int* fragment_size = nullptr;
    const std::string* compression = nullptr;
};

struct AdvertiseService : public BridgeMessage
{
    const std::string op = "advertise_service";
    const std::string type;
    const std::string service;
};

struct UnadvertiseService : public BridgeMessage
{
    const std::string op = "unadvertise_service";
    const std::string service;
};

struct ServiceResponse : public BridgeMessage
{
    const std::string op = "service_response";
    const std::string* id = nullptr;
    const std::string service;
    const std::string* values = nullptr; // Should be custom datatype? List<json>?
    const bool result;
};

#endif