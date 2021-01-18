#ifndef ROSMESSAGEHANDLER_HPP
#define ROSMESSAGEHANDLER_HPP

#include "BridgeMessageHandler.hpp" // The interface this class complies with.
#include <iostream>                 // std::cout, std::cerr
#include <map>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>



enum STATUS
{
    NONE,
    ERROR,
    WARNING,
    INFO
};


class RosMessageHandler : public BridgeMessageHandler
{
private:

    STATUS current_status = STATUS::ERROR;
    Authenticate last_authentication;


    ros::NodeHandle n;
    std::map<std::string, ros::Publisher> publishers_by_topic;


public:
    virtual Status HandleSetStatusLevel(const SetStatusLevel &message)
    {
        if( message.level == "none")
        {
            current_status = STATUS::NONE;
            return Status( message.id, message.level, "set_level_ack");
        }
        else if(message.level == "error")
        {
            current_status = STATUS::ERROR;
            return Status( message.id, message.level, "set_level_ack");
        }
        else if( message.level == "warning")
        {
            current_status = STATUS::WARNING;
            return Status( message.id, message.level, "set_level_ack");
        }
        else if( message.level == "info")
        {
            current_status = STATUS::INFO;
            return Status( message.id, message.level, "set_level_ack");
        }
        else
        {
            if(current_status == (STATUS::ERROR | STATUS::WARNING | STATUS::INFO) )
            return Status(message.id, "error", "set_level_nack");
        }
        
    }

    virtual Status HandleStatus(const Status &message)
    {
        if(message.level == "info")
        { 
            std::cout << message.msg + "\n"; 
        }
        else if(message.level == "error")
        {
            std::cerr << message.msg + "\n";
        }
    }

    virtual Status HandleAuthenticate(const Authenticate &message)
    {
        return Status( std::string(),"warning", "auth_nack_not_enabled");
    }

    virtual Status HandleAdvertise(const Advertise& message)
    {
        if(message.type == "/nav_msgs/Odometry")
        {
            const auto pub = n.advertise<nav_msgs::Odometry>(message.topic, 1, true);
            publishers_by_topic.emplace( std::make_pair(message.topic, pub) );
            return Status(message.id, "info", "advertise_ack");
        }
        else
        {
            return Status(message.id, "error", "advertise_nack_invalid_topic");
        }

    }

    virtual Status HandleUnadvertise(const Unadvertise& message)
    {
        if(publishers_by_topic.erase(message.topic) > 0)
        {
            return Status(message.id, "info", "unadvertise_ack");
        }
        else
        {
            return Status(message.id, "error", "unadvertise_nack_not_advertised");
        }
    }

    virtual Status HandlePublish(const Publish &message)
    {
        return Status();
    }
    virtual Status HandleSubscribe(const Subscribe& message, const std::function<void(const Publish&)> &callback)
    {
        return Status();
    }
    virtual Status HandleUnsubscribe(const Unsubscribe &message)
    {
        return Status();
    }
    virtual Status HandleCallService(const CallService &message)
    {
        return Status();
    }
    virtual Status HandleAdvertiseService(const AdvertiseService &message, const std::function<void(const CallService &)> &callback)
    {
        return Status();
    }
    virtual Status HandleUnadvertiseService(const UnadvertiseService &message)
    {
        return Status();
    }
    virtual Status HandleServiceResponse(const ServiceResponse &message)
    {
        return Status();
    }
    virtual std::unique_ptr<BridgeMessageHandler> copy() const
    {
        //auto ret = RosMessageHandler(*this);
        return std::make_unique<RosMessageHandler>(RosMessageHandler(*this));
    }

    RosMessageHandler() {}
    ~RosMessageHandler() {}
};

#endif // ROSMESSAGEHANDLER_HPP