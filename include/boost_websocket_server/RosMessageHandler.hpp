#ifndef ROSMESSAGEHANDLER_HPP
#define ROSMESSAGEHANDLER_HPP

#include "BridgeMessageHandler.hpp" // The interface this class complies with.
#include <iostream>                 // std::cout, std::cerr
#include <map>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>



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
    int copy_count = 0;


    
    ros::NodeHandle* n = nullptr;   // To defer initialization until after ros::init is called with a unique nodename.
    std::map<std::string, ros::Publisher> publishers_by_topic;

    template<typename MessageType>
    Status advertise_handled(const Advertise& message)
    {
        const auto pub = n->advertise<MessageType>(message.topic, 1, true);
        publishers_by_topic.emplace( message.topic, pub );
        return Status(message.id, "info", "advertise_ack");
    }

public:

    RosMessageHandler(const std::string& handler_name = "RosBridgeMessageHandler") 
    {
        if(!ros::isStarted())
        {
            char* dummy = nullptr;
            std::string nodename = handler_name + std::to_string(copy_count);
            int lvalue = 0;
            ros::init( lvalue, &dummy, nodename, 0 );
        }

        n = new ros::NodeHandle() ;
    }

    RosMessageHandler(const RosMessageHandler& other)
    {
        copy_count++;
        n = new ros::NodeHandle(); // Deep copy the nodehandle to ensure that this raw pointer is not shared.
    }

    ~RosMessageHandler() {
        delete n;
        n = nullptr;
    }

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
        return Status(); // TODO: REMOVE THIS
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
        return Status(message.id, "info", "status_ack");
    }

    virtual Status HandleAuthenticate(const Authenticate &message)
    {
        return Status( std::string(),"warning", "auth_nack_not_enabled");
    }

    virtual Status HandleAdvertise(const Advertise& message)
    {
        if(message.type == "/nav_msgs/Odometry")
        { return advertise_handled<nav_msgs::Odometry>(message); }
        else if(message.type == "/sensor_msgs/NavSatFix")
        { return advertise_handled<sensor_msgs::NavSatFix>(message); }
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
        return std::make_unique<RosMessageHandler>(RosMessageHandler(*this));
    }


};

#endif // ROSMESSAGEHANDLER_HPP