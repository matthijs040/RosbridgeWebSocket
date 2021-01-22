#ifndef ROSMESSAGEHANDLER_HPP
#define ROSMESSAGEHANDLER_HPP

#include "RosMessageJsonSerializer.hpp" // Serializer
#include "BridgeMessageHandler.hpp" // The interface this class complies with.
#include <iostream>                 // std::cout, std::cerr
#include <map>

#include <ros/ros.h>
#include <ros/spinner.h>

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

    int copy_count = 0;

    
    std::unique_ptr<ros::NodeHandle> n;   // To defer initialization until after ros::init is called with a unique nodename.
    std::map<std::string, ros::Publisher> publishers_by_topic;
    std::map<std::string, ros::Subscriber> subscribers_by_topic;

    ros::AsyncSpinner spinner;

    /**
     * @brief Appends a new publisher for the given topic in the advertisemessage.
     * 
     * @tparam MessageType 
     * @param message 
     * @return Status 
     */
    template<typename MessageType>
    Status advertise_handled(const Advertise& message)
    {
        const auto pub = n->advertise<MessageType>(message.topic, 1, true);
        
        if( publishers_by_topic.emplace( message.topic, pub ).second ) 
            return Status(message.id, "info", "advertise_ack"); 
        
        return Status(message.id, "error", "advertise_nack_already_advertised");
    }

public:

    const RosMessageJsonSerializer& serializer;

    RosMessageHandler(const RosMessageJsonSerializer& serializer, const std::string& handler_name = "RosBridgeMessageHandler")
    : serializer(serializer)
    , publishers_by_topic()     // Is this default initialization?
    , subscribers_by_topic()
    , spinner( ros::AsyncSpinner(0) )
    {
        if(!ros::isStarted())
        {
            char* dummy = nullptr;
            std::string nodename = handler_name + std::to_string(copy_count);
            int lvalue = 0;
            ros::init( lvalue, &dummy, nodename, 0 );
        }

        n = std::make_unique<ros::NodeHandle>(ros::NodeHandle());

        if(spinner.canStart())  // If another spinner is already active, the callbacks registered here will already be called.
            spinner.start();
    }

    RosMessageHandler(const RosMessageHandler& other)
    : spinner(ros::AsyncSpinner(other.spinner)) // Asyncspinner might contain ros related resources. Should be deep copied.
    , serializer(other.serializer)              // Serializer does not contain state. it may be moved.
    {
        copy_count++; // Might be unessecary? ros runtime might only be initialized once?
        n = std::make_unique<ros::NodeHandle>(ros::NodeHandle());
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
            if( current_status == STATUS::ERROR 
             || current_status == STATUS::WARNING
             || current_status == STATUS::INFO )
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
        { return Status(message.id, "info", "unadvertise_ack"); }
        else
        { return Status(message.id, "error", "unadvertise_nack_not_advertised"); }
    }

    virtual Status HandlePublish(const Publish &message)
    {
        return Status();
    }
    virtual Status HandleSubscribe(const Subscribe& message, const std::function<void(const Publish&)> &callback)
    {
        if(subscribers_by_topic.count(message.topic))
        {
            return Status(message.id, "error", "subscribe_nack_already_subscribed");
        }
        try
        {
            auto sub = n->subscribe(message.topic,message.queue_length, 
                [this](const geometry_msgs::Vector3::ConstPtr msg){
                    serializer.Serialize()

                });
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        



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