#ifndef BRIDGEMESSAGEJSONSERIALIZER_HPP
#define BRIDGEMESSAGEJSONSERIALIZER_HPP

#include "BridgeMessageSerializer.hpp"  // Interface to implement.
#include "BridgeMessages.hpp"           // Messages to parse.
#include <boost/json.hpp>

// https://www.boost.org/doc/libs/develop/libs/json/doc/html/json/quick_look.html#json.quick_look.value_conversion
// Function overloads for serializing the BridgeMessage types with boost::json.

using namespace boost::json;

namespace BridgeMessages {

    // from: https://www.boost.org/doc/libs/develop/libs/json/doc/html/json/quick_look.html#json.quick_look.value_conversion
    // This helper function deduces the type and assigns the value with the matching key
    template<class T>
    void extract( object const& obj, T& t, string_view key )
    {
        t = value_to<T>( obj.at( key ) );
    }

    SetStatusLevel tag_invoke( value_to_tag< SetStatusLevel >, const value& v )
    {
        SetStatusLevel ret;
        const object& obj = v.as_object();
        extract( obj, ret.op, "op" );
        extract( obj, ret.level, "level" );
        extract( obj, ret.id, "id" ); // NOTE: Assumes that library handles pointers as optional params.

        return ret;
    }


    void tag_invoke( value_from_tag, value& v, const SetStatusLevel& m )
    {
        v = {
            { "op", m.op },
            { "level", m.level },
            { "id", m.id}
        };  
    }
};

class BridgeMessageJsonSerializer : public BridgeMessageSerializer
{
private:
    
public:
    BridgeMessageJsonSerializer(/* args */) {}
    ~BridgeMessageJsonSerializer() {}

    // virtual std::string Serialize(const BridgeMessage& data) = 0;
    // virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data) = 0;

    virtual std::string Serialize(const BridgeMessage& message)
    {
        return serialize( value_from(message) );
    }

    
    virtual std::unique_ptr<BridgeMessage> Deserialize(const std::string& data)
    {
        return std::make_unique( value_to<BridgeMessage>( parse(data) ) );
    }
};


#endif // BRIDGEMESSAGEJSONSERIALIZER_HPP