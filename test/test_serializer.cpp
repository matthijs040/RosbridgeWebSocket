//#include <boost/json/src.hpp>

#include "../include/boost_websocket_server/BridgeMessageJsonSerializer.hpp"
#include "../include/boost_websocket_server/BridgeMessages.hpp"

#include <iostream>


#include "../include/boost_websocket_server/ChronoBenchmarker.hpp"

/**
 * @brief Namespace containing all tests for the BridgeMessageJsonSerializer class.
 */
namespace test
{

    BridgeMessageJsonSerializer serializer = BridgeMessageJsonSerializer();

    /**
     * @brief Utility function that tells if a given JSON key-value-pair is contained within a given string. 
     * NOTE: that only values typeof string are encapsulated with additional quotationmarks as used here.
     * 
     * @param data as a json serialized string.
     * @param key of the json field.
     * @param value of the json field. 
     * @return true if the key-value are in the string in json format.
     * @return false otherwise
     */
    bool contains_keyvalue_pair(const std::string& data, const std::string& key, const std::string& value)
    {
        return data.find("\"" + key + "\":\"" + value +"\"") != std::string::npos;
    }

    template<typename T>
    bool contains_keyvalue_pair(const std::string& data, const std::string& key, const T value)
    {
        return data.find("\"" + key + "\":" + std::to_string(value) ) != std::string::npos; 
    }


    bool contains(const std::string& data, const std::string& contains)
    {
        return data.find(contains) != std::string::npos;
    }

    void SetStatusLevel_serialized_preserving_all_fields()
    {
        // Given:
        auto message = SetStatusLevel();
        message.level = "error";
        message.id = "id_10";

        // When:
        auto data = serializer.Serialize(message);

        // Then:
        assert(contains_keyvalue_pair(data, "op", message.op));
        assert(contains_keyvalue_pair(data, "level", message.level));
        assert(contains_keyvalue_pair(data, "id", message.id));
    }

    void optional_field_serialized_as_empty()
    {
        auto m = Status();
        
        auto data = serializer.Serialize(m);
        // std::cout << data << '\n';
        assert(contains(data, "id"));
    }

    void Status_serialized_preserving_all_fields()
    {
        auto m = Status();
        m.level = "error";
        m.id = "id_string";
        m.msg = "msg_string";

        auto data = serializer.Serialize(m);
        
        assert(contains_keyvalue_pair(data, "op", m.op) );
        assert(contains_keyvalue_pair(data, "level", m.level) );
        assert(contains_keyvalue_pair(data, "id",m.id) );
        assert(contains_keyvalue_pair(data, "msg", m.msg) );
    }

    void Authenticate_serialized_preserving_all_fields()
    {
        auto m = Authenticate();

        m.mac = "mac_string";
        m.client= "client_string";
        m.dest = "dest_string";
        m.rand = "rand_string";
        m.t = 12;
        m.level = "level_string";
        m.end= 13;

        auto data = serializer.Serialize(m);

        assert(contains_keyvalue_pair(data, "op", m.op) );
        assert(contains_keyvalue_pair(data, "mac", m.mac) );
        assert(contains_keyvalue_pair(data, "client", m.client) );
        assert(contains_keyvalue_pair(data, "dest", m.dest) );
        assert(contains_keyvalue_pair(data, "rand", m.rand) );
        assert(contains_keyvalue_pair(data, "t", m.t) );   
        assert(contains_keyvalue_pair(data, "level", m.level) );
        assert(contains_keyvalue_pair(data, "end", m.end));
    }

    void Advertise_serializes_preserving_all_fields()
    {
        auto m = Advertise();
        m.id = "id_string";
        m.topic = "topic_string";
        m.type = "type_string";

        auto data = serializer.Serialize(m);

        assert(contains_keyvalue_pair(data, "op", m.op) );
        assert(contains_keyvalue_pair(data, "id", m.id) );
        assert(contains_keyvalue_pair(data, "topic", m.topic) );
        assert(contains_keyvalue_pair(data, "type", m.type) );
                                
    }

    void Unadvertise_serializes_preserving_all_fields()
    {
        auto m = Unadvertise();
        m.id = "id_string";
        m.topic = "topic_string";

        auto data = serializer.Serialize(m);  

        assert(contains_keyvalue_pair(data, "op", m.op) );
        assert(contains_keyvalue_pair(data, "id", m.id) );
        assert(contains_keyvalue_pair(data, "topic", m.topic) );
    }

    void Publish_serializes_preserving_all_fields()
    {
        auto m = Publish();
        m.topic = "topic_string";       

        auto data = serializer.Serialize(m);  
        
        assert(contains_keyvalue_pair(data, "op", m.op) );
        assert(contains_keyvalue_pair(data, "topic", m.topic) );
    }

    void Subscribe_serializes_preserving_all_fields()
    {
        auto m = Subscribe();
        
        m.id = "id_string";
        m.topic = "topic_string";
        m.type = "type_string";
        m.throttle_rate = 1;
        m.queue_length = 2;
        m.fragment_size = 3;
        m.compression = "compression_string";

        auto data = serializer.Serialize(m);  

        assert(contains_keyvalue_pair(data, "op", m.op) );      

    }

    void SetLevel_deserializes_unchanged_after_serialization()
    {
        auto m = SetStatusLevel();
    
        assert( *dynamic_cast<SetStatusLevel*>( serializer.Deserialize(serializer.Serialize(m)).get() ) == m);
    }
    
};


int main(int argc, char const *argv[])
{
    test::SetStatusLevel_serialized_preserving_all_fields();
    test::Status_serialized_preserving_all_fields();
    test::optional_field_serialized_as_empty();
    test::Authenticate_serialized_preserving_all_fields();
    test::Advertise_serializes_preserving_all_fields();
    test::Unadvertise_serializes_preserving_all_fields();
    test::Publish_serializes_preserving_all_fields();
    test::Subscribe_serializes_preserving_all_fields();
    test::SetLevel_deserializes_unchanged_after_serialization();

    bool do_benchmark = true;
    if(do_benchmark)
    {
        std::cout << "without dyncast x10000\n";
        auto start = std::chrono::steady_clock::now();
        
        for(int i = 10000; i > 0; i--)
       {
            test::serializer.Serialize(ServiceResponse());
            test::serializer.Serialize(Status());
            test::serializer.Serialize(Advertise());
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
        
        


        start = std::chrono::steady_clock::now();
        std::cout << "without dyncast x10000\n";

        for(int i = 10000; i > 0; i--)
        {
            test::serializer.Serialize(ServiceResponse());
            test::serializer.Serialize(Status());
            test::serializer.Serialize(Advertise());
        }

        end = std::chrono::steady_clock::now();
        elapsed_seconds = end-start;
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";



        
        start = std::chrono::steady_clock::now();
        std::cout << "with upcast en dyncast x10000\n";

        for(int i = 10000; i > 0; i--)
        {
            test::serializer.Serialize((BridgeMessage)ServiceResponse());
            test::serializer.Serialize((BridgeMessage)Status());
            test::serializer.Serialize((BridgeMessage)Advertise());
        }


        end = std::chrono::steady_clock::now();
        elapsed_seconds = end-start;
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
    }

    return EXIT_SUCCESS;
}
