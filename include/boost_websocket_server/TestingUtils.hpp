#ifndef TESTINGUTILS_HPP
#define TESTINGUTILS_HPP

#include <chrono>       // td::chrono::steady_clock, std::chrono::duration
#include <functional>   // std::function
#include <iostream>     // std::cout 
#include <cctype>       // std::isdigit

#include <sstream> // std::stringstream
namespace test
{
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

    bool contains_keyvalue_pair(const std::string& data, const std::string& key, const int value)
    {
        return data.find("\"" + key + "\":" + std::to_string(value) ) != std::string::npos; 
    }

    std::vector<double> getCSVs(const std::string& values)
    {
        auto ret = std::vector<double>();
        std::stringstream ss(values);



        for(double i; ss >> i;)
        {
            ret.push_back(i);
            if(ss.peek() == ',')        // Seperation character.
                ss.ignore();
            else if(ss.peek() == ']')   // Terminiation character.
            {
                break;
            }
            else if(!std::isdigit(ss.peek()))   // Non-data character. Invalid string.
            {
                ret.clear();    
                return ret;
            }
        }

    }

    /**
     * @brief Function for checking if the given key-value string exists in a given data string.
     * specialized for a list of comma seperated numerical values. Assumes that formatting is the following:
     * [N1,N2,N3,Nn]
     * 
     * @param data 
     * @param key 
     * @param value 
     * @return true 
     * @return false 
     */
    bool contains_keyvalue_pair(const std::string& data, const std::string& key, const std::vector<double>& value)
    {
        // First checks if the key is present. Followed by an array opener.
        const auto array_key = "\"" + key + "\":[";
        const auto start = data.find(array_key);
        if(start == std::string::npos)
        {
            return false;
        }
        const auto result = getCSVs( data.substr(start + array_key.length()) );
        if(result == value)
            return true;
        
        return false;

        // auto read_values = std::vector<double>();
        // read_values.reserve(values.size());
        // 
        // 
        // valuesString.erase(valuesString.end() - 1); //remove the last added ','
        // return contains_keyvalue_pair(data, key, "[" + valuesString + "]");
    }

};


namespace benchmark 
{
    double run(std::function<void(void)> function, int times)
    {
        auto start = std::chrono::steady_clock::now();

        for(int i = 0; i < times; i++)
            function();
            
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        return elapsed_seconds.count();
    }

};

#endif