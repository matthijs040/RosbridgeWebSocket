#ifndef TESTINGUTILS_HPP
#define TESTINGUTILS_HPP

#include <chrono>       // td::chrono::steady_clock, std::chrono::duration
#include <functional>   // std::function
#include <iostream>     // std::cout 
#include <sstream>

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

    /**
     * @brief helper function to split comma seperated decimal values from a std::string into a vector.  
     * Based on: https://stackoverflow.com/questions/289347/using-strtok-with-a-stdstring
     * Assumes that the data string is properly started terminated, and separated.
     * @throws std::invalid_argument if the string does not consist of only CSVs
     * @param values 
     * @return std::vector<double> 
     */
    std::vector<double> getCSVs(const std::string& values)
    {
        auto ret = std::vector<double>();        
        std::istringstream iss(values);
        std::string token;

        while (std::getline(iss, token, ','))
        {
            ret.push_back(std::stod(token));
        }
        return ret;
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
        const auto start = data.find(array_key) + array_key.length();
        if(start == std::string::npos)
        {
            return false;
        }
        const auto end = data.find(']', start) - 1;
        const auto result = getCSVs( data.substr(start, end - start) );
        if(result == value)
            return true;
        
        return false;
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