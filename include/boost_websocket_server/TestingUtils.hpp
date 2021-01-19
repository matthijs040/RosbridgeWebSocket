#ifndef TESTINGUTILS_HPP
#define TESTINGUTILS_HPP

#include <chrono>
#include <functional>
#include <iostream>

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

};


namespace benchmark 
{
    double run(std::function<void(void)> function)
    {
        auto start = std::chrono::steady_clock::now();
        function();
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        return elapsed_seconds.count();
    }

};

#endif