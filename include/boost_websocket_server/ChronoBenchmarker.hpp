#ifndef CHRONOBENCHMARKER_HPP
#define CHRONOBENCHMARKER_HPP

#include <chrono>
#include <functional>
#include <iostream>

namespace benchmark 
{
    double run( std::function<void(void)> function, bool do_print = false)
    {
        auto start = std::chrono::steady_clock::now();
        function();
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        if(do_print)
            std::cout << "function ran in: " << elapsed_seconds.count() << "\n";
        return elapsed_seconds.count();
    }

};

#endif