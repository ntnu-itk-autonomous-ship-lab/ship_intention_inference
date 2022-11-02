#pragma once

#include <map>
#include <sstream>
#include <string>
#include <iostream>

namespace INTENTION_INFERENCE
{
    template <typename map_t>
    auto &better_at(map_t &map, std::string key)
    {
        try
        {
            return map.at(key);
        }
        catch (const std::out_of_range &error)
        {
            std::stringstream ss;
            ss << "Trying to get nonexisting key: " << key << "\n";
            ss << "The following keys exist: ";
            for (const auto &[key, value] : map)
            {
                (void)value;
                ss << key << ", ";
            }
            std::cout << ss.str() << std::flush;
            throw std::runtime_error("Trying to get nonexisting key");
        }
        return map.begin()->second;
    }

    template <typename map_t>
    auto &better_at(map_t &map, int key)
    {
        try
        {
            return map.at(key);
        }
        catch (const std::out_of_range &error)
        {
            std::stringstream ss;
            ss << "Trying to get nonexisting key: " << key << "\n";
            ss << "The following keys exist: ";
            for (const auto &[key, value] : map)
            {
                (void)value;
                ss << key << ", ";
            }
            std::cout << ss.str() << std::flush;
            throw std::runtime_error("Trying to get nonexisting key");
        }
        return map.begin()->second;
    }

    double mean(std::map<std::string,double> input){
        int number_of_states = input.size();
        double mean = 0;
        double prob_sum = 0;
        for(int i=1; i<=number_of_states; ++i){
            mean+=i*input.at("State"+std::to_string(i));
            prob_sum += input.at("State"+std::to_string(i));
        }
        if(prob_sum<0.98 || prob_sum > 1.02){
            throw std::string("Probability sum does not sum to 1 when evaluating mean!");
        }
        return mean/number_of_states;
    }

    double max(std::map<std::string,double> input){
        int number_of_states = input.size();
        double max_index = 0;
        double max_value = 0;
        for(int i=1; i<=number_of_states; ++i){
            double res = input.at("State"+std::to_string(i));
            if(res > max_value){
                max_index = i;
                max_value = res;
            }
        }
        return max_index/number_of_states;
    }
}