#pragma once

#include <map>
#include <sstream>
#include <string>
#include <stdio.h>
#include <iostream>
//#include <ros/ros.h>

namespace INTENTION_INFERENCE
{
    double prob_saturation(double val, std::string key = "") {
        if ((key != "") && ((val < 0.0) || (val > 1.0))){
            std::cout<<"WARNING: " << key << " = " << val << " is out of scope [0, 1]." << std::endl;
        }
        return std::max(0.0, std::min(val, 1.0));
    }
    template <typename map_t>
    /**
     * @brief Standard .at() operator for maps, just with more debugging
     * features
     * 
     * @param map 
     * @param key 
     * @return auto& 
     */
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
            //ROS_ERROR("%s", ss.str().c_str());
            std::cout << ss.rdbuf() << std::endl;
            assert(false);
        }
        return map.begin()->second;
    }

    template <typename map_t>
    /**
     * @brief Standard .at() operator for maps, just with more debugging
     * features
     * 
     * @param map 
     * @param key 
     * @return auto& 
     */
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
            //printf("ERROR: %s", ss.str().c_str());
            std::cout << ss.rdbuf() << std::endl;
            assert(false);
        }
        return map.begin()->second;
    }
}
