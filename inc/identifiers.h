/*
This file contains functions that translates continious values into discrete states used by the bayesian network.
*/

#pragma once
#include <math.h>
#include <map>
#include <algorithm>
#include <limits.h>
#include <string>
#include "geometry.h"
#include "parameters.h"
#include "utils.h"

namespace INTENTION_INFERENCE
{
    unsigned discretizer(double input, int max, int n_bins)
    {
        if (!std::isfinite(input) || std::floor(input * n_bins) > INT_MAX)
            return n_bins-1;
        else
            return std::clamp(int(std::floor(input * n_bins / max)), 0, n_bins-1);
    }

    unsigned timeIdentifier(const IntentionModelParameters &parameters, double time_s)
    {
        return discretizer(time_s, parameters.ample_time_s.max, parameters.ample_time_s.n_bins);
    }
    unsigned distanceIdentifier(const IntentionModelParameters &parameters, double distance_m)
    {
        return discretizer(distance_m, parameters.distance_midpoint_m.max, parameters.distance_midpoint_m.n_bins);
    }
    unsigned frontDistanceIdentifier(const IntentionModelParameters &parameters, double distance_m)
    {
        return discretizer(2 * distance_m, parameters.distance_front_m.max, parameters.distance_front_m.n_bins);
    }

    std::string hasPassedIdentifier(double time)
    {
        if (time <= 0)
            return "true";
        else
            return "false";
    }

    std::string crossing_port_starboard_identifier(double relative_bearing)
    {
        if (relative_bearing < 0)
            return "port";
        else
            return "starboard";
    }

   bool currentChangeInCourseIdentifier(const IntentionModelParameters &parameters, double current_course, double last_course){
        auto minimal_change = parameters.change_in_course_rad.minimal_change_since_last_state;
        if (wrapPI(current_course - last_course) > minimal_change)
        {
            return true;
        }
        else if (wrapPI(last_course - current_course) > minimal_change)
        {
            return true;
        }
        return false;
    }

    /**
     * @brief 
     * 
     * @param parameters Uses the \ref parameters.change_in_course_rad.minimal_change
     * parameter
     * @param current_course 
     * @param initial_course 
     * @return std::string "starboardwards", "portwards", or "none"
     */
    std::string changeInCourseIdentifier(const IntentionModelParameters &parameters, double current_course, double initial_course)
    {
        auto minimal_change = parameters.change_in_course_rad.minimal_change_since_init_state;
        if (wrapPI(current_course - initial_course) > minimal_change)
        {
            return "starboardwards";
        }
        else if (wrapPI(initial_course - current_course) > minimal_change)
        {
            return "portwards";
        }
        return "none";
    }

    /**
     * @brief 
     * 
     * @param parameters Uses the \ref parameters.change_in_speed_m_s.minimal_change
     * parameter
     * @param current_speed 
     * @param initial_speed 
     * @return std::string "higher", "lower", or "similar"
     */
    std::string changeInSpeedIdentifier(const IntentionModelParameters &parameters, double current_speed, double initial_speed)
    {
        auto minimal_change = parameters.change_in_speed_m_s.minimal_change;
        if (current_speed - initial_speed > minimal_change)
        {
            return "higher";
        }
        else if (initial_speed - current_speed > minimal_change)
        {
            return "lower";
        }
        return "similar";
    }
/*
    std::map<std::string, double> evaluateSitution(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state)
    {
        const auto &p = parameters.colregs_situation_borders_rad;

        double relative_heading = obstacle_state(CHI) - ownship_state(CHI);
        wrapPI(&relative_heading);

        //Check bearing relative to ownship heading to see if we are being overtaken
        const double angle_from_own_to_obstacle_ship = std::atan2(obstacle_state(PY) - ownship_state(PY), obstacle_state(PX) - ownship_state(PX));
        double bearing_relative_to_ownship_heading = angle_from_own_to_obstacle_ship - ownship_state(CHI);
        wrapPI(&bearing_relative_to_ownship_heading);

        //Check bearing realtive to obstacleship heading to see if we are overtaking
        const double angle_from_obstacle_to_own = std::atan2(ownship_state(PY) - obstacle_state(PY), ownship_state(PX) - obstacle_state(PX));
        double bearing_relative_to_obstacle_heading = angle_from_obstacle_to_own - obstacle_state(CHI);
        wrapPI(&bearing_relative_to_obstacle_heading);

        std::map<std::string, double> result;
        result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;

        double CR = 0; //crossing is added to the results later di be ble to distinguish between CR_PS adn CR_SS
        if (relative_heading > p.HO_start || relative_heading < p.HO_stop) 
        {
            result.at("HO") = 1;
        }
        else if (relative_heading > p.HO_uncertainty_start && relative_heading < p.HO_start) 
        {
            result.at("HO") = (relative_heading - p.HO_uncertainty_start) / (p.HO_start - p.HO_uncertainty_start);
            CR = (p.HO_start - relative_heading) / (p.HO_start - p.HO_uncertainty_start);
        }
        else if (relative_heading < p.HO_uncertainty_stop && relative_heading > p.HO_stop)
        {
            result.at("HO") = (relative_heading - p.HO_uncertainty_stop) / (p.HO_stop - p.HO_uncertainty_stop);
            CR = (p.HO_stop - relative_heading) / (p.HO_stop - p.HO_uncertainty_stop);
        }
        else if (bearing_relative_to_ownship_heading > p.OT_start || bearing_relative_to_ownship_heading < p.OT_stop) 
        {
            result.at("OT_en") = 1;
        }
        else if (bearing_relative_to_ownship_heading > p.OT_uncertainty_start && bearing_relative_to_ownship_heading < p.OT_start)
        {
            result.at("OT_en") = (bearing_relative_to_ownship_heading - p.OT_uncertainty_start) / (p.OT_start - p.OT_uncertainty_start);
            CR = (p.OT_start - bearing_relative_to_ownship_heading) / (p.OT_start - p.OT_uncertainty_start);
        }
        else if (bearing_relative_to_ownship_heading < p.OT_uncertainty_stop && bearing_relative_to_ownship_heading > p.OT_stop)
        {
            result.at("OT_en") = (bearing_relative_to_ownship_heading - p.OT_uncertainty_stop) / (p.OT_stop - p.OT_uncertainty_stop);
            CR = (p.OT_stop - bearing_relative_to_ownship_heading) / (p.OT_stop - p.OT_uncertainty_stop);
        }
        else if (bearing_relative_to_obstacle_heading > p.OT_start || bearing_relative_to_obstacle_heading < p.OT_stop)
        {
            result.at("OT_ing") = 1;
        }
        else if (bearing_relative_to_obstacle_heading > p.OT_uncertainty_start && bearing_relative_to_obstacle_heading < p.OT_start)
        {
            result.at("OT_ing") = (bearing_relative_to_obstacle_heading - p.OT_uncertainty_start) / (p.OT_start - p.OT_uncertainty_start);
            CR = (p.OT_start - bearing_relative_to_obstacle_heading) / (p.OT_start - p.OT_uncertainty_start);
        }
        else if (bearing_relative_to_obstacle_heading < p.OT_uncertainty_stop && bearing_relative_to_obstacle_heading > p.OT_stop)
        {
            result.at("OT_ing") = (bearing_relative_to_obstacle_heading - p.OT_uncertainty_stop) / (p.OT_stop - p.OT_uncertainty_stop);
            CR = (p.OT_stop - bearing_relative_to_obstacle_heading) / (p.OT_stop - p.OT_uncertainty_stop);
        }
        else
        {
            CR = 1;
        }

        if (bearing_relative_to_ownship_heading < 0)
        {
            result.at("CR_PS") = CR;
        }
        else
        {
            result.at("CR_SS") = CR;
        }

        return result;
    }

    std::map<std::string, double> evaluateRelativeSituation1(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state, double time_to_cpa) //method Bjørn-Olav
    {
        const auto &p = parameters.colregs_situation_borders_rad;

        double relative_heading = obstacle_state(CHI) - ownship_state(CHI);
        wrapPI(&relative_heading);

        //Check bearing relative to ownship heading to see if we are being overtaken
        const double angle_from_own_to_obstacle_ship = std::atan2(obstacle_state(PY) - ownship_state(PY), obstacle_state(PX) - ownship_state(PX));
        double bearing_relative_to_ownship_heading = angle_from_own_to_obstacle_ship - ownship_state(CHI);
        wrapPI(&bearing_relative_to_ownship_heading);

        //Check bearing realtive to obstacleship heading to see if we are overtaking
        const double angle_from_obstacle_to_own = std::atan2(ownship_state(PY) - obstacle_state(PY), ownship_state(PX) - obstacle_state(PX));
        double bearing_relative_to_obstacle_heading = angle_from_obstacle_to_own - obstacle_state(CHI);
        wrapPI(&bearing_relative_to_obstacle_heading);

        std::map<std::string, double> result;
        result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;

        //double theta1 = 22.5*M_PI/180; 
        //double theta2 = 90*M_PI/180;
        //double theta3 = 112.5*M_PI/180;
        //double theta4 = 145*M_PI/180;

        double theta1 = M_PI/8;
        double theta2 = M_PI/2;
        double theta3 = 5*M_PI/8;
        double theta4 = 5*M_PI/8;
        double uncertainty = 0.3;

        double CR = 0; //crossing is added to the results later di be ble to distinguish between CR_PS adn CR_SS
        
        if (time_to_cpa <= 0){
            result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;
        }
        else {
        if ((relative_heading > M_PI-theta1 || relative_heading < -M_PI+theta1) && (bearing_relative_to_ownship_heading <= theta2 && bearing_relative_to_ownship_heading >= -theta2)) // must add relative bearing
        {
            result.at("HO") = 1;
        }
        else if ((relative_heading > M_PI-theta1-uncertainty && relative_heading < M_PI-theta1) && (bearing_relative_to_ownship_heading <= theta2 && bearing_relative_to_ownship_heading >= -theta2))
        {
            result.at("HO") = (relative_heading - (M_PI-theta1-uncertainty)) / (uncertainty);
            CR = (M_PI-theta1 - relative_heading) / (uncertainty);
        }
        else if ((relative_heading < -M_PI+theta1 && relative_heading > -M_PI+theta1+uncertainty) && (bearing_relative_to_ownship_heading <= theta2 && bearing_relative_to_ownship_heading >= -theta2))
        {
            result.at("HO") = (relative_heading - (-M_PI+theta1+uncertainty)) / (uncertainty);
            CR = (-M_PI+theta1 - relative_heading) / (uncertainty);
        }
        else if ((bearing_relative_to_ownship_heading > theta2 || bearing_relative_to_ownship_heading < -theta2) && (relative_heading <= (M_PI - theta3) && relative_heading >= -(M_PI - theta3)))
        {
            result.at("OT_en") = 1;
        }
        else if (((bearing_relative_to_ownship_heading > theta2) || (bearing_relative_to_ownship_heading < -theta2)) && ((relative_heading > (M_PI - theta3)) && (relative_heading < (M_PI - theta3+uncertainty))))
        {
            result.at("OT_en") = ((M_PI-theta3+uncertainty)- relative_heading) / (uncertainty);
            CR = (relative_heading - (M_PI-theta3)) / (uncertainty);
        }
        else if (((bearing_relative_to_ownship_heading > theta2) || (bearing_relative_to_ownship_heading < -theta2)) && ((relative_heading < (-M_PI + theta3)) && (relative_heading > (-M_PI + theta3-uncertainty))))
        {
            result.at("OT_en") = (relative_heading - (-M_PI+theta3-uncertainty)) / (uncertainty);
            CR = (-M_PI+theta3-relative_heading) / (uncertainty);
        }
        else if ((bearing_relative_to_ownship_heading < theta2 && bearing_relative_to_ownship_heading > -theta2) && (relative_heading < (M_PI - theta4) && relative_heading > -(M_PI - theta4)))
        {
            result.at("OT_ing") = 1;
            std::cout<<relative_heading<<std::endl;
        }
        else if ((bearing_relative_to_ownship_heading < theta2 && bearing_relative_to_ownship_heading > -theta2) && ((relative_heading > (M_PI - theta4)) && (relative_heading < (M_PI - theta4+uncertainty))))
        {
            result.at("OT_ing") = ((M_PI-theta4+uncertainty)- relative_heading) / (uncertainty);
            CR = (relative_heading - (M_PI-theta4)) / (uncertainty);
            std::cout<<"oting2"<<std::endl;
        }
        else if ((bearing_relative_to_ownship_heading < theta2 && bearing_relative_to_ownship_heading > -theta2) && (relative_heading < (-M_PI + theta4) && relative_heading > (-M_PI + theta4-uncertainty)))
        {
            result.at("OT_ing") = (relative_heading - (-M_PI+theta4-uncertainty)) / (uncertainty);
            CR = (-M_PI+theta4-relative_heading) / (uncertainty);
            std::cout<<"oting3"<<std::endl;
        }
        else
        {
            CR = 1;
        }

        if ((bearing_relative_to_ownship_heading > -theta3 && bearing_relative_to_ownship_heading < -theta2 ) && (relative_heading > (M_PI - theta3) && relative_heading < (M_PI - theta1) ))
        {
            result.at("CR_PS") = CR;
        }
         if ((bearing_relative_to_ownship_heading > -theta2 && bearing_relative_to_ownship_heading < theta2 ) && (relative_heading > (M_PI - theta4) && relative_heading < (M_PI - theta1) ))
        {
            result.at("CR_PS") = CR;
        }
        else if ((bearing_relative_to_ownship_heading < M_PI && bearing_relative_to_ownship_heading > theta3) && (relative_heading > (M_PI - theta2) && relative_heading < (M_PI - theta1) ))
        {
            result.at("CR_PS") = CR;
        }
        else if ((bearing_relative_to_ownship_heading > -theta2 && bearing_relative_to_ownship_heading < theta2 ) && (relative_heading > -(M_PI - theta1) && relative_heading < -(M_PI - theta4) ))
        {
            result.at("CR_SS") = CR;
        }
        else if ((bearing_relative_to_ownship_heading > theta2 && bearing_relative_to_ownship_heading < theta3 ) && (relative_heading > -(M_PI - theta1) && relative_heading < -(M_PI - theta3) ))
        {
            result.at("CR_SS") = CR;
        }
        else if ((bearing_relative_to_ownship_heading > -M_PI && bearing_relative_to_ownship_heading < -(M_PI - theta3) ) && (relative_heading > -(M_PI - theta2) && relative_heading < -(M_PI - theta1) ))
        {
            result.at("CR_SS") = CR;
        }
        else {
            result.at("CR_SS") = CR/2;
            result.at("CR_PS") = CR/2;
        }
        }
        return result;
    }

    std::map<std::string, double> evaluateRelativeSituation2(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state) //method Emil
    {
        const auto &p = parameters.colregs_situation_borders_rad;

        double relative_heading = obstacle_state(CHI) - ownship_state(CHI);
        wrapPI(&relative_heading);

        //Check bearing relative to ownship heading to see if we are being overtaken
        const double angle_from_own_to_obstacle_ship = std::atan2(obstacle_state(PY) - ownship_state(PY), obstacle_state(PX) - ownship_state(PX));
        double bearing_relative_to_ownship_heading = angle_from_own_to_obstacle_ship - ownship_state(CHI);
        wrapPI(&bearing_relative_to_ownship_heading);

        std::map<std::string, double> result;
        result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;

        //Must be tuned correctly
        double theta1 = 22.5*M_PI/180;
        double theta2 = 5*M_PI/8;
        double theta3 = 112.5*M_PI/180;
        double uncertainty = 0.2;

        if ((bearing_relative_to_ownship_heading < theta1) && (bearing_relative_to_ownship_heading > -theta1)){
            if ((relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading) || (relative_heading< -(M_PI-theta1+uncertainty)+bearing_relative_to_ownship_heading)){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("HO") = prob_saturation(-((M_PI-theta1+bearing_relative_to_ownship_heading-uncertainty)- relative_heading) / (uncertainty), "HO");
                result.at("CR_PS") = prob_saturation(-(relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty), "CR_PS");
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_PS") = prob_saturation(((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty), "CR_PS");
                result.at("OT_ing") = prob_saturation((relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty), "OT_ing");
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_SS") = prob_saturation(((-M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty), "CR_SS");
                result.at("OT_ing") = prob_saturation((relative_heading - (-M_PI+theta2+bearing_relative_to_ownship_heading)) / (uncertainty), "OT_ing");
            }
            else if (relative_heading< -M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") =prob_saturation(((-M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty), "HO");
                result.at("CR_SS") =prob_saturation((relative_heading - (-M_PI+theta1+bearing_relative_to_ownship_heading)) / (uncertainty), "CR_SS");
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            //std::cout<<relative_heading<<std::endl;
        }
        else if (bearing_relative_to_ownship_heading < theta2 && bearing_relative_to_ownship_heading > theta1){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -(M_PI-theta1+uncertainty)+bearing_relative_to_ownship_heading){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("CR_SS") = prob_saturation(((M_PI-theta1+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty), "CR_SS");
                result.at("HO") = prob_saturation((relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty), "HO");
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("HO") = prob_saturation(((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty), "HO");
                result.at("OT_ing") = prob_saturation((relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty), "OT_ing");
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_SS") = prob_saturation((-relative_heading+(-M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty)) / (uncertainty), "CR_SS");
                result.at("OT_ing") = prob_saturation((-(-M_PI+theta2+bearing_relative_to_ownship_heading)+relative_heading) / (uncertainty), "OT_ing");
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            //std::cout<<relative_heading<<std::endl;
            /*if (relative_heading > bearing_relative_to_ownship_heading+M_PI-theta1 || relative_heading< bearing_relative_to_ownship_heading-M_PI+theta1){
                result["CR_SS"] = 1;
            }
             else if (relative_heading > bearing_relative_to_ownship_heading+M_PI-theta2){
                result["HO"] = 1;
            }
            else if (relative_heading< bearing_relative_to_ownship_heading-M_PI+theta2){
                result["CR_SS"] = 1;
            }
            else if (relative_heading < bearing_relative_to_ownship_heading+M_PI-theta2 && relative_heading > bearing_relative_to_ownship_heading-M_PI+theta2){
                result["OT_ing"] = 1;
            }* /

        }
        else if (bearing_relative_to_ownship_heading > theta2 || bearing_relative_to_ownship_heading < -theta2){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -(M_PI-theta1+uncertainty)+bearing_relative_to_ownship_heading){
                result.at("OT_en") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("OT_en") = prob_saturation(((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty), "OT_en");
                result.at("HO") = prob_saturation((relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty), "HO");
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_en") = prob_saturation((relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty), "OT_en");
                result.at("HO") = prob_saturation(((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty), "HO");
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = 1;
            }
            //std::cout<<relative_heading<<std::endl;
            /*
            if (relative_heading > M_PI-theta1+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1+bearing_relative_to_ownship_heading){
                result["OT_en"] = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading){
                result["OT_en"] = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading){
                result["OT_en"] = 1;
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading){
                result["HO"] = 1;
            }* /
        }
        else if (bearing_relative_to_ownship_heading < -theta1 && bearing_relative_to_ownship_heading > -theta2){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -(M_PI-theta1+uncertainty)+bearing_relative_to_ownship_heading){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_PS") = prob_saturation(((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty), "CR_PS");
                result.at("OT_ing") = prob_saturation((relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty), "OT_ing");
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("HO") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = prob_saturation((relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty), "HO");
                result.at("OT_ing") = prob_saturation(((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty), "OT_ing");
            }
            else if (relative_heading< -M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_PS") = prob_saturation(((-M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty), "CR_PS");
                result.at("HO") = prob_saturation((relative_heading - (-M_PI+theta1+bearing_relative_to_ownship_heading)) / (uncertainty), "HO");
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            /*
            if (relative_heading > M_PI-theta1+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1+bearing_relative_to_ownship_heading){
                result["CR_PS"] = 1;
            }
             else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading){
                result["CR_PS"] = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading){
                result["HO"] = 1;
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading && relative_heading > -M_PI+theta2+bearing_relative_to_ownship_heading){
                result["OT_ing"] = 1;
            }* /
        }
        return result;
    }

    std::map<std::string, double> evaluateImprovedRelativeSituation2(std::map<std::string, double> result, const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state, double time_to_cpa) //method Emil
    {
        const auto &p = parameters.colregs_situation_borders_rad;

        double relative_heading = obstacle_state(CHI) - ownship_state(CHI);
        wrapPI(&relative_heading);

        //Check bearing relative to ownship heading to see if we are being overtaken
        const double angle_from_own_to_obstacle_ship = std::atan2(obstacle_state(PY) - ownship_state(PY), obstacle_state(PX) - ownship_state(PX));
        double bearing_relative_to_ownship_heading = angle_from_own_to_obstacle_ship - ownship_state(CHI);
        wrapPI(&bearing_relative_to_ownship_heading);

        

        //Must be tuned correctly
        double theta1 = 22.5*M_PI/180;
        double theta2 = 5*M_PI/8;
        double theta3 = 112.5*M_PI/180;
        double uncertainty = 0.2;
        if (time_to_cpa <= 0){
            result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;
        }
        else if (time_to_cpa < 1){

        }
         else if (bearing_relative_to_ownship_heading < theta1 && bearing_relative_to_ownship_heading > -theta1){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("HO") = ((M_PI-theta1+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("CR_PS") = (relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_PS") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_SS") = (relative_heading - (-M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty)) / (uncertainty);
                result.at("OT_ing") = ((-M_PI+theta2+bearing_relative_to_ownship_heading)- relative_heading) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = ((-M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty);
                result.at("CR_SS") = (relative_heading - (-M_PI+theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            //std::cout<<relative_heading<<std::endl;
        }
        else if (bearing_relative_to_ownship_heading < theta2 && bearing_relative_to_ownship_heading > theta1){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("CR_SS") = ((M_PI-theta1+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("HO") = (relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("HO") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_SS") = (relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty);
                result.at("OT_ing") = (relative_heading -(-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            std::cout<<relative_heading<<std::endl;
            
        }
        else if (bearing_relative_to_ownship_heading > theta2 || bearing_relative_to_ownship_heading < -theta2){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("OT_en") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("OT_en") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("HO") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_en") = (relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty);
                result.at("HO") = ((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = 1;
            }
            
        }
        else if (bearing_relative_to_ownship_heading < -theta1 && bearing_relative_to_ownship_heading > -theta2){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("CR_PS") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty){
                result.at("HO") = 1;
            }
            else if (relative_heading< -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = (relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty);
                result.at("OT_ing") = ((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty){
                result.at("CR_PS") = ((-M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty);
                result.at("HO") = (relative_heading - (-M_PI+theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            //std::cout<<relative_heading<<std::endl;
            
        }
        return result;
    }

    std::map<std::string, double> evaluateRelativeSituation3(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state, double time_to_cpa) //method Emil
    {
        const auto &p = parameters.colregs_situation_borders_rad;

        double relative_heading = obstacle_state(CHI) - ownship_state(CHI);
        wrapPI(&relative_heading);

        //Check bearing relative to ownship heading to see if we are being overtaken
        const double angle_from_own_to_obstacle_ship = std::atan2(obstacle_state(PY) - ownship_state(PY), obstacle_state(PX) - ownship_state(PX));
        double bearing_relative_to_ownship_heading = angle_from_own_to_obstacle_ship - ownship_state(CHI);
        wrapPI(&bearing_relative_to_ownship_heading);

        std::map<std::string, double> result;
        result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;

        //Must be tuned correctly
        double theta1 = M_PI/8;
        double theta2 = 5*M_PI/8;
        double theta3 = 5*M_PI/8;
        double uncertainty = 0.2;
        if (time_to_cpa <= 0){
            result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;
        }
         else if (bearing_relative_to_ownship_heading < theta1 && bearing_relative_to_ownship_heading > -theta1){
            if (relative_heading > M_PI-theta1+uncertainty || relative_heading< -M_PI+theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty){
                result.at("HO") = ((M_PI-theta1+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("CR_PS") = (relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading > M_PI-theta2+uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2-uncertainty){
                result.at("CR_PS") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading< -M_PI+theta2+uncertainty){
                result.at("CR_SS") = ((-M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (-M_PI+theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta1+uncertainty){
                result.at("HO") = ((-M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty);
                result.at("CR_SS") = (relative_heading - (-M_PI+theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            //std::cout<<relative_heading<<std::endl;
        }
        else if (bearing_relative_to_ownship_heading < theta2 && bearing_relative_to_ownship_heading > theta1){
            if (relative_heading > M_PI-theta1+uncertainty || relative_heading< -M_PI+theta1-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty){
                result.at("CR_SS") = ((M_PI-theta1+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("HO") = (relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading > M_PI-theta2+uncertainty){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta2-uncertainty){
                result.at("HO") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2-uncertainty){
                result.at("CR_SS") = 1;
            }
            else if (relative_heading< -M_PI+theta2+uncertainty){
                result.at("CR_SS") = (relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty);
                result.at("OT_ing") = ((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            
            
        }
        else if (bearing_relative_to_ownship_heading > theta2 || bearing_relative_to_ownship_heading < -theta2){
            if (relative_heading > M_PI-theta1+uncertainty || relative_heading< -M_PI+theta1-uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading > M_PI-theta2+uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading > M_PI-theta2-uncertainty){
                result.at("OT_en") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("HO") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2-uncertainty){
                result.at("OT_en") = 1;
            }
            else if (relative_heading< -M_PI+theta2+uncertainty){
                result.at("OT_en") = (relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty);
                result.at("HO") = ((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("HO") = 1;
            }
            
        }
        else if (bearing_relative_to_ownship_heading < -theta1 && bearing_relative_to_ownship_heading > -theta2){
            if (relative_heading > M_PI-theta1+uncertainty || relative_heading< -M_PI+theta1-uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2+uncertainty){
                result.at("CR_PS") = 1;
            }
            else if (relative_heading > M_PI-theta2-uncertainty){
                result.at("CR_PS") = ((M_PI-theta2+bearing_relative_to_ownship_heading+uncertainty)- relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (M_PI-theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta2-uncertainty){
                result.at("HO") = 1;
            }
            else if (relative_heading< -M_PI+theta2+uncertainty){
                result.at("HO") = (relative_heading-(-M_PI+theta2+bearing_relative_to_ownship_heading-uncertainty)) / (uncertainty);
                result.at("OT_ing") = ((-M_PI+theta2+bearing_relative_to_ownship_heading)-relative_heading) / (uncertainty);
            }
            else if (relative_heading< -M_PI+theta1+uncertainty){
                result.at("CR_PS") = ((-M_PI+theta1+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty);
                result.at("HO") = (relative_heading - (-M_PI+theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            }
            else if (relative_heading < M_PI-theta2-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            std::cout<<relative_heading<<std::endl;
            
        }
        return result;
    }*/

    std::map<std::string, double> evaluateRelativeSituation(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state) //method Jon Eivind
    {
        const auto p = parameters.colregs_situation_borders_rad;

        double relative_course = wrapPI(obstacle_state(CHI) - ownship_state(CHI));

        std::map<std::string, double> result;
        result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;

        double ot_probability = 0;

        if (relative_course <= -p.HO_certain || relative_course > p.HO_certain) {
            result.at("HO") = 1;
        }
        else if (relative_course <= -p.HO_uncertain && relative_course > -p.HO_certain) {
            result.at("HO") = (relative_course + p.HO_uncertain)/(-p.HO_certain + p.HO_uncertain);
            result.at("CR_PS") = (-p.HO_certain - relative_course)/(-p.HO_certain + p.HO_uncertain);
        }
        else if (relative_course <= -p.OT_uncertain && relative_course > -p.HO_uncertain) {
            result.at("CR_PS") = 1;
        }
        else if (relative_course <= -p.OT_certain && relative_course > -p.OT_uncertain) {
            result.at("CR_PS") = (relative_course + p.OT_certain)/(-p.OT_uncertain + p.OT_certain);
            ot_probability = (-p.OT_uncertain - relative_course)/(-p.OT_uncertain + p.OT_certain);
        }
        else if (relative_course <= p.OT_certain && relative_course > -p.OT_certain) {
            ot_probability = 1;
        }
        else if (relative_course <= p.OT_uncertain && relative_course > p.OT_certain) {
            ot_probability = (relative_course - p.OT_uncertain)/(p.OT_certain - p.OT_uncertain);
            result.at("CR_SS") = (p.OT_certain - relative_course)/(p.OT_certain - p.OT_uncertain);
        }
        else if (relative_course <= p.HO_uncertain && relative_course > p.OT_uncertain) {
            result.at("CR_SS") = 1;
        }
        else if (relative_course <= p.HO_certain && relative_course > p.HO_uncertain) {
            result.at("CR_SS") = (relative_course - p.HO_certain)/(p.HO_uncertain - p.HO_certain);
            result.at("HO") = (p.HO_uncertain - relative_course)/(p.HO_uncertain - p.HO_certain);
        }
        
        const double speed_difference = ownship_state(U) - obstacle_state(U);
        if (speed_difference > p.U_uncertainty_m_s) {
            result.at("OT_ing") = ot_probability;
        }
        else if (speed_difference <= p.U_uncertainty_m_s && speed_difference > -p.U_uncertainty_m_s) {
            result.at("OT_ing") = ot_probability * (speed_difference + p.U_uncertainty_m_s) / (2 * p.U_uncertainty_m_s);
            result.at("OT_en") = ot_probability * (p.U_uncertainty_m_s - speed_difference) / (2 * p.U_uncertainty_m_s);
        }
        else if (speed_difference <= -p.U_uncertainty_m_s) {
            result.at("OT_en") = ot_probability;
        }

        // if (result.at("HO")+result.at("CR_PS")+result.at("CR_SS")+result.at("OT_en")+result.at("OT_ing") != 1) {
        //     std::cout<<"Situation probabilities sum to "<<result.at("HO")+result.at("CR_PS")+result.at("CR_SS")+result.at("OT_en")+result.at("OT_ing");
        // }
        return result;
    }
}