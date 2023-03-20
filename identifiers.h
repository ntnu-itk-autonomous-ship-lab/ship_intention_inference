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

namespace INTENTION_INFERENCE
{
    unsigned discretizer(double input, int max, int n_bins)
    {
        if (!std::isfinite(input) || std::floor(input * n_bins / max) > INT_MAX)
            return n_bins - 1;
        else
            return std::clamp(int(std::floor(input * n_bins / max)), 0, n_bins - 1);
    }

    unsigned timeIdentifier(const IntentionModelParameters &parameters, double time_s)
    {
        return discretizer(time_s, parameters.ample_time_s.max, parameters.ample_time_s.n_bins);
    }
    unsigned highresCPADistanceIdentifier(const IntentionModelParameters &parameters, double distance_m)
    {
        return discretizer(distance_m, parameters.safe_distance_m.max, parameters.safe_distance_m.n_bins);
    }
    unsigned twotimesDistanceToMidpointIdentifier(const IntentionModelParameters &parameters, double distance_to_midpoint_m)
    {
        return discretizer(2 * distance_to_midpoint_m, parameters.safe_distance_midpoint_m.max, parameters.safe_distance_midpoint_m.n_bins);
    }
    unsigned crossInFrontHighresIdentifier(const IntentionModelParameters &parameters, double distance_m)
    {
        return discretizer(2 * distance_m, parameters.safe_distance_front_m.max, parameters.safe_distance_front_m.n_bins);
    }

    //Side of other ship at CPA
    std::string sideIdentifier(double angle_diff)
    {
        return angle_diff >= 0 ? "port" : "starboard";
    }

    std::string frontAftIdentifier(bool passing_in_front)
    {
        return passing_in_front ? "front" : "aft";
    }

    std::string crossingWithMidpointOnSideIdentifier(bool port_side)
    {
        return port_side ? "port" : "starboard";
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

   Eigen::Vector4d newInitialStatesIdentifier(Eigen::Vector4d ship_states, Eigen::Vector4d initial_ship_states, int time){
        Eigen::Vector4d new_initial_states;
        new_initial_states = initial_ship_states;
        if (time % 600==0){
            new_initial_states = ship_states;
        }
        return new_initial_states;
    }

    std::string changeInCourseIdentifier(const IntentionModelParameters &parameters, double current_course, double initial_course)
    {
        auto minimal_change = parameters.change_in_course_rad.minimal_change;
        if (current_course - initial_course > minimal_change)
        {
            return "starboardwards";
        }
        else if (initial_course - current_course > minimal_change)
        {
            return "portwards";
        }
        return "none";
    }

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
    


    std::map<std::string, double> evaluateRelativeSituation2(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state, double time_to_cpa) //method Emil
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
        /*if (time_to_cpa <= 0){
            result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;
        }*/

        std::cout<< "relative heading: " << relative_heading;
        std::cout<< "\n relative bearing: " << bearing_relative_to_ownship_heading << std::endl;
         if (bearing_relative_to_ownship_heading < theta1 && bearing_relative_to_ownship_heading > -theta1){
            if (relative_heading > M_PI-theta1+uncertainty+bearing_relative_to_ownship_heading || relative_heading< -M_PI+theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("HO") = 1;
            }
            else if (relative_heading > M_PI-theta1-uncertainty+bearing_relative_to_ownship_heading){
                result.at("HO") = -((M_PI-theta1+bearing_relative_to_ownship_heading-uncertainty)- relative_heading) / (uncertainty);
                result.at("CR_PS") = -(relative_heading - (M_PI-theta1+bearing_relative_to_ownship_heading)) / (uncertainty);
            std::cout << "hei\n";
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
                result.at("CR_SS") = ((-M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty)-relative_heading) / (uncertainty);
                result.at("OT_ing") = (relative_heading - (-M_PI+theta2+bearing_relative_to_ownship_heading)) / (uncertainty);
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
                result.at("CR_SS") = (-relative_heading+(-M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty)) / (uncertainty);
                result.at("OT_ing") = (-(-M_PI+theta2+bearing_relative_to_ownship_heading)+relative_heading) / (uncertainty);
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
            }*/
            
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
            }*/
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
                std::cout << "hei2\n";
            }
            else if (relative_heading < M_PI-theta2+bearing_relative_to_ownship_heading-uncertainty && relative_heading> -M_PI+theta2+bearing_relative_to_ownship_heading+uncertainty){
                result.at("OT_ing") = 1;
            }
            std::cout<<relative_heading<<std::endl;
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
            }*/
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
    }

    std::string compareSpeed(double ownship_speed, double obstacle_speed)
    {
        if (ownship_speed > obstacle_speed)
        {
            return "higher";
        }
        else
        {
            return "lower";
        }
    }

    /*
DENNE er feil, port/starboard gir ikke mening her
std::string compareCourse(double ownship_course, double obstacle_course)
{
    double velocity_angle_diff = ownship_course - obstacle_course;
    wrapPI(&velocity_angle_diff);

    if (velocity_angle_diff >= 0)
        return "port";
    else
        return "starboard";
}*/
}
