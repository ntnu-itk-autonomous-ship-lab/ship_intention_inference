#pragma once
#include <map>
#include <string>
#include <vector>

namespace INTENTION_INFERENCE
{
struct IntentionModelParameters{
    unsigned number_of_network_evaluation_samples; /* This parameter specifies the precision, higher number means more precise. */
    unsigned max_number_of_obstacles;
    unsigned time_into_trajectory; /* How far into trajectory to see if there is course or speed change */
    double starting_distance; /* At what distance between the ships the intention model starts */
    double starting_cpa_distance;
    
    /**
     * @brief  Add a new timestep in the DBN no more often than min_time_s, and no more seldom than max_time_s. Within this interval, add a new timestep if there has been a significant course or speed change
     * 
     */
    struct{
        double min_time_s;
        double max_time_s;
        double min_course_change_rad;
        double min_speed_change_m_s;
    }expanding_dbn;
    
    /* #The normal distributions are discretized into 30 bins. Everything above the max value is in the highest bin. Max should probably be around mu+2sigma */
    
    /**
     * @brief The ship must do an evasive action before ample time if it plans to give way
     * 
     */
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
        double minimal_accepted_by_ownship; /* If the obstacle ship has a definition of ample time lower than this then the own ship should give way */
    }ample_time_s;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }safe_distance_m;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }risk_distance_m;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }risk_distance_front_m;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }safe_distance_midpoint_m;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }safe_distance_front_m;

    /**
     * @brief Course change smaller than this is considered as keeping course
     * 
     */
    struct{
        double minimal_change_since_init_state;
        double minimal_change_since_last_state;
    }change_in_course_rad;

    /**
     * @brief Speed change smaller than this is considered as keeping speed
     * 
     */
    struct{
        double minimal_change;
    }change_in_speed_m_s;
    struct{
        double HO_uncertainty_start;
        double HO_start;
        double HO_stop;
        double HO_uncertainty_stop;
        double OT_uncertainty_start;
        double OT_start;
        double OT_stop;
        double OT_uncertainty_stop;
    }colregs_situation_borders_rad;

    /**
     * @brief Parameters of initial start point of intention calculations.
     * 
     */
    struct  {
        double min_time_cpa; /* Minimum time to CPA before startpoint is set */
    }set_startpoint;

    /**
     * @brief Parameters for timestep removal if intention model calculates unmodeled behaviour
     * 
     */
    struct  {
        int min_timesteps_in_state_history; /* Minimum saved timesteps of ship states */
        double unmodeled_behaviour_threshold; /* Tolerance of unmodeled behaviour */
        double time_cpa_threshold; /* Smallest time before cpa where we can remove timesteps */
    }time_step_removal;

    double ignoring_safety_probability; /* Any behaviour not captured by the model, such as changing course that does not give way */
    double colregs_compliance_probability; /* If not colregs compliant, then the ship only needs to pass at a safe distance */
    double good_seamanship_probability; /* Prevents the ship from switching which side it is giving way towards */
    double unmodeled_behaviour;
    std::map<std::string, double> priority_probability;
};

    /**
     * @brief 
     * 
     * @param num_ships number of ships including own ship
     * @return IntentionModelParameters class with set parameters
     */
    IntentionModelParameters default_parameters(int num_ships){
        IntentionModelParameters param;
        param.number_of_network_evaluation_samples = 100000;
        param.max_number_of_obstacles = num_ships-1; //must be set to num_ships-1 or else segmantation fault
        param.time_into_trajectory = 0;
        param.starting_distance = 10000;
        param.starting_cpa_distance = 15000;
        param.expanding_dbn.min_time_s = 20;
        param.expanding_dbn.max_time_s = 300;
        param.expanding_dbn.min_course_change_rad = 0.2617994; /* 15 degrees */
        param.expanding_dbn.min_speed_change_m_s = 1.5;
        param.ample_time_s.mu = 200;
        param.ample_time_s.sigma = 100;
        param.ample_time_s.max = 1000;
        param.ample_time_s.n_bins = 30; // this value must match the bayesian network
        param.ample_time_s.minimal_accepted_by_ownship = 20;
        param.safe_distance_m.mu = 200;
        param.safe_distance_m.sigma = 30;
        param.safe_distance_m.max = 800;
        param.safe_distance_m.n_bins = 30;
        param.risk_distance_m.mu = 1500;
        param.risk_distance_m.sigma = 250;
        param.risk_distance_m.max = 2500;
        param.risk_distance_m.n_bins = 30; // this value must match the bayesian network
        param.risk_distance_front_m.mu = 1500;
        param.risk_distance_front_m.sigma = 250;
        param.risk_distance_front_m.max = 2500;
        param.risk_distance_front_m.n_bins = 30;
        param.safe_distance_midpoint_m.mu = 600;
        param.safe_distance_midpoint_m.sigma = 20;
        param.safe_distance_midpoint_m.max = 2500;
        param.safe_distance_midpoint_m.n_bins = 30; // this value must match the bayesian network
        param.safe_distance_front_m.mu = 100;
        param.safe_distance_front_m.sigma = 50;
        param.safe_distance_front_m.max = 1000;
        param.safe_distance_front_m.n_bins = 30; // this value must match the bayesian network
        param.change_in_course_rad.minimal_change_since_init_state = 0.2617994; /* 15 degrees */
        param.change_in_course_rad.minimal_change_since_last_state = 0.06;
        param.change_in_speed_m_s.minimal_change = 1.5;
        param.colregs_situation_borders_rad.HO_uncertainty_start = 2.79;
        param.colregs_situation_borders_rad.HO_start = 2.96;
        param.colregs_situation_borders_rad.HO_stop = -2.96;
        param.colregs_situation_borders_rad.HO_uncertainty_stop = -2.79;
        param.colregs_situation_borders_rad.OT_uncertainty_start = 1.74;
        param.colregs_situation_borders_rad.OT_start = 2.18;
        param.colregs_situation_borders_rad.OT_stop = -2.18;
        param.colregs_situation_borders_rad.OT_uncertainty_stop = -1.74;
        param.set_startpoint.min_time_cpa = 60;
        param.time_step_removal.min_timesteps_in_state_history = 7;
        param.time_step_removal.unmodeled_behaviour_threshold = 0.1;
        param.time_step_removal.time_cpa_threshold = 50;
        param.ignoring_safety_probability = 0;
        param.colregs_compliance_probability = 0.99;
        param.good_seamanship_probability = 0.99;
        param.unmodeled_behaviour = 0.00001;
        param.priority_probability["lower"] = 0.05;
        param.priority_probability["similar"] = 0.90;
        param.priority_probability["higher"] = 0.05;
        return param;
    }

    /**
     * @brief Creates an IntentionModelParameters instance from another IntentionModelParameters instance
     * 
     * @param IntentionModelParameters instance of IntentionModelParameters
     * @return IntentionModelParameters class
     */
    IntentionModelParameters copy_parameters_to_new_instance(const IntentionModelParameters &other) 
    {
        IntentionModelParameters copy;
        copy = other;
        return copy;
    }
}
