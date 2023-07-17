/**
 * @file generate_intention_model.cpp
 * @author Steffen Folåsen
 * @brief Generates prior distributions for risk of collision, safe distance, and ample time 
 * using classified ais data, and writes these to a new intention model file. Other priors will
 * be based on an intention model file given as argument. 
 * @version 1.0
 * @date 2023-07-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <math.h>
#include "../inc/bayesian_network.h"
#include "../inc/parameters.h"
#include "../external/Eigen/Dense"

INTENTION_INFERENCE::IntentionModelParameters setModelParameters(int num_ships){
    INTENTION_INFERENCE::IntentionModelParameters param;
    param.number_of_network_evaluation_samples = 100000;
 	param.max_number_of_obstacles = num_ships-1; //must be set to num_ships-1 or else segmantation fault
	param.time_into_trajectory = 10;
    param.starting_distance = 10000;
    param.starting_cpa_distance = 15000;
	param.expanding_dbn.min_time_s = 10;
	param.expanding_dbn.max_time_s = 1200;
	param.expanding_dbn.min_course_change_rad = 0.18;
	param.expanding_dbn.min_speed_change_m_s = 2;
	param.ample_time_s.mu = 200;
	param.ample_time_s.sigma = 100;
	param.ample_time_s.max = 1000;
	param.ample_time_s.n_bins = 30; // this value must match the bayesian network
	param.ample_time_s.minimal_accepted_by_ownship = 20;
    param.safe_distance_m.mu = 200;
	param.safe_distance_m.sigma = 30;
	param.safe_distance_m.max = 800;
    param.safe_distance_m.n_bins = 30;
    param.risk_distance_m.mu = 1800;
	param.risk_distance_m.sigma = 500;
	param.risk_distance_m.max = 2500;
	param.risk_distance_m.n_bins = 30; // this value must match the bayesian network
	param.risk_distance_front_m.mu = 1900;
	param.risk_distance_front_m.sigma = 500;
	param.risk_distance_front_m.max = 2000;
	param.risk_distance_front_m.n_bins = 30;
    param.safe_distance_midpoint_m.mu = 600;
	param.safe_distance_midpoint_m.sigma = 20;
	param.safe_distance_midpoint_m.max = 2500;
	param.safe_distance_midpoint_m.n_bins = 30; // this value must match the bayesian network
	param.safe_distance_front_m.mu = 100;
	param.safe_distance_front_m.sigma = 50;
	param.safe_distance_front_m.max = 1000;
	param.safe_distance_front_m.n_bins = 30; // this value must match the bayesian network
	param.change_in_course_rad.minimal_change = 0.18;
	param.change_in_speed_m_s.minimal_change = 2;
	param.colregs_situation_borders_rad.HO_uncertainty_start = 2.79;
	param.colregs_situation_borders_rad.HO_start = 2.96;
	param.colregs_situation_borders_rad.HO_stop = -2.96;
	param.colregs_situation_borders_rad.HO_uncertainty_stop = -2.79;
	param.colregs_situation_borders_rad.OT_uncertainty_start = 1.74;
	param.colregs_situation_borders_rad.OT_start = 2.18;
	param.colregs_situation_borders_rad.OT_stop = -2.18;
	param.colregs_situation_borders_rad.OT_uncertainty_stop = -1.74;
	param.ignoring_safety_probability = 0;
	param.colregs_compliance_probability = 0.97;
    param.good_seamanship_probability = 0.99;
	param.unmodeled_behaviour = 0.001;
	param.priority_probability["lower"] = 0.05;
	param.priority_probability["similar"] = 0.90;
	param.priority_probability["higher"] = 0.05;
    return param;
}

int main(){
    using namespace INTENTION_INFERENCE;

    int num_ships = 2;
    std::string network_file_name = "files/intention_models/intention_model_with_risk_of_collision_no_startpoint_3.xdsl";

    IntentionModelParameters parameters = setModelParameters(num_ships);
    BayesianNetwork net(network_file_name, parameters.number_of_network_evaluation_samples);

    net.setBinaryPriors("intention_ignoring_safety", parameters.ignoring_safety_probability);
    net.setBinaryPriors("intention_colregs_compliant", parameters.colregs_compliance_probability);
    net.setBinaryPriors("intention_good_seamanship", parameters.good_seamanship_probability);
    net.setBinaryPriors("unmodelled_behaviour", parameters.unmodeled_behaviour);
    net.setPriorNormalDistribution("intention_distance_risk_of_collision", parameters.risk_distance_m.mu, parameters.risk_distance_m.sigma, parameters.risk_distance_m.max / ( parameters.safe_distance_m.n_bins));
    net.setPriorNormalDistribution("intention_distance_risk_of_collision_front", parameters.risk_distance_front_m.mu, parameters.risk_distance_front_m.sigma, parameters.risk_distance_front_m.max / parameters.risk_distance_front_m.n_bins);

    //int colreg_idx = 7;
    //int cpa_ts_idx = 4;  // per nå lik r_maneuver_own (skal byttes til cpa_ts_idx)

    net.setPriorNormalDistribution("intention_safe_distance_front", parameters.safe_distance_front_m.mu, parameters.safe_distance_front_m.sigma, parameters.safe_distance_front_m.max / parameters.safe_distance_front_m.n_bins);


    int cpa_dist_idx = 6;
    int colreg_idx = 7;
    int cpa_ample_time_idx = 8;

    int timestep = 60;
    int n_bins = 30;
    int multiply =1;

    int head_on = 3;
    int overtake = -2;
    int crossing = -1;

    // Cpa distance
    net.setAisDistribution("intention_safe_distance_midpoint", "files/classified/classified_south.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, head_on);
    net.setAisDistribution("intention_safe_distance", "files/classified/classified_south.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, overtake);
    //net.setAisDistribution("intention_distance_risk_of_collision", "files/classified/classified_west_5.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, overtake);
    //net.setAisDistribution("intention_distance_risk_of_collision_front", "files/classified/classified_west_5.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, crossing);

    // Cpa time, the model does NOT differ for the different situations
    //net.setAisDistribution("intention_ample_time", "files/classified/classified_west_5.csv", colreg_idx, cpa_ample_time_idx, multiply, n_bins, head_on);  //head on
    net.setAmpleTimeDistribution("intention_ample_time", "files/classified/classified_west_5.csv", cpa_ample_time_idx, timestep, n_bins);
    net.save_network("files/intention_models/intention_model_from_code.xdsl");
}
