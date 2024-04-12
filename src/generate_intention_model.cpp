/**
 * @file generate_intention_model.cpp
 * @author Steffen Folåsen
 * @brief Generates prior distributions for risk of collision, safe distance, and ample time 
 * using classified ais data, and writes these to a new intention model file. Other priors will
 * be based on an intention model file given as argument. Default number of ships is 2.
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
#include <Eigen/Dense>

int main(int argc, char* argv[]){
    using namespace INTENTION_INFERENCE;

    int num_ships;
    if (argc > 1) {num_ships = std::atoi(argv[1]);}
    else{num_ships = 2;}

    std::string network_file_name = "files/intention_models/intention_model_with_grounding_manually_adjusted.xdsl";

    IntentionModelParameters parameters = default_parameters(num_ships);
    BayesianNetwork net(network_file_name, parameters.number_of_network_evaluation_samples);

    net.setBinaryPriors("intention_ignoring_safety", parameters.ignoring_safety_probability);
    net.setBinaryPriors("intention_colregs_compliant", parameters.colregs_compliance_probability);
    net.setBinaryPriors("intention_good_seamanship", parameters.good_seamanship_probability);
    net.setBinaryPriors("unmodelled_behaviour", parameters.unmodeled_behaviour);
    net.setPriorNormalDistribution("intention_distance_risk_of_collision", parameters.distance_midpoint_m.risk_mu, parameters.distance_midpoint_m.risk_sigma, parameters.distance_midpoint_m.max / ( parameters.distance_midpoint_m.n_bins));
    net.setPriorNormalDistribution("intention_safe_distance_midpoint", parameters.distance_midpoint_m.safe_mu, parameters.distance_midpoint_m.safe_sigma, parameters.distance_midpoint_m.max / parameters.distance_midpoint_m.n_bins);
    net.setPriorNormalDistribution("intention_safe_distance_front", parameters.distance_front_m.safe_mu, parameters.distance_front_m.safe_sigma, parameters.distance_front_m.max / parameters.distance_front_m.n_bins);
    net.setPriorNormalDistribution("intention_distance_risk_of_collision_front", parameters.distance_front_m.risk_mu, parameters.distance_front_m.risk_sigma, parameters.distance_front_m.max / parameters.distance_front_m.n_bins);

    int cpa_dist_idx = 6;
    int colreg_idx = 7;
    int cpa_ample_time_idx = 8;

    int timestep = 60;
    int n_bins = 30;
    int multiply = 1;

    int head_on = 3;
    int overtake = -2;
    int crossing = -1;

    // Cpa distance
    net.setAisDistribution("intention_safe_distance_midpoint", "files/classified/classified_south.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, head_on);
    net.setAisDistribution("intention_safe_distance_front", "files/classified/classified_south.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, crossing);
    //net.setAisDistribution("intention_distance_risk_of_collision", "files/classified/classified_west_5.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, overtake);
    //net.setAisDistribution("intention_distance_risk_of_collision_front", "files/classified/classified_west_5.csv", colreg_idx, cpa_dist_idx, multiply, n_bins, crossing);

    // Cpa time, the model does NOT differ for the different situations
    net.setAmpleTimeDistribution("intention_ample_time", "files/classified/classified_west_5.csv", cpa_ample_time_idx, timestep, n_bins);
    net.save_network("files/intention_models/intention_model_with_grounding_manually_adjusted.xdsl");
}
