// This file includes the intentio model class. This saves all the information needed and implements the needed functions for intention inference.
#pragma once

#include <map>
#include "Eigen/Dense"
#include <string>
#include <sstream>
//#include <boost/math/special_functions/sign.hpp>
#include <math.h>
#include <optional>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "bayesian_network.h"
#include "identifiers.h"
#include "geometry.h"
#include "parameters.h"
#include "utils.h"


namespace INTENTION_INFERENCE
{
	class IntentionModel
	{
	private:
		const IntentionModelParameters &parameters;
		BayesianNetwork net;
		const int my_id;
		std::map<int, std::string> ship_name_map;
		std::vector<std::string> ship_names;
		const std::vector<std::string> intention_node_names_ship_specific = {"colav_situation_towards_", "priority_intention_to_", "disable_"};
		const std::vector<std::string> intention_node_names_general = {"intention_colregs_compliant", "intention_good_seamanship", "intention_safe_distance", "intention_safe_distance_front", "intention_safe_distance_midpoint", "intention_ample_time", "intention_ignoring_safety", "unmodelled_behaviour","intention_distance_risk_of_collision","intention_distance_risk_of_collision_front","intention_situation_start_distance"};
		//const std::vector<std::string> intention_node_names_general = {"intention_colregs_compliant", "intention_good_seamanship", "intention_safe_distance", "intention_safe_distance_front", "intention_safe_distance_midpoint", "intention_ample_time", "intention_ignoring_safety", "unmodelled_behaviour"};
		const std::vector<std::string> intermediate_node_names_ship_specific = {"is_pre_ample_time_to_", "safe_distance_at_CPA_towards_", "safe_crossing_front_towards_", "safe_distance_to_", "safe_distance_to_midpoint_", "gives_way_correct_towards_", "Observation_applicable_towards_", "role_towards_", "good_seamanship_to_","lowres_distance_at_cpa_towards_","lowres_crossing_distance_front_towards_","risk_of_collision_towards_","situation_started_towards_","will_give_way_to_","safely_passed_", "change_in_course_towards_", "change_in_speed_towards_","initial_course_towards_","initial_speed_towards_"};
		//const std::vector<std::string> intermediate_node_names_ship_specific = {"is_pre_ample_time_to_", "safe_distance_at_CPA_towards_", "safe_crossing_front_towards_", "safe_distance_to_", "safe_distance_to_midpoint_", "gives_way_correct_towards_", "Observation_applicable_towards_", "role_towards_", "good_seamanship_to_"};
		std::vector<std::string> intermediate_node_names_general = {"has_turned_portwards", "has_turned_starboardwards", "observation_applicable", "stands_on_correct"};
		std::vector<std::string> all_node_names;
		const std::string output_name = "observation_applicable";

		const std::map<int, Eigen::Vector4d> initial_ship_states;
		std::map<int, Eigen::Vector4d> previously_saved_ship_states;
		std::map<int, double> time_last_saved_shipstate;

		bool doSave(const std::map<int, Eigen::Vector4d> &ship_states, double time)
		{
			const auto min_time_between_saved_states = parameters.expanding_dbn.min_time_s;
			const auto max_time_between_saved_states = parameters.expanding_dbn.max_time_s;
			const auto heading_change_to_save = parameters.expanding_dbn.min_course_change_rad;
			const auto speed_change_to_save = parameters.expanding_dbn.min_speed_change_m_s;
			if (!previously_saved_ship_states.size())
			{
				previously_saved_ship_states = ship_states;
				for (const auto &[ship_id, state] : previously_saved_ship_states)
				{
					(void)state; // discard state to avoid compiler warning for unused variable
					time_last_saved_shipstate[ship_id] = time;
				}
				return false;
			}
			else
			{
				for (auto const &[ship_id, current_ship_state] : ship_states)
				{
					const auto &past_ship_state = better_at(previously_saved_ship_states, ship_id);
					const auto time_passed = time - better_at(time_last_saved_shipstate, ship_id);
					if (time_passed > min_time_between_saved_states && (time_passed > max_time_between_saved_states || std::abs(current_ship_state[CHI] - past_ship_state[CHI]) > heading_change_to_save || std::abs(current_ship_state[U] - past_ship_state[U]) > speed_change_to_save))
					{
						previously_saved_ship_states = ship_states;
						time_last_saved_shipstate[ship_id] = time;
						return true;
					}
				}
			}
			return false;
		}

		bool evaluate_nodes(std::ofstream &intentionFile, double time, double x, double y, double sog, double cog){
			intentionFile << my_id << ",";
            intentionFile << x << ",";
            intentionFile << y << ","; 
			intentionFile << cog << ",";
			intentionFile << sog << ",";
            intentionFile << time << ",";

			auto result = net.evaluateStates(all_node_names);
			bool has_started = true;

			auto intention_colregs_compliant = better_at(better_at(result, "intention_colregs_compliant"), "true");
			intentionFile << intention_colregs_compliant << ",";
			
			/*auto intention_distance_risk_of_collision = better_at(better_at(result, "intention_distance_risk_of_collision"), "true");
			intentionFile << intention_distance_risk_of_collision << ",";
			std::cout << "intention_distance_risk_of_collision: " << intention_distance_risk_of_collision << std::endl << std::flush;
			auto intention_distance_risk_of_collision_front = better_at(better_at(result, "intention_distance_risk_of_collision_front"), "true");
			intentionFile << intention_distance_risk_of_collision_front << ",";
			std::cout << "intention_distance_risk_of_collision_front: " << intention_distance_risk_of_collision_front << std::endl << std::flush;*/
			intentionFile << "0,0,";

			auto intention_ignoring_safety = better_at(better_at(result, "intention_ignoring_safety"), "true");
			auto intention_good_seamanship = better_at(better_at(result, "intention_good_seamanship"), "true");
			intentionFile << intention_good_seamanship << ",";
			auto intention_unmodeled_behaviour = better_at(better_at(result, "unmodelled_behaviour"), "true");
			intentionFile << intention_unmodeled_behaviour << ",";
			auto has_turned_portwards = better_at(better_at(result, "has_turned_portwards"), "true");
			intentionFile << has_turned_portwards << ",";
			auto has_turned_starboardwards = better_at(better_at(result, "has_turned_starboardwards"), "true");
			intentionFile << has_turned_starboardwards << ",";
			auto stands_on_correct = better_at(better_at(result, "stands_on_correct"), "true");
			intentionFile << stands_on_correct << ",";
			auto observation_applicable = better_at(better_at(result, "observation_applicable"), "true");
			


			/*// Convert from map to vector
			std::transform(better_at(result, "intention_ample_time").begin(), better_at(result, "intention_ample_time").end(), std::back_inserter(node_state_msg->intention_ample_time), [](std::pair<std::string, double> v)
						   { return (float)v.second; });
			std::transform(better_at(result, "intention_safe_distance_front").begin(), better_at(result, "intention_safe_distance_front").end(), std::back_inserter(node_state_msg->intention_safe_distance_front), [](std::pair<std::string, double> v)
						   { return (float)v.second; });
			std::transform(better_at(result, "intention_safe_distance").begin(), better_at(result, "intention_safe_distance").end(), std::back_inserter(node_state_msg->intention_safe_distance), [](std::pair<std::string, double> v)
						   { return (float)v.second; });
			std::transform(better_at(result, "intention_safe_distance_midpoint").begin(), better_at(result, "intention_safe_distance_midpoint").end(), std::back_inserter(node_state_msg->intention_safe_distance_midpoint), [](std::pair<std::string, double> v)
						   { return (float)v.second; });*/


			for (auto const &[ship_id, ship_name] : ship_name_map)
			{
				if (ship_id != my_id)
				{
					
					auto intention_colav_situation_CR_PS = better_at(better_at(result, "colav_situation_towards_" + ship_name), "CR_PS");
					intentionFile << intention_colav_situation_CR_PS << ",";
				
					auto intention_colav_situation_CR_SS = better_at(better_at(result, "colav_situation_towards_" + ship_name), "CR_SS");
					intentionFile << intention_colav_situation_CR_SS << ",";
		
					auto intention_colav_situation_HO = better_at(better_at(result, "colav_situation_towards_" + ship_name), "HO");
					intentionFile << intention_colav_situation_HO << ",";

					auto intention_colav_situation_OT_en = better_at(better_at(result, "colav_situation_towards_" + ship_name), "OT_en");
					intentionFile << intention_colav_situation_OT_en << ",";

					auto intention_colav_situation_OT_ing = better_at(better_at(result, "colav_situation_towards_" + ship_name), "OT_ing");
					intentionFile << intention_colav_situation_OT_ing << ",";


					auto priority_intention_lower = better_at(better_at(result, "priority_intention_to_" + ship_name), "lower");
					intentionFile << priority_intention_lower << ",";
					auto priority_intention_similar = better_at(better_at(result, "priority_intention_to_" + ship_name), "similar");
					intentionFile << priority_intention_similar << ",";
					auto priority_intention_higher = better_at(better_at(result, "priority_intention_to_" + ship_name), "higher");
					intentionFile << priority_intention_higher << ",";

					auto intention_is_risk_of_colision = better_at(better_at(result, "risk_of_collision_towards_" + ship_name), "true");
					intentionFile << intention_is_risk_of_colision << ",";
					auto situation_started = better_at(better_at(result, "situation_started_towards_"+ ship_name), "true");
					intentionFile << situation_started << ",";
					has_started = has_started && (situation_started>0.98);
					std::cout << "Situation started: " << situation_started << std::endl << std::flush;
					auto will_give_way = better_at(better_at(result, "will_give_way_to_"+ ship_name), "true");
					intentionFile << will_give_way << ",";
					auto is_pre_ample_time = better_at(better_at(result, "is_pre_ample_time_to_"+ ship_name), "true");
					intentionFile << is_pre_ample_time << ",";
					auto safe_distance = better_at(better_at(result, "safe_distance_to_"+ ship_name), "true");
					intentionFile << safe_distance << ",";
					auto safe_distance_to_midpoint = better_at(better_at(result, "safe_distance_to_midpoint_"+ ship_name), "true");
					intentionFile << safe_distance_to_midpoint << ",";
					auto gives_way_correct = better_at(better_at(result, "gives_way_correct_towards_"+ ship_name), "true");
					intentionFile << gives_way_correct << ",";
					auto stand_on_role = better_at(better_at(result, "role_towards_"+ ship_name), "SO");
					intentionFile << stand_on_role << ",";
					auto change_in_course_port = better_at(better_at(result, "change_in_course_towards_"+ ship_name), "portwards");
					intentionFile << change_in_course_port << ",";
					auto change_in_course_starboard = better_at(better_at(result, "change_in_course_towards_"+ ship_name), "starboardwards");
					intentionFile << change_in_course_starboard << ",";
					auto change_in_speed_lower = -1*better_at(better_at(result, "change_in_speed_towards_"+ ship_name), "lower");
					intentionFile << change_in_speed_lower << ",";
					auto change_in_speed_higher = -1*better_at(better_at(result, "change_in_speed_towards_"+ ship_name), "higher");
					intentionFile << change_in_speed_higher << ",";

					auto mean_initial_course = mean(better_at(result, "initial_course_towards_"+ ship_name))*360-180;
					intentionFile << mean_initial_course << ",";
					auto max_initial_course = max(better_at(result, "initial_course_towards_"+ ship_name))*360-180;
					intentionFile << max_initial_course << ",";
					auto mean_initial_speed = mean(better_at(result, "initial_speed_towards_"+ ship_name))*parameters.speed.max;
					intentionFile << mean_initial_speed << ",";
					auto max_initial_speed = max(better_at(result, "initial_speed_towards_"+ ship_name))*parameters.speed.max;
					intentionFile << max_initial_speed << ",";


					//{"", "safe_distance_at_CPA_towards_", "safe_crossing_front_towards_", "", "Observation_applicable_towards_", "", "good_seamanship_to_","lowres_distance_at_cpa_towards_","lowres_crossing_distance_front_towards_","safely_passed_", }
				}
			}

			intentionFile << "\n";
			return has_started;
		}


	public:
		IntentionModel(std::string network_file_name, const IntentionModelParameters &parameters, int my_id, const std::map<int, Eigen::Vector4d> &ship_states) : IntentionModel(network_file_name, parameters, my_id, ship_states, std::map<std::string, std::string>{}) {}

		IntentionModel(std::string network_file_name, const IntentionModelParameters &parameters, int my_id, const std::map<int, Eigen::Vector4d> &ship_states, const std::map<std::string, std::string> &priors) : parameters(parameters),
																																																					net(network_file_name, parameters.number_of_network_evaluation_samples),
																																																					my_id(my_id),
																																																					initial_ship_states(ship_states)
		{
			ship_names.clear();
			for (unsigned i = 0; i < parameters.max_number_of_obstacles; ++i)
			{
				std::string name = "ship" + std::to_string(i);
				ship_names.push_back(name);
			}

			// Convert ship-ids to the names used in the network
			size_t i = 0;
			for (auto const &[ship_id, ship_state] : ship_states)
			{
				(void)ship_state; // discard unused variable
				if (ship_id != my_id)
				{
					ship_name_map[ship_id] = ship_names[i];
					printf("\nAdded ship name \"%s\" for ship id %d", ship_names[i].c_str(), ship_id);
					++i;
				}
			}

			// Generate node names
			all_node_names.insert(all_node_names.end(), intention_node_names_general.begin(), intention_node_names_general.end());
			all_node_names.insert(all_node_names.end(), intermediate_node_names_general.begin(), intermediate_node_names_general.end());
			for (auto [ship_id, ship_name] : ship_name_map)
			{
				(void)ship_id;
				for (auto node : intention_node_names_ship_specific)
				{
					all_node_names.push_back(node + ship_name);
				}
				for (auto node : intermediate_node_names_ship_specific)
				{
					all_node_names.push_back(node + ship_name);
				}
			}

	
			net.setEvidence(priors);

			// Initiate colregs situation
			/*for (auto const &[ship_id, ship_state] : ship_states)
			{
				if (ship_id != my_id)
				{
					std::string ship_name = better_at(ship_name_map, ship_id);
					const auto situation = evaluateSitution(parameters, better_at(ship_states, my_id), ship_state);
					net.setPriors("colav_situation_towards_" + ship_name, situation);

					std::stringstream situation_ss;
					situation_ss.precision(2);
					situation_ss << "Ship " << my_id << " init colav situation towards ship " << ship_id << " as: ";
					for (const auto &[name, value] : situation)
					{
						situation_ss << name << "=" << value << ", ";
					}
					auto s = situation_ss.str();
					printf("%s", s.c_str());
				}
			}*/

			net.setBinaryPriors("intention_ignoring_safety", parameters.ignoring_safety_probability);
			net.setBinaryPriors("intention_colregs_compliant", parameters.colregs_compliance_probability);
			net.setBinaryPriors("intention_good_seamanship", parameters.good_seamanship_probability);
			net.setBinaryPriors("unmodelled_behaviour", parameters.unmodeled_behaviour);

			net.setHardLimitBinaryOutcomePriors("ample_time_acceptable", parameters.ample_time_s.minimal_accepted_by_ownship, parameters.ample_time_s.max);
			net.setHardLimitBinaryOutcomePriors("safe_distance_front_acceptable", parameters.safe_distance_front_m.minimal_accepted_by_ownship, parameters.safe_distance_front_m.max);
			net.setHardLimitBinaryOutcomePriors("safe_distance_acceptable", parameters.safe_distance_m.minimal_accepted_by_ownship, parameters.safe_distance_m.max);
			net.setHardLimitBinaryOutcomePriors("safe_distance_midpoint_acceptable", parameters.safe_distance_midpoint_m.minimal_accepted_by_ownship, parameters.safe_distance_midpoint_m.max);

			for (auto [ship_id, ship_name] : ship_name_map)
			{
				if (ship_id != my_id)
				{
					net.setPriors("priority_intention_to_" + ship_name, parameters.priority_probability);
				}
			}

			if(!parameters.use_ais_distributions){
				net.setPriorNormalDistribution("intention_ample_time", parameters.ample_time_s.mu, parameters.ample_time_s.sigma, parameters.ample_time_s.max / parameters.ample_time_s.n_bins);
				net.setPriorNormalDistribution("intention_distance_risk_of_collision", parameters.risk_distance_m.mu, parameters.risk_distance_m.sigma, parameters.risk_distance_m.max / parameters.risk_distance_m.n_bins);
				net.setPriorNormalDistribution("intention_distance_risk_of_collision_front", parameters.risk_distance_front_m.mu, parameters.risk_distance_front_m.sigma, parameters.risk_distance_front_m.max / parameters.risk_distance_front_m.n_bins);
				net.setPriorNormalDistribution("intention_safe_distance_midpoint", parameters.safe_distance_midpoint_m.mu, parameters.safe_distance_midpoint_m.sigma, parameters.safe_distance_midpoint_m.max / parameters.safe_distance_midpoint_m.n_bins);
				net.setPriorNormalDistribution("intention_safe_distance", parameters.safe_distance_m.mu, parameters.safe_distance_m.sigma, parameters.safe_distance_m.max / parameters.safe_distance_m.n_bins);
				net.setPriorNormalDistribution("intention_safe_distance_front", parameters.safe_distance_front_m.mu, parameters.safe_distance_front_m.sigma, parameters.safe_distance_front_m.max / parameters.safe_distance_front_m.n_bins);
				net.setPriorNormalDistribution("intention_situation_start_distance", parameters.situation_start_distance.mu, parameters.situation_start_distance.sigma, parameters.situation_start_distance.max / parameters.situation_start_distance.n_bins);
			}
			else{
				// MOVE LATER
				int cpa_dist_idx = 0;
				int colreg_idx = 1;
				// TODO: Find indexes automatically
				int ample_time_idx = 2;

				int timestep = 60;
				int multiply =1;

				//TODO these are used hard-coded inside other functions. This makes it look like you can change them even though its not possible. Maybe use #define or a global static constant variable?
				int head_on = 3;
				int overtake = -2;
				int crossing = -1;

				net.setPriorNormalDistribution("intention_situation_start_distance", parameters.situation_start_distance.mu, parameters.situation_start_distance.sigma, parameters.situation_start_distance.max / parameters.situation_start_distance.n_bins);

				net.setPriorNormalDistribution("intention_distance_risk_of_collision", parameters.risk_distance_m.mu, parameters.risk_distance_m.sigma, parameters.risk_distance_m.max / parameters.risk_distance_m.n_bins);
				net.setPriorNormalDistribution("intention_distance_risk_of_collision_front", parameters.risk_distance_front_m.mu, parameters.risk_distance_front_m.sigma, parameters.risk_distance_front_m.max / parameters.risk_distance_front_m.n_bins);

				// Cpa distance
				net.setAisDistribution("intention_safe_distance_midpoint", "west_rel.csv", colreg_idx, cpa_dist_idx, multiply, parameters.safe_distance_midpoint_m.n_bins, 0, parameters.safe_distance_midpoint_m.max);
				net.setAisDistribution("intention_safe_distance", "west_rel.csv", colreg_idx, cpa_dist_idx, multiply, parameters.safe_distance_m.n_bins,0, parameters.safe_distance_m.max);
				net.setAisDistribution("intention_safe_distance_front", "west_rel.csv", colreg_idx, cpa_dist_idx, multiply, parameters.safe_distance_front_m.n_bins,0, parameters.safe_distance_m.max);
				//net.setPriorNormalDistribution("intention_safe_distance_front", parameters.safe_distance_front_m.mu, parameters.safe_distance_front_m.sigma, parameters.safe_distance_front_m.max / parameters.safe_distance_front_m.n_bins);
				//TODO: Set safe distance front based on the safe_distance distribution somehow
				
				// Cpa time, the model does NOT differ for the different situations
				//net.setPriorNormalDistribution("intention_ample_time", parameters.ample_time_s.mu, parameters.ample_time_s.sigma, parameters.ample_time_s.max / parameters.ample_time_s.n_bins);
				//ample_time <800, found in python
				std::vector<double> west_unfiltered = {0.1686746987951807, 0.321285140562249, 0.21686746987951808, 0.08433734939759036, 0.04618473895582329, 0.03614457831325301, 0.02610441767068273, 0.018072289156626505, 0.01606425702811245, 0.004016064257028112, 0.002008032128514056, 0.008032128514056224, 0.006024096385542169, 0.006024096385542169, 0.002008032128514056, 0.002008032128514056, 0.002008032128514056, 0.0, 0.002008032128514056, 0.0, 0.002008032128514056, 0.002008032128514056, 0.0, 0.002008032128514056, 0.002008032128514056, 0.0, 0.002008032128514056, 0.0, 0.0, 0.002008032128514056};
				//ample_time < 400
				std::vector<double> west_filtered = {0.09437751004016057, 0.10040160642570281, 0.1465863453815261, 0.14859437751004015, 0.13253012048192772, 0.10441767068273092, 0.04819277108433735, 0.04618473895582329, 0.02208835341365462, 0.02208835341365462, 0.014056224899598393, 0.02208835341365462, 0.01606425702811245, 0.01606425702811245, 0.004016064257028112, 0.012048192771084338, 0.006024096385542169, 0.010040160642570281, 0.006024096385542169, 0.0, 0.002008032128514056, 0.002008032128514056, 0.002008032128514056, 0.004016064257028112, 0.004016064257028112, 0.004016064257028112, 0.002008032128514056, 0.002008032128514056, 0.0, 0.006024096385542169};
				
				net.setAmpleTimeDistribution("intention_ample_time", "west_rel.csv", ample_time_idx, parameters.ample_time_s.n_bins, 0, parameters.ample_time_s.max, west_filtered);
			}
			net.save_network("modified_network.xdsl");
		}

		bool insertObservation(const IntentionModelParameters parameters, int &ot_en, const std::map<int, Eigen::Vector4d> ship_states, std::vector<int> currently_tracked_ships, bool is_changing_course, double time, std::ofstream &intentionFile,std::ofstream &measurementFile, std::ofstream &measurementIdentifiersFile)
		{
			measurementFile << my_id << ",";
			measurementIdentifiersFile << my_id << ",";
            measurementFile << time << ",";
			measurementIdentifiersFile << time << ",";
			bool did_save = false;

			if (doSave(ship_states, time))
			{
				net.incrementTime();
				did_save = true;
			}
			measurementFile << did_save << ",";
			
		
			net.setEvidence("measured_course",courseIdentifier(parameters, better_at(ship_states,my_id)[CHI]));
			measurementFile << better_at(ship_states,my_id)[CHI] << ",";
			measurementIdentifiersFile << courseIdentifier(parameters, better_at(ship_states,my_id)[CHI]) << ",";

			net.setEvidence("measured_speed",speedIdentifier(parameters, better_at(ship_states,my_id)[U]));
			measurementFile << better_at(ship_states,my_id)[U] << ",";
			measurementIdentifiersFile << speedIdentifier(parameters, better_at(ship_states,my_id)[U]) << ",";

			net.setEvidence("is_changing_course", is_changing_course);  
			measurementFile << is_changing_course << ",";
			
			bool has_passed = true;
			std::vector<std::string> handled_ship_names;
			for (auto const &ship_id : currently_tracked_ships)
			{
				if (ship_id != my_id)
				{
					const std::string ship_name = better_at(ship_name_map, ship_id);
					const auto ship_state = better_at(ship_states, ship_id);
					handled_ship_names.push_back(ship_name);

					
					const auto current_colav_situation = evaluateSitution(parameters,better_at(ship_states,my_id), ship_state);
					net.setVirtualEvidence("measured_colav_situation_towards_"+ship_name, current_colav_situation);
					for(const auto key : {"CR_PS","CR_SS","HO","OT_en","OT_ing"}){
						measurementFile << better_at(current_colav_situation,key) << ",";
					}

					auto currentDistance = evaluateDistance(better_at(ship_states, my_id)[PX]-better_at(ship_states, ship_id)[PX], better_at(ship_states, my_id)[PY]-better_at(ship_states,ship_id)[PY]);
					net.setEvidence("distance_to_"+ship_name, currentDistanceIdentifier(parameters, currentDistance));
					measurementFile << currentDistance << ",";
					measurementIdentifiersFile << currentDistanceIdentifier(parameters, currentDistance) << ",";

					CPA cpa = evaluateCPA(better_at(ship_states, my_id), ship_state);

					net.setEvidence("disable_" + ship_name, "enabled");

					net.setEvidence("time_untill_closest_point_of_approach_towards_" + ship_name, timeIdentifier(parameters, cpa.time_untill_CPA));
					measurementFile << cpa.time_untill_CPA << ",";
					measurementIdentifiersFile << timeIdentifier(parameters, cpa.time_untill_CPA) << ",";

					net.setEvidence("distance_at_cpa_towards_" + ship_name, highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));
					measurementIdentifiersFile << highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA) << ",";
					net.setEvidence("lowres_distance_at_cpa_towards_" + ship_name, lowresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));
					measurementFile <<  cpa.distance_at_CPA << ",";
					measurementIdentifiersFile << lowresCPADistanceIdentifier(parameters, cpa.distance_at_CPA) << ",";

					double crossing_in_front_distance = crossingInFrontDistance(better_at(ship_states, my_id), ship_state);
					net.setEvidence("crossing_distance_front_towards_" + ship_name, crossInFrontHighresIdentifier(parameters, crossing_in_front_distance));
					measurementIdentifiersFile << crossInFrontHighresIdentifier(parameters, crossing_in_front_distance) << ",";
					net.setEvidence("lowres_crossing_distance_front_towards_" + ship_name, crossInFrontLowresIdentifier(parameters, crossing_in_front_distance));
					measurementFile << crossing_in_front_distance << ",";
					measurementIdentifiersFile << crossInFrontLowresIdentifier(parameters, crossing_in_front_distance) << ",";

					auto distanceToMidpointResult = distanceToMidpointCourse(better_at(ship_states, my_id), ship_state);
				
					net.setEvidence("two_times_distance_to_midpoint_at_cpa_to_" + ship_name, twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint));
					measurementFile << distanceToMidpointResult.distance_to_midpoint << ",";
					measurementIdentifiersFile << twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint) << ",";
					
					net.setEvidence("crossing_with_midpoint_on_side_"+ship_name, crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side));
					measurementFile << distanceToMidpointResult.crossing_with_midpoint_on_port_side << ",";
					measurementIdentifiersFile << crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side) << ",";

					net.setEvidence("passed_" + ship_name, hasPassedIdentifier(cpa.time_untill_CPA));
					measurementFile << (hasPassedIdentifier(cpa.time_untill_CPA)=="true") << ",";
					has_passed = has_passed && (hasPassedIdentifier(cpa.time_untill_CPA)=="true");


					net.setEvidence("crossing_wiht_other_on_port_side_to_" + ship_name, crossing_port_starboard_identifier(cpa.bearing_relative_to_heading));
					measurementFile << (crossing_port_starboard_identifier(cpa.bearing_relative_to_heading)=="port") << ",";
				}
			}
			measurementFile << "\n";
			measurementIdentifiersFile << "\n";
	
			for (const auto ship_name1 : ship_names)
			{
				if (!std::count(handled_ship_names.begin(), handled_ship_names.end(), ship_name1))
				{
					net.setEvidence("disable_" + ship_name1, "disabled");
				}
			}

			net.setEvidence(output_name, "true");
			bool has_started = evaluate_nodes(intentionFile, time, better_at(ship_states, my_id)[PX], better_at(ship_states, my_id)[PY], better_at(ship_states, my_id)[U], better_at(ship_states, my_id)[CHI]);
			
			return has_started && has_passed;
			
		}

	};
}
