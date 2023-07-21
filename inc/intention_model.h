/**
 * @file intention_model.h
 * @author Sverre Velten Rothmund
 * @brief This file includes the intention model class. This saves
*         all the information needed and implements the needed
*         functions for intention inference.
 * @version 1.0
 * @date 2022
 * 
 * @copyright Copyright (c) 2022
 * 
 */
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
#include <deque>

//TODO fiks:
// ros::Time::now() <- tidspunktet for denne målingen
// custom_msgs::... <- er bare brukt for å vise frem resultatet, kan fjernes (erstattes?)
// putt inn tid som input i observation

namespace INTENTION_INFERENCE
{
	/**
	 * @brief
	 *
	 */
	class IntentionModel
	{
	private:
		const IntentionModelParameters &parameters;
		BayesianNetwork net;
		const int my_id; /* mmsi of own ship */
		Eigen::Vector4d my_initial_ship_state; /* Ship state from where intentions are calculated */
		std::map<int, std::string> ship_name_map;
		std::vector<std::string> ship_names;
		std::map<int, Eigen::Vector4d> ship_states_before_cpa_limit; /* Used for the insertObservation function to store last ship state while CPA > 240 */
		std::map<std::string,std::map<std::string,double>> intention_model_predictions; /* Predictions from observations. Is changed by the insertObservation fuction */
		int write_start_to_file = 0; /* Either 1 or 0. For writing to file*/
		int remove_start = 0; /* Either 1 or 0. For writing to file*/
		bool my_current_risk = false;
		bool my_start = false;
		bool my_risk_of_collision = false;
		std::deque<std::map<int, Eigen::Vector4d >> ship_states_history;

		const std::vector<std::string> intention_node_names_ship_specific = {"colav_situation_towards_"
                                                                             , "priority_intention_to_"
                                                                             , "disable_"};
		const std::vector<std::string> intention_node_names_general = {"intention_colregs_compliant"
                                                                       , "intention_good_seamanship"
                                                                       , "intention_safe_distance"
                                                                       , "intention_safe_distance_front"
                                                                       , "intention_safe_distance_midpoint"
                                                                       , "intention_ample_time"
                                                                       , "intention_ignoring_safety"
                                                                       , "unmodelled_behaviour"
                                                                       ,"intention_distance_risk_of_collision"
                                                                       ,"intention_distance_risk_of_collision_front"};
		//const std::vector<std::string> intention_node_names_general = {"intention_colregs_compliant", "intention_good_seamanship", "intention_safe_distance", "intention_safe_distance_front", "intention_safe_distance_midpoint", "intention_ample_time", "intention_ignoring_safety", "unmodelled_behaviour"};
		const std::vector<std::string> intermediate_node_names_ship_specific = {"risk_of_collision_towards_"
                                                                                , "is_pre_ample_time_to_"
                                                                                , "safe_distance_at_CPA_towards_"
                                                                                , "safe_crossing_front_towards_"
                                                                                , "safe_distance_to_"
                                                                                , "safe_distance_to_midpoint_"
                                                                                , "gives_way_correct_towards_"
                                                                                , "Observation_applicable_towards_"
                                                                                , "role_towards_"
                                                                                , "good_seamanship_to_"
                                                                                , "Current_risk_of_collision_CPA_towards_"
                                                                                , "Current_risk_of_collision_front_towards_"
                                                                                , "Current_risk_of_collision_towards_"};
		//const std::vector<std::string> intermediate_node_names_ship_specific = {"is_pre_ample_time_to_", "safe_distance_at_CPA_towards_", "safe_crossing_front_towards_", "safe_distance_to_", "safe_distance_to_midpoint_", "gives_way_correct_towards_", "Observation_applicable_towards_", "role_towards_", "good_seamanship_to_"};
		std::vector<std::string> intermediate_node_names_general = {"has_turned_portwards"
                                                                    , "has_turned_starboardwards"
                                                                    , "observation_applicable"
                                                                    , "stands_on_correct"
                                                                    , "change_in_speed"
                                                                    , "is_changing_course"};
		std::vector<std::string> all_node_names;
		const std::string output_name = "observation_applicable";

		std::map<int, Eigen::Vector4d> previously_saved_ship_states;
		std::map<int, double> time_last_saved_shipstate;

		/**
		 * @brief 
		 * 
		 * @param ship_states 
		 * @param time 
		 * @return true 
		 * @return false 
		 */
		bool doSave(const std::map<int, Eigen::Vector4d> &ship_states, double time)
		{
			const auto min_time_between_saved_states = 20;
			const auto max_time_between_saved_states = 1200;
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

		/**
		 * @brief
		 *
		 * @param time_cpa
		 */
		bool check_remove_steps(double time_cpa) {
			double unmodeled = better_at(better_at(intention_model_predictions, "unmodelled_behaviour"), "true");
			double cost_remove_steps = time_cpa*unmodeled;
			if ((unmodeled>0.5 ) && time_cpa > 60 && cost_remove_steps>60){
				return true;
			}
			return false;
		}

		/**
		 * @brief
		 *
		 */
		void restart(){
			net.clearEvidence();
			net.restartTime();
		}

		/**
		 * @brief Adds elements to file in the following format (only new line at tje end):
		 * mmsi,x,y,time,colreg_compliant,good_seamanship,unmodeled_behaviour,
		 * has_turned_portwards,has_turned_starboardwards,change_in_speed,is_changing_course,
		 * CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher,risk_of_collision,current_risk_of_collision,start
		 *
		 * @param intentionFile path to csv file to be written to. Should already be opened
		 * @param time
		 * @param x
		 * @param y
		 */
		void write_results_to_file(std::ofstream &intentionFile,
								   double time, double x, double y){
			intentionFile << my_id << ",";
            intentionFile << x << ",";
            intentionFile << y << ",";
            intentionFile << time << ",";

			auto intention_colregs_compliant = better_at(better_at(intention_model_predictions, "intention_colregs_compliant"), "true");
			intentionFile << intention_colregs_compliant << ",";
			auto intention_good_seamanship = better_at(better_at(intention_model_predictions, "intention_good_seamanship"), "true");
			intentionFile << intention_good_seamanship << ",";
			auto intention_unmodeled_behaviour = better_at(better_at(intention_model_predictions, "unmodelled_behaviour"), "true");
			intentionFile << intention_unmodeled_behaviour << ",";
			auto has_turned_portwards = better_at(better_at(intention_model_predictions, "has_turned_portwards"), "true");
			intentionFile << has_turned_portwards << ",";
			auto has_turned_starboardwards = better_at(better_at(intention_model_predictions, "has_turned_starboardwards"), "true");
			intentionFile << has_turned_starboardwards << ",";
			auto change_in_speed = !better_at(better_at(intention_model_predictions, "change_in_speed"), "similar");
			intentionFile << change_in_speed << ",";
			auto check_is_changing_course = better_at(better_at(intention_model_predictions, "is_changing_course"), "true");
			intentionFile << check_is_changing_course << ",";

			for (auto const &[ship_id, ship_name] : ship_name_map)
			{
				if (ship_id != my_id)
				{
					auto intention_colav_situation_CR_PS = better_at(better_at(intention_model_predictions, "colav_situation_towards_" + ship_name), "CR_PS");
					intentionFile << intention_colav_situation_CR_PS << ",";
					auto intention_colav_situation_CR_SS = better_at(better_at(intention_model_predictions, "colav_situation_towards_" + ship_name), "CR_SS");
					intentionFile << intention_colav_situation_CR_SS << ",";
					auto intention_colav_situation_HO = better_at(better_at(intention_model_predictions, "colav_situation_towards_" + ship_name), "HO");
					intentionFile << intention_colav_situation_HO << ",";
					auto intention_colav_situation_OT_en = better_at(better_at(intention_model_predictions, "colav_situation_towards_" + ship_name), "OT_en");
					intentionFile << intention_colav_situation_OT_en << ",";
					auto intention_colav_situation_OT_ing = better_at(better_at(intention_model_predictions, "colav_situation_towards_" + ship_name), "OT_ing");
					intentionFile << intention_colav_situation_OT_ing << ",";
					auto priority_intention_lower = better_at(better_at(intention_model_predictions, "priority_intention_to_" + ship_name), "lower");
					intentionFile << priority_intention_lower << ",";
					auto priority_intention_similar = better_at(better_at(intention_model_predictions, "priority_intention_to_" + ship_name), "similar");
					intentionFile << priority_intention_similar << ",";
					auto priority_intention_higher = better_at(better_at(intention_model_predictions, "priority_intention_to_" + ship_name), "higher");
					intentionFile << priority_intention_higher<< ",";
					auto risk_of_collision = better_at(better_at(intention_model_predictions, "risk_of_collision_towards_"+ ship_name), "true");
					intentionFile << risk_of_collision<< ",";
					auto current_risk_of_collision = better_at(better_at(intention_model_predictions, "Current_risk_of_collision_towards_" + ship_name), "true");
					intentionFile << current_risk_of_collision<<",";
					if (remove_start){
						write_start_to_file = 1;
						remove_start = 0;
					}
					intentionFile << write_start_to_file << std::endl;
				}
			}
		}

	public:
		/**
		 * @brief Construct a new Intention Model object. Initializes a bayesian network using priors
		 * from an .xdsl file. 
		 * 
		 * @param network_file_name File name of intention model. Typically a path to an .xdsl file
		 * @param parameters Parameter object used to construct prior distributions
		 * @param my_id mmsi of own ship
		 * @param ship_states a map of states of all ships with mmsi as key and
		 * object as a 4d vector consisting of x, y, sog, cog, where this acts
		 * as startpoint or initial state for the intention inference
		 */
		IntentionModel(std::string network_file_name,
					   const IntentionModelParameters &parameters,
					   int my_id, const std::map<int,
					   Eigen::Vector4d> &ship_states) : IntentionModel(network_file_name,
                                                                       parameters,
                                                                       my_id,
                                                                       ship_states,
                                                                       std::map<std::string, std::string>{}){}

		IntentionModel(std::string network_file_name,
					   const IntentionModelParameters &parameters,
					   int my_id, const std::map<int, Eigen::Vector4d> &ship_states,
					   const std::map<std::string, std::string> &priors) : parameters(parameters),
                                                                           net(network_file_name, parameters.number_of_network_evaluation_samples),
                                                                           my_id(my_id),
																		   ship_states_before_cpa_limit(ship_states),
                                                                           my_initial_ship_state(better_at(ship_states,my_id))
		{
			ship_names.clear();
			ship_states_history.clear();

			ship_states_history.push_back(ship_states);
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
			for (auto const &[ship_id, ship_state] : ship_states)
			{
				if (ship_id != my_id)
				{
					std::string ship_name = better_at(ship_name_map, ship_id);
					const auto situation = evaluateRelativeSituation2(parameters, better_at(ship_states, my_id), ship_state);
					net.setPriors("colav_situation_towards_" + ship_name, situation);
				}
			}

			for (auto [ship_id, ship_name] : ship_name_map)
			{
				if (ship_id != my_id)
				{
					net.setPriors("priority_intention_to_" + ship_name, parameters.priority_probability);
				}
			}
		}

		/**
		 * @brief Updates intention model based on given data.
		 *
		 * @param parameters Parameter object of intention model
 		 * @param ship_states
 		 * @param last_ship_states
 		 * @param currently_tracked_ships ship list of all ships (including own)
 		 * @param check_changing_course
 		 * @param risk_of_collision A map of ships and if they have a risk of collision. This
 		 * map needs to be parsed between all intention models.
		 * @param new_timestep if true, indicates that all ships intention models have been updated for
		 * the previous time step, and that we now are in a new time step
		 * @param start indicator if startpoint has been set
		 * @return bool true or false. Returns start for pybind11 compatability
		 */
		bool insertObservation(const std::map<int, Eigen::Vector4d> ship_states
							   , std::map<int, Eigen::Vector4d> last_ship_states
							   , std::vector<int> currently_tracked_ships
							   )
		{
			bool did_save = false;

			net.setEvidence("change_in_course", changeInCourseIdentifier(parameters, better_at(ship_states, my_id)[CHI], my_initial_ship_state[CHI]));
			net.setEvidence("change_in_speed", changeInSpeedIdentifier(parameters, better_at(ship_states, my_id)[U], my_initial_ship_state[U]));

			bool is_changing_course = currentChangeInCourseIdentifier(better_at(ship_states, my_id)[CHI], last_ship_states[my_id][CHI]);
			net.setEvidence("is_changing_course", is_changing_course);
			bool other_is_changing_course = false;
			std::vector<std::string> handled_ship_names;
			CPA cpa;
			for (auto const &ship_id : currently_tracked_ships)
			{
				if (ship_id != my_id)
				{
					other_is_changing_course = currentChangeInCourseIdentifier(better_at(ship_states, ship_id)[CHI], last_ship_states[ship_id][CHI]);
					const std::string ship_name = better_at(ship_name_map, ship_id);
					const auto ship_state = better_at(ship_states, ship_id);
					handled_ship_names.push_back(ship_name);

					cpa = evaluateCPA(better_at(ship_states, my_id), ship_state);

					net.setEvidence("disable_" + ship_name, "enabled");
					net.setEvidence("time_untill_closest_point_of_approach_towards_" + ship_name, timeIdentifier(parameters, cpa.time_untill_CPA));
					net.setEvidence("distance_at_cpa_towards_" + ship_name, highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));

					double crossing_in_front_distance = crossingInFrontDistance(better_at(ship_states, my_id), ship_state);
					net.setEvidence("crossing_distance_front_towards_" + ship_name, crossInFrontHighresIdentifier(parameters, crossing_in_front_distance));

					auto distanceToMidpointResult = distanceToMidpointCourse(better_at(ship_states, my_id), ship_state);
					net.setEvidence("two_times_distance_to_midpoint_at_cpa_to_" + ship_name, twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint));
					net.setEvidence("crossing_with_midpoint_on_side_"+ship_name, crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side));

					net.setEvidence("aft_front_crossing_side_to_" + ship_name, frontAftIdentifier(cpa.passing_in_front));
					net.setEvidence("passed_" + ship_name, hasPassedIdentifier(cpa.time_untill_CPA));
					net.setEvidence("crossing_with_other_on_port_side_to_" + ship_name, crossing_port_starboard_identifier(cpa.bearing_relative_to_heading));

					net.setEvidence("lowres_distance_at_cpa_towards_" + ship_name, lowresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));
					net.setEvidence("lowres_crossing_distance_front_towards_" + ship_name, crossInFrontLowresIdentifier(parameters, crossing_in_front_distance));

					if (cpa.time_untill_CPA > 240){
						ship_states_before_cpa_limit = ship_states;
				    }
					std::map<std::string, double> situation = evaluateRelativeSituation2(parameters, better_at(ship_states_before_cpa_limit, my_id), better_at(ship_states_before_cpa_limit, ship_id));
					net.setVirtualEvidence("colav_situation_towards_" + ship_name, situation);
				}
			}

			for (const auto ship_name : ship_names)
			{
				if (!std::count(handled_ship_names.begin(), handled_ship_names.end(), ship_name))
				{
					net.setEvidence("disable_" + ship_name, "disabled");
				}
			}

			net.setEvidence(output_name, "true");
			intention_model_predictions = net.evaluateStates(all_node_names);
			int other_ship_id;

			for (auto const &ship_id : currently_tracked_ships)
			{
				if (ship_id != my_id)
				{
					my_current_risk = false;
					const std::string ship_name = better_at(ship_name_map, ship_id);
					auto result_risk_of_collision = better_at(better_at(intention_model_predictions, "risk_of_collision_towards_"+ ship_name), "true");
					bool check_changing_course = better_at(better_at(intention_model_predictions, "is_changing_course"), "true");
					other_ship_id = ship_id;
					//&& !check_changing_course[my_id] &&!check_changing_course[other_ship_id] && (cpa.time_untill_CPA<600)
					if(result_risk_of_collision>0.9  && !check_changing_course){
						my_risk_of_collision = true;
						my_current_risk = true;
					}
				}
			}

			write_start_to_file = 0;
			/* This sets new initial conditions if we either are on a start or a new timestep and we are in risk of collision */
			/* TODO: Add a CPA limit on how long a start poin can be delayed */
			if(my_risk_of_collision){
				if((!my_start && my_current_risk) || my_start){
					if((!my_start && !is_changing_course && !other_is_changing_course)  || my_start){
						net.add_to_dequeue();
						net.incrementTime();
						did_save = true;

						/* For remove start  */
						if(check_remove_steps(cpa.time_untill_CPA)){
							net.restartTime();
							net.clearEvidence();
							my_start = true;
							write_start_to_file = 1;
							if(cpa.time_untill_CPA > 600){
								my_initial_ship_state = better_at(ship_states, my_id);
							}
							else{
								std::string throw_object = "Remove time steps";
								throw throw_object;
							}
						}

						if (!my_start){
							my_initial_ship_state = better_at(ship_states, my_id);
							write_start_to_file = 1;
						}
						my_start = true;
					}
				}
				else{
					net.clearEvidence();
					my_initial_ship_state = better_at(ship_states, my_id);
				}
			}
			else{
				net.clearEvidence();
				my_initial_ship_state = better_at(ship_states, my_id);
			}

			return did_save;
		}

		double evaluateTrajectory(const Eigen::MatrixXd &trajectory
								  , const std::map<int, Eigen::Vector4d> &ship_states
								  , std::vector<int> currently_tracked_ships
								  , double dt
//								  , custom_msgs::IntentionMeasurement *measurement_msgs
)
		{
			//measurement_msgs->header.stamp = ros::Time::now();
			//measurement_msgs->reference_ship_id = my_id;

			net.incrementTime();

			unsigned steps_into_trajectory = parameters.time_into_trajectory;

			//measurement_msgs->change_in_course_deg = RAD2DEG * (trajectory(CHI, steps_into_trajectory) - my_initial_ship_state[CHI]);
			//measurement_msgs->change_in_course_state_name = changeInCourseIdentifier(parameters, trajectory(CHI, steps_into_trajectory), my_initial_ship_state[CHI]);
			net.setEvidence("change_in_course", changeInCourseIdentifier(parameters, trajectory(CHI, steps_into_trajectory), my_initial_ship_state[CHI]));

			//measurement_msgs->change_in_speed_m_s = trajectory(U, steps_into_trajectory) - my_initial_ship_state[U];
			//measurement_msgs->change_in_speed_state_name = changeInSpeedIdentifier(parameters, trajectory(U, steps_into_trajectory), my_initial_ship_state[U]);
			net.setEvidence("change_in_speed", changeInSpeedIdentifier(parameters, trajectory(U, steps_into_trajectory), my_initial_ship_state[U]));

			//measurement_msgs->is_changing_course = false;
			net.setEvidence("is_changing_course", false);

			//measurement_msgs->ship_measurements.clear();
			std::vector<std::string> handled_ship_names;
			for (auto const ship_id : currently_tracked_ships)
			{
				if (ship_id != my_id)
				{
					std::string ship_name = better_at(ship_name_map, ship_id);
					handled_ship_names.push_back(ship_name);
					const auto ship_state = better_at(ship_states, ship_id);
					//custom_msgs::IntentionMeasurementSingleShip single_ship_meas_msg;
					//single_ship_meas_msg.measured_ship_id = ship_id;

					net.setEvidence("disable_" + ship_name, "enabled");

					CPA cpa = evaluateCPA(trajectory, ship_state, dt);
					auto minimum_acceptable_ample_time = parameters.ample_time_s.minimal_accepted_by_ownship;

					//single_ship_meas_msg.time_untill_CPA_sec = minimum_acceptable_ample_time;
					//single_ship_meas_msg.time_untill_CPA_state_id = timeIdentifier(parameters, minimum_acceptable_ample_time);
					net.setEvidence("time_untill_closest_point_of_approach_towards_" + ship_name, timeIdentifier(parameters, minimum_acceptable_ample_time));

					//single_ship_meas_msg.distance_at_CPA_m = cpa.distance_at_CPA;
					//single_ship_meas_msg.distance_at_CPA_state_id = highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA);
					net.setEvidence("distance_at_cpa_towards_" + ship_name, highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));

					double crossing_in_front_distance = crossingInFrontDistanceTrajectory(trajectory, ship_state, dt);

					//single_ship_meas_msg.crossing_distance_front_m = crossing_in_front_distance;
					//single_ship_meas_msg.crossing_distance_front_state_id = crossInFrontHighresIdentifier(parameters, crossing_in_front_distance);
					net.setEvidence("crossing_distance_front_towards_" + ship_name, crossInFrontHighresIdentifier(parameters, crossing_in_front_distance));

					auto distanceToMidpointResult = distanceToMidpointTrajectory(trajectory, ship_state, dt);
					//single_ship_meas_msg.two_times_distance_to_midpoint_at_cpa_to_m = distanceToMidpointResult.distance_to_midpoint;
					//single_ship_meas_msg.two_times_distance_to_midpoint_at_cpa_to_state_id = twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint);
					net.setEvidence("two_times_distance_to_midpoint_at_cpa_to_" + ship_name, twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint));

					//single_ship_meas_msg.crossing_with_midpoint_on_side = crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side);
					net.setEvidence("crossing_with_midpoint_on_side_"+ship_name, crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side));

					//single_ship_meas_msg.aft_front_crossing_side = frontAftIdentifier(cpa.passing_in_front);
					net.setEvidence("aft_front_crossing_side_to_" + ship_name, frontAftIdentifier(cpa.passing_in_front));

					//single_ship_meas_msg.passed = hasPassedIdentifier(cpa.time_untill_CPA);
					net.setEvidence("passed_" + ship_name, hasPassedIdentifier(cpa.time_untill_CPA));

					//single_ship_meas_msg.port_starboard_crossing_side = crossing_port_starboard_identifier(cpa.bearing_relative_to_heading);
					net.setEvidence("crossing_wiht_other_on_port_side_to_" + ship_name, crossing_port_starboard_identifier(cpa.bearing_relative_to_heading));

					//measurement_msgs->ship_measurements.push_back(single_ship_meas_msg);
				}
			}

			for (const auto ship_name : ship_names)
			{
				if (!std::count(handled_ship_names.begin(), handled_ship_names.end(), ship_name))
				{
					net.setEvidence("disable_" + ship_name, "disabled");
				}
			}

			// evaluate_nodes(node_state_msg);
			auto res = net.evaluateStates(all_node_names);
			//measurement_msgs->probability_of_observation = better_at(better_at(res, output_name), "true");

			net.decrementTime();
			return better_at(better_at(res, {output_name}), "true");
			}

		/**
		 * @brief Opens given filename and calls the private
		 * \ref write_results_to_file() function
		 *
		 * @param filename Path to file to be written to.
		 * @param x x coordinate at time \ref time
		 * @param y y coordinate at time \ref time
		 * @param time current time, just used for writing to file
		 */
		void save_intention_predictions_to_file(std::string filename, double x, double y, double time){
			std::ofstream intentionFile;

			intentionFile.open (filename, std::ios_base::app);
			if (intentionFile.is_open()) {
				write_results_to_file(intentionFile, time, x, y);
				intentionFile.close();
			} else {
				std::cout << "ERROR: Failed to open " << filename << std::endl;
				assert(false);
			}
		}

		/**
		 * @brief Removes lines from prediction files after the given time, and
		 * after given mmsi index in ship list.
		 *
		 * @param filename path to intention file
		 * @param time not timestep, but as given in the file
		 * @param ship_list list of tracked ships, including own ship
		 * @param mmsi id of ship where exception was thrown.
		 */
		void remove_intention_predictions_from_file(std::string filename, double time, std::vector<int> ship_list, int mmsi){
			std::ifstream intentionFileRead;

			intentionFileRead.open (filename);
			if (!intentionFileRead.is_open()) {
				std::cout << "ERROR: Failed to open " << filename << std::endl;
				assert(false);
			}

			std::vector<std::string> lines;
			std::string line;
			bool is_header = true;

			while (std::getline(intentionFileRead, line)) {
				if (is_header){
					is_header = false;
					continue;
				}
				std::istringstream ss(line);
				std::vector<std::string> fields;
				std::string field;

				while (std::getline(ss, field, ',')) {
					fields.push_back(field);
				}
				if (std::stoi(fields[3]) < time) {
					lines.push_back(line);
				}
				else if ((std::stoi(fields[3]) == time) &&
						 (std::find(ship_list.begin(), ship_list.end(), std::stoi(fields[0])) <= std::find(ship_list.begin(), ship_list.end(), mmsi))){
					lines.push_back(line);
				}
				else if (std::stoi(fields[0]) != my_id){
					lines.push_back(line);
				}
				else{break;}
			}

			intentionFileRead.close();

			std::ofstream intentionFile;

			intentionFile.open(filename);
			if (!intentionFile.is_open()) {
				std::cerr << "Error opening file for writing: " << filename << std::endl;
				assert(false);
			}

			intentionFile << "mmsi,x,y,time,colreg_compliant,good_seamanship,unmodeled_behaviour,has_turned_portwards,has_turned_starboardwards,change_in_speed,is_changing_course,CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher,risk_of_collision,current_risk_of_collision,start\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2\n";
			for (const auto& l : lines) {
				intentionFile << l << std::endl;
			}

			intentionFile.close();
		}

		std::map<std::string,std::map<std::string,double>> get_intention_model_predictions(){
			return intention_model_predictions;
		}

		void set_initial_state(Eigen::Vector4d initial_ship_state){
			my_initial_ship_state = initial_ship_state;
		}

		void run_inference(const std::map<int, Eigen::Vector4d> ship_states, std::vector<int> ship_list
								//, std::map<int, Eigen::MatrixXd> trajectory_candidates = {}, double dt
								)
		{
			ship_states_history.push_back(ship_states);
			auto ship_states_history_it = std::prev(ship_states_history.end(), 1);
			int remove_timesteps_timeout = 0;
			while((ship_states_history_it != ship_states_history.end()) && (remove_timesteps_timeout < 7)){
				try{
					if(ship_states_history_it == ship_states_history.begin()) {ship_states_history_it++;}

					insertObservation(*ship_states_history_it, *std::prev(ship_states_history_it), ship_list);
					ship_states_history_it++;
				}
				catch (std::string throw_object){
					++remove_timesteps_timeout;
					if(ship_states_history.size() > 10){
						for (int i = 0; i < 4; ++i) {
							ship_states_history.pop_front();
						}
					}
					set_initial_state(ship_states_history.front()[my_id]);
					ship_states_history_it = ship_states_history.begin();
				}
			}

			//for (auto const &[traj_id, trajecotry] : trajectory_candidates)
			//{
				//TODO implement for trajectories!

				//res.emplace(traj_id, evaluateTrajectory(trajecotry, ship_states, dt));
			//}
		}	
	};
}
