#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <math.h>
#include "intention_model.h"
#include "parameters.h"
#include "geometry.h"
#include "utils.h"
#include "Eigen/Dense"
#include <map>

void readFileToVecs (std::string filename, std::vector<int> &mmsi_vec, std::vector<double> &time_vec, std::vector<double> &x_vec, std::vector<double> &y_vec, std::vector<double> &sog_vec, std::vector<double> &cog_vec){
    std::string filename_open = "files/"+filename;
    std::ifstream ifile(filename_open);

    int mmsi;
    time_t time;
    double time_d;
    double x, y, sog, cog;
    std::string str;
    double time_null = 0;
    double time_from_null;

    if(ifile.is_open()){
        getline(ifile,str);
        while(getline(ifile,str)){
            std::istringstream iss(str);
            std::string token;
            getline(iss, token, ',');
            mmsi = std::stoi(token);
            
            getline(iss, token, ',');
            struct std::tm td;
            std::istringstream ss(token);
            ss >> std::get_time(&td, "%Y-%m-%d %H:%M:%S"); // or just %T in this case
            std::time_t time = mktime(&td);
            time_d = time;
            //std::tm local = *std::localtime(&time);
            //std::cout << "local: " << std::put_time(&local, "%c %Z") << '\n' << std::flush;
            
            getline(iss, token, ',');
            x = stod(token);
            int new_x = round(x);
            getline(iss, token, ',');
            y = stod(token);
            getline(iss, token, ',');
            sog = stod(token);
            getline(iss, token, ',');
            cog = stod(token);

            mmsi_vec.push_back(mmsi);
            x_vec.push_back(x);
            y_vec.push_back(y);
            sog_vec.push_back(sog);

            double cog_rad = cog*M_PI/180;
            cog_vec.push_back(cog_rad);
            if (time_vec.empty()){
                time_vec.push_back(time_d);
            }
            else {
                double time_from_null = time_d-time_vec[0];
                time_vec.push_back(time_from_null);
            }
        }
        time_vec[0]=0;
    }
    else{
				std::cout << "Could not open file" << std::flush;
    }
}


void vecsToShipStateVectorMap(std::vector<std::map<int, Eigen::Vector4d > > &ship_state, std::vector<double> &unique_time_vec, int num_ships, const std::vector<int> &mmsi_vec, const std::vector<double> &time_vec, const std::vector<double> &x_vec, const std::vector<double> &y_vec, const std::vector<double> &sog_vec, const std::vector<double> &cog_vec){
    int mmsi;
    time_t time;
    double time_1;
    double x, y, sog, cog;
    std::string str;

	for (int i = 0; i < time_vec.size()/num_ships; i++ ) {
        bool skip_line = false;
        std::map<int, Eigen::Vector4d> current_ship_states;
        for (int c = 0; c < num_ships; c++){
            int index = c*time_vec.size()/num_ships + i;
            if(isnan(x_vec[index]+y_vec[index]+cog_vec[index]+sog_vec[index])){
                skip_line = true;
                break;
            }
            Eigen::Vector4d states(x_vec[index],y_vec[index],cog_vec[index],sog_vec[index]);
            std::map<int,Eigen::Vector4d>::iterator it = current_ship_states.end();
            current_ship_states.insert(it, std::pair<int, Eigen::Vector4d>(mmsi_vec[index],states));
        }
        if(!skip_line){
            ship_state.push_back(current_ship_states);
            unique_time_vec.push_back(time_vec[i]);
        }
    }
}


std::vector<int> getShipList(std::vector<int> mmsi_vec){
    std::vector<int> ship_list( mmsi_vec.begin(), mmsi_vec.end() );
    auto it = unique(ship_list.begin(), ship_list.end());
    ship_list.resize(distance(ship_list.begin(), it));
    return ship_list;
}

/*void ensure_initialization(std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions){
		for (auto& [ship_id, current_ship_intention_model] : ship_intentions){
			// Initiate ships that are not the own-ship, that have not already been initiated, and that are sufficiently close
			if (ship_id != OWN_SHIP_ID && !ship_intentions.count(ship_id) && evaluateDistance(better_at(ship_states, ship_id)[PX] - better_at(ship_states, OWN_SHIP_ID)[PX], better_at(ship_states, ship_id)[PY] - better_at(ship_states, OWN_SHIP_ID)[PY]) < starting_distance){
				ship_intentions.insert({ship_id, IntentionModel(network_filename, intention_model_parameters, ship_id, ship_states)});
			}
		}
	}*/

int getShipListIndex(int mmsi, std::vector<int> ship_list){
    int index = -1;
    for (int i = 0; i < ship_list.size(); i++){
        if(mmsi==ship_list[i]){
            index = i;
        }
    }
    if (index == -1){
        std::cout<<"ERROR: mmsi not found in ship list\n" << std::flush;
    }
    return index;
}


void writeIntentionToFile(int timestep, INTENTION_INFERENCE::IntentionModelParameters parameters, std::string filename, std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions, std::vector<std::map<int, Eigen::Vector4d > > ship_state, std::vector<int> ship_list, std::vector<double> unique_time_vec){
    std::ofstream intentionFile;
    std::ofstream measurementFile;
    std::ofstream measurementIdentifiersFile;
    std::string filename_intention = "intention_files/dist_intention_"+filename;
    std::string filename_measurements = "intention_files/measurements_"+filename;
    std::string filename_measurementIdentifiers = "intention_files/measurements_identifiers_"+filename;
    intentionFile.open(filename_intention);
    measurementFile.open(filename_measurements);
    measurementIdentifiersFile.open(filename_measurementIdentifiers);
    intentionFile << "mmsi,x,y,cog,sog,time,colreg_compliant,distance_risk_of_collision,distance_risk_of_collision_front,good_seamanship,unmodeled_behaviour,has_turned_portwards,has_turned_starboardwards,stands_on_correct,ample_time_acceptable,safe_distance_front_acceptable,safe_distance_acceptable,safe_distance_midpoint_acceptable,CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher,is_risk_of_collision,situation_started,will_give_way,is_pre_ample_time,safe_distance,safe_distance_to_midpoint,gives_way_correct,stand_on_role,change_in_course_port,change_in_course_starboard,change_in_speed_lower,change_in_speed_higher,mean_initial_course,max_initial_course,mean_initial_speed,max_initial_speed,\n";
    measurementFile << "mmsi,time,did_save,course,speed,is_changing_course,CR_PS,CR_SS,HO,OT_en,OT_ing,current_distance,time_untill_CPA,distance_cpa,crossing_in_front_distance,distance_to_midpoint,crossing_with_midpoint_on_port_side,hasPassed,crossing_with_other_on_port_side,\n";
    measurementIdentifiersFile << "mmsi,time,course,speed,current_distance,time_untill_CPA,highres_distance_cpa,lowres_distance_cpa,highres_crossing_in_front_distance,lowres_crossing_in_front_distance,distance_to_midpoint,crossing_with_midpoint_on_port_side,\n";
    //intentionFile << "mmsi,x,y,time,colreg_compliant,good_seamanship,unmodeled_behaviour,CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2\n";

    bool is_finished;
    for(int i = timestep; !is_finished && i < unique_time_vec.size() ; i++){ //from 1 because first state might be NaN
        std::cout << "timestep: " << i << std::endl << std::flush;
        for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
            bool is_changing_course = false;
            if(i>timestep){
                double change_in_course = INTENTION_INFERENCE::better_at(ship_state[i],ship_id)[INTENTION_INFERENCE::CHI] - INTENTION_INFERENCE::better_at(ship_state[i-1],ship_id)[INTENTION_INFERENCE::CHI];
                is_changing_course = std::abs(change_in_course) > (10*INTENTION_INFERENCE::DEG2RAD);
                if(is_changing_course) {
                    std::cout << "CHANGING COURSE!" << std::endl << std::flush;
                }
            }
            std::cout << "ship_id" << ship_id << std::endl << std::flush;
            int j = getShipListIndex(ship_id,ship_list);
            is_finished = current_ship_intention_model.insertObservation(parameters, ship_state[i], ship_list, is_changing_course, unique_time_vec[i], intentionFile, measurementFile, measurementIdentifiersFile); //writes intantion variables to file as well
        }
    }
    intentionFile.close(); 
    measurementFile.close();
    printf("Finished writing intentions to file \n");
}


INTENTION_INFERENCE::IntentionModelParameters setModelParameters(int num_ships){
    INTENTION_INFERENCE::IntentionModelParameters param;
    param.use_ais_distributions = true;
    param.number_of_network_evaluation_samples = 100000;
	param.max_number_of_obstacles = num_ships-1; //must be set to num_ships-1 or else segmantation fault
	param.time_into_trajectory = 10;
    param.starting_distance =7000;
	param.expanding_dbn.min_time_s = 10;
	param.expanding_dbn.max_time_s = 200;
	param.expanding_dbn.min_course_change_rad = 0.10;
	param.expanding_dbn.min_speed_change_m_s = 1;
    param.situation_start_distance.mu = 5500;
    param.situation_start_distance.sigma = 500;
    param.situation_start_distance.n_bins = 30;
    param.situation_start_distance.max = 7000;
    param.speed.max = 18;
    param.speed.n_bins = 25;
    param.course.n_bins = 36;
	//param.ample_time_s.mu = 100;
	//param.ample_time_s.sigma = 25;
	param.ample_time_s.max = 120;
	param.ample_time_s.n_bins = 30; // this value must match the bayesian network
	param.ample_time_s.minimal_accepted_by_ownship = 250;
    //param.safe_distance_m.mu = 200;
	//param.safe_distance_m.sigma = 5;
	param.safe_distance_m.max = 1000;
	param.safe_distance_m.n_bins = 30; // this value must match the bayesian network
    param.safe_distance_m.minimal_accepted_by_ownship = 250;
	//param.safe_distance_midpoint_m.mu = 600;
	//param.safe_distance_midpoint_m.sigma = 20;
	param.safe_distance_midpoint_m.max = 1000;
	param.safe_distance_midpoint_m.n_bins = 30; // this value must match the bayesian network
    param.safe_distance_midpoint_m.minimal_accepted_by_ownship = 250;
	//param.safe_distance_front_m.mu = 200;
	//param.safe_distance_front_m.sigma = 10;
	param.safe_distance_front_m.max = 1000;
	param.safe_distance_front_m.n_bins = 30; // this value must match the bayesian network
    param.safe_distance_front_m.minimal_accepted_by_ownship = 50;
	param.risk_distance_m.mu = 1500;
	param.risk_distance_m.sigma = 250;
	param.risk_distance_m.max = 2500;
    param.risk_distance_m.n_bins = 30; // this value must match the bayesian network
    param.risk_distance_front_m.mu = 1500;
	param.risk_distance_front_m.sigma = 250;
	param.risk_distance_front_m.max = 2500;
    param.risk_distance_front_m.n_bins = 30;  // this value must match the bayesian network
	param.colregs_situation_borders_rad.HO_uncertainty_start = 2.79;
	param.colregs_situation_borders_rad.HO_start = 2.96;
	param.colregs_situation_borders_rad.HO_stop = -2.96;
	param.colregs_situation_borders_rad.HO_uncertainty_stop = -2.79;
	param.colregs_situation_borders_rad.OT_uncertainty_start = 1.614;
	param.colregs_situation_borders_rad.OT_start = 2.3126;
	param.colregs_situation_borders_rad.OT_stop = -2.3126;
	param.colregs_situation_borders_rad.OT_uncertainty_stop = -1.614;
	param.ignoring_safety_probability = 0;
	param.colregs_compliance_probability = 0.98;
    param.good_seamanship_probability = 0.99;
	param.unmodeled_behaviour = 0.001;
	param.priority_probability["lower"] = 0.05;
	param.priority_probability["similar"] = 0.9;
	param.priority_probability["higher"] = 0.05;
    return param;
}


int main(){
    using namespace INTENTION_INFERENCE;
    
	int num_ships = 2;
    //std::string filename = "new_case_LQLVS-60-sec.csv"; //crossing
    //Korrekt oppførsel, potensielt litt sen action

    //std::string filename = "new_case_2ZC9Z-60-sec-two-ships.csv"; //head on
    //En tidlig bevegelse feil vei på ship1, så endrer det til at begge skipene oppfører seg rett

    //std::string filename = "new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"; //overtaking must start at timestep 4
    //Noe rart i starten, så svinger ship2 mot kollisjon egentlig og ship1 svinger unna

    //std::string filename = "new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"; //overtaking
    //Funker dårlig, ship1 har allerde startet å gjøre en unnamanøver når dataen starter

    //std::string filename = "new_Case - 01-09-2018, 01-11-37 - RT3LY-60-sec-two-ships-filled.csv"; //head-on
    //HO, rett oppførsel men egentlig ikke noe risiko for kollisjon i det hele tatt. 

    //std::string filename = "new_Case - 01-09-2018, 01-45-02 - 19JNJ-60-sec-two-ships.csv";
    //HO ingenting spennende

    //std::string filename  = "new_Case - 01-11-2019, 02-30-00 - LP84U-60-sec.csv";
    //En CR/OT situasjon som ikke egentlig gir mening, skjønner ikke hva båtene gjør...

    //std::string filename  = "new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";
    //HO hvor den ene svinger feil vei, den andre holder stø kurs

    //std::string filename = "new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv";
    //CR hvor den ene svinger feil vei. Noe surr på starten som må ryddes vekk som situasjonen ikke startet

    //SOuth cases:
    std::string filename = "new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv";

    std::string intentionModelFilename = "intention_model_combined_discretized.xdsl";

    std::vector<std::map<int, Eigen::Vector4d> > ship_state;
    std::vector<double> unique_time_vec;
    std::vector<int> mmsi_vec;
    {
        std::vector<double> time_vec, x_vec, y_vec, sog_vec, cog_vec;

        readFileToVecs(filename, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec);
        vecsToShipStateVectorMap(ship_state, unique_time_vec, num_ships, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec);
    }

    std::vector<int> ship_list = getShipList(mmsi_vec);


    INTENTION_INFERENCE::IntentionModelParameters parameters = setModelParameters(num_ships);

    std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions;

    int timestep = 0;
    bool inserted = false;

    while (!inserted){
        for (int i = 0; i < num_ships; i++){
            std::cout<< INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PX]- INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2])[INTENTION_INFERENCE::PX] << std::endl << std::flush;
            double dist = evaluateDistance(INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PX] - INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2])[INTENTION_INFERENCE::PX], INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PY] - INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2])[INTENTION_INFERENCE::PY]);
            std::cout<< "dist: " << dist << std::endl << std::flush;
            if ((dist < parameters.starting_distance) && (better_at(ship_state[timestep],mmsi_vec[0])[U]>1) && (better_at(ship_state[timestep],mmsi_vec[1])[U]>1)){ //only checks the speed for two ships
                ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[i], INTENTION_INFERENCE::IntentionModel(intentionModelFilename,parameters,ship_list[i],ship_state[timestep]))); //ship_state[1] as initial as first state might be NaN
            inserted = true;
            }
        }
        timestep ++;
    }
    
    writeIntentionToFile(timestep, parameters,filename, ship_intentions, ship_state, ship_list, unique_time_vec); //intentionfile is called: intention_<filename>  NB: not all intentions!
    

    /* OLD PRINTS
    for (int i= 0; i <2; i++){
        std::cout << "mmsi: " << mmsi_vec[i] << std::endl << std::flush;
        std::cout << "time: " << time_vec[i] << std::endl << std::flush;
        std::cout << "x: " << x_vec[i] << std::endl << std::flush;
        std::cout << "y: " << y_vec[i] << std::endl << std::flush;
        std::cout << "sog: " << sog_vec[i] << std::endl << std::flush;
        std::cout << "cog: " << cog_vec[i] << std::endl << std::flush;
    } 
    for (int i = 0; i < 5; i++){
            for(auto it = ship_state[i].cbegin(); it != ship_state[i].cend(); ++it){
            std::cout << it->first << " -> " << it->second << std::endl << std::flush;
            std::cout << " time: " << unique_time_vec[i] << std::endl << std::flush;
        }
    } */
}