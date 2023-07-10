#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <math.h>
#include "../inc/intention_model.h"
#include "../inc/parameters.h"
#include "../inc/geometry.h"
#include "../inc/utils.h"
#include "../external/Eigen/Dense"
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
            //std::cout << "local: " << std::put_time(&local, "%c %Z") << '\n';
            
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
				std::cout << "Could not open file";
    }
}


void vecsToShipStateVectorMap(std::vector<std::map<int, Eigen::Vector4d > > &ship_state, std::vector<double> &unique_time_vec, int num_ships, std::vector<int> mmsi_vec, std::vector<double> time_vec, std::vector<double> x_vec, std::vector<double> y_vec, std::vector<double> sog_vec, std::vector<double> cog_vec){
    int mmsi;
    time_t time;
    double time_1;
    double x, y, sog, cog;
    std::string str;

	for (int i = 0; i < time_vec.size()/num_ships; i++ ) {
        std::map<int, Eigen::Vector4d> current_ship_states;
        for (int c = 0; c < num_ships; c++){
            int index = c*time_vec.size()/num_ships + i;
            Eigen::Vector4d states(x_vec[index],y_vec[index],cog_vec[index],sog_vec[index]);
            std::map<int,Eigen::Vector4d>::iterator it = current_ship_states.end();
            current_ship_states.insert(it, std::pair<int, Eigen::Vector4d>(mmsi_vec[index],states));
        }
        ship_state.push_back(current_ship_states);
				unique_time_vec.push_back(time_vec[i]);
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
        std::cout<<"ERROR: mmsi not found in ship list\n";
    }
    return index;
}


void writeIntentionToFile(int timestep,
                          INTENTION_INFERENCE::IntentionModelParameters parameters,
                          std::string filename,
                          std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions,
                          std::vector<std::map<int, Eigen::Vector4d > > ship_state,
                          std::vector<int> ship_list, std::vector<double> unique_time_vec,
                          std::vector<double> x_vec,
                          std::vector<double> y_vec){


    std::ofstream intentionFile;
    std::string filename_intention = "files/intention_files/nostart_intention_"+filename;
    intentionFile.open (filename_intention);
    intentionFile << "mmsi,x,y,time,colreg_compliant,good_seamanship,unmodeled_behaviour,has_turned_portwards,has_turned_starboardwards,change_in_speed,is_changing_course,CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher,risk_of_collision,current_risk_of_collision,start\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2\n";
    //intentionFile << "mmsi,x,y,time,colreg_compliant,good_seamanship,unmodeled_behaviour,CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2\n";
    //intentionFile << "mmsi,x,y,time,CR_PS,CR_SS,HO,OT_en,OT_ing\n";
    intentionFile.close();

    std::map<int, bool> risk_of_collision;
    std::map<int,bool> current_risk;
    std::map<int,Eigen::Vector4d> new_initial_ship_states;
    std::map<int,double> check_changing_course;
    for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
        risk_of_collision[ship_id] = false;
        current_risk[ship_id] = false;
    }

    bool start = false;
    bool new_timestep;
   
    for(int i = timestep; i < unique_time_vec.size() ; i++){ 
        std::cout << "timestep: " << i << std::endl;
        new_timestep = true;
        
        for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
            //if ((i-timestep)>4){
                //new_initial_ship_states[ship_id] = INTENTION_INFERENCE::better_at(ship_state[i-4], ship_id); used when no start-point
            //}
            std::cout << "ship_id: " << ship_id << std::endl;
            int j = getShipListIndex(ship_id,ship_list);
            current_ship_intention_model.insertObservation(parameters,start,new_timestep,
                                                           check_changing_course,
                                                           risk_of_collision,
                                                           ship_state[i],
                                                           ship_state[i-1],
                                                           ship_list,
                                                           unique_time_vec[i],
                                                           x_vec[unique_time_vec.size()*j+i], /* x pos for ship j at time i */
                                                           y_vec[unique_time_vec.size()*j+i],
                                                           "intention_files/nostart_intention_"+filename
                                                           ); //writes intention variables to file as well

            new_timestep = false;
    }
   }
    printf("Finished writing intentions to file \n");
}


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
    
	int num_ships = 2; /* Total number of ships, including own ship*/
    //std::string filename = "new_Case_LQLVS-60-sec.csv"; //crossing
    //std::string filename = "new_case_2ZC9Z-60-sec-two-ships.csv"; //head on
    //std::string filename = "new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"; //overtaking must start at timestep 4
    //std::string filename = "new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"; //overtaking
    //std::string filename  = "new_Case - 01-17-2018, 06-26-20 - W4H51-60-sec.csv";
    //std::string filename = "new_Case - 05-26-2019, 20-39-57 - 60GEW-60-sec.csv";
    //std::string filename = "new_Case - 01-04-2020, 15-34-37 - 7SWX4-60-sec.csv"; //overtake
    //std::string filename = "new_Case - 02-01-2018, 15-50-25 - C1401-60-sec.csv"; //head-on corr
    //std::string filename = "new_Case - 01-09-2018, 03-55-18 - QZPS3-60-sec.csv"; //ho wr
    //std::string filename = "new_1_Case - 07-09-2019, 05-52-22 - O7LU9-60-sec.csv"; //weird start
    std::string filename = "new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv"; //not unmodeled
    //std::string filename = "new_1_Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv"; //not unmodeled
    //std::string filename = "new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv"; //unmodeled
    //std::string filename = "new_1_Case - 07-18-2019, 05-46-19 - W6ZUC-60-sec.csv";
    //std::string filename = "new_1_Case - 09-17-2018, 18-24-32 - 0URFX-60-sec.csv";
    //std::string filename = "new_Case - 01-12-2018, 03-56-43 - WRNUL-60-sec.csv";
    //std::string filename = "new_Case - 01-02-2018, 01-05-22 - GP38T-60-sec.csv"; //crossing wrong both
    //std::string filename = "new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";

    //std::string intentionModelFilename = "intention_model_with_risk_of_collision.xdsl";
    //std::string intentionModelFilename = "intention_model_two_ships.xdsl";
    std::string intentionModelFilename = "files/intention_models/intention_model_from_code.xdsl";

    std::vector<std::map<int, Eigen::Vector4d> > ship_state;
    std::vector<int> mmsi_vec;
    std::vector<double> time_vec, x_vec, y_vec, sog_vec, cog_vec, unique_time_vec;;

    readFileToVecs(filename, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec);


    vecsToShipStateVectorMap(ship_state, unique_time_vec, num_ships, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec);

    std::vector<int> ship_list = getShipList(mmsi_vec);


    INTENTION_INFERENCE::IntentionModelParameters parameters = setModelParameters(num_ships);

    std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions;

    int timestep = 0;
    bool inserted = false;

    while (!inserted){
        for (int i = 0; i < num_ships; i++){
            std::cout<< INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PX]- INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2])[INTENTION_INFERENCE::PX] << std::endl;
            double dist = evaluateDistance(INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PX] - INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2])[INTENTION_INFERENCE::PX], INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PY] - INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2])[INTENTION_INFERENCE::PY]);
            std::cout<< "dist: " << dist << std::endl;
            auto CPA = evaluateCPA(INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1]), INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[2]));
            std::cout<< "CPA dist: " << CPA.distance_at_CPA << std::endl;
            
            if ((dist < parameters.starting_distance) && (sog_vec[timestep]>0.1) && (sog_vec[unique_time_vec.size()+timestep]>0.1) && timestep>0){ // && (CPA.distance_at_CPA < parameters.starting_cpa_distance) ){ //only checks the speed for two ships
                ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[i], INTENTION_INFERENCE::IntentionModel(intentionModelFilename,parameters,ship_list[i],ship_state[timestep]))); 
                inserted = true;
            }
        }
        timestep ++;
    }

    writeIntentionToFile(timestep, parameters,filename, ship_intentions, ship_state, ship_list, unique_time_vec, x_vec,y_vec); //intentionfile is called: intention_<filename>  NB: not all intentions!
    

    /* OLD PRINTS
    for (int i= 0; i <2; i++){
        std::cout << "mmsi: " << mmsi_vec[i] << std::endl;
        std::cout << "time: " << time_vec[i] << std::endl;
        std::cout << "x: " << x_vec[i] << std::endl;
        std::cout << "y: " << y_vec[i] << std::endl;
        std::cout << "sog: " << sog_vec[i] << std::endl;
        std::cout << "cog: " << cog_vec[i] << std::endl;
    } */
    for (int i = 0; i < 5; i++){
            for(auto [key, item]: ship_state[i]){
            std::cout << key << " -> " << item << std::endl;
            std::cout << " time: " << unique_time_vec[i] << std::endl;
        }
    } 
}
