#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <math.h>
#include <random>
#include "../inc/intention_model.h"
#include "../inc/parameters.h"
#include "../inc/geometry.h"
#include "../inc/utils.h"
#include <Eigen/Dense>
#include <map>

void readFileToVecs (std::string filename, std::vector<int> &mmsi_vec, std::vector<double> &time_vec, std::vector<double> &x_vec, std::vector<double> &y_vec, std::vector<double> &sog_vec, std::vector<double> &cog_vec, std::vector<bool> &land_port_vec, std::vector<bool> &land_front_vec, std::vector<bool> &land_starboard_vec){
    std::string filename_open = "files/"+filename;
    std::ifstream ifile(filename_open);

    int mmsi;
    time_t time;
    double time_d;
    double x, y, sog, cog;
    bool land_port, land_front, land_starboard;
    std::string str;
    double time_null = 0;
    double time_from_null;

    if(ifile.is_open()){
        getline(ifile,str);
        double min_time;
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

            getline(iss, token, ',');
            x = stod(token);
            int new_x = round(x);
            getline(iss, token, ',');
            y = stod(token);
            getline(iss, token, ',');
            sog = stod(token);
            getline(iss, token, ',');
            cog = stod(token);
            getline(iss, token, ',');
            land_port = stod(token);
            getline(iss, token, ',');
            land_front = stod(token);
            getline(iss, token, ',');
            land_starboard = stod(token);
            mmsi_vec.push_back(mmsi);
            x_vec.push_back(x);
            y_vec.push_back(y);
            sog_vec.push_back(sog);

            double cog_rad = cog*M_PI/180;
            cog_vec.push_back(cog_rad);

            land_port_vec.push_back(land_port);
            land_front_vec.push_back(land_front);
            land_starboard_vec.push_back(land_starboard);

            if (time_vec.empty()){
                min_time = time_d;
            }
            else if (min_time > time_d) {
                min_time = time_d;
            }
            time_vec.push_back(time_d);
        }
        for (int i = 0; i < time_vec.size(); i++){
            time_vec[i] -= min_time;
        }
    }
    else{
        std::cout << "Could not open file";
    }
}


std::vector<int> getShipList(std::vector<int> mmsi_vec){
    std::sort(mmsi_vec.begin(), mmsi_vec.end());
    auto it = std::unique(mmsi_vec.begin(), mmsi_vec.end());
    mmsi_vec.resize(std::distance(mmsi_vec.begin(), it));
    return mmsi_vec;
}

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

void vecsToStateVectorMap(std::vector<std::map<int, Eigen::Vector4d >> &ship_state, std::vector<std::map<int, Eigen::Vector3d >> &land_state, std::vector<double> &unique_time_vec, std::vector<int> ship_list, std::vector<double> time_vec, std::vector<double> x_vec, std::vector<double> y_vec, std::vector<double> sog_vec, std::vector<double> cog_vec, std::vector<bool> land_port_vec, std::vector<bool> land_front_vec, std::vector<bool> land_starboard_vec){
    time_t time;
    double time_1;
    double x, y, sog, cog;
    bool land_port, land_front, land_starboard;
    std::string str;
    int num_ships = ship_list.size();

	for (int i = 0; i < time_vec.size()/num_ships; i++ ) {
        std::map<int, Eigen::Vector4d> current_ship_states;
        std::map<int, Eigen::Vector3d> current_land_states;
        for (auto & ship_id : ship_list){
            int j = getShipListIndex(ship_id,ship_list);
            int index = j*time_vec.size()/num_ships + i;

            Eigen::Vector4d states(x_vec[index],y_vec[index],cog_vec[index],sog_vec[index]);
            std::map<int,Eigen::Vector4d>::iterator it = current_ship_states.end();
            current_ship_states.insert(it, std::pair<int, Eigen::Vector4d>(ship_id,states));
            Eigen::Vector3d states_land(land_port_vec[index],land_front_vec[index],land_starboard_vec[index]);
            std::map<int,Eigen::Vector3d>::iterator it_land = current_land_states.end();
            current_land_states.insert(it_land, std::pair<int, Eigen::Vector3d>(ship_id,states_land));
        }
        ship_state.push_back(current_ship_states);
        land_state.push_back(current_land_states);
        unique_time_vec.push_back(time_vec[i]);
    }
}

std::map<int, Eigen::MatrixXd> generate_trajectories(Eigen::Vector4d ship_state, double dt, int num_timesteps, Eigen::MatrixXd scenarios) {

    std::map<int, Eigen::MatrixXd> trajectories;

    for (int traj_id = 0; traj_id < scenarios.cols(); ++traj_id) {
        Eigen::MatrixXd trajectory(4, num_timesteps);
        double x = ship_state[0];
        double y = ship_state[1];
        double cog = ship_state[2];
        double sog = ship_state[3];

        int sign = 1;
        double coeff = 1.0;

        for (int t = 0; t < num_timesteps; ++t) {
            if (t == 1) {sign = sign*(-1);}
            if (t >= 1) {coeff *= 0.5;}
            cog += coeff * sign*scenarios(1, traj_id);
            //sog += sog * scenarios(0, traj_id);

            double dx = sog * std::cos(cog);
            double dy = sog * std::sin(cog);
            x += dx * dt/2;
            y += dy * dt/2;

            trajectory(0, t) = x;
            trajectory(1, t) = y;
            trajectory(2, t) = cog;
            trajectory(3, t) = sog;
        }
        trajectories[traj_id] = trajectory;
    }

    return trajectories;
}

void writeIntentionToFile(int timestep,
                          INTENTION_INFERENCE::IntentionModelParameters parameters,
                          std::string filename,
                          std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions,
                          std::vector<std::map<int, Eigen::Vector4d > > ship_state,
                          std::vector<std::map<int, Eigen::Vector3d > > land_state,
                          std::vector<int> ship_list, std::vector<double> unique_time_vec,
                          std::vector<double> x_vec,
                          std::vector<double> y_vec,){


    std::ofstream intentionFile;
    std::string filename_intention = "files/intention_files/nostart_intention_"+filename;
    std::string filename_trajectories = "files/trajectory_files/nostart_trajectory_"+filename;
    intentionFile.open (filename_intention);
    intentionFile << "mmsi,x,y,time,unmodeled_behaviour,colregs_compliant,good_seamanship,ignoring_safety,CR_PS,CR_SS,HO,OT_en,OT_ing,priority_lower,priority_similar,priority_higher,risk_of_collision,start\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2,risk_of_collision2\n";
    intentionFile.close();

    intentionFile.open (filename_trajectories);
    intentionFile << "mmsi,time,traj_id,x,y,prob\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2\n";
    intentionFile.close();

    Eigen::MatrixXd traj_scenarios(2,9);
    traj_scenarios << 0, 0, 0, 0, 0, 0, 0, 0, 0, /* Pertubations in sog */
                 0, 0.3, -0.3, 0.6, -0.6, 0.9 ,-0.9, 1.2, -1.2; /* Perturbations in cog */

    for(int i = timestep; i < unique_time_vec.size() ; i++){
        std::cout << "timestep: " << unique_time_vec[i] << std::endl;

        for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
            int j = getShipListIndex(ship_id,ship_list);
            std::cout << "ship_id: " << ship_id << std::endl;
            double dt = unique_time_vec[i] - unique_time_vec[i-1];

            //auto trajectory_candidates = generate_random_trajectories(ship_state[i][ship_id], dt, parameters.time_into_trajectory, 10);
            auto trajectory_candidates = generate_trajectories(ship_state[i][ship_id], dt, 6, traj_scenarios);
            bool did_save = current_ship_intention_model.run_intention_inference(ship_state[i], land_state[i], ship_list, unique_time_vec[i]);
            current_ship_intention_model.save_intention_predictions_to_file(filename_intention,
                                                                            x_vec[unique_time_vec.size()*j+i], /* x pos for ship j at time i */
                                                                            y_vec[unique_time_vec.size()*j+i],
                                                                            unique_time_vec[i]);
            current_ship_intention_model.run_trajectory_inference(ship_state[i] ,ship_list, trajectory_candidates, dt);
            current_ship_intention_model.save_trajectories_to_file(filename_trajectories,
                                                                   unique_time_vec[i],
                                                                   trajectory_candidates);
        }
    }
    std::cout << "Finished writing intentions to file \n";
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
    //std::string filename = "new_1_Case - 08-09-2018, 19-12-24 - 4XJ3B-60-sec.csv"; //not unmodeled
    //std::string filename = "new_1_Case - 06-25-2019, 14-22-43 - OO430-60-sec.csv"; //not unmodeled
    //std::string filename = "new_1_Case - 12-02-2018, 20-10-07 - PW6UL-60-sec.csv"; //unmodeled
    //std::string filename = "new_1_Case - 07-18-2019, 05-46-19 - W6ZUC-60-sec.csv";
    //std::string filename = "new_1_Case - 09-17-2018, 18-24-32 - 0URFX-60-sec.csv";
    //std::string filename = "new_Case - 01-12-2018, 03-56-43 - WRNUL-60-sec.csv";
    //std::string filename = "new_Case - 01-02-2018, 01-05-22 - GP38T-60-sec.csv"; //crossing wrong both
    //std::string filename = "new_Case - 05-09-2018, 10-05-48 - 9PNLJ-60-sec.csv";
    std::string filename = "input_ready_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships-radius-300.csv";

    std::string intentionModelFilename = "files/intention_models/intention_model_with_grounding_manually_adjusted.xdsl";

    std::vector<std::map<int, Eigen::Vector4d> > ship_state;
    std::vector<std::map<int, Eigen::Vector3d> > land_state;
    std::vector<int> mmsi_vec;
    std::vector<double> time_vec, x_vec, y_vec, sog_vec, cog_vec, unique_time_vec;
    std::vector<bool> land_port_vec, land_front_vec, land_starboard_vec;
    readFileToVecs(filename, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec, land_port_vec, land_front_vec, land_starboard_vec);

    std::vector<int> ship_list = getShipList(mmsi_vec);

    vecsToStateVectorMap(ship_state, land_state, unique_time_vec, ship_list, time_vec, x_vec, y_vec, sog_vec, cog_vec, land_port_vec, land_front_vec, land_starboard_vec);

    INTENTION_INFERENCE::IntentionModelParameters parameters = default_parameters(num_ships);

    std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions;

    int timestep = 0;
    bool inserted = false;

    while (!inserted){
        for (int i = 0; i < num_ships; i++){
            std::cout<< INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PX]- INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[0])[INTENTION_INFERENCE::PX] << std::endl;
            double dist = evaluateDistance(INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PX] - INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[0])[INTENTION_INFERENCE::PX], INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1])[INTENTION_INFERENCE::PY] - INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[0])[INTENTION_INFERENCE::PY]);
            std::cout<< "dist: " << dist << std::endl;
            auto CPA = evaluateCPA(INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[1]), INTENTION_INFERENCE::better_at(ship_state[timestep], ship_list[0]));
            std::cout<< "CPA dist: " << CPA.distance_at_CPA << std::endl;
            
            if ((dist < parameters.starting_distance) && (sog_vec[timestep]>0.1) && (sog_vec[unique_time_vec.size()+timestep]>0.1) && timestep>0){ //only checks the speed for two ships
                ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[i], INTENTION_INFERENCE::IntentionModel(intentionModelFilename,parameters,ship_list[i],ship_state[timestep], land_state[timestep])));
                inserted = true;
            }
        }
        timestep ++;
    }
    writeIntentionToFile(timestep, parameters, filename, ship_intentions, ship_state, land_state, ship_list, unique_time_vec, x_vec, y_vec, land_port_vec, land_front_vec, land_starboard_vec); //intentionfile is called: intention_<filename>  NB: not all intentions!
    
}
