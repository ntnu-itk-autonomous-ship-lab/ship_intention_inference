/**
 * @file bayesian_network.h
 * @author Sverre Velten Rothmund
 * @brief This file interfaces the smile library. 
 * It implements commonly used functions. 
 * This file should not care what the BBN is used for.

 * @version 1.0
 * @date 2022
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "../external/smile/smile.h"
#include "../external/smile/smile_license.h"
#include <assert.h>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include "geometry.h"
#include <stdio.h>
#include <iostream>
#include <deque>

namespace INTENTION_INFERENCE
{
class BayesianNetwork{
    private:
    DSL_network net;
    std::deque<std::map<int, int>> temporal_evidence_; // que over timesteps, map <node_id, evidence_id>
    std::deque<std::map<int, std::vector<double>>> temporal_virtual_evidence_;
    std::map<int, int> evidence_;
    std::map<int, std::vector<double>> virtual_evidence_;

    auto getNodeId(std::string name) const{
        const auto node_id = net.FindNode(name.c_str());
        if(node_id<0) std::cout << "ERROR: Node name " << name.c_str() << " resulted in error" << std::endl;
        assert(node_id>=0);
        return node_id;
    }

    auto getOutcomeId(std::string node_name, std::string outcome_name){
        const auto node_id = getNodeId(node_name);
        const auto outcome_id = net.GetNode(node_id)->Definition()->GetOutcomesNames()->FindPosition(outcome_name.c_str());
        if(outcome_id<0) std::cout << "ERROR: Outcome name " << outcome_name.c_str() << " resulted in error for node name " << node_name.c_str() << std::endl;
        assert(outcome_id>=0);
        return outcome_id;
    }

    auto getNumberOfOutcomes(std::string node_name){
        const auto node_id = getNodeId(node_name);
        const auto outcome_count = net.GetNode(node_id)->Definition()->GetNumberOfOutcomes();
        if(outcome_count<=0) std::cout << "ERROR: No outcomes for node_id " << node_name.c_str() << std::endl;
        assert(outcome_count>0);
        return outcome_count;
    }

    void setDefinition(std::string node_name, DSL_doubleArray& CPT){
        double sum=0;
        for(int i=0; i<CPT.GetSize(); ++i){
            sum += CPT[i];
        }
        //if(sum<0.9999 || sum>1.00001 || !isfinite(sum)) printf("ERROR: Prior distribution on \"%s\" sums to %f, should be 1", node_name.c_str(), sum);
        if(sum<0.8 || sum>1.1 || !isfinite(sum)) std::cout << "ERROR: Prior distribution on " << node_name.c_str() << " sums to " << sum << ", should be 1" << std::endl;
        assert(sum>=0.8 && sum<=1.1);
        const auto node_id = getNodeId(node_name);
        auto result =  net.GetNode(node_id)->Definition()->SetDefinition(CPT);
        if(result<0) std::cout << "ERROR: Setting priors failed on node " << node_name.c_str() << std::endl;
        assert(result>=0);
    }

    void wrapTimeSlice(int* time_slice){
        const auto num_slices = net.GetNumberOfSlices();
        if (*time_slice < 0){
            *time_slice = num_slices+*time_slice;
        }
        if(*time_slice>=num_slices) std::cout << "ERROR: Attempted to access nonexisting timeslice " << *time_slice << std::endl;
        assert(*time_slice<num_slices);
    }

    bool isTemporal(int node_id){
        return net.GetTemporalType(node_id) == dsl_temporalType::dsl_plateNode;
    }

    double getTemporalOutcome(int node_id,int time_slice, int outcome_id){
        wrapTimeSlice(&time_slice);
        const auto outcome_count = net.GetNode(node_id)->Definition()->GetNumberOfOutcomes();
        int index = time_slice*outcome_count+outcome_id;
        //Temporal data is saved as a large vector the outcome for each timestep saved after each other
        const auto outcomes = net.GetNode(node_id)->Value()->GetMatrix();
        return (*outcomes)[index];
    }

    auto getTemporalOutcome(std::string node_name,int time_slice, std::string outcome){
        const auto node_id = getNodeId(node_name);
        const auto outcome_id = getOutcomeId(node_name, outcome);
        return getTemporalOutcome(node_id,time_slice,outcome_id);
    }

    double getOutcome(int node_id, int outcome_id){
        return net.GetNode(node_id)->Value()->GetMatrix()->operator[](outcome_id);
    }

    auto getOutcome(std::string node_name,int time_slice=-1){
        std::map<std::string,double> return_value;
        const auto node_id = getNodeId(node_name);
        const auto is_temporal = isTemporal(node_id);
        const auto outcome_count = getNumberOfOutcomes(node_name);
        const auto outcome_names = net.GetNode(node_id)->Definition()->GetOutcomesNames();
        for(int i=0; i<outcome_count;++i){
            if(is_temporal)
                return_value[(*outcome_names)[i]] = getTemporalOutcome(node_id,time_slice,i);
            else
                return_value[(*outcome_names)[i]] = getOutcome(node_id,i);
        }
        return return_value;
    }

    void apply_evidence()
    {
        clearEvidence();
        restartTime();

        std::cout << "Evidence Size: " << evidence_.size() << std::endl;
        std::cout << "Virtual Evidence Size: " << virtual_evidence_.size() << std::endl; 
        std::cout << "Temporal Evidence Size: " << temporal_evidence_.size() << std::endl;
        std::cout << "Temporal Virtual Evidence Size: " << temporal_virtual_evidence_.size() << std::endl; 
        
        // normal evidence
        // for (const auto &[node_id, outcome_id] : evidence_)
        // {
        //     const auto res = net.GetNode(node_id)->Value()->SetEvidence(outcome_id);
        //     if (res < 0)
        //         printf("ERROR: Set evidence (outcome_id=%d) on node \"%d\" resulted in error", outcome_id, node_id);
        //     assert(res >= 0);
        // }
        // for (const auto &[node_id, evidence_vec] : virtual_evidence_)
        // {
        //     const auto res = net.GetNode(node_id)->Value()->SetVirtualEvidence(evidence_vec);
        //     if (res < 0)
        //         printf("ERROR: Set virtual evidence on node \"%d\" resulted in error", node_id);
        //     assert(res >= 0);
        // }
        
        auto evidence_it = temporal_evidence_.begin();
        auto virtual_evidence_it = temporal_virtual_evidence_.begin();
        int i = 0;

        while (evidence_it != temporal_evidence_.end() && virtual_evidence_it != temporal_virtual_evidence_.end())
        {
            for (const auto &[node_id, outcome_id] : *evidence_it)
            {
                const auto res = net.GetNode(node_id)->Value()->SetEvidence(outcome_id);
                if (res < 0)
                    std::cout << "ERROR: Set temporal evidence (slice=" << i << " outcome_id=" << outcome_id << ") on node " << node_id << " resulted in an error" << std::endl; 
                assert(res >= 0);
            }

            for (const auto &[node_id, evidence_vec] : *virtual_evidence_it)
            {
                const auto res = net.GetNode(node_id)->Value()->SetTemporalEvidence(i, evidence_vec);
                if (res < 0)
                    std::cout << "ERROR: Set temporal evidence (slice=" << i << ") on node " << node_id << " resulted in an error" << std::endl;
                assert(res >= 0);
            }

            ++i;
            incrementTime();
            ++evidence_it;
            ++virtual_evidence_it;
        }
    }

public:
    BayesianNetwork(){}

    /**
     * @brief Construct a new Bayesian Network object. Constructor just calls \ref init
     * 
     * @param file_name File name of intention model. Typically a path to an .xdsl file
     * @param num_network_evaluation_samples 
     */
    BayesianNetwork(std::string file_name, unsigned num_network_evaluation_samples){
        init(file_name, num_network_evaluation_samples);
    }

    /**
     * @brief 
     * 
     * @param file_name File name of intention model. Typically a path to an .xdsl file
     * @param num_network_evaluation_samples 
     */
    void init(std::string file_name, unsigned num_network_evaluation_samples){
        DSL_errorH().RedirectToFile(stdout); //Rederects errors to standard output
        std::string full_path = file_name;
        const auto result = net.ReadFile(full_path.c_str());
        if(result<0) std::cout << "ERROR: Unable to read file: " << full_path.c_str() << std::endl;
        assert(result>=0);
        const auto result_slice = net.SetNumberOfSlices(1);
        if(result_slice<0) std::cout << "ERROR: Insufficient number of time slices" << std::endl;
        assert(result_slice>=0);
        net.SetDefaultBNAlgorithm(DSL_ALG_BN_EPISSAMPLING);
        const auto return_set_samples = net.SetNumberOfSamples(num_network_evaluation_samples);
        if(return_set_samples<0) std::cout << "ERROR: Illigal number of samples set: " << num_network_evaluation_samples << std::endl;
        assert(return_set_samples>=0);

        temporal_evidence_.push_back(std::map<int, int>{});
        temporal_virtual_evidence_.push_back(std::map<int, std::vector<double>>{});
    }

    void save_network(std::string file_name){
        std::string full_path = file_name;
        const auto result = net.WriteFile(full_path.c_str());
        if(result<0) std::cout << "ERROR: Unable to write file: " << full_path.c_str() << std::endl;
        assert(result>=0);
        std::cout << "Network saved to " << full_path.c_str() << std::endl;
    }

    void add_to_dequeue(){
        temporal_evidence_.push_back(std::map<int, int>{});
        temporal_virtual_evidence_.push_back(std::map<int, std::vector<double>>{});
    }

    void setEvidence(std::string node_name, int outcome_id,int time_slice=-1){
        wrapTimeSlice(&time_slice);
        const auto node_id = getNodeId(node_name);
        if(isTemporal(node_id)){
            temporal_evidence_.back()[node_id] = outcome_id;
            //std::map<int, int> new_temporal_evidence { { node_id, outcome_id} };
            //temporal_evidence_.push_back(new_temporal_evidence);
            const auto res = net.GetNode(node_id)->Value()->SetTemporalEvidence(time_slice,outcome_id);
            if(res<0) std::cout << "ERROR: Set temporal evidence (t=" << time_slice << ", outcome_id=" << outcome_id << ") on node " << node_name.c_str() << " resulted in error" << std::endl;
            assert(res>=0);
        }
        else{
            evidence_[node_id] = outcome_id;
            const auto res = net.GetNode(node_id)->Value()->SetEvidence(outcome_id);
            if(res<0) std::cout << "ERROR: Set evidence (outcome_id=" << outcome_id << ") on node " << node_name.c_str() <<" resulted in error" <<std::endl;
            assert(res>=0);
        }
    }

    void setEvidence(std::string node_name, std::string observed_outcome,int time_slice=-1){
        const auto outcome_id = getOutcomeId(node_name, observed_outcome);
        setEvidence(node_name, outcome_id,time_slice);
    }

    //Input <node_name, observation>
    void setEvidence(const std::map<std::string,std::string>&  evidence){
        for(auto e : evidence){
            setEvidence(e.first,e.second);
        }
    }
    

    void setEvidence(std::string node_name, const std::map<std::string,std::string>&  evidence){
        for(auto e : evidence){
            setEvidence(e.first,e.second);
        }
    }

    void setPriors(std::string node_name, const std::map<std::string, double>& prior_distribution){
        const auto node_id = getNodeId(node_name);
        auto node_definition = net.GetNode(node_id)->Definition();
        DSL_doubleArray CPT(node_definition->GetMatrix()->GetSize());
        for(const auto& [outcome_name, outcome_prob] : prior_distribution){
            const auto outcome_id = getOutcomeId(node_name, outcome_name);
            CPT[outcome_id] = outcome_prob;
        }
        setDefinition(node_name, CPT);
    }

    void setBinaryPriors(std::string node_name, double probablity_of_true){
        setPriors(node_name, {{"false", 1-probablity_of_true},{"true", probablity_of_true}});
    }

    // for each elemeent 
    void setPriorNormalDistribution(const std::string node_name, const double mu, const double sigma, const double bin_width){
        const auto node_id = getNodeId(node_name);
        auto node_definition = net.GetNode(node_id)->Definition();
        DSL_doubleArray CPT(node_definition->GetMatrix()->GetSize());
        CPT[0] = evaluateBinProbability(-INFINITY, bin_width, mu, sigma); //First bin takes everything below 0 aswell
        std::cout << "\n Distribution added for colreg sit (" <<node_name<<") :\n" ;
        std::cout << "CPT0: " << CPT[0] << "\n";
        for(auto i = 1; i < CPT.GetSize(); ++i){
            CPT[i] = evaluateBinProbability(i*bin_width, (i+1)*bin_width, mu, sigma);
            std::cout << CPT[i] << " ";
        }
        CPT[CPT.GetSize()-1] = evaluateBinProbability((CPT.GetSize()-1)*bin_width, INFINITY, mu, sigma); //Last bin takes everything above max
        setDefinition(node_name, CPT);
    }

    void setAisDistribution(const std::string node_name, std::string filename, int colreg_idx, int cpa_dist_idx, int multiply, int n_bins, int col_sit){
        std::vector<std::vector<std::string> > content = read_file(filename);
        std::map<int, std::vector<double> > ais_cpa_map = aisMap(content, colreg_idx, cpa_dist_idx, multiply);
        std::map<int, std::vector<double> > distr_cpa_map = distributionMap(ais_cpa_map, n_bins);
        const auto node_id = getNodeId(node_name);
        auto node_definition = net.GetNode(node_id)->Definition();
        DSL_doubleArray CPT(node_definition->GetMatrix()->GetSize());  //henter ut matrix i baysian network
        // CPT = distr_map[-2];
        std::cout << "\n Distribution added for colreg sit (" <<col_sit<<") :\n" ;
        double sum = 0;
        for(auto [col, inVect] : distr_cpa_map){
            if(col == col_sit){
                for(int i=0; i < CPT.GetSize(); ++i){
                    CPT[i]= inVect[i];
                    std::cout << CPT[i] << " ";
                    sum += CPT[i];
                }
            }
        }
        if(sum<0.9999 || sum>1.0001 || !isfinite(sum)){
            double error = 1.0000-sum;
            CPT[CPT.GetSize()-1] +=  error;
        }
        setDefinition(node_name, CPT);
        std::cout << "\n";
    }

      void setAmpleTimeDistribution(const std::string node_name, std::string filename, int ample_time_idx, int timestep, int n_bins){
        std::vector<std::vector<std::string> > content = read_file(filename);
        std::vector<double> ample_time_vec = ampleTimeVec(content, ample_time_idx, timestep);
        std::vector<double> distr_ample_time_vec = find_distribution(ample_time_vec, n_bins);
        const auto node_id = getNodeId(node_name);
        auto node_definition = net.GetNode(node_id)->Definition();
        DSL_doubleArray CPT(node_definition->GetMatrix()->GetSize());  //henter ut matrix i baysian network

        std::cout << "\n Distribution added for ample time :\n" ;
        double sum = 0;
        
        for(int i=0; i < CPT.GetSize(); ++i){
                    CPT[i]= distr_ample_time_vec[i];
                    std::cout << CPT[i] << " ";
                    sum += CPT[i];
            }
        if(sum<0.9999 || sum>1.0001 || !isfinite(sum)){
            double error = 1.0000-sum;
            CPT[CPT.GetSize()-1] +=  error;
        }
        setDefinition(node_name, CPT);
        std::cout << "\n";
    }

    void setVirtualEvidence(std::string node_name,const std::vector<double>& virtualEvidence, int time_slice=-1){
        wrapTimeSlice(&time_slice);
        const auto node_id = getNodeId(node_name);
        if(isTemporal(node_id)){
            temporal_virtual_evidence_.back()[node_id] = virtualEvidence;
            //std::map<int, std::vector<double>> new_temporal_virtual_evidence { { node_id, virtualEvidence} };
            //temporal_virtual_evidence_.push_back(new_temporal_virtual_evidence);
            const auto result = net.GetNode(node_id)->Value()->SetTemporalEvidence(time_slice,virtualEvidence);
            if(result<0) std::cout << "ERROR: Set virtual evidence (t=" << time_slice <<") on node " << node_name.c_str() <<" resulted in error" << std::endl;
            assert(result>=0);
        }
        else{
            virtual_evidence_[node_id] = virtualEvidence;
            const auto result = net.GetNode(node_id)->Value()->SetVirtualEvidence(virtualEvidence);
            if(result<0) std::cout<<"ERROR: Set virtual evidence on node " << node_name.c_str() << " resulted in error" << std::endl;
            assert(result>=0);
        }
    }

    void setVirtualEvidence(std::string node_name, const std::map<std::string, double>& virtualEvidence){
        const auto num_outcomes = getNumberOfOutcomes(node_name);
        assert(num_outcomes>0);
        std::vector<double> virtual_evidence_vector(num_outcomes, 0.0);
        for(auto evidence:virtualEvidence){
            const auto outcome_id = getOutcomeId(node_name, evidence.first);
            virtual_evidence_vector[outcome_id] = evidence.second;
        }
        setVirtualEvidence(node_name,virtual_evidence_vector);
    }

    void incrementTime(){
        const auto number_of_time_slices = net.GetNumberOfSlices();
        const auto res = net.SetNumberOfSlices(number_of_time_slices+1);
        if(res<0) std::cout << "ERROR: Increment time failed" << std::endl;
        assert(res>=0);
    }

    void decrementTime(){
        const auto number_of_time_slices = net.GetNumberOfSlices();
        const auto res = net.SetNumberOfSlices(number_of_time_slices-1);
        if(res<0) std::cout << "ERROR: Decrement time failed" << std::endl;
        assert(res>=0);
    }

    void restartTime(){
        const auto res = net.SetNumberOfSlices(1);
        assert(res>=0);
    }

    auto evaluateStates(std::vector<std::string> node_names){
        const auto current_time_slice = net.GetNumberOfSlices()-1;

        //Target nodes are the nodes the inference algorithm tries to solve.
        for(auto node_name:node_names){
            net.SetTarget(getNodeId(node_name));
        }
        const auto update_res = net.UpdateBeliefs();
        if(update_res<0){
            std::cout << "ERROR: Unable to update beliefs" << std::endl;
            throw "Unable to update beliefs";
        } 
        assert(update_res>=0);

        std::map<std::string,std::map<std::string,double>> return_value;
        for(auto node_name:node_names){
            return_value[node_name] = getOutcome(node_name,current_time_slice);
        }
        return return_value;
    }

    void removeEarlyTimeSteps(int min_number_of_timesteps, int num_of_timesteps_to_be_removed) //must decrement time equal times to number of pop front of evidence
    {
        if (min_number_of_timesteps < 0 || temporal_evidence_.size() < min_number_of_timesteps)
        {
            incrementTime();
        }
        else
        {
            int num_pop = 0;
            while (num_pop<num_of_timesteps_to_be_removed){
                std::cout << "Before: " << temporal_evidence_.size() << std::endl;
                temporal_evidence_.pop_front();
                std::cout << " After pop: " << temporal_evidence_.size() << std::endl;
                temporal_virtual_evidence_.pop_front();
                decrementTime();
                num_pop ++;
            }
            apply_evidence();
            //std::cout << " After apply: " << temporal_evidence_.size();
            
        }
        //temporal_evidence_.push_back(std::map<int, int>{});
        std::cout << " After push: " << temporal_evidence_.size() << std::endl;
        //temporal_virtual_evidence_.push_back(std::map<int, std::vector<double>>{});
    }

    void clearEvidence(){
        net.ClearAllEvidence();
    }

    auto getNumberOfTimeSteps()const{
        return net.GetNumberOfSlices();
    }

    auto isEvidence(){
        return net.IsThereAnyEvidence();
    }
     
    auto isDecisions(){
        return net.IsThereAnyDecision();
    }
};
}
