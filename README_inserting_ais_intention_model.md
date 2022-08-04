## Using AIS data to evaluate intention model

The goal of this work is to evaluate the intention model using AIS data.

To run AIS data on the intention model a case file with two (three) ships is used. The file must contain mmsi, times (YYYY-mm-dd HH:MM:SS), x (north), y (east), sog and cog. The file must have consistent time intervals that are the same for both ships. A matlab file (extract_file.m) was used to filter case files to only get the wanted mmsis and x,y coordinates instead of latitude and longitude.

insert_ais.cpp reads the AIS data file and uses the intention model. For each timestep, it inserts an observation of the ships into the model and writes the intentions at that timestep to an intention file. These variables are written to the intention file: mmsi, x, y, and time. As well as the intention variables: colreg_compliant, good_semanship, unmodeled_behaviour. Probability of what kind of situation it is in, and the probability that the priority is lower, similar, or higher.

Historic AIS data is used to find distributions for safe distance and ample time.

intention_model_two_ships.xdsl is used as the intentionModelFile, but can also use intention_model_three_ships.xdsl with a few modifications of insert_ais.cpp.


Can also use a different intentionModelFile: intention_model_with_risk_of_collision.xdsl. This is currently working on a different branch called riskofCollision. Here the risk of collision is taken into consideration, a risk distance distribution is used and if r_cpa is high there is no risk of collision and maneuvers will not affect the intentions.

To evaluate the intentions, by plotting the values from the intention file, a matlab file (plot_intentions.m) was used. The ship positions and intention variables were plotted with circles moving indicating each timestep.

When evaluating the intention model using different situations it seems like the intentions are mostly correct. However, it is difficult to decide when a situation has actually started. Now the situation is defined as started if the distance between the ships is under a certain value and both ships have a speed above 1 m/s. It is also difficult to know for sure if the maneuvers made are related to the situation or not.
