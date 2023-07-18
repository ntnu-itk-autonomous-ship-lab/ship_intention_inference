# ship_intention_inference
Intention model to predict intentions of ships using ais data. To use in another project, the files in `inc/` are of interest, in particular `intention_model.h`. In addition, a range of .cpp files are made to test and improve the intention model. **Currently, only insert_ais.cpp and generate_intention_model.cpp are supported**. The rest of the cpp files are reminants from older versions. 

## Setup
To get the intention model to run we need the smile library. Therefore, you need to download the correct version of this for your computer from https://www.bayesfusion.com. Add the contents of the smile folder to `external/smile`. We also need `smile_licence.h`, that must be added to this folder as well, and can be downloaded from the same page. 

To build, run

```bash
cmake -S . -B build
cmake --build build
```
After that, the compiled programs will be in the `build` folder. Then to generate priors for the intention model, run

```bash
./build/generate_intention_model
```
and then run to test the intention model with the `insert_ais` program, run the following command:

```bash
./build/insert_ais
```
## Using AIS data to evaluate intention model

The goal of this work is to evaluate the intention model using AIS data.

To run AIS data on the intention model a case file with two (three) ships is used. The file must contain mmsi, times (YYYY-mm-dd HH:MM:SS), x (north), y (east), sog (speen over ground) and cog (course over ground). The file must have consistent time intervals that are the same for both ships. A matlab file (*extract_file.m*) was used to filter case files to only get the wanted mmsis and x,y coordinates instead of latitude and longitude.

*insert_ais.cpp* reads the AIS data file and uses the intention model. For each timestep, it inserts an observation of the ships into the model and writes the intentions at that timestep to an intention file. These variables are written to the intention file: mmsi, x, y, and time. As well as the intention variables: colreg_compliant, good_semanship, unmodeled_behaviour. In addition to the probability of what kind of situation it is in, and the probability that the priority is lower, similar, or higher.

`intention_model_two_ships.xdsl` is used as the intentionModelFile, but can also use `intention_model_three_ships.xdsl` with a few modifications of `insert_ais.cpp`.

Can also use a different intentionModelFile: `intention_model_with_risk_of_collision.xdsl`. This is currently working on a different branch called `riskofCollision`. Here the risk of collision is taken into consideration, a risk distance distribution is used and if `r_cpa` is high there is no risk of collision and maneuvers will not affect the intentions.

Also `generate_intention_model.cpp` will generate the intentionModelFile `intention_model_from_code.xdsl`, which will use historic AIS data to find distributions for safe distance and ample time. The rest of the prior distributions will directly be transferred from another given intentionModelFIle.

In the file `parameters.h`, a function with default parameters is defined, that is used both for `insert_ais` and `generate_intention_model`. 

To evaluate the intentions, by plotting the values from the intention file, a matlab file (*plot_intentions.m*) was used. The ship positions and intention variables were plotted with circles moving indicating each timestep.

When evaluating the intention model using different situations it seems like the intentions are mostly correct. However, it is difficult to decide when a situation has actually started. Now the situation is defined as started if the distance between the ships is under a certain value and both ships have a speed above 1 m/s. It is also difficult to know for sure if the maneuvers made are related to the situation or not.

## Working with genie
Going from continuous to discrete:
intention_model_continious.xdsl is used to design the network but must be discretized before use. 
This is done by opening the model in Genie (https://www.bayesfusion.com/) then opening the menu "Network" and pressing Discretize. 
The network must then be made dynamic, this is done by choosing "Network" - "Dynamic Models" - "Enable temporal plate"
All nodes that are supposed to be temporal (all except for intentions nodes) must then be marked and dragged into the temporal plate. 
Special care must be given to the "has turned starboardwards/portwards" node as they depend on their previous state. 
Make a new arc from these nodes to themselves and choose "Order 1" and delete the "has turned starboardwards/portwards t-1" nodes. Open the definition of the "has turned starboardwards/portwards" nodes, for "t = 0" the CPT of the "has turned starboardwards/portwards t-1" in the continuous model node is used, for "t = 1" the CPT of the "has turned starboardwards/portwards" in the continuous model node is used.

Making a model for more ships:
Mark all nodes that should be repeated for multiple ships (these are all noes with ship0 at the end of their name).
Copy and paste these node (within the temporal plate). 
Save the network and open it in sublime text (or another text editing tool). We now need to rename the new nodes from "Copy\_of\_[node\_name]\_ship0" to "[node\_name]\_ship1" (or 2 or 3 etc).
This can be done with the regex expression:
```
Find: Copy_of_(.*?)ship0
Replace: $1ship1
```
And then:
```
Find: Copy of (.*?)ship0
Replace: $1ship1
```
Save and open the file in genie
Next the observation_applicable node must be updated such that both "applicable_or_disabled_ship0" and "applicable_or_disabled_ship1" must be true for it to be true.
Then "stands_on_correct_or_forced_to_give_way_ship0/1" must be updated. For ship 0 it should be true if the ship_stands_on_correct or if ship1 is enabled, it has a gw role to ship1, gives way correct to ship1, and has not safely passed ship1. Similarly for ship1. 