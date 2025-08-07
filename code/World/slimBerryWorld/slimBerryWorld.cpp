//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/Hintzelab/MABE/wiki/License

#include "slimBerryWorld.h"

// this is how you setup a parameter in MABE, the function Parameters::register_parameter()takes the
// name of the parameter (catagory-name), default value (which must conform with the type), a the useage message
shared_ptr<ParameterLink<int>> slimBerryWorld::evaluationsPerGenerationPL =
Parameters::register_parameter("WORLD_slimBerry-evaluationsPerGeneration", 6,
    "how many times should each organism be tested in each generation?");

shared_ptr<ParameterLink<int>> slimBerryWorld::swarmSizePL =
Parameters::register_parameter("WORLD_slimBerry-swarmSize", 8,
    "size of the swarm (number of clones)");

shared_ptr<ParameterLink<double>> slimBerryWorld::switchCostPL =
Parameters::register_parameter("WORLD_slimBerry-switchCost", 10.0,
    "cost payed to swich eat a different food type");

shared_ptr<ParameterLink<int>> slimBerryWorld::directionsPL =
Parameters::register_parameter("WORLD_slimBerry-directions", 4,
    "number of directions, 4 or 8. this effects both agent turning and sensors");

shared_ptr<ParameterLink<int>> slimBerryWorld::scoreRulePL =
Parameters::register_parameter("WORLD_slimBerry-scoreRule", 2,
    "0: min, 1: ave, 2: max");


/*
// like vectorToBitToInt, but converts node values to trits (-1, 0, or 1) adds one to
// the result and then generates a number in base 3 rather then base 2.
template <typename Type>
inline int test_vectorToTritToInt(const std::vector<Type>& nodes,
    const std::vector<int>& nodeAddresses, bool reverseOrder = false) {
    if (reverseOrder)
        return std::accumulate(nodeAddresses.crbegin(), nodeAddresses.crend(), 0,
            [&nodes](int result, int na) {return result * 3 + Trit(nodes.at(na)) + 1; });
    else
        return std::accumulate(nodeAddresses.cbegin(), nodeAddresses.cend(), 0,
            [&nodes](int result, int na) {return result * 3 + Trit(nodes.at(na)) + 1; });
}

void testTriTupdate() {
    std::vector<double> inputs1 = { 0,1,1,0 };
    std::vector<double> inputs2 = { 0,2,1,0 };
    std::vector<int> addreses = { 0,1,2,3 };

    int input1 = test_vectorToTritToInt(inputs1, addreses, true);  // converts the input values into an index
    int input2 = test_vectorToTritToInt(inputs2, addreses, true);  // converts the input values into an index
    std::cout << input1 << " " << input2 << std::endl;
    std::cout << "done" << std::endl;
    exit(1);
}
*/

// the constructor gets called once when MABE starts up. use this to set things up
slimBerryWorld::slimBerryWorld(shared_ptr<ParametersTable> PT) : AbstractWorld(PT) {
    
    //localize a parameter value for faster access
    evaluationsPerGeneration = evaluationsPerGenerationPL->get(PT);
    swarmSize = swarmSizePL->get(PT);
    switchCost = switchCostPL->get(PT);
    scoreRule = scoreRulePL->get(PT);

    directions = directionsPL->get(PT);
    sensorCount = (directions == 8) ? 5 : 3;
    forwardIndex = (directions == 8) ? 2 : 1;

    std::cout << "setting up slimBerryWorld..." << std::endl;
    std::cout << "  evals..............." << evaluationsPerGeneration << std::endl;
    std::cout << "  swarmSize..........." << swarmSize << std::endl;
    std::cout << "  switchCost.........." << switchCost << std::endl;
    std::cout << "  scoreRule...........";
    if (scoreRule == 0) {
        std::cout << "min" << std::endl;
    }
    if (scoreRule == 1) {
        std::cout << "ave" << std::endl;
    }
    if (scoreRule == 2) {
        std::cout << "max" << std::endl;
    }
    std::cout << "  directions.........." << directions << std::endl;

    // set up a clean slate - this will be the map for agents and walls, but here we just set up walls
    // this will be copied each generation. in this array, 9 is wall, 0 is empty and 1..8 is swarm ID
    foodGridMask.fill(1); // everything is allowed then remove the walls
    for (int x = 0; x < worldEdgeSize; x++) {
        agentGridEmpty[getIndex(x, 0)] = 9;           // Top edge (y = 0)
        agentGridEmpty[getIndex(x, worldEdgeSize - 1)] = 9;    // Bottom edge (y = size - 1)
        foodGridMask[getIndex(x, 0)] = 0;        // Top edge (y = 0)
        foodGridMask[getIndex(x, worldEdgeSize - 1)] = 0; // Bottom edge (y = size - 1)
    }
    for (int y = 1; y < worldEdgeSize - 1; y++) { // Avoid corners being set twice
        agentGridEmpty[getIndex(0, y)] = 9;           // Left edge (x = 0)
        agentGridEmpty[getIndex(worldEdgeSize - 1, y)] = 9;    // Right edge (x = size - 1)
        foodGridMask[getIndex(0, y)] = 0;        // Left edge (x = 0)
        foodGridMask[getIndex(worldEdgeSize - 1, y)] = 0; // Right edge (x = size - 1)
    }

    int k = 0;
    for (int y = 1; y < worldEdgeSize - 1; y++) { // set up validStartLocations with all non-wall locations
        for (int x = 1; x < worldEdgeSize - 1; x++) {
            validStartLocations[k++] = getIndex(x, y);
        }
    }
    

    // set up an array that is 1/2 1 and 1/2 2 to use for food placement
    int halfSize = (worldEdgeSize * worldEdgeSize) / 2;
    for (int i = 0; i < worldEdgeSize * worldEdgeSize; i++) {
        if (i < halfSize) {
            foodGridInit[i] = 1;
        }
        else {
            foodGridInit[i] = 2;
        }
    }

    // update order will be shuffled every world update and used to determine agent update order
    updateOrder.resize(swarmSize);
    std::iota(updateOrder.begin(), updateOrder.end(), 0);


    // sensors are set up with 8 directions, the offset arrays provide the offset for each sensor position given the direction
    // direction = 0 is up, rotation is clockwise
    sensorXoffsets.resize(sensorCount);
    sensorYoffsets.resize(sensorCount);

    if (directions == 8) {
        sensorXoffsets[2] = {  0, 1, 1, 1, 0,-1,-1,-1 };   // forward
        sensorYoffsets[2] = { -1,-1, 0, 1, 1, 1, 0,-1 };
        sensorXoffsets[1] = { -1, 0, 1, 1, 1, 0,-1,-1 };  // left forward 
        sensorYoffsets[1] = { -1,-1,-1, 0, 1, 1, 1, 0 };
        sensorXoffsets[3] = {  1, 1, 1, 0,-1,-1,-1, 0 };   // rigth forward
        sensorYoffsets[3] = { -1, 0, 1, 1, 1, 0,-1,-1 };
        sensorXoffsets[0] = { -1,-1, 0, 1, 1, 1, 0,-1 };  // left
        sensorYoffsets[0] = {  0,-1,-1,-1, 0, 1, 1, 1 };
        sensorXoffsets[4] = {  1, 1, 0,-1,-1,-1, 0, 1 };   // right
        sensorYoffsets[4] = {  0, 1, 1, 1, 0,-1,-1,-1 };
    }
    else { // directions = 4
        sensorXoffsets[1] = {  0, 1, 0,-1 };   // forward
        sensorYoffsets[1] = { -1, 0, 1, 0 };
        sensorXoffsets[0] = { -1, 0, 1, 0 };  // left
        sensorYoffsets[0] = {  0,-1, 0, 1 };
        sensorXoffsets[2] = {  1, 0,-1, 0 };   // right
        sensorYoffsets[2] = {  0, 1, 0,-1 };
    }

    swarmFood1Eats.resize(swarmSize);
    swarmFood2Eats.resize(swarmSize);
    swarmSwitches.resize(swarmSize);

    // these are used durring update to know where in the world food, agents and walls are. This needs to be pregenerated for each eval
    foodGridSafeList.resize(evaluationsPerGeneration);
    agentGridSafeList.resize(evaluationsPerGeneration);

    // these hold the start location and direction for each agent in the swarm. This needs to be pregenerated for each eval
    agentXSafeList.resize(evaluationsPerGeneration);
    agentYSafeList.resize(evaluationsPerGeneration);
    agentDSafeList.resize(evaluationsPerGeneration);
    for (int eval = 0; eval < evaluationsPerGeneration; eval++) {
        agentXSafeList[eval].resize(swarmSize);
        agentYSafeList[eval].resize(swarmSize);
        agentDSafeList[eval].resize(swarmSize);
    }

    agentX.resize(swarmSize);
    agentY.resize(swarmSize);
    agentD.resize(swarmSize);
    agentLastFood.resize(swarmSize);

    // popFileColumns tell MABE what data should be saved to pop.csv files
	popFileColumns.clear();
    popFileColumns.push_back("score");
    popFileColumns.push_back("switchAve");
    popFileColumns.push_back("switchMax");
    popFileColumns.push_back("switchMin");

    popFileColumns.push_back("food1Ave");
    popFileColumns.push_back("food1Max");
    popFileColumns.push_back("food1Min");

    popFileColumns.push_back("food2Ave");
    popFileColumns.push_back("food2Max");
    popFileColumns.push_back("food2Min");
}

// the evaluate function gets called every generation. evaluate should set values on organisms datamaps
// that will be used by other parts of MABE for things like reproduction and archiving
auto slimBerryWorld::evaluate(map<string, shared_ptr<Group>>& groups, int analyze, int visualize, int debug) -> void {

    int popSize = groups[groupName]->population.size();

    // in order to be able to collect all of the information for a swarm, that swarm must exist over all evals.
    // this means that we must create all start conditions for all evals, then we can eval each swarm on each start condition

    // set up the eval inital conditions, then we will itterate over every agent in the popultion
    // create a swarm for that agent and evaluate that swarm on all evals

    for (int eval = 0; eval < evaluationsPerGeneration; eval++) {

        // this sets 1/2 the food to 1 and 1/2 to 2
        foodGridSafeList[eval] = foodGridInit;
        // this randomizes the locations
        std::shuffle(foodGridSafeList[eval].begin(), foodGridSafeList[eval].end(), Random::getCommonGenerator());
        // this removes the food from the wall locations
        applyMask(foodGridSafeList[eval], foodGridMask);

        // randomize the order of validStartLocations, we will use the first swarmSize elements to initalize agent locations
        std::shuffle(validStartLocations.begin(), validStartLocations.end(), Random::getCommonGenerator());

        agentGridSafeList[eval] = agentGridEmpty; // clear the agent grid - now it only has walls
        // place the agents in their inital positions
        for (int i = 0; i < swarmSize; i++) {
            auto loc = getLoc(validStartLocations[i]);
            agentXSafeList[eval][i] = loc.first;
            agentYSafeList[eval][i] = loc.second;
            agentDSafeList[eval][i] = Random::getIndex(directions);       // direction the agent is facing
            agentGridSafeList[eval][validStartLocations[i]] = 1; // this location is occupoied by an agent
        }
    }

    // now that the evals are all set up, itterate over the population,
    // create a swarm from each agent in the population
    // evaluate the swarm on each eval setup
    for (int pop_id = 0; pop_id < popSize; pop_id++) { // for each agent in the population
        
        auto org = groups[groupName]->population[pop_id];
        auto brain = org->brains[brainName];
        
        std::vector<std::vector<std::vector<int>>> lastBerryHistory; // this will allow us to ask: does the agent know what berry it ate last?
        swarmBrains.clear();
        for (int i = 0; i < swarmSize; i++) { // make a swarm
            swarmBrains.push_back(brain->makeCopy(brain->PT));
            if (analyze) {
                swarmBrains.back()->setRecordActivity(true);
                lastBerryHistory.emplace_back();  // this will allow us to ask: does the agent know what berry it ate last?
            }
        }



        for (int eval = 0; eval < evaluationsPerGeneration; eval++) {
        //std::cout << "eval: " << eval << std::endl;

            // copy the food gird for this eval, we will be changing foodGrid, this way everyone gets the same setup
            foodGrid = foodGridSafeList[eval];

            // set the agent locations on the grid for this eval
            agentX = agentXSafeList[eval];
            agentY = agentYSafeList[eval];
            agentD = agentDSafeList[eval];

            // copy the agent grid (where are agents and walls) for this eval
            agentGrid = agentGridSafeList[eval];

            if (debug) {
                statusReport();
            }

            // setup this swarm
            std::fill(swarmFood1Eats.begin(), swarmFood1Eats.end(), 0);
            std::fill(swarmFood2Eats.begin(), swarmFood2Eats.end(), 0);
            std::fill(swarmSwitches.begin(), swarmSwitches.end(), 0);

            std::fill(agentLastFood.begin(), agentLastFood.end(), 0); // no agent has eaten any food yet


            if (visualize || analyze) {
                std::string vis_str = "**InitializeWorld**\n";
                vis_str += std::to_string(directions) + "," + std::to_string(worldEdgeSize) + "," + std::to_string(worldEdgeSize) + "," + std::to_string(swarmSize) + "\n";
                vis_str += world2str();
                vis_str += "-\n";
                vis_str += "**InitializeHarvesters**\n";
                for (int i = 0; i < swarmSize; i++) {
                    vis_str += std::to_string(i) + "," + std::to_string(agentX[i]+.5) + "," + std::to_string(agentY[i]+.5) + "," + std::to_string(agentD[i]) + "," + std::to_string(i) + "\n";
                }
                vis_str += "-\n";
                FileManager::writeToFile("HarvestWorldData.txt", vis_str);
            }


            for (int i = 0; i < swarmSize; i++) { // make a swarm
                swarmBrains[i]->resetBrain();
            }

            int evalTime = 200;
            for (int t = 0; t < evalTime; t++) {
                if (visualize || analyze) {
                    std::string vis_str = "U," + std::to_string(t) + "\n";
                    FileManager::writeToFile("HarvestWorldData.txt", vis_str);
                }

                // randomize the order agents update
                std::shuffle(updateOrder.begin(), updateOrder.end(), Random::getCommonGenerator());

                for (auto id : updateOrder) {
                    if (0) {
                        std::cout << Global::update << " " << pop_id << " " << eval << " " << t << "   " << id << std::endl;
                    }

                    if (analyze) {
                        lastBerryHistory[id].push_back({ agentLastFood[id] });
                    }
                    int inputIndex = 0;
                    // food here?
                    swarmBrains[id]->setInput(inputIndex++, foodGrid[getIndex(agentX[id], agentY[id])] == 1); // food1 here?
                    swarmBrains[id]->setInput(inputIndex++, foodGrid[getIndex(agentX[id], agentY[id])] == 2); // food2 here?

                    // food, wall, and other agent at each other sensor location
                    for (int si = 0; si < sensorCount;  si++) {
                        swarmBrains[id]->setInput(inputIndex++, foodGrid[getIndex(agentX[id] + sensorXoffsets[si][agentD[id]], agentY[id] + sensorYoffsets[si][agentD[id]])] == 1); // food1 here?
                        swarmBrains[id]->setInput(inputIndex++, foodGrid[getIndex(agentX[id] + sensorXoffsets[si][agentD[id]], agentY[id] + sensorYoffsets[si][agentD[id]])] == 2); // food2 here?
                        //swarmBrains[id]->setInput(inputIndex++, agentGrid[getIndex(agentX[id] + sensorXoffsets[si][agentD[id]], agentY[id] + sensorYoffsets[si][agentD[id]])] == 9); // wall here?
                        //swarmBrains[id]->setInput(inputIndex++, agentGrid[getIndex(agentX[id] + sensorXoffsets[si][agentD[id]], agentY[id] + sensorYoffsets[si][agentD[id]])] == 1); // other agent here?
                    }

                    swarmBrains[id]->setInput(inputIndex++, agentGrid[getIndex(agentX[id] + sensorXoffsets[forwardIndex][agentD[id]], agentY[id] + sensorYoffsets[forwardIndex][agentD[id]])] == 9); // wall here?
                    swarmBrains[id]->setInput(inputIndex++, agentGrid[getIndex(agentX[id] + sensorXoffsets[forwardIndex][agentD[id]], agentY[id] + sensorYoffsets[forwardIndex][agentD[id]])] == 1); // other agent here?

                    if (0) {
                        printGrid(foodGrid);
                        std::cout << std::endl;
                        printGrid(agentGrid);
                        std::cout << std::endl;
                        std::cout << id << " " << agentX[id] << "," << agentY[id] << "(" << agentD[id] << ")" << std::endl;
                    }

                    swarmBrains[id]->update();
                    int motorLeft = swarmBrains[id]->readOutput(0);
                    int motorRight = swarmBrains[id]->readOutput(1);
                    int actionEat = swarmBrains[id]->readOutput(2);

                    int foodHere = foodGrid[getIndex(agentX[id], agentY[id])];
                    if (actionEat>0) {
                        if (foodHere > 0) { // agent wants to eat, and there is food here
                            if (agentLastFood[id] != foodHere && agentLastFood[id] > 0) {
                                swarmSwitches[id]++;
                            }
                            agentLastFood[id] = foodHere;
                            if (foodHere == 1) {
                                swarmFood1Eats[id]++;
                            }
                            else { // food here must be 2
                                swarmFood2Eats[id]++;
                            }

                            if (visualize || analyze) {
                                std::string vis_str = "E," + std::to_string(agentX[id]) + "," + std::to_string(agentY[id]) + "," + "\n";
                                FileManager::writeToFile("HarvestWorldData.txt", vis_str);
                            }

                            foodGrid[getIndex(agentX[id], agentY[id])] = 0; // the food here was gathered
                        }
                    }
                    else { // agent did not try to eat
                        if (motorLeft != motorRight) { // turn - the moters have different values
                            agentD[id] = loopMod(agentD[id] + std::min(1, std::max(-1, motorRight - motorLeft)), directions);
                            if (visualize || analyze) {
                                std::string vis_str = "";
                                if (motorLeft > motorRight) {
                                    vis_str = "TL," + std::to_string(id) + "," + std::to_string(agentD[id]) + "\n";
                                }
                                else {
                                    vis_str = "TR," + std::to_string(id) + "," + std::to_string(agentD[id]) + "\n";

                                }
                                FileManager::writeToFile("HarvestWorldData.txt", vis_str);
                            }

                        }
                        if (motorLeft == motorRight && motorLeft > 0) { // move - equal values to both motors
                            int targetX = agentX[id] + sensorXoffsets[forwardIndex][agentD[id]];
                            int targetY = agentY[id] + sensorYoffsets[forwardIndex][agentD[id]];
                            if (0) {
                                std::cout << Global::update << " " << eval << " " << pop_id << " " << t << "   " << id << " " << agentX[id] << "," << agentY[id] << "(" << agentD[id] << ")" << sensorXoffsets[forwardIndex][agentD[id]] << "," << sensorYoffsets[forwardIndex][agentD[id]] << ">>" << targetX << "," << targetY << std::endl;
                            }
                            if (agentGrid[getIndex(targetX, targetY)] == 0) { // forward location is empty
                                agentGrid[getIndex(targetX, targetY)] = 1;
                                agentGrid[getIndex(agentX[id], agentY[id])] = 0;
                                if (foodHere == 0) { // if the current locaiton is empty
                                    foodGrid[getIndex(agentX[id], agentY[id])] = Random::getInt(1, 2);
                                    if (visualize || analyze) {
                                        std::string vis_str = "R," + std::to_string(agentX[id]) + "," + std::to_string(agentY[id]) + "," + std::to_string(foodGrid[getIndex(agentX[id], agentY[id])]) + "\n";
                                        FileManager::writeToFile("HarvestWorldData.txt", vis_str);
                                    }
                                }
                                agentX[id] = targetX;
                                agentY[id] = targetY;
                                if (visualize || analyze) {
                                    std::string vis_str = "M," + std::to_string(id) + "," +  std::to_string(agentX[id]+.5) + "," + std::to_string(agentY[id]+.5) + "\n";
                                    FileManager::writeToFile("HarvestWorldData.txt", vis_str);
                                }
                            }
                        }
                    }

                    if (0) {
                        printGrid(foodGrid);
                        std::cout << std::endl;
                        printGrid(agentGrid);
                        std::cout << std::endl;
                        std::cout << id << " " << agentX[id] << "," << agentY[id] << "(" << agentD[id] << ")" << std::endl;
                    }

                    if (visualize || analyze) {
                        std::string vis_str  = std::to_string(id) + "," + std::to_string(agentX[id]) + "," + std::to_string(agentY[id]) + "," + std::to_string(agentD[id]) + "," + std::to_string(id) + "\n";
                        FileManager::writeToFile("HarvestWorldData.txt", vis_str);
                    }

                } // all agents have been updated
            } // swarm eval complete
            
            std::vector<double> scores;
            double scoreTotal = 0;
            double minScore = 1000000;
            double maxScore = -1000;
            double switchTotal = 0;
            double switchMax = 0;
            double switchMin = 1000000;
            double food1Total = 0;
            double food1Max = 0;
            double food1Min = 1000000;
            double food2Total= 0;
            double food2Max = 0;
            double food2Min = 1000000;

            for (int id = 0; id < swarmSize; id++) {
                //std::cout << id << " " << swarmFood1Eats[id] << " " << swarmFood2Eats[id] << " " << swarmSwitches[id] << std::endl;
                scores.push_back(swarmFood1Eats[id] + swarmFood2Eats[id] - (swarmSwitches[id] * switchCost));
                scoreTotal += scores.back();
                maxScore = std::max(scores.back(), maxScore);
                minScore = std::min(scores.back(), minScore);

                switchTotal += swarmSwitches[id];
                switchMax = std::max(switchMax, double(swarmSwitches[id]));
                switchMin = std::min(switchMin, double(swarmSwitches[id]));

                food1Total += swarmFood1Eats[id];
                food1Max = std::max(food1Max, double(swarmFood1Eats[id]));
                food1Min = std::min(food1Min, double(swarmFood1Eats[id]));

                food2Total += swarmFood2Eats[id];
                food2Max = std::max(food2Max, double(swarmFood2Eats[id]));
                food2Min = std::min(food2Min, double(swarmFood2Eats[id]));
            }
            if (scoreRule == 0) { // min
                org->dataMap.append("score", minScore);
            }
            if (scoreRule == 1) { // ave
                org->dataMap.append("score", scoreTotal / swarmSize);
            }
            if (scoreRule == 2) { // max
                org->dataMap.append("score", maxScore);
            }
            org->dataMap.append("switchAve", switchTotal / swarmSize);
            org->dataMap.append("switchMax", switchMax);
            org->dataMap.append("switchMin", switchMin);
            org->dataMap.append("food1Ave", food1Total / swarmSize);
            org->dataMap.append("food1Max", food1Max);
            org->dataMap.append("food1Min", food1Min);
            org->dataMap.append("food2Ave", food2Total / swarmSize);
            org->dataMap.append("food2Max", food2Max);
            org->dataMap.append("food2Min", food2Min);

        } // done with one eval using the current agent from the population

        if (analyze) { // oh boy, this should get interesting

            int agentID = org->ID;
            std::cout << "\nAlalyze Mode:  organism with ID " << agentID << " scored " << org->dataMap.getAverage("score") << std::endl;

            if (1) { // saveBrainStructureAndConnectome
                std::cout << "  saving brain connectome and structrue..." << std::endl;
                brain->saveConnectome("brainConnectome_id_" + std::to_string(agentID) + ".py");
                brain->saveStructure("brainStructure_id_" + std::to_string(agentID) + ".dot");
            }

            FileManager::writeToFile("score.txt", std::to_string(org->dataMap.getAverage("score")));

            for (int C_id = 0; C_id < swarmSize; C_id++) { // for each swarm member...

                std::cout << "  processing clone " << C_id << "..." << std::endl;
                auto lifeTimes = swarmBrains[C_id]->getLifeTimes();
                auto inputStateSet = TS::remapToIntTimeSeries(swarmBrains[C_id]->getInputStates(), TS::RemapRules::TRIT);

                auto outputStateSet = TS::remapToIntTimeSeries(swarmBrains[C_id]->getOutputStates(), TS::RemapRules::TRIT);

                auto hiddenFullStatesSet = TS::remapToIntTimeSeries(swarmBrains[C_id]->getHiddenStates(), TS::RemapRules::TRIT);
                //auto hiddenFullStatesSet = TS::remapToIntTimeSeries(swarmBrains[C_id]->getHiddenStates(), TS::RemapRules::TRIT);
                auto hiddenAfterStateSet = TS::trimTimeSeries(hiddenFullStatesSet, TS::Position::FIRST, lifeTimes);
                auto hiddenBeforeStateSet = TS::trimTimeSeries(hiddenFullStatesSet, TS::Position::LAST, lifeTimes);

                std::string fullID = std::to_string(agentID) + "_" + std::to_string(C_id);

                if (1) { //saveStateToState
                    std::cout << "  saving state to state..." << std::endl;
                    std::string fileName = "StateToState_id_" + fullID + ".txt";
                    S2S::saveStateToState({ hiddenFullStatesSet, TS::extendTimeSeries(outputStateSet, lifeTimes, {0}, TS::Position::FIRST) }, { inputStateSet }, lifeTimes, "H_O__I_" + fileName);
                    S2S::saveStateToState({ hiddenFullStatesSet }, { outputStateSet, inputStateSet }, lifeTimes, "H__O_I_" + fileName);
                    S2S::saveStateToState({ hiddenFullStatesSet }, { inputStateSet }, lifeTimes, "H_I_" + fileName);
                }
                if (1) { // save_R_FragMatrix
                    std::cout << "  saving R frag matrix..." << std::endl;
                    FRAG::saveFragMatrix( lastBerryHistory[C_id] , hiddenAfterStateSet, "R_FragmentationMatrix_id_" + fullID + ".py", "feature", { "last_food" });
                }
                if (1) { // saveFlowMatrix
                    std::cout << "  saving flow matix..." << std::endl;

                    // save data flow information - 
                    //std::vector<std::pair<double, double>> flowRanges = { {0,1},{0,.333},{.333,.666},{.666,1},{0,.5},{.5,1} };
                    //std::vector<std::pair<double, double>> flowRanges = { {0,1},{0,.1},{.9,1} };
                    std::vector<std::pair<double, double>> flowRanges = { {0,.25}, {.75,1}, {0,1} };//, { 0,.1 }, { .9,1 }};

                    //std::cout << TS::TimeSeriesToString(TS::trimTimeSeries(brainStates, TS::Position::LAST, lifeTimes), ",",",") << std::endl;
                    //std::cout << TS::TimeSeriesToString(TS::trimTimeSeries(brainStates, TS::Position::FIRST, lifeTimes), ",",",") << std::endl;
                    FRAG::saveFragMatrixSet(
                        TS::Join(TS::trimTimeSeries(hiddenFullStatesSet, TS::Position::FIRST, lifeTimes), outputStateSet),
                        TS::Join(TS::trimTimeSeries(hiddenFullStatesSet, TS::Position::LAST, lifeTimes), inputStateSet),
                        lifeTimes, flowRanges, "flowMap_id_" + fullID + ".py", "shared", -1, true); 
                }
                if (1) { // saveStates
                    std::cout << "  saving brain states information..." << std::endl;
                    std::string fileStr = "";

                    auto discreetInput = inputStateSet;
                    auto discreetOutput = outputStateSet;
                    auto discreetHiddenBefore = TS::trimTimeSeries(hiddenFullStatesSet, TS::Position::LAST, lifeTimes);
                    auto discreetHiddenAfter = TS::trimTimeSeries(hiddenFullStatesSet, TS::Position::FIRST, lifeTimes);
                    int timeCounter = 0;
                    int lifeCounter = 0;
                    int lifeTimeCounter = 0;
                    fileStr += "input,output,hiddenBefore,hiddenAfter,time,life,lifeTime\n";
                    for (int i = 0; i < discreetInput.size(); i++) {
                        fileStr += "\"" + TS::TimeSeriesSampleToString(discreetInput[i], ",") + "\",";
                        fileStr += "\"" + TS::TimeSeriesSampleToString(discreetOutput[i], ",") + "\",";
                        fileStr += "\"" + TS::TimeSeriesSampleToString(discreetHiddenBefore[i], ",") + "\",";
                        fileStr += "\"" + TS::TimeSeriesSampleToString(discreetHiddenAfter[i], ",") + "\",";

                        fileStr += std::to_string(timeCounter) + ",";
                        fileStr += std::to_string(lifeCounter) + ",";
                        fileStr += std::to_string(lifeTimeCounter) + "\n";

                        timeCounter++;
                        lifeTimeCounter++;
                        if (lifeTimes[lifeCounter] == lifeTimeCounter) { // if we are at the end of the current lifetime
                            lifeCounter++;
                            lifeTimeCounter = 0;
                        }
                    }
                    FileManager::writeToFile("PathFollow_BrainActivity_id_" + fullID + ".csv", fileStr);
                }
                std::cout << "  ... analyze done" << std::endl;
            }
        }
    } // done with the current agent from the population
    if (visualize || analyze) {
        FileManager::writeToFile("HarvestWorldData.txt", "EOF");
    }
}

// the requiredGroups function lets MABE know how to set up populations of organisms that this world needs
auto slimBerryWorld::requiredGroups() -> unordered_map<string,unordered_set<string>> {
    std::cout << "slimBerryWorld is being run with direction = " << directions << " brains will have: " << 2 + (sensorCount * 2) + 2 << " inputs and 3 outputs" << std::endl;
	return { { groupName, { "B:"+brainName+","+std::to_string(2 + (sensorCount * 2) + 2) + ",3" } } };
}
