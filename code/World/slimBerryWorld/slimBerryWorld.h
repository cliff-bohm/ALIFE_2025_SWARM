//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/Hintzelab/MABE/wiki/License

#pragma once    // directive to insure that this .h file is only included one time

#include <World/AbstractWorld.h> // AbstractWorld defines all the basic function templates for worlds
#include <string>
#include <memory> // shared_ptr
#include <map>

using std::shared_ptr;
using std::string;
using std::map;
using std::unordered_map;
using std::unordered_set;
using std::to_string;

#include <Analyze/neurocorrelates.h>
#include <Analyze/brainTools.h>
#include <Analyze/fragmentation.h>
#include <Analyze/smearedness.h>

class slimBerryWorld : public AbstractWorld {

public:
    // parameters for group and brain namespaces
    static shared_ptr<ParameterLink<int>> evaluationsPerGenerationPL;
    static shared_ptr<ParameterLink<int>> swarmSizePL;
    static shared_ptr<ParameterLink<double>> switchCostPL;
    static shared_ptr<ParameterLink<int>> directionsPL;
    static shared_ptr<ParameterLink<int>> scoreRulePL;

    // a local variable used for faster access to the ParameterLink value
    int evaluationsPerGeneration;
    int swarmSize;
    double switchCost;
    int directions;
    int sensorCount;
    int forwardIndex;
    int scoreRule;

    static constexpr int worldEdgeSize = 8;

    std::array<int, worldEdgeSize* worldEdgeSize> foodGridInit{}; // 1/2 1, 1/2 2 has food on all locations
    std::array<int, worldEdgeSize* worldEdgeSize> foodGridMask{}; // mask to remove food on wall locations
    std::vector<std::array<int, worldEdgeSize* worldEdgeSize>> foodGridSafeList{}; // holds starting food placement
    std::array<int, worldEdgeSize* worldEdgeSize> foodGrid{};     // active grid durring eval

    std::array<int, (worldEdgeSize - 2)* (worldEdgeSize - 2)> validStartLocations{}; // locations where agents can start
    std::array<int, worldEdgeSize* worldEdgeSize> agentGridEmpty{};    // agent grid with just walls
    std::vector < std::array<int, worldEdgeSize* worldEdgeSize>> agentGridSafeList{};     // holds starting agent placement 
    std::array<int, worldEdgeSize* worldEdgeSize> agentGrid{};         // active grid durring eval 

    std::string groupName = "root::";
    std::string brainName = "root::";
    
    std::vector<std::shared_ptr<AbstractBrain >> swarmBrains;
    std::vector<int> swarmFood1Eats;
    std::vector<int> swarmFood2Eats;
    std::vector<int> swarmSwitches;

    std::vector < std::vector<int>> agentXSafeList; // agents location in X
    std::vector < std::vector<int>> agentYSafeList; // agetns location in Y
    std::vector < std::vector<int>> agentDSafeList; // direction agents are facing
    std::vector<int> agentX; // agents location in X used durring eval
    std::vector<int> agentY; // agents location in Y used durring eval
    std::vector<int> agentD; // direction agents are facing used during eval - D = 0 is up, rotation is clockwise

    std::vector<int> agentLastFood; // last food gathered by this agent, 0 = no food gathered yet

    std::vector<int> updateOrder; // used durring eval, the order the agents update in, shuffled every world update

    // D = 0 is up, rotation is clockwise
    std::vector<std::vector<int>> sensorXoffsets;
    std::vector<std::vector<int>> sensorYoffsets;


    inline int getIndex(int x, int y) {
        return y * worldEdgeSize + x;
    }

    inline std::pair<int, int> getLoc(int index) {
        int x = index % worldEdgeSize;
        int y = index / worldEdgeSize;
        return { x, y };
    }

    void applyMask(std::array<int, worldEdgeSize* worldEdgeSize>& target, const std::array<int, worldEdgeSize* worldEdgeSize>& mask) {
        for (int i = 0; i < worldEdgeSize * worldEdgeSize; i++) {
            if (mask[i] == 0) {
                target[i] = 0;
            }
        }
    }

    void printGrid(const std::array<int, worldEdgeSize* worldEdgeSize>& locations) {
        for (int y = 0; y < worldEdgeSize; y++) {
            for (int x = 0; x < worldEdgeSize; x++) {
                std::cout << locations[y * worldEdgeSize + x] << " ";
            }
            std::cout << std::endl; // New line at the end of each row
        }
    }

    std::string world2str() {
        
        std::string gridStr = "";
        for (int y = 0; y < worldEdgeSize; y++) {
            for (int x = 0; x < worldEdgeSize; x++) {
                if (agentGrid[y * worldEdgeSize + x] == 9) {
                    gridStr += "9,";
                }
                else {
                    gridStr += std::to_string(foodGrid[y * worldEdgeSize + x]) + ",";
                }
            }
            gridStr += "\n"; // New line at the end of each row
        }
        return gridStr;
    }

    void statusReport() {
        std::cout << "foodGridInit" << std::endl;
        printGrid(foodGridInit);

        std::cout << "foodGridMask" << std::endl;
        printGrid(foodGridMask);

        std::cout << "validStartLocations" << std::endl;
        for (auto v : validStartLocations) {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        std::cout << "agentGridEmpty" << std::endl;
        printGrid(agentGridEmpty);
        std::cout << std::endl << std::endl;

        std::cout << "foodGrid (active)" << std::endl;
        printGrid(foodGrid);
        std::cout << std::endl;
        std::cout << "agentGrid (active)" << std::endl;
        printGrid(agentGrid);
        std::cout << std::endl;

        std::cout << "validStartLocations (active)" << std::endl;
        for (int i = 0; i < swarmSize; i++) {
            std::cout << validStartLocations[i] << " : " << agentX[i] << "," << agentY[i] << "," << agentD[i] << "  ";
        }
        std::cout << std::endl << std::endl;
    }

    slimBerryWorld(shared_ptr<ParametersTable> PT);
    virtual ~slimBerryWorld() = default;

    virtual auto evaluate(map<string, shared_ptr<Group>>& /*groups*/, int /*analyze*/, int /*visualize*/, int /*debug*/) -> void override;

    virtual auto requiredGroups() -> unordered_map<string,unordered_set<string>> override;
};

