#include "mission.h"

int main(int argc, char* argv[])
{
    if(argc < 1) {
        std::cout << "Error! Pathfinding task file is not specified!" << std::endl;
        return 0;
    }

    Mission mission(argv[1]);

    std::cout << argv[1] << std::endl;
    std::cout << "Parsing the map from XML:" << std::endl;

    if(!mission.getMap()) {
        std::cout << "Incorrect map! Program halted!" << std::endl;
    }
    else {
        std::cout << "Map OK!" << std::endl << "Parsing configurations (algorithm, log) from XML:" <<std::endl;
        if(!mission.getConfig())
            std::cout << "Incorrect configurations! Program halted!" <<std::endl;
        else {
            std::cout << "Configurations OK!" << std::endl << "Creating log channel:" <<std::endl;

            if(!mission.createLog())
                std::cout << "Log chanel has not been created! Program halted!" << std::endl;
            else {
                std::cout << "Log OK!" << std::endl << "Start searching" << std::endl;
                // mission.createSearch();
                mission.createAlgorithm();
                int tasksCount = mission.getSingleExecution() ? 1 : mission.getTasksCount();
                for (int i = 0; i < tasksCount; ++i) {
                    std::string agentsFile = mission.getAgentsFile() + "-" + std::to_string(i + mission.getFirstTask()) + ".xml";
                    if (!mission.getAgents(agentsFile.c_str()))
                        std::cout << "Agent set has not been created! Program halted!" << std::endl;
                    else if (mission.checkAgentsCorrectness(agentsFile)) {
                        std::cout << "Starting search for agents file " << agentsFile << std::endl;
                        mission.startSearch(agentsFile);
                    }
                }
                if (!mission.getSingleExecution()) {
                    if (mission.getSaveAggregatedResults()) {
                        mission.saveAggregatedResultsToLog();
                    } else {
                        mission.saveSeparateResultsToLog();
                    }
                }
                std::cout << "All searches are finished!" << std::endl;
            }
        }
    }
    return 0;
}
