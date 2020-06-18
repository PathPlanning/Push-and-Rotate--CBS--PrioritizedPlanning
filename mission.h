#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "agent_set.h"
#include "push_and_rotate.h"
#include "conflict_based_search.h"
#include "prioritized_planning.h"
#include "config.h"
#include "isearch.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "astar.h"
#include "dijkstra.h"
#include "xmllogger.h"
#include <algorithm>

//That's the wrap up class that first creates all the needed objects (Map, Search etc.)
//and then runs the search and then cleans everything up.

//Hint: Create Mission object in the main() function and then use it 1) to retreive all the data from input XML
//2) run the search 3) flush the results to output XML

class Mission
{
    public:
        Mission();
        Mission (const char* MapFile);
        ~Mission();

        bool getMap();
        bool getAgents(const char* agentsFile);
        bool getConfig();
        bool createLog();
        void createSearch();
        void createAlgorithm();
        void createEnvironmentOptions();
        void startSearch(const std::string &agentsFile);
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();
        void saveAgentsPathsToLog(const std::string &agentsFile, int makespan, int flowtime);
        bool checkCorrectness();
        void saveAggregatedResultsToLog();
        std::pair<int, int> getCosts();
        int getTasksCount();
        std::string getAgentsFile();
        bool getSingleExecution();

    private:
        Map                                   map;
        AgentSet                              agentSet;
        Config                                config;
        EnvironmentOptions                    options;
        ISearch*                              search;
        ConflictBasedSearch*                  conflictBasedSearch;
        PushAndRotate*                        pushAndRotate;
        PrioritizedPlanning*                  prioritizedPlanning;
        ILogger*                              logger;
        const char*                           mapFile;
        MultiagentSearchResult                sr;
        std::vector<std::vector<Node>>        agentsPaths;
        std::map<int, std::vector<int>>       makespans;
        std::map<int, std::vector<int>>       timeflows;
        std::map<int, std::vector<double>>    times;
};

#endif

