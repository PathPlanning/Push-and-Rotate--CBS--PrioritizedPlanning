#ifndef MULTIAGENT_SEARCH_RESULT_H
#define MULTIAGENT_SEARCH_RESULT_H

#include <vector>
#include "node.h"
#include "agent_move.h"

struct MultiagentSearchResult
{
    bool                            pathfound;
    std::vector<AgentMove>*         agentsMoves;
    std::vector<std::vector<Node>>* agentsPaths;
    std::vector<double>             time;
    std::vector<double>             AvgLLExpansions;
    std::vector<double>             AvgLLNodes;
    std::vector<int>                cost;
    std::vector<double>             HLExpansions;
    std::vector<double>             HLExpansionsStart;
    std::vector<double>             HLNodes;
    std::vector<double>             HLNodesStart;
    double                          finalHLExpansions = 0.0;
    double                          finalHLExpansionsStart = 0.0;
    double                          finalHLNodes = 0.0;
    double                          finalHLNodesStart = 0.0;
    std::vector<double>             focalW;
    std::vector<double>             flowtime;
    std::vector<double>             makespan;
    std::vector<double>             totalNodes;
    int                             finalTotalNodes;

    MultiagentSearchResult(bool Pathfound = false) {
        pathfound = Pathfound;
    }

    MultiagentSearchResult& operator+=(const MultiagentSearchResult& other) {
        pathfound = other.pathfound;
        agentsPaths = other.agentsPaths;
        agentsMoves = other.agentsMoves;

        time.insert(time.end(), other.time.begin(), other.time.end());
        AvgLLExpansions.insert(AvgLLExpansions.end(), other.AvgLLExpansions.begin(), other.AvgLLExpansions.end());
        AvgLLNodes.insert(AvgLLNodes.end(), other.AvgLLNodes.begin(), other.AvgLLNodes.end());
        cost.insert(cost.end(), other.cost.begin(), other.cost.end());
        HLExpansionsStart.insert(HLExpansionsStart.end(), other.HLExpansionsStart.begin(), other.HLExpansionsStart.end());
        HLNodesStart.insert(HLNodesStart.end(), other.HLNodesStart.begin(), other.HLNodesStart.end());
        HLExpansions.insert(HLExpansions.end(), other.HLExpansions.begin(), other.HLExpansions.end());
        HLNodes.insert(HLNodes.end(), other.HLNodes.begin(), other.HLNodes.end());
        focalW.insert(focalW.end(), other.focalW.begin(), other.focalW.end());
        flowtime.insert(flowtime.end(), other.flowtime.begin(), other.flowtime.end());
        makespan.insert(makespan.end(), other.makespan.begin(), other.makespan.end());
        totalNodes.insert(totalNodes.end(), other.totalNodes.begin(), other.totalNodes.end());
        return *this;
    }

    std::pair<int, int> getCosts() const {
        size_t makespan = 0, timeflow = 0;
        for (int i = 0; i < agentsPaths->size(); ++i) {
            makespan = std::max(makespan, (*agentsPaths)[i].size() - 1);
            int lastMove;
            for (lastMove = (*agentsPaths)[i].size() - 1;
                 lastMove > 1 && (*agentsPaths)[i][lastMove] == (*agentsPaths)[i][lastMove - 1];
                 --lastMove);
            timeflow += lastMove;
        }
        return std::make_pair(makespan, timeflow);
    }
};

#endif // MULTIAGENT_SEARCH_RESULT_H
