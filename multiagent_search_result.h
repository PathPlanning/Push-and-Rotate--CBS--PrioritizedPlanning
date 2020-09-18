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
    double                          time;
    double                          AvgLLExpansions = 0;
    double                          AvgLLNodes = 0;
    int                             HLExpansions = 0;
    int                             HLNodes = 0;

    MultiagentSearchResult(bool Pathfound = false) {
        pathfound = Pathfound;
    }
};

#endif // MULTIAGENT_SEARCH_RESULT_H
