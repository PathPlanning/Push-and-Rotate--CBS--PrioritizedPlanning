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

    MultiagentSearchResult(bool Pathfound = false) {
        pathfound = Pathfound;
    }
};

#endif // MULTIAGENT_SEARCH_RESULT_H
