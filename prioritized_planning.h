#ifndef PRIORITIZEDPLANNING_H
#define PRIORITIZEDPLANNING_H

#include "config.h"
#include "agent_set.h"
#include "map.h"
#include "isearch.h"
#include "astar.h"
#include "multiagent_search_result.h"
#include <vector>

class PrioritizedPlanning
{
public:
    PrioritizedPlanning();
    PrioritizedPlanning(ISearch* Search);
    ~PrioritizedPlanning(void);
    MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &AgentSet);
    void clear();

private:
    ISearch*                        search;
    std::vector<std::vector<Node>>  agentsPaths;
};

#endif // PRIORITIZEDPLANNING_H
