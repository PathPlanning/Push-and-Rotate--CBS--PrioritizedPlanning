#ifndef MULTIAGENT_SEARCH_INTEFACE_H
#define MULTIAGENT_SEARCH_INTEFACE_H

#include "config.h"
#include "agent_set.h"
#include "map.h"
#include "multiagent_search_result.h"
#include <vector>

class MultiagentSearchInterface
{
public:
    virtual ~MultiagentSearchInterface(void) {}
    virtual MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &AgentSet) = 0;
    virtual void clear() {
        agentsPaths.clear();
    }

protected:
    std::vector<std::vector<Node>>  agentsPaths;
};


#endif // MULTIAGENT_SEARCH_INTEFACE_H
