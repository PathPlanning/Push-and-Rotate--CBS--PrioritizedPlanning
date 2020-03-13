#ifndef CONFLICT_BASED_SEARCH_H
#define CONFLICT_BASED_SEARCH_H

#include "isearch.h"
#include "astar.h"
#include "cbs_node.h"
#include "multiagent_search_result.h"
#include "config.h"
#include "conflict.h"
#include "conflict_avoidance_table.h"

class ConflictBasedSearch
{
    public:
        ConflictBasedSearch();
        ConflictBasedSearch(ISearch* Search);
        ~ConflictBasedSearch(void);
        MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &agentSet);
        void clear();
    private:
        CBSNode createNode(const Map &map, const AgentSet &agentSet, const Conflict &conflict,
                           const std::vector<int> &costs, ConstraintsSet &constraints, int agentId,
                           const Node &pos1, const Node &pos2,
                           std::list<Node>::iterator pathStart, std::list<Node>::iterator pathEnd,
                           ConflictAvoidanceTable& CAT, bool withCAT,
                           CBSNode *parentPtr);


        ISearch*                        search;
        std::vector<std::vector<Node>>  agentsPaths;
};

#endif // CONFLICT_BASED_SEARCH_H
