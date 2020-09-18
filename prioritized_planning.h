#ifndef PRIORITIZEDPLANNING_H
#define PRIORITIZEDPLANNING_H

#include "config.h"
#include "agent_set.h"
#include "map.h"
#include "isearch.h"
#include "astar.h"
#include "sipp.h"
#include "scipp.h"
#include "zero_scipp.h"
#include "multiagent_search_result.h"
#include "multiagent_search_interface.h"
#include <vector>
#include <numeric>

template <typename SearchType = Astar<>>
class PrioritizedPlanning : public MultiagentSearchInterface
{
public:
    PrioritizedPlanning();
    PrioritizedPlanning(SearchType* Search);
    ~PrioritizedPlanning(void);
    MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &AgentSet) override;

private:
    SearchType*                      search;
};

#endif // PRIORITIZEDPLANNING_H
