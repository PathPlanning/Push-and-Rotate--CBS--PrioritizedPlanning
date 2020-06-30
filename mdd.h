#ifndef MDD_H
#define MDD_H

#include <vector>
#include <unordered_set>
#include "constraints_set.h"
#include "map.h"
#include "agent_set.h"
#include "isearch.h"
#include "astar.h"

class MDD
{
public:
    MDD();
    MDD(const Map& map, const AgentSet& agentSet, ISearch* search, int agentId, int cost,
        const ConstraintsSet& constraints = ConstraintsSet());

    int getLayerSize(int cost) const;

//private:
    std::vector<int> layerSizes;
};

#endif // MDD_H
