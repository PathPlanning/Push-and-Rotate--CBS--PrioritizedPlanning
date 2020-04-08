#ifndef CONFLICT_BASED_SEARCH_H
#define CONFLICT_BASED_SEARCH_H

#include "isearch.h"
#include "astar.h"
#include "cbs_node.h"
#include "multiagent_search_result.h"
#include "config.h"
#include "conflict.h"
#include "conflict_avoidance_table.h"
#include "mdd.h"

class ConflictBasedSearch
{
    public:
        ConflictBasedSearch();
        ConflictBasedSearch(ISearch* Search);
        ~ConflictBasedSearch(void);
        MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &agentSet);
        void clear();
        template<typename Iter> static Conflict findConflict(const std::vector<Iter> &starts, const std::vector<Iter> &ends,
                                                             bool withCardinalConflicts = false,
                                                             const std::vector<MDD> &mdds = std::vector<MDD>());
    private:
        CBSNode createNode(const Map &map, const AgentSet &agentSet, const Conflict &conflict,
                           const std::vector<int> &costs, ConstraintsSet &constraints, int agentId,
                           const Node &pos1, const Node &pos2,
                           std::list<Node>::iterator pathStart, std::list<Node>::iterator pathEnd,
                           ConflictAvoidanceTable& CAT, bool withCAT, bool withCardinalConflicts,
                           CBSNode *parentPtr);


        ISearch*                        search;
        std::vector<std::vector<Node>>  agentsPaths;
};

template<typename Iter>
Conflict ConflictBasedSearch::findConflict(const std::vector<Iter> &starts, const std::vector<Iter> &ends,
                                           bool withCardinalConflicts, const std::vector<MDD> &mdds) {
    std::vector<Conflict> semiCardinal, nonCardinal;
    std::vector<Iter> iters = starts;
    for (int time = 0;; ++time) {
        std::unordered_multimap<Node, int> positions;
        std::unordered_multimap<std::pair<Node, Node>, int> edges;
        std::vector<int> ids;

        int finished = 0;
        for (int i = 0; i < iters.size(); ++i) {
            auto posRange = positions.equal_range(*iters[i]);
            for (auto posIt = posRange.first; posIt != posRange.second; ++posIt) {
                Conflict conflict(i, posIt->second, *iters[i], *iters[i], time, false);
                if (!withCardinalConflicts) {
                    return conflict;
                }
                int id1 = i, id2 = posIt->second;
                int size1 = mdds[id1].getLayerSize(time);
                int size2 = mdds[id2].getLayerSize(time);

                if (size1 == 1 && size2 == 1) {
                    return conflict;
                } else if (size1 == 1 || size2 == 1) {
                    semiCardinal.push_back(conflict);
                } else {
                    nonCardinal.push_back(conflict);
                }
            }
            positions.insert(std::make_pair(*iters[i], i));

            if (std::next(iters[i]) != ends[i]) {
                auto edgeRange = edges.equal_range(std::make_pair(*std::next(iters[i]), *iters[i]));
                for (auto edgeIt = edgeRange.first; edgeIt != edgeRange.second; ++edgeIt) {
                    Conflict conflict(i, edgeIt->second, *iters[i], *std::next(iters[i]), time + 1, true);
                    if (!withCardinalConflicts) {
                        return conflict;
                    }
                    int id1 = i, id2 = edgeIt->second;
                    int inSize1 = mdds[id1].getLayerSize(time), outSize1 = mdds[id1].getLayerSize(time + 1);
                    int inSize2 = mdds[id2].getLayerSize(time), outSize2 = mdds[id2].getLayerSize(time + 1);

                    if ((inSize1 == 1 && outSize1 == 1) && (inSize1 == 2 && outSize2 == 1)) {
                        return conflict;
                    } else if ((inSize1 == 1 && outSize1 == 1) || (inSize2 == 1 && outSize2 == 1)) {
                        semiCardinal.push_back(conflict);
                    } else {
                        nonCardinal.push_back(conflict);
                    }
                }
                edges.insert(std::make_pair(std::make_pair(*iters[i], *std::next(iters[i])), i));
            }

            if (std::next(iters[i]) != ends[i]) {
                ++iters[i];
            } else {
                ++finished;
            }
        }

        if (finished == iters.size()) {
            break;
        }
    }
    if (!semiCardinal.empty()) {
        return semiCardinal[0];
    } else if (!nonCardinal.empty()) {
        return nonCardinal[0];
    }
    return Conflict(false);
}

#endif // CONFLICT_BASED_SEARCH_H
