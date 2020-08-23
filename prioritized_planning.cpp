#include "prioritized_planning.h"

template <typename SearchType>
PrioritizedPlanning<SearchType>::PrioritizedPlanning()
{
    search = nullptr;
}

template <typename SearchType>
PrioritizedPlanning<SearchType>::PrioritizedPlanning (SearchType *Search)
{
    search = Search;
}

template <typename SearchType>
PrioritizedPlanning<SearchType>::~PrioritizedPlanning()
{
    if (search)
        delete search;
}

template <typename SearchType>
MultiagentSearchResult PrioritizedPlanning<SearchType>::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // std::cout << agentSet.getAgentCount() << std::endl;

    if (config.withPerfectHeuristic) {
        search->getPerfectHeuristic(map, agentSet);
    }

    std::vector<int> order;
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        order.push_back(i);
    }

    if (config.ppOrder == 1 || config.ppOrder == 2) {
        std::vector<int> dist;
        for (int i = 0; i < agentSet.getAgentCount(); ++i) {
            Agent agent = agentSet.getAgent(i);
            dist.push_back(search->computeHFromCellToCell(
                               agent.getStart_i(), agent.getStart_j(), agent.getGoal_i(), agent.getGoal_j()));
        }

        std::sort(order.begin(), order.end(), [&dist, &config](int i, int j) {
            return (dist[i] < dist[j] && config.ppOrder == 1) ||
                   (dist[i] > dist[j] && config.ppOrder == 2);
        });
    }

    ConflictAvoidanceTable CAT;
    std::vector<std::list<Node>> individualPaths;
    if (config.lowLevel == CN_SP_ST_SCIPP || config.lowLevel == CN_SP_ST_ZSCIPP) {
        Astar<> astar(false);
        for (int i = 0; i < agentSet.getAgentCount(); ++i) {
            Agent agent = agentSet.getAgent(i);
            SearchResult sr = astar.startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                        agent.getGoal_i(), agent.getGoal_j());
            individualPaths.push_back(*sr.lppath);
            CAT.addAgentPath(individualPaths.back().begin(), individualPaths.back().end());
        }
    }

    MultiagentSearchResult result(false);
    ConstraintsSet constraints;
    agentsPaths.resize(agentSet.getAgentCount());
    size_t maxDepth = 0;
    std::vector<double> LLExpansions;
    std::vector<double> LLNodes;
    for (int i : order) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            result.pathfound = false;
            return result;
        }

        if (config.lowLevel == CN_SP_ST_SCIPP || config.lowLevel == CN_SP_ST_ZSCIPP) {
            CAT.removeAgentPath(individualPaths[i].begin(), individualPaths[i].end());
        }

        Agent agent = agentSet.getAgent(i);
        SearchResult searchResult = search->startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                        agent.getGoal_i(), agent.getGoal_j(), nullptr,
                                                        true, true, 0, -1, maxDepth + map.getEmptyCellCount(),
                                                        {}, constraints, false, CAT);

        if (!searchResult.pathfound) {
            return result;
        }

        LLExpansions.push_back(searchResult.nodesexpanded);
        LLNodes.push_back(searchResult.nodescreated);

        auto path = *searchResult.lppath;
        maxDepth = std::max(maxDepth, path.size());
        agentsPaths[i] = std::vector<Node>(path.begin(), path.end());

        for (auto it = path.begin(); it != path.end(); ++it) {
            if (std::next(it) == path.end()) {
                constraints.addGoalNodeConstraint(it->i, it->j, it->g, i);
            } else {
                constraints.addNodeConstraint(it->i, it->j, it->g, i);
            }
            if (it != path.begin()) {
                constraints.addEdgeConstraint(std::prev(it)->i, std::prev(it)->j, it->g, i, it->i, it->j);
            }
        }
    }

    result.pathfound = true;
    result.agentsPaths = &agentsPaths;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    result.time = static_cast<double>(elapsedMilliseconds) / 1000;
    result.AvgLLExpansions = (double)std::accumulate(LLExpansions.begin(), LLExpansions.end(), 0) / LLExpansions.size();
    result.AvgLLNodes = (double)std::accumulate(LLNodes.begin(), LLNodes.end(), 0) / LLNodes.size();
    return result;
}

template class PrioritizedPlanning<Astar<>>;
template class PrioritizedPlanning<SIPP<>>;
template class PrioritizedPlanning<SCIPP<>>;
template class PrioritizedPlanning<ZeroSCIPP<>>;
