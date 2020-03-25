#include "prioritized_planning.h"

PrioritizedPlanning::PrioritizedPlanning()
{
    search = nullptr;
}

PrioritizedPlanning::PrioritizedPlanning (ISearch *Search)
{
    search = Search;
}

PrioritizedPlanning::~PrioritizedPlanning()
{
    if (search)
        delete search;
}

void PrioritizedPlanning::clear() {
    agentsPaths.clear();
}

MultiagentSearchResult PrioritizedPlanning::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::cout << agentSet.getAgentCount() << std::endl;

    if (config.withPerfectHeuristic) {
        search->getPerfectHeuristic(map, agentSet);
    }

    std::vector<int> order;
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        order.push_back(i);
    }

    if (config.ppOrder == 1 || config.ppOrder == 2) {
        Astar astar(false);
        std::vector<int> pathLength;
        for (int i = 0; i < agentSet.getAgentCount(); ++i) {
            Agent agent = agentSet.getAgent(i);
            SearchResult searchResult = astar.startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                            agent.getGoal_i(), agent.getGoal_j());
            pathLength.push_back(searchResult.pathlength);
        }

        std::sort(order.begin(), order.end(), [&pathLength, &config](int i, int j) {
            return (pathLength[i] < pathLength[j] && config.ppOrder == 1) ||
                   (pathLength[i] > pathLength[j] && config.ppOrder == 2);
        });
    }

    MultiagentSearchResult result(false);
    ConstraintsSet constraints;
    agentsPaths.resize(agentSet.getAgentCount());
    size_t maxDepth = 0;
    for (int i : order) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            result.pathfound = false;
            return result;
        }

        Agent agent = agentSet.getAgent(i);
        SearchResult searchResult = search->startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                        agent.getGoal_i(), agent.getGoal_j(), nullptr,
                                                        true, true, maxDepth + map.getEmptyCellCount(), {}, constraints);
        if (!searchResult.pathfound) {
            return result;
        }
        auto path = *searchResult.lppath;
        maxDepth = std::max(maxDepth, path.size());
        agentsPaths[i] = std::vector<Node>(path.begin(), path.end());
        for (auto it = path.begin(); it != path.end(); ++it) {
            if (std::next(it) == path.end()) {
                constraints.addGoalNodeConstraint(it->i, it->j, it->depth, i);
            } else {
                constraints.addNodeConstraint(it->i, it->j, it->depth, i);
            }
            if (it != path.begin()) {
                constraints.addEdgeConstraint(std::prev(it)->i, std::prev(it)->j, it->depth, i, it->i, it->j);
            }
        }
    }

    result.pathfound = true;
    result.agentsPaths = &agentsPaths;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    result.time = static_cast<double>(elapsedMilliseconds) / 1000;
    return result;
}
