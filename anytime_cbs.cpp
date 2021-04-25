#include "anytime_cbs.h"

AnytimeCBS::AnytimeCBS() {}

AnytimeCBS::AnytimeCBS(ConflictBasedSearch<Astar<>>* Search)
{
    search = Search;
}

AnytimeCBS::~AnytimeCBS()
{
    if (search)
        delete search;
}

void AnytimeCBS::clear()
{
    search->clear();
}

MultiagentSearchResult AnytimeCBS::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    MultiagentSearchResult result(false);
    Config curConfig = config;
    curConfig.LogParams = nullptr;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<std::vector<Node>> agentsPaths;

    search->search->perfectHeuristic.clear();
    if (config.withPerfectHeuristic) {
        search->search->getPerfectHeuristic(map, agentSet);
    }

    while (curConfig.focalW > 1.0) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            break;
        }

        MultiagentSearchResult newResult = search->startSearch(map, curConfig, agentSet);
        if (!newResult.pathfound) {
            break;
        }

        result += newResult;
        result.time.back() = (double)std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() / 1000;

        double minF = std::min(*search->sumLb.begin(), (double)result.cost.back());
        result.focalW.back() = result.cost.back() / minF;

        agentsPaths = *(result.agentsPaths);
        search->agentsPaths.clear();

        if (search->open.empty() && search->focal.empty()) {
            result.focalW.back() = 1.0;
            break;
        }

        curConfig.focalW = result.cost.back() / minF - 0.0001;
        if (curConfig.focalW < 1.0) {
            curConfig.focalW = 1.0;
        }

        //std::cout << result.cost[0] << " " << curConfig.focalW << std::endl;

        auto it = search->focal.begin();
        while (it != search->focal.end()) {
            if (it->cost > minF * curConfig.focalW) {
                search->open.insert(*it);
                it = search->focal.erase(it);
            } else {
                ++it;
            }
        }
    }
    search->agentsPaths = agentsPaths;
    result.agentsPaths = &search->agentsPaths;
    return result;
}
