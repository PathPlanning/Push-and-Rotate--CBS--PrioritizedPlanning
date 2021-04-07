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
    for (int i = 0; i <= 10 && curConfig.focalW > 1.0; ++i) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            break;
        }

        MultiagentSearchResult newResult = search->startSearch(map, curConfig, agentSet);
        if (!newResult.pathfound) {
            break;
        }
        result = newResult;
        agentsPaths = *(result.agentsPaths);
        search->agentsPaths.clear();

        if (search->open.empty() && search->focal.empty()) {
            break;
        }

        double minF = *search->sumLb.begin();
        curConfig.focalW = result.cost / *search->sumLb.begin() - 0.01;
        if (curConfig.focalW < 1.0 || i == 10) {
            curConfig.focalW = 1.0;
        }

        std::cout << result.cost << " " << curConfig.focalW << std::endl;

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
