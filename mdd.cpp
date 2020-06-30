#include "mdd.h"

MDD::MDD() {};

MDD::MDD(const Map& map, const AgentSet& agentSet, ISearch* search, int agentId, int cost, const ConstraintsSet& constraints) {
    bool oldWithIntervals = search->getWithIntervals();
    search->setWithIntervals(false);
    Agent agent = agentSet.getAgent(agentId);
    Node start = agent.getStartPosition(), goal = agent.getGoalPosition();
    std::vector<std::unordered_set<Node>> layers;
    layers.push_back({start});

    int t = 0;
    int q = 0;

    for (int i = 0; i < cost - 1; ++i) {
        layers.push_back({});
        for (auto node : layers[i]) {

            std::chrono::steady_clock::time_point a = std::chrono::steady_clock::now();

            std::list<Node> successors = search->findSuccessors(node, map, goal.i, goal.j, agentId, {}, constraints);

            std::chrono::steady_clock::time_point b = std::chrono::steady_clock::now();
            q += std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
            ++t;

            for (auto neigh : successors) {
                if (neigh.H <= cost - i - 1) {
                    layers.back().insert(neigh);
                }
            }
        }
    }

    //std::cout << q << std::endl;
    //std::cout << t << std::endl;

    layerSizes.resize(cost + 1, 0);
    layerSizes[cost] = 1;
    std::unordered_set<Node> lastLayer = {goal};
    for (int i = cost - 1; i >= 0; --i) {
        std::unordered_set<Node> newLastLayer;
        for (auto node : layers[i]) {
            std::list<Node> successors = search->findSuccessors(node, map, goal.i, goal.j, agentId, {}, constraints);
            for (auto neigh : successors) {
                if (lastLayer.find(neigh) != lastLayer.end()) {
                    newLastLayer.insert(node);
                    break;
                }
            }
        }
        layerSizes[i] = newLastLayer.size();
        lastLayer = newLastLayer;
    }
    search->setWithIntervals(oldWithIntervals);
}

int MDD::getLayerSize(int cost) const {
    if (cost > layerSizes.size()) {
         return 1;
    }
    return layerSizes[cost];
}
