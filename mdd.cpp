#include "mdd.h"

MDD::MDD() {};

MDD::MDD(const Map& map, const AgentSet& agentSet, int agentId, int cost, const ConstraintsSet& constraints) {
    Astar search = Astar(true);
    Agent agent = agentSet.getAgent(agentId);
    Node start = agent.getStartPosition(), goal = agent.getGoalPosition();
    std::vector<std::unordered_set<Node>> layers;
    layers.push_back({start});
    for (int i = 0; i < cost - 1; ++i) {
        layers.push_back({});
        for (auto node : layers[i]) {
            std::list<Node> successors = search.findSuccessors(node, map, goal.i, goal.j, agentId, {}, constraints);
            for (auto neigh : successors) {
                if (neigh.H <= cost - i - 1) {
                    layers.back().insert(neigh);
                }
            }
        }
    }

    layerSizes.resize(cost + 1, 0);
    layerSizes[cost] = 1;
    std::unordered_set<Node> lastLayer = {goal};
    for (int i = cost - 1; i >= 0; --i) {
        std::unordered_set<Node> newLastLayer;
        for (auto node : layers[i]) {
            std::list<Node> successors = search.findSuccessors(node, map, goal.i, goal.j, agentId, {}, constraints);
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
}

int MDD::getLayerSize(int cost) const {
    if (cost > layerSizes.size()) {
         return 1;
    }
    return layerSizes[cost];
}
