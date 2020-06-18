#include "conflict_avoidance_table.h"

void ConflictAvoidanceTable::addAgentPosition(const Node &node, const Map& map) {
    int conv = SearchQueue::convolution(node, map, true);
    if (agentsCount.find(conv) == agentsCount.end()) {
        agentsCount[conv] = 1;
    } else {
        ++agentsCount[conv];
    }
}

void ConflictAvoidanceTable::addAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map) {
    for (auto it = start; it != end; ++it) {
        addAgentPosition(*it, map);
    }
}

void ConflictAvoidanceTable::removeAgentPosition(const Node &node, const Map& map) {
    int conv = SearchQueue::convolution(node, map, true);
    --agentsCount[conv];
}

void ConflictAvoidanceTable::removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map) {
    for (auto it = start; it != end; ++it) {
        removeAgentPosition(*it, map);
    }
}

int ConflictAvoidanceTable::getAgentsCount(const Node &node, const Map& map) const {
    int conv = SearchQueue::convolution(node, map, true);
    if (agentsCount.find(conv) == agentsCount.end()) {
        return 0;
    }
    return agentsCount.at(conv);
}
