#include "conflict_avoidance_table.h"

void ConflictAvoidanceTable::addAgentPosition(int i, int j, int time, const Map& map) {
    int conv = SearchQueue::convolution(i, j, map, time, true);
    if (agentsCount.find(conv) == agentsCount.end()) {
        agentsCount[conv] = 1;
    } else {
        ++agentsCount[conv];
    }
}

void ConflictAvoidanceTable::addAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map) {
    for (auto it = start; it != end; ++it) {
        addAgentPosition(it->i, it->j, it->depth, map);
    }
}

void ConflictAvoidanceTable::removeAgentPosition(int i, int j, int time, const Map& map) {
    int conv = SearchQueue::convolution(i, j, map, time, true);
    --agentsCount[conv];
}

void ConflictAvoidanceTable::removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map) {
    for (auto it = start; it != end; ++it) {
        removeAgentPosition(it->i, it->j, it->depth, map);
    }
}

int ConflictAvoidanceTable::getAgentsCount(int i, int j, int time, const Map& map) const {
    int conv = SearchQueue::convolution(i, j, map, time, true);
    if (agentsCount.find(conv) == agentsCount.end()) {
        return 0;
    }
    return agentsCount.at(conv);
}
