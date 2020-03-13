#include "conflict_avoidance_table.h"

int convolution(int i, int j, const Map &map, int time, bool withTime) {
    int res = withTime ? map.getMapWidth() * map.getMapHeight() * time : 0;
    return res + i * map.getMapWidth() + j;
}


void ConflictAvoidanceTable::addAgentPosition(int i, int j, int time, const Map& map) {
    int conv = convolution(i, j, map, time, true);
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
    int conv = convolution(i, j, map, time, true);
    --agentsCount[conv];
}

void ConflictAvoidanceTable::removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map) {
    for (auto it = start; it != end; ++it) {
        removeAgentPosition(it->i, it->j, it->depth, map);
    }
}

int ConflictAvoidanceTable::getAgentsCount(int i, int j, int time, const Map& map) const {
    int conv = convolution(i, j, map, time, true);
    if (agentsCount.find(conv) == agentsCount.end()) {
        return 0;
    }
    return agentsCount.at(conv);
}
