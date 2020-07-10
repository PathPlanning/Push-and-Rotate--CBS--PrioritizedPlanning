#include "conflict_avoidance_table.h"

void ConflictAvoidanceTable::addAgentPosition(const Node &node) {
    auto tuple = std::make_tuple(node.i, node.j, node.g);
    if (agentsCount.find(tuple) == agentsCount.end()) {
        agentsCount[tuple] = 1;
    } else {
        ++agentsCount[tuple];
    }
}

void ConflictAvoidanceTable::addAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end) {
    for (auto it = start; it != end; ++it) {
        addAgentPosition(*it);
    }
}

void ConflictAvoidanceTable::removeAgentPosition(const Node &node) {
    auto tuple = std::make_tuple(node.i, node.j, node.g);
    if (agentsCount[tuple] == 1) {
        agentsCount.erase(tuple);
    } else {
        --agentsCount[tuple];
    }
}

void ConflictAvoidanceTable::removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end) {
    for (auto it = start; it != end; ++it) {
        removeAgentPosition(*it);
    }
}

int ConflictAvoidanceTable::getAgentsCount(const Node &node) const {
    auto tuple = std::make_tuple(node.i, node.j, node.g);
    if (agentsCount.find(tuple) == agentsCount.end()) {
        return 0;
    }
    return agentsCount.at(tuple);
}

int ConflictAvoidanceTable::getFirstSoftConflict(const Node & node, int startTime, int endTime) const {
    auto it = agentsCount.lower_bound(std::make_tuple(node.i, node.j, startTime));
    if (std::get<0>(it->first) == node.i && std::get<1>(it->first) == node.j
            && std::get<2>(it->first) <= endTime) {
        return std::get<2>(it->first);
    }
    return -1;
}

void ConflictAvoidanceTable::getSoftConflictIntervals(std::vector<std::pair<int, int>> &res, const Node & node,
                                                      int startTime, int endTime, bool binary) const {
    int count = 0, prevTime = startTime - 1, beg = -1;
    auto it = agentsCount.lower_bound(std::make_tuple(node.i, node.j, startTime));
    auto end = agentsCount.upper_bound(std::make_tuple(node.i, node.j, endTime));
    for (it; it != end; ++it) {
        int time = std::get<2>(it->first);
        if (time > prevTime + 1 || count == 0 || (!binary && it->second != count)) {
            if (beg != -1) {
                res.push_back(std::make_pair(beg, count));
            }
            if (time > prevTime + 1) {
                res.push_back(std::make_pair(prevTime + 1, 0));
            }
            beg = time;
            count = it->second;
        }
        prevTime = time;
    }
    if (beg != -1) {
        res.push_back(std::make_pair(beg, count));
    }
    if (prevTime < endTime) {
        res.push_back(std::make_pair(prevTime + 1, 0));
    }
}
