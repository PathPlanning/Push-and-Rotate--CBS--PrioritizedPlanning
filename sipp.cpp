#include "sipp.h"

int SIPP::getEndTime(int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) {
    int endTime = constraints.getFirstConstraintTime(start_i, start_j, startTime, agentId);
    if (endTime < CN_INFINITY) {
        --endTime;
    }
    return endTime;
}

void SIPP::createSuccessorsFromNode(const Node &cur, Node &neigh, std::list<Node> &successors,
                                    int agentId, const ConstraintsSet &constraints,
                                    const ConflictAvoidanceTable &CAT) {
    std::vector<std::pair<int, int>> safeIntervals = constraints.getSafeIntervals(
                neigh.i, neigh.j, agentId, cur.time + 1,
                cur.endTime + (cur.endTime != CN_INFINITY));
    for (auto interval : safeIntervals) {
        std::vector<std::pair<int, int>> softConflictIntervals = splitBySoftConflicts(neigh, interval, CAT);
        for (int i = 0; i < softConflictIntervals.size(); ++i) {
            neigh.startTime = softConflictIntervals[i].first;
            neigh.endTime = (i == softConflictIntervals.size() - 1) ? interval.second : softConflictIntervals[i + 1].first - 1;
            neigh.hc = cur.hc + softConflictIntervals[i].second;

            for (neigh.time = std::max(neigh.startTime, cur.time + 1); neigh.time <= neigh.endTime; ++neigh.time) {
                if (!constraints.hasEdgeConstraint(neigh.i, neigh.j, neigh.time, agentId, cur.i, cur.j)) {
                    break;
                }
            }
            if (neigh.time <= cur.endTime + 1 && neigh.time <= neigh.endTime) {
                neigh.g = neigh.time;
                neigh.F = neigh.g + neigh.H;
                successors.push_back(neigh);
            }
        }
    }
}

std::vector<std::pair<int, int>> SIPP::splitBySoftConflicts(const Node & node, std::pair<int, int> interval,
                                                            const ConflictAvoidanceTable &CAT) {
    return {std::make_pair(interval.first, 0)};
}

bool SIPP::checkGoal(const Node &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.time <= goalTime;
}


