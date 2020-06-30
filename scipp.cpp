#include "scipp.h"

SCIPP::~SCIPP() {}

std::vector<std::pair<int, int>> SCIPP::splitBySoftConflicts(const Node & node, std::pair<int, int> interval,
                                                      const ConflictAvoidanceTable &CAT) {
    std::vector<std::pair<int, int>> res;
    CAT.getSoftConflictIntervals(res, node, interval.first, interval.second);
    return res;
}
