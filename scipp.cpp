#include "scipp.h"

template<typename NodeType>
SCIPP<NodeType>::~SCIPP() {}

template<typename NodeType>
void SCIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                                  const NodeType & node, std::pair<int, int> interval,
                                                  const ConflictAvoidanceTable &CAT) {
    CAT.getSoftConflictIntervals(softConflictIntervals, node, interval.first, interval.second, false);
}

template class SCIPP<SCIPPNode>;
