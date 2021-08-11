#include "scipp.h"

template<typename NodeType>
SCIPP<NodeType>::~SCIPP() {}

template<typename NodeType>
void SCIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                                  const NodeType & node, const NodeType & prevNode, std::pair<int, int> interval,
                                                  const ConflictAvoidanceTable &CAT) {
    CAT.getSoftConflictIntervals(softConflictIntervals, node, prevNode, interval.first, interval.second, false);
}

template<typename NodeType>
void SCIPP<NodeType>::addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    this->updateEndTimeBySoftConflicts(node, CAT);
    this->open.insert(map, node, ISearch<NodeType>::withTime);
}

template<typename NodeType>
void SCIPP<NodeType>::setHC(NodeType &neigh, const NodeType &cur,
                            const ConflictAvoidanceTable &CAT, bool isGoal) {
    neigh.hc = cur.hc + cur.conflictsCount * (neigh.g - cur.g - 1) + neigh.conflictsCount;
    if (isGoal) {
        this->addFutureConflicts(neigh, CAT);
    }
}

template<typename NodeType>
void SCIPP<NodeType>::createNeighborsByEdges(const NodeType &cur, NodeType &neigh,
    std::list<NodeType> &successors, int agentId,
    const ConstraintsSet &constraints, const ConflictAvoidanceTable &CAT)
{
    int minConflicts = CN_INFINITY;
    for (neigh.g = std::max(neigh.startTime, cur.g + 1); neigh.g <= neigh.endTime; ++neigh.g) {
        if (constraints.hasEdgeConstraint(neigh.i, neigh.j, neigh.g, agentId, cur.i, cur.j)) {
            continue;
        }
        int conflictsCount = CAT.getEdgeAgentsCount(neigh, cur);
        if (conflictsCount < minConflicts) {
            if (neigh.g <= cur.endTime + 1) {
                neigh.conflictsCount += conflictsCount;
                neigh.hc = cur.hc + cur.conflictsCount * (neigh.g - cur.g - 1) + neigh.conflictsCount;
                neigh.F = neigh.g + neigh.H;
                successors.push_back(neigh);
            }
        }
        if (conflictsCount == 0) {
            break;
        }
    }
}

template class SCIPP<SCIPPNode>;
