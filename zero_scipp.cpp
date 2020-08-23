#include "zero_scipp.h"

template<typename NodeType>
ZeroSCIPP<NodeType>::~ZeroSCIPP() {}

template<typename NodeType>
void ZeroSCIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                                  const NodeType & node, const NodeType & prevNode,
                                                  std::pair<int, int> interval, const ConflictAvoidanceTable &CAT) {
    CAT.getSoftConflictIntervals(softConflictIntervals, node, prevNode, interval.first, interval.second, true);
}

template<typename NodeType>
void ZeroSCIPP<NodeType>::addOptimalNode(const NodeType& cur, NodeType &neigh, std::pair<int, int> interval,
                                            int agentId, const ConstraintsSet &constraints,
                                            std::list<NodeType> &successors) {
    if (cur.optimal) {
        neigh.optimal = true;
        neigh.startTime = interval.first;
        neigh.endTime = interval.second;
        this->setNeighG(cur, neigh, agentId, constraints);
        if (neigh.g <= cur.endTime + 1 && neigh.g <= neigh.endTime) {
            neigh.F = weight * (neigh.g + neigh.H);
            successors.push_back(neigh);
        }

    }
}

template<typename NodeType>
bool ZeroSCIPP<NodeType>::checkSuboptimal(const NodeType &cur) {
    return genSuboptFromOpt || !cur.optimal;
}

template<typename NodeType>
bool ZeroSCIPP<NodeType>::getOptimal(const NodeType &neigh) {
    return neigh.optimal;
}

template<typename NodeType>
void ZeroSCIPP<NodeType>::setOptimal(NodeType &neigh, bool val) {
    neigh.optimal = val;
}

template<typename NodeType>
void ZeroSCIPP<NodeType>::addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    int oldF = node.F;
    node.F *= weight;
    this->open.insert(map, node, ISearch<NodeType>::withTime);
    node.F = oldF;
}

template<typename NodeType>
void ZeroSCIPP<NodeType>::addSuboptimalNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    this->updateEndTimeBySoftConflicts(node, CAT);
    node.optimal = false;
    this->open.insert(map, node, ISearch<NodeType>::withTime);
}

template class ZeroSCIPP<ZeroSCIPPNode>;
