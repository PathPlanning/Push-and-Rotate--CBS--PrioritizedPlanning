#include "weighted_sipp.h"

template<typename NodeType>
WeightedSIPP<NodeType>::~WeightedSIPP() {}

template<typename NodeType>
void WeightedSIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                                  const NodeType & node, std::pair<int, int> interval,
                                                  const ConflictAvoidanceTable &CAT) {
    CAT.getSoftConflictIntervals(softConflictIntervals, node, interval.first, interval.second, true);
}

template<typename NodeType>
void WeightedSIPP<NodeType>::addOptimalNode(const NodeType& cur, NodeType &neigh, std::pair<int, int> interval,
                                            int agentId, const ConstraintsSet &constraints,
                                            std::list<NodeType> &successors) {
    if (cur.optimal) {
        neigh.optimal = true;
        neigh.startTime = interval.first;
        neigh.endTime = interval.second;
        this->setNeighG(cur, neigh, agentId, constraints);
        neigh.F = weight * (neigh.g + neigh.H);
        successors.push_back(neigh);
    }
}


template<typename NodeType>
bool WeightedSIPP<NodeType>::getOptimal(const NodeType &neigh) {
    return neigh.optimal;
}

template<typename NodeType>
void WeightedSIPP<NodeType>::setOptimal(NodeType &neigh, bool val) {
    neigh.optimal = val;
}

template class WeightedSIPP<WeightedSIPPNode>;
