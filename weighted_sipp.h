#ifndef WEIGHTEDSIPP_H
#define WEIGHTEDSIPP_H

#include "sipp.h"
#include "weighted_sipp_node.h"

template <typename NodeType = WeightedSIPPNode>
class WeightedSIPP : public SIPP<NodeType>
{
public:
    WeightedSIPP(double Weight = 1.0, double HW = 1.0, bool BT = true) :
        SIPP<NodeType>(HW, BT), weight(Weight) {}
    virtual ~WeightedSIPP();

protected:
    void splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                              const NodeType & node, std::pair<int, int> interval,
                              const ConflictAvoidanceTable &CAT) override;
    bool binarySplitting() { return true; }
    bool getOptimal(const NodeType &neigh) override;
    void setOptimal(NodeType &neigh, bool val) override;
    void addOptimalNode(const NodeType& cur, NodeType &neigh, std::pair<int, int> interval,
                        int agentId, const ConstraintsSet &constraints,
                        std::list<NodeType> &successors) override;

    double weight;
};


#endif // WEIGHTEDSIPP_H
