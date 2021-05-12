#ifndef SCIPP_H
#define SCIPP_H

#include "focalsearch.h"
#include "sipp.h"
#include "scipp_node.h"

template <typename NodeType = SCIPPNode>
class SCIPP : public SIPP<NodeType>, public FocalSearch<NodeType>
{
public:
    SCIPP(double FocalW = 1.0, double HW = 1.0, bool BT = true) :
        Astar<NodeType>(true, HW, BT), SIPP<NodeType>(HW, BT), FocalSearch<NodeType>(true, FocalW, HW, BT) {}
    SCIPP(SCIPP& other) = default;
    SCIPP& operator=(SCIPP& other) = default;
    SCIPP(SCIPP&& other) = default;
    SCIPP& operator=(SCIPP&& other) = default;
    virtual ~SCIPP();

protected:
    void splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                              const NodeType & node, const NodeType & prevNode, std::pair<int, int> interval,
                              const ConflictAvoidanceTable &CAT) override;
    bool canStay() override { return true; }
    void addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) override;
    void setHC(NodeType &neigh, const NodeType &cur,
               const ConflictAvoidanceTable &CAT, bool isGoal) override;
};

#endif // SCIPP_H
