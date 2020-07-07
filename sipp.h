#ifndef SIPP_H
#define SIPP_H

#include "astar.h"
#include "sipp_node.h"
#include "weighted_sipp_node.h"

template <typename NodeType = SIPPNode>
class SIPP : virtual public Astar<NodeType>
{
public:
    SIPP(double HW = 1.0, bool BT = true) : Astar<NodeType>(true, HW, BT) {
        this->withTime = true;
        this->withIntervals = true;
    }
    virtual ~SIPP() {}

protected:
    void setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) override;
    void createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                  int agentId, const ConstraintsSet &constraints,
                                  const ConflictAvoidanceTable &CAT) override;
    bool checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) override;
    virtual void splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                      const NodeType & node, std::pair<int, int> interval,
                                      const ConflictAvoidanceTable &CAT);
    virtual void setNeighG(const NodeType &cur, NodeType &neigh,
                   int agentId, const ConstraintsSet &constraints);
    virtual void addOptimalNode(const NodeType& cur, NodeType &neigh, std::pair<int, int> interval,
                                int agentId, const ConstraintsSet &constraints,
                                std::list<NodeType> &successors) {}
    virtual bool getOptimal(const NodeType &neigh) { return false; }
    virtual void setOptimal(NodeType &neigh, bool val) {}
};

#endif // SIPP_H
