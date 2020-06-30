#ifndef SIPP_H
#define SIPP_H

#include "astar.h"

class SIPP : virtual public Astar
{
public:
    SIPP(double HW = 1.0, bool BT = true) : Astar(true, HW, BT) { withIntervals = true; }
    virtual ~SIPP() {}

protected:
    int getEndTime(int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) override;
    void createSuccessorsFromNode(const Node &cur, Node &neigh, std::list<Node> &successors,
                                  int agentId, const ConstraintsSet &constraints,
                                  const ConflictAvoidanceTable &CAT) override;
    bool checkGoal(const Node &cur, int goalTime, int agentId, const ConstraintsSet &constraints) override;
    virtual std::vector<std::pair<int, int>> splitBySoftConflicts(const Node & node, std::pair<int, int> interval,
                                                                  const ConflictAvoidanceTable &CAT);
};

#endif // SIPP_H
