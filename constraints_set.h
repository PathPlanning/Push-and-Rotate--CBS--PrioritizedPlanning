#ifndef CONSTRAINTSSET_H
#define CONSTRAINTSSET_H

#include "constraint.h"
#include <set>

class ConstraintsSet
{
public:
    void addNodeConstraint(int i, int j, int time, int agentId);
    void addGoalNodeConstraint(int i, int j, int time, int agentId);
    void addEdgeConstraint(int i, int j, int time, int agentId, int prevI, int prevJ);
    void addConstraint(Constraint &constraint);
    ConstraintsSet getAgentConstraints(int agentId) const;

    bool hasNodeConstraint(int i, int j, int time, int agentId) const;
    bool hasFutureConstraint(int i, int j, int time, int agentId) const;
    bool hasEdgeConstraint(int i, int j, int time, int agentId, int prevI, int prevJ) const;
private:
    std::set<Constraint> nodeConstraints;
    std::set<Constraint> edgeConstraints;
    std::set<Constraint> goalNodeConstraints;
};

#endif // CONSTRAINTSSET_H
