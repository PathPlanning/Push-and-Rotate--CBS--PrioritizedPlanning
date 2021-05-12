#ifndef CONSTRAINTSSET_H
#define CONSTRAINTSSET_H

#include "constraint.h"
#include "gl_const.h"
#include <set>
#include <vector>
#include <algorithm>

class ConstraintsSet
{
public:
    void addNodeConstraint(int i, int j, int time, int agentId);
    void addGoalNodeConstraint(int i, int j, int time, int agentId);
    void addEdgeConstraint(int i, int j, int time, int agentId, int prevI, int prevJ);
    void addPositiveConstraint(int i, int j, int time, int agentId, int prevI = -1, int prevJ = -1);
    void addConstraint(Constraint &constraint);
    void removeConstraint(Constraint &constraint);
    template<typename Iter> void addAgentPath(Iter start, Iter end, int agentId);

    void removeNodeConstraint(int i, int j, int time, int agentId);
    void removeGoalNodeConstraint(int i, int j, int time, int agentId);
    void removeEdgeConstraint(int i, int j, int time, int agentId, int prevI, int prevJ);
    void removeLastPositiveConstraint();
    template<typename Iter> void removeAgentPath(Iter start, Iter end, int agentId);

    ConstraintsSet getAgentConstraints(int agentId) const;
    std::vector<Constraint> getPositiveConstraints() const;
    int getFirstConstraintTime(int i, int j, int startTime, int agentId) const;
    std::vector<std::pair<int, int>> getSafeIntervals(int i, int j, int agentId, int startTime, int endTime) const;

    bool hasNodeConstraint(int i, int j, int time, int agentId) const;
    bool hasFutureConstraint(int i, int j, int time, int agentId) const;
    bool hasEdgeConstraint(int i, int j, int time, int agentId, int prevI, int prevJ) const;
//private:
    std::set<Constraint> nodeConstraints;
    std::set<Constraint> edgeConstraints;
    std::set<Constraint> goalNodeConstraints;
    std::vector<Constraint> positiveConstraints;
};

template<typename Iter>
void ConstraintsSet::addAgentPath(Iter start, Iter end, int agentId) {
    int time = 0;
    for (auto it = start; it != end; ++it) {
        if (std::next(it) == end) {
            addGoalNodeConstraint(it->i, it->j, time, agentId);
        } else {
            addNodeConstraint(it->i, it->j, time, agentId);
        }
        if (it != start) {
            addEdgeConstraint(std::prev(it)->i, std::prev(it)->j, time, agentId, it->i, it->j);
        }
        ++time;
    }
}

template<typename Iter>
void ConstraintsSet::removeAgentPath(Iter start, Iter end, int agentId) {
    int time = 0;
    for (auto it = start; it != end; ++it) {
        if (std::next(it) == end) {
            removeGoalNodeConstraint(it->i, it->j, time, agentId);
        } else {
            removeNodeConstraint(it->i, it->j, time, agentId);
        }
        if (it != start) {
            removeEdgeConstraint(std::prev(it)->i, std::prev(it)->j, time, agentId, it->i, it->j);
        }
        ++time;
    }
}


#endif // CONSTRAINTSSET_H
