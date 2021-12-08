#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <unordered_set>
#include <tuple>

struct Constraint
{
    int     i, j; //grid cell coordinates
    int     time;
    int     dur;
    int     agentId;
    int     prev_i, prev_j;
    bool    goalNode;
    bool    positive;

    Constraint(int x = 0, int y = 0, int Time = 0, int AgentId = 0, int PrevI = -1, int PrevJ = -1, bool GoalNode = false) {
        i = x;
        j = y;
        time = Time;
        agentId = AgentId;
        prev_i = PrevI;
        prev_j = PrevJ;
        goalNode = GoalNode;
        positive = false;
        dur = 1;
    }

    bool operator== (const Constraint &other) const {
        return i == other.i && j == other.j && time == other.time &&
            prev_i == other.prev_i && prev_j == other.prev_j &&
            goalNode == other.goalNode && agentId == other.agentId;
    }
    bool operator!= (const Constraint &other) const {
        return !(*this == other);
    }

    bool operator< (const Constraint &other) const {
        return std::tuple<int, int, int, int, int, bool, int>(i, j, prev_i, prev_j, time, goalNode, agentId) <
            std::tuple<int, int, int, int, int, bool, int>(
                other.i, other.j, other.prev_i, other.prev_j, other.time, other.goalNode, other.agentId);
    }
};

#endif // CONSTRAINT_H
