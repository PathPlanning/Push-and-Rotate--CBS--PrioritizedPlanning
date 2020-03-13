#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <unordered_set>
#include <tuple>

struct Constraint
{
    int     i, j; //grid cell coordinates
    int     time;
    int     agentId;
    int     prev_i, prev_j;
    bool    goalNode;

    Constraint(int x = 0, int y = 0, int Time = 0, int AgentId = 0, int PrevI = -1, int PrevJ = -1, bool GoalNode = false) {
        i = x;
        j = y;
        time = Time;
        agentId = AgentId;
        prev_i = PrevI;
        prev_j = PrevJ;
        goalNode = GoalNode;
    }

    bool operator== (const Constraint &other) const {
        return i == other.i && j == other.j && time == other.time &&
                prev_i == other.prev_i && prev_j == other.prev_j && goalNode == other.goalNode;
    }
    bool operator!= (const Constraint &other) const {
        return !(*this == other);
    }

    bool operator< (const Constraint &other) const {
        return std::tuple<int, int, int, int, int, bool>(i, j, time, prev_i, prev_j, goalNode) <
               std::tuple<int, int, int, int, int, bool>(other.i, other.j, other.time, other.prev_i, other.prev_j, other.goalNode);
    }
};

#endif // CONSTRAINT_H
