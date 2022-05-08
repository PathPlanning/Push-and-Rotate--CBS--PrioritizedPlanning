#ifndef REPLANNING_ASTAR_NODE_H
#define REPLANNING_ASTAR_NODE_H

#include <tuple>
#include <unordered_set>
#include "node.h"

struct ReplanningAstarNode : virtual public Node
{
    int predCount = 0;

    ReplanningAstarNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, double H_ = 0, int ConflictsCount = 0) :
        Node(x, y, p, g_, H_, ConflictsCount) {}

    bool operator< (const ReplanningAstarNode &other) const {
        return std::tuple<double, int, int, int, int>(F, conflictsCount, -g, i, j) <
                std::tuple<double, int, int, int, int>(other.F, other.conflictsCount, -other.g, other.i, other.j);
    }
};

#endif // REPLANNING_ASTAR_NODE_H
