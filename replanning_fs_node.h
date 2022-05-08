#ifndef REPLANNING_FS_NODE_H
#define REPLANNING_FS_NODE_H

#include <tuple>
#include <unordered_set>
#include "replanning_astar_node.h"

struct ReplanningFSNode : virtual public ReplanningAstarNode
{
    int hc;

    ReplanningFSNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount), hc(hc_) {}

    bool operator< (const ReplanningFSNode &other) const {
        return std::tuple<int, int, int, int, int>(F, -g, hc, i, j) <
                std::tuple<int, int, int, int, int>(other.F, -other.g, other.hc, other.i, other.j);
    }

    virtual int getHC() const override {
        return hc;
    }
};

#endif // REPLANNING_FS_NODE_H
