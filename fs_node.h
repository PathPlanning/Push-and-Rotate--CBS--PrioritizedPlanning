#ifndef FS_NODE_H
#define FS_NODE_H

#include "node.h"

struct FSNode : virtual public Node
{
    int hc;
    int futureConflictsCount;

    FSNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount), hc(hc_), futureConflictsCount(0) {}

    FSNode(const Node &other) : Node(other) {}

    virtual int convolution(int width, int height, bool withTime = true) const {
        int res = withTime ? width * height * g : 0;
        return res + i * width + j;
    }

    bool operator< (const FSNode &other) const {
        return std::tuple<int, int, int, int, int>(F, -g, hc, i, j) <
                std::tuple<int, int, int, int, int>(other.F, -other.g, other.hc, other.i, other.j);
    }

    virtual int getHC() const override {
        return hc;
    }
};

#endif
