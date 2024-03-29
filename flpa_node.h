#ifndef FLPA_NODE_H
#define FLPA_NODE_H

#include "node.h"
#include "gl_const.h"

struct FLPANode : virtual public Node
{
    int hc;
    int futureConflictsCount;
    int time;
    int rhs;
    int minDst;

    FLPANode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, double H_ = 0, int ConflictsCount = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount), hc(hc_), futureConflictsCount(0) {}

    FLPANode(const Node &other) : Node(other) {}

    void setF() {
        minDst = std::min(g, rhs);
        F = (minDst == CN_INFINITY) ? CN_INFINITY : double(minDst) + H;
    }

    virtual int convolution(int width, int height, bool withTime = true) const {
        int res = withTime ? width * height * time : 0;
        return res + i * width + j;
    }

    bool operator< (const FLPANode &other) const {
        return std::tuple<double, int, int, int, int>(F, minDst, -time, i, j) <
                std::tuple<double, int, int, int, int>(other.F, other.minDst, -other.time, other.i, other.j);
    }
};

#endif // FLPA_NODE_H
