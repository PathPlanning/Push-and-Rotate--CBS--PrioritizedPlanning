#ifndef SIPP_NODE_H
#define SIPP_NODE_H

#include "node.h"

struct SIPPNode : virtual public Node
{
    int     startTime;
    int     endTime;

    SIPPNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount), startTime(g_), endTime(g_) {}

    SIPPNode(const Node &other) : Node(other) {}

    virtual int convolution(int width, int height, bool withTime = true) const {
        int res = withTime ? width * height * startTime : 0;
        return res + i * width + j;
    }

    bool operator< (const SIPPNode &other) const {
        return std::tuple<int, int, int, int, int>(F, -g, i, j, -startTime) <
                std::tuple<int, int, int, int, int>(other.F, -other.g, other.i, other.j, -other.startTime);
    }

};

#endif
