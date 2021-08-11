#ifndef SCIPP_NODE_H
#define SCIPP_NODE_H

#include "sipp_node.h"
#include "fs_node.h"

struct SCIPPNode : public SIPPNode, FSNode
{
    SCIPPNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount),
        SIPPNode(x, y, p, g_, H_, ConflictsCount),
        FSNode(x, y, p, g_, H_, ConflictsCount, hc_)
    {}

    SCIPPNode(const Node &other) : Node(other) {}

    virtual int convolution(int width, int height, bool withTime = true) const {
        int cantor = (hc + startTime) * (hc + startTime + 1) / 2 + hc;
        int res = withTime ? width * height * cantor : 0;
        return res + i * width + j;
    }

    bool operator< (const SCIPPNode &other) const {
        return std::tuple<int, int, int, int, int, int>(F, -g, hc, i, j, -startTime) <
                std::tuple<int, int, int, int, int, int>(other.F, -other.g, other.hc, other.i, other.j, -other.startTime);
    }

};

#endif
