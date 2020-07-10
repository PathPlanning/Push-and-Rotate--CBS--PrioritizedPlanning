#ifndef WEIGHTED_SIPP_NODE_H
#define WEIGHTED_SIPP_NODE_H

#include "SIPP_node.h"

struct WeightedSIPPNode : public SIPPNode
{
    bool optimal;

    WeightedSIPPNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0, bool Optimal = true) :
        Node(x, y, p, g_, H_, ConflictsCount),
        SIPPNode(x, y, p, g_, H_, ConflictsCount),
        optimal(Optimal) {}

    //WeightedSIPPNode(const Node &other) : Node(other) {}

    virtual int convolution(int width, int height, bool withTime = true) const {
        int res = withTime ? width * height * startTime : 0;
        return (res + i * width + j) * (optimal ? -1 : 1);
    }

    bool operator< (const WeightedSIPPNode &other) const {
        return std::tuple<int, int, int, int, bool>(F, -g, i, j, optimal) <
                std::tuple<int, int, int, int, bool>(other.F, -other.g, other.i, other.j, other.optimal);
    }

};

#endif // WEIGHTED_SIPP_NODE_H
