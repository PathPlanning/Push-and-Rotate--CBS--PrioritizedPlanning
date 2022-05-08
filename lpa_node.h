#ifndef FLPA_NODE_H
#define FLPA_NODE_H

#include "node.h"
/*#include "gl_const.h"

struct LPANode : virtual public Node
{
    int futureConflictsCount;
    int time;
    int rhs;
    int minDst;

    LPANode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount), futureConflictsCount(0) {}

    LPANode(const Node &other) : Node(other) {}

    void setF() {
        minDst = std::min(g, rhs);
        F = (minDst == CN_INFINITY) ? CN_INFINITY : minDst + H;
    }

    virtual int convolution(int width, int height, bool withTime = true) const {
        int res = withTime ? width * height * time : 0;
        return res + i * width + j;
    }

    bool operator< (const LPANode &other) const {
        return std::tuple<int, int, int, int, int>(F, minDst, -time, i, j) <
                std::tuple<int, int, int, int, int>(other.F, other.minDst, -other.time, other.i, other.j);
    }
};

#endif // FLPA_NODE_H
*/
