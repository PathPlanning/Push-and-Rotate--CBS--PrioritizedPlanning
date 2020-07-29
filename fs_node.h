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

};

#endif
