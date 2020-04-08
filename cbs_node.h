#ifndef CBS_NODE_H
#define CBS_NODE_H

#include <vector>
#include <unordered_map>
#include <list>
#include "node.h"
#include "constraint.h"
#include "mdd.h"

struct CBSNode
{
    static int curId;

    Constraint                                  constraint;
    std::unordered_map<int, std::list<Node>>    paths;
    std::unordered_map<int, MDD>                mdds;
    CBSNode*                                    parent;
    int                                         cost;
    int                                         id;

    CBSNode(CBSNode *p = nullptr, int Cost = 0) {
        parent = p;
        cost = Cost;
        id = curId++;
    }

    bool operator< (const CBSNode &other) const {
        return cost < other.cost || (cost == other.cost && id < other.id);
    }
};

#endif // CBS_NODE_H
