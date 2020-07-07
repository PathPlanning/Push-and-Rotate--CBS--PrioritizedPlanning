#ifndef NODE_H
#define NODE_H

#include <unordered_set>
#include <tuple>

struct Node
{
    int     i, j; //grid cell coordinates
    int     F, g, H; //f-, g- and h-values of the search node
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)
    int     conflictsCount;

    Node(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, int H_ = 0, int ConflictsCount = 0) {
        i = x;
        j = y;
        parent = p;
        g = g_;
        H = H_;
        F = g_ + H_;
        conflictsCount = ConflictsCount;
    }

    bool operator== (const Node &other) const {
        return i == other.i && j == other.j;
    }
    bool operator!= (const Node &other) const {
        return i != other.i || j != other.j;
    }
    bool operator< (const Node &other) const {
        return std::tuple<int, int, int, int>(F, -g, i, j) <
                std::tuple<int, int, int, int>(other.F, -other.g, other.i, other.j);
    }

    virtual int convolution(int width, int height, bool withTime = false) const {
        int res = withTime ? width * height * g : 0;
        return res + i * width + j;
    }
};

struct NodeHash {
    size_t operator()(const Node& node) const {
        return (node.i + node.j) * (node.i + node.j + 1) + node.j;
    }
};

struct NodePairHash {
    size_t operator()(const std::pair<Node, Node>& pair) const {
        NodeHash nodeHash;
        size_t hash1 = nodeHash(pair.first), hash2 = nodeHash(pair.second);
        return (hash1 + hash2) * (hash1 + hash2 + 1) + hash2;
    }
};

#endif
