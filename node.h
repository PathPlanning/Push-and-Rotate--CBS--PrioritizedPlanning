#ifndef NODE_H
#define NODE_H

#include <unordered_set>
#include <tuple>

//That's the data structure for storing a single search node.
//Although one might realize A* pathfinder relying only on g-value,
//it's a good idea to store f- and h-values explicitly for the sake of simplicity
//(and either h- or f-value is definetely needed for realizing different tie-breaking strategies).
//Backpointer is obligatory for any-angle algorithms, e.g. Theta*, and it makes sense to utilize it
//in A*-like algorithms as well for reconstructing path (after the main search phase is finished).

//So, in the end of the day, we have a sort of "universal" search-node structure
//compatable with various types of grid pathfinders (Dijkstra, A*, Jump Point Search, Theta* etc.)
//which means - that's all you need for that project.

struct Node
{
    int     i, j; //grid cell coordinates
    double  F, g, H; //f-, g- and h-values of the search node 
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)
    int     subgraph = -1;
    int     time;
    int     startTime;
    int     endTime;
    int     conflictsCount;
    int     hc;
    static bool breakingties;

    Node(int x = 0, int y = 0, Node *p = nullptr, double g_ = 0, double H_ = 0, int Time = 0, int ConflictsCount = 0, int hc_ = 0) {
        i = x;
        j = y;
        parent = p;
        g = g_;
        H = H_;
        F = g_ + H_;
        time = Time;
        startTime = Time;
        endTime = Time;
        conflictsCount = ConflictsCount;
        hc = hc_;
    }

    bool operator== (const Node &other) const {
        return i == other.i && j == other.j;
    }
    bool operator!= (const Node &other) const {
        return i != other.i || j != other.j;
    }
    bool operator< (const Node &other) const {
        if (F == other.F) {
            if (conflictsCount != other.conflictsCount) {
                return conflictsCount < other.conflictsCount;
            } else {
                if (g == other.g) {
                    return std::tuple<int, int, int>(i, j, time) < std::tuple<int, int, int>(other.i, other.j, other.time);
                } else {
                    if (breakingties) {
                        return g > other.g;
                    } else {
                        return g < other.g;
                    }
                }
            }
        }
        return F < other.F;
    }
};

namespace std {
    template<>
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            return (node.i + node.j) * (node.i + node.j + 1) + node.j;
        }
    };

    template<>
    struct hash<std::pair<Node, Node>> {
        size_t operator()(const std::pair<Node, Node>& pair) const {
            hash<Node> nodeHash;
            size_t hash1 = nodeHash(pair.first), hash2 = nodeHash(pair.second);
            return (hash1 + hash2) * (hash1 + hash2 + 1) + hash2;
        }
    };
}

#endif
