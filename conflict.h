#include "node.h"
#include <vector>
#include <unordered_map>

struct Conflict {
    int id1, id2;
    Node pos1, pos2;
    int time;
    bool edgeConflict;
    bool conflictFound;

    Conflict(bool ConflictFound) {
        conflictFound = ConflictFound;
    }

    Conflict(int Id1, int Id2, Node Pos1, Node Pos2, int Time, bool EdgeConflict) {
        id1 = Id1;
        id2 = Id2;
        pos1 = Pos1;
        pos2 = Pos2;
        time = Time;
        edgeConflict = EdgeConflict;
        conflictFound = true;
    }

    template<typename Iter>
    static Conflict findConflict(const std::vector<Iter> &starts, const std::vector<Iter> &ends);
};

template<typename Iter>
Conflict Conflict::findConflict(const std::vector<Iter> &starts, const std::vector<Iter> &ends) {
    std::vector<Iter> iters = starts;
    for (int time = 0;; ++time) {
        std::unordered_map<Node, int> positions;
        std::unordered_map<std::pair<Node, Node>, int> edges;
        std::vector<int> ids;

        int finished = 0;
        for (int i = 0; i < iters.size(); ++i) {
            auto posIt = positions.find(*iters[i]);
            if (posIt != positions.end()) {
                return Conflict(i, posIt->second, *iters[i], *iters[i], time, false);
            }
            positions[*iters[i]] = i;

            if (std::next(iters[i]) != ends[i]) {
                auto edgeIt = edges.find(std::make_pair(*std::next(iters[i]), *iters[i]));
                if (edgeIt != edges.end()) {
                    return Conflict(i, edgeIt->second, *iters[i], *std::next(iters[i]), time + 1, true);
                }
                edges[std::make_pair(*iters[i], *std::next(iters[i]))] = i;
            }

            if (std::next(iters[i]) != ends[i]) {
                ++iters[i];
            } else {
                ++finished;
            }
        }

        if (finished == iters.size()) {
            break;
        }
    }
    return Conflict(false);
}

