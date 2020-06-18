#include "sipp.h"

std::list<Node> findSuccessors(const Node &curNode, const Map &map, int goal_i = 0, int goal_j = 0, int agentId = -1,
                               const std::unordered_set<Node> &occupiedNodes = std::unordered_set<Node>(),
                               const ConstraintsSet &constraints = ConstraintsSet(),
                               const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable()) {
    std::list<Node> successors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            int depth = curNode.time;
            if ((di == 0 || dj == 0) && (di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes)) {

            }
        }
    }
}
