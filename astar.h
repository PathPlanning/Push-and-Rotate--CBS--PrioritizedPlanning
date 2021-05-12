#ifndef ASTAR_H
#define ASTAR_H
#include "dijkstra.h"

//A* search.
template <typename NodeType = Node>
class Astar : public Dijkstra<NodeType>
{
    public:
        Astar(bool WithTime = false, double HW = 1.0, bool BT = true);
        Astar(Astar& other) = default;
        Astar& operator=(Astar& other) = default;
        Astar(Astar&& other) = default;
        Astar& operator=(Astar&& other) = default;
        virtual ~Astar() {}
        void setPerfectHeuristic(std::unordered_map<std::pair<Node, Node>, int, NodePairHash>* PerfectHeuristic) {
            perfectHeuristic = PerfectHeuristic;
        }
        double computeHFromCellToCell(int i1, int j1, int i2, int j2) override;

        std::unordered_map<std::pair<Node, Node>, int, NodePairHash>* perfectHeuristic = nullptr;

    protected:
        double euclideanDistance(int x1, int y1, int x2, int y2);
        double manhattanDistance(int x1, int y1, int x2, int y2);
        double chebyshevDistance(int x1, int y1, int x2, int y2);
        double diagonalDistance(int x1, int y1, int x2, int y2);
        double metric(int x1, int y1, int x2, int y2);


};

#endif
