#include "astar.h"

template<typename NodeType>
Astar<NodeType>::Astar(bool WithTime, double HW, bool BT)
{
    this->hweight = 1;
    this->breakingties = BT;
    this->withTime = WithTime;
}

template<typename NodeType>
double Astar<NodeType>::computeHFromCellToCell(int i1, int j1, int i2, int j2)
{
    if (this->perfectHeuristic != nullptr) {
        auto it = this->perfectHeuristic->find(std::make_pair(NodeType(i1, j1), NodeType(i2, j2)));
        if (it != this->perfectHeuristic->end()) {
            return it->second;
        }
    }
    return metric(i1, j1, i2, j2) * this->hweight;
}

template<typename NodeType>
double Astar<NodeType>::manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

template<typename NodeType>
double Astar<NodeType>::metric(int x1, int y1, int x2, int y2) {
    return manhattanDistance(x1, y1, x2, y2);
}

template class Astar<Node>;
template class Astar<SIPPNode>;
template class Astar<ZeroSCIPPNode>;
template class Astar<SCIPPNode>;
template class Astar<FSNode>;
