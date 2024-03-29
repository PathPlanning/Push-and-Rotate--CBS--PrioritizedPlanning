#include "focalsearch.h"

template<typename NodeType>
int FocalSearch<NodeType>::Time = 0;

template<typename NodeType>
FocalSearch<NodeType>::FocalSearch(bool WithTime, double FocalW, double HW, bool BT) :
    Astar<NodeType>(WithTime, HW, BT) {
    auto focalCmp = [](const NodeType &lhs, const NodeType &rhs) {
        return lhs.hc < rhs.hc || lhs.hc == rhs.hc && lhs < rhs;
    };
    focal = SearchQueue<NodeType>(focalCmp);
    focalW = FocalW;
}

template<typename NodeType>
bool FocalSearch<NodeType>::checkOpenEmpty() {
    return this->open.empty() && focal.empty();
}

template<typename NodeType>
NodeType FocalSearch<NodeType>::getCur(const Map& map) {
    if (!this->open.empty()) {
        double minF = this->open.getFront().F;
        if (!focalF.empty()) {
            minF = std::min(minF, *focalF.begin());
        }
        this->open.moveByUpperBound(focal, minF * focalW, map, focalF, this->withTime);
    }
    NodeType cur = focal.getFront();
    return cur;
}

template<typename NodeType>
void FocalSearch<NodeType>::removeCur(const NodeType& cur, const Map& map) {
    focal.erase(map, cur, this->withTime);
    auto it = focalF.find(cur.F);
    focalF.erase(focalF.find(cur.F));
}

template<typename NodeType>
bool FocalSearch<NodeType>::updateFocal(const NodeType& neigh, const Map& map) {
    NodeType old = focal.getByIndex(map, neigh, this->withTime);
    if (old.i != -1) {
        if (focal.insert(map, neigh, this->withTime, true, old)) {
            auto it = focalF.find(old.F);
            focalF.erase(focalF.find(old.F));
            focalF.insert(neigh.F);
        }
        return true;
    }
    return false;
}

template<typename NodeType>
double FocalSearch<NodeType>::getMinFocalF() {
    if (focalF.empty()) {
        return CN_INFINITY;
    }
    return *focalF.begin();
}

template<typename NodeType>
void FocalSearch<NodeType>::clearLists() {
    ISearch<NodeType>::clearLists();
    focal.clear();
    focalF.clear();
}

template<typename NodeType>
void FocalSearch<NodeType>::setHC(NodeType &neigh, const NodeType &cur,
                                  const ConflictAvoidanceTable &CAT, bool isGoal) {
    neigh.hc = cur.hc + neigh.conflictsCount;
    if (isGoal) {
        addFutureConflicts(neigh, CAT);
    }
}

template<typename NodeType>
void FocalSearch<NodeType>::addFutureConflicts(NodeType &neigh, const ConflictAvoidanceTable &CAT) {
    neigh.futureConflictsCount = CAT.getFutureConflictsCount(neigh, neigh.g);
    neigh.hc += neigh.futureConflictsCount;
}

template<typename NodeType>
void FocalSearch<NodeType>::updateFocalW(double newFocalW, const Map& map) {
    focalW = newFocalW;
    if (focal.empty()) {
        return;
    }
    double minF = *focalF.begin();
    this->focal.moveByLowerBound(this->open, minF * focalW, map, focalF, this->withTime);
}

template class FocalSearch<FSNode>;
template class FocalSearch<SCIPPNode>;
