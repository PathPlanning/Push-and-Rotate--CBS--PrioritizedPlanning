#include "focalsearch.h"

FocalSearch::FocalSearch(bool WithTime, double FocalW, double HW, bool BT) : Astar(WithTime, HW, BT) {
    auto focalCmp = [](const Node &lhs, const Node &rhs) {
        return lhs.hc < rhs.hc || lhs.hc == rhs.hc && lhs < rhs;
    };
    focal = SearchQueue(focalCmp);
    focalW = FocalW;
}

bool FocalSearch::checkOpenEmpty() {
    return open.empty() && focal.empty();
}

Node FocalSearch::getCur(const Map& map) {
    if (!open.empty()) {
        double minF = open.getFront().F;
        if (!focalF.empty()) {
            minF = std::min(minF, *focalF.begin());
        }
        open.moveByThreshold(focal, minF * focalW, map, focalF, withTime, withIntervals);
    }
    Node cur = focal.getFront();
    focal.erase(map, cur, withTime, withIntervals);
    focalF.erase(focalF.find(cur.F));
    return cur;
}

bool FocalSearch::updateFocal(const Node& neigh, const Map& map) {
    Node old = focal.getByIndex(map, neigh, withTime, withIntervals);
    if (old.i != -1) {
        if (focal.insert(map, neigh, withTime, withIntervals, true, old)) {
            focalF.erase(focalF.find(old.F));
            focalF.insert(neigh.F);
        }
        return true;
    }
    return false;
}

double FocalSearch::getMinFocalF() {
    if (focalF.empty()) {
        return CN_INFINITY;
    }
    return *focalF.begin();
}

void FocalSearch::clearLists() {
    ISearch::clearLists();
    focal.clear();
    focalF.clear();
}
