#include "search_queue.h"

SearchQueue::SearchQueue(bool (*_cmp)(const Node&, const Node&)) {
    cmp = _cmp;
    sortByKey = std::set<Node, decltype (cmp)>(cmp);
}

int SearchQueue::convolution(const Node &node, const Map &map, bool withTime, bool withIntervals) {
    int time = withIntervals ? node.startTime : node.time;
    int res = withTime ? map.getMapWidth() * map.getMapHeight() * time : 0;
    return res + node.i * map.getMapWidth() + node.j;
}

bool SearchQueue::insert(const Map& map, Node node, bool withTime, bool withIntervals, bool withOld, Node old) {
    if (!withOld) {
        old = getByIndex(map, node, withTime, withIntervals);
    }
    if (old.i == -1 || cmp(node, old)) {
        if (old.i != -1) {
            sortByKey.erase(old);
        }
        sortByIndex[convolution(node, map, withTime, withIntervals)] = node;
        sortByKey.insert(node);
        return true;
    }
    return false;
}

void SearchQueue::erase(const Map& map, Node node, bool withTime, bool withIntervals) {
    sortByKey.erase(node);
    sortByIndex.erase(convolution(node, map, withTime, withIntervals));
}

Node SearchQueue::getByIndex(const Map& map, Node node, bool withTime, bool withIntervals) {
    auto it = sortByIndex.find(convolution(node, map, withTime, withIntervals));
    if (it == sortByIndex.end()) {
        return Node(-1, -1);
    }
    return it->second;
}

void SearchQueue::moveByThreshold(SearchQueue& other, double threshold, const Map& map,
                                  std::multiset<double>& otherF, bool withTime, bool withIntervals) {
    auto it = sortByKey.begin();
    for (it; it != sortByKey.end() && it->F <= threshold; ++it) {
        other.insert(map, *it, withTime, withIntervals);
        otherF.insert(it->F);
        sortByIndex.erase(convolution(*it, map, withTime, withIntervals));
    }
    sortByKey.erase(sortByKey.begin(), it);
}

Node SearchQueue::getFront() const {
    return *sortByKey.begin();
}

bool SearchQueue::empty() const {
    return sortByKey.empty();
}

int SearchQueue::size() const {
    return sortByKey.size();
}

void SearchQueue::clear() {
    sortByKey.clear();
    sortByIndex.clear();
}

