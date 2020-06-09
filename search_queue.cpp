#include "search_queue.h"

SearchQueue::SearchQueue(bool (*_cmp)(const Node&, const Node&)) {
    cmp = _cmp;
    sortByKey = std::set<Node, decltype (cmp)>(cmp);
}

int SearchQueue::convolution(int i, int j, const Map &map, int time, bool withTime) {
    int res = withTime ? map.getMapWidth() * map.getMapHeight() * time : 0;
    return res + i * map.getMapWidth() + j;
}

bool SearchQueue::insert(const Map& map, Node node, bool withTime, bool withOld, Node old) {
    if (!withOld) {
        old = getByIndex(map, node, withTime);
    }
    if (old.i == -1 || cmp(node, old)) {
        if (old.i != -1) {
            sortByKey.erase(old);
        }
        sortByIndex[convolution(node.i, node.j, map, node.depth, withTime)] = node;
        sortByKey.insert(node);
        return true;
    }
    return false;
}

void SearchQueue::erase(const Map& map, Node node, bool withTime) {
    sortByKey.erase(node);
    sortByIndex.erase(convolution(node.i, node.j, map, node.depth, withTime));
}

Node SearchQueue::getByIndex(const Map& map, Node node, bool withTime) {
    auto it = sortByIndex.find(convolution(node.i, node.j, map, node.depth, withTime));
    if (it == sortByIndex.end()) {
        return Node(-1, -1);
    }
    return it->second;
}

void SearchQueue::moveByThreshold(SearchQueue& other, double threshold, const Map& map,
                                  bool withTime, std::multiset<double>& otherF) {
    auto it = sortByKey.begin();
    for (it; it != sortByKey.end() && it->F <= threshold; ++it) {
        other.insert(map, *it, withTime);
        otherF.insert(it->F);
        sortByIndex.erase(convolution(it->i, it->j, map, it->depth, withTime));
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

