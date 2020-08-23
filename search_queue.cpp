#include "search_queue.h"

template<typename NodeType>
SearchQueue<NodeType>::SearchQueue(bool (*_cmp)(const NodeType&, const NodeType&)) {
    cmp = _cmp;
    sortByKey = std::set<NodeType, decltype (cmp)>(cmp);
}

template<typename NodeType>
bool SearchQueue<NodeType>::insert(const Map& map, NodeType node, bool withTime, bool withOld, NodeType old) {
    if (!withOld) {
        old = getByIndex(map, node, withTime);
    }
    if (old.i == -1 || cmp(node, old)) {
        if (old.i != -1) {
            sortByKey.erase(old);
        }
        sortByIndex[node.convolution(map.getMapWidth(), map.getMapHeight(), withTime)] = node;
        sortByKey.insert(node);
        return true;
    }
    return false;
}

template<typename NodeType>
void SearchQueue<NodeType>::erase(const Map& map, NodeType node, bool withTime) {
    sortByKey.erase(node);
    sortByIndex.erase(node.convolution(map.getMapWidth(), map.getMapHeight(), withTime));
}

template<typename NodeType>
NodeType SearchQueue<NodeType>::getByIndex(const Map& map, NodeType node, bool withTime) {
    auto it = sortByIndex.find(node.convolution(map.getMapWidth(), map.getMapHeight(), withTime));
    if (it == sortByIndex.end()) {
        return NodeType(-1, -1);
    }
    return it->second;
}

template<typename NodeType>
void SearchQueue<NodeType>::moveByThreshold(SearchQueue<NodeType>& other, double threshold, const Map& map,
                                  std::multiset<double>& otherF, bool withTime) {
    auto it = sortByKey.begin();
    for (it; it != sortByKey.end() && it->F <= threshold; ++it) {
        other.insert(map, *it, withTime);
        otherF.insert(it->F);
        sortByIndex.erase(it->convolution(map.getMapWidth(), map.getMapHeight(), withTime));
    }
    sortByKey.erase(sortByKey.begin(), it);
}

template<typename NodeType>
NodeType SearchQueue<NodeType>::getFront() const {
    return *sortByKey.begin();
}

template<typename NodeType>
bool SearchQueue<NodeType>::empty() const {
    return sortByKey.empty();
}

template<typename NodeType>
int SearchQueue<NodeType>::size() const {
    return sortByKey.size();
}

template<typename NodeType>
void SearchQueue<NodeType>::clear() {
    sortByKey.clear();
    sortByIndex.clear();
}

template class SearchQueue<Node>;
template class SearchQueue<SIPPNode>;
template class SearchQueue<ZeroSCIPPNode>;
template class SearchQueue<SCIPPNode>;
template class SearchQueue<FSNode>;
