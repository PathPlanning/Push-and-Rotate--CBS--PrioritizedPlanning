#ifndef SEARCHQUEUE_H
#define SEARCHQUEUE_H

#include "node.h"
#include "zero_scipp_node.h"
#include "scipp_node.h"
#include "fs_node.h"
#include "map.h"
#include <set>
#include <unordered_map>

template <typename NodeType = Node>
class SearchQueue
{
public:
    SearchQueue(bool (*cmp)(const NodeType&, const NodeType&) = [](const NodeType& lhs, const NodeType& rhs) {return lhs < rhs;});

    bool insert(const Map& map, NodeType node, bool withTime,
                bool withOld = false, NodeType old = NodeType(-1, -1));
    void erase(const Map& map, NodeType node, bool withTime);
    void moveByThreshold(SearchQueue<NodeType>& other, double threshold, const Map& map, std::multiset<double>& otherF,
                         bool withTime = false);
    NodeType getByIndex(const Map& map, NodeType node, bool withTime);
    NodeType getFront() const;
    bool empty() const;
    int size() const;
    void clear();

//private:
    std::set<NodeType, bool (*)(const NodeType&, const NodeType&)> sortByKey;
    std::unordered_map<int, NodeType> sortByIndex;
    bool (*cmp)(const NodeType&, const NodeType&);
};

#endif // SEARCHQUEUE_H
