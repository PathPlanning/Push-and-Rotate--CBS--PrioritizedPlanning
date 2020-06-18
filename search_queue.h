#ifndef SEARCHQUEUE_H
#define SEARCHQUEUE_H

#include "node.h"
#include "map.h"
#include <set>
#include <unordered_map>

class SearchQueue
{
public:
    SearchQueue(bool (*cmp)(const Node&, const Node&) = [](const Node& lhs, const Node& rhs) {return lhs < rhs;});

    bool insert(const Map& map, Node node, bool withTime, bool withIntervals = false,
                bool withOld = false, Node old = Node(-1, -1));
    static int convolution(const Node &node, const Map &map, bool withTime = false, bool withIntervals = false);
    void erase(const Map& map, Node node, bool withTime, bool withIntervals = false);
    void moveByThreshold(SearchQueue& other, double threshold, const Map& map, std::multiset<double>& otherF,
                         bool withTime = false, bool withIntervals = false);
    Node getByIndex(const Map& map, Node node, bool withTime, bool withIntervals = false);
    Node getFront() const;
    bool empty() const;
    int size() const;
    void clear();

private:
    std::set<Node, bool (*)(const Node&, const Node&)> sortByKey;
    std::unordered_map<int, Node> sortByIndex;
    bool (*cmp)(const Node&, const Node&);
};

#endif // SEARCHQUEUE_H
