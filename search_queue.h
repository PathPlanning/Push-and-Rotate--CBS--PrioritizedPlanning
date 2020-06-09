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

    bool insert(const Map& map, Node node, bool withTime, bool withOld = false, Node old = Node(-1, -1));
    static int convolution(int i, int j, const Map &map, int time = 0, bool withTime = false);
    void erase(const Map& map, Node node, bool withTime);
    void moveByThreshold(SearchQueue& other, double threshold, const Map& map,
                         bool withTime, std::multiset<double>& otherF);
    Node getByIndex(const Map& map, Node node, bool withTime);
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
