#ifndef CONFLICTAVOIDANCETABLE_H
#define CONFLICTAVOIDANCETABLE_H

#include <unordered_map>
#include <list>
#include "node.h"
#include "map.h"
#include "search_queue.h"

class ConflictAvoidanceTable
{
public:
    void addAgentPosition(const Node &node, const Map& map);
    void removeAgentPosition(const Node &node, const Map& map);
    void addAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map);
    void removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map);
    int getAgentsCount(const Node &node, const Map& map) const;

private:
    std::unordered_map<int, int> agentsCount;
};

#endif // CONFLICTAVOIDANCETABLE_H
