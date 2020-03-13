#ifndef CONFLICTAVOIDANCETABLE_H
#define CONFLICTAVOIDANCETABLE_H

#include <unordered_map>
#include <list>
#include "node.h"
#include "map.h"

class ConflictAvoidanceTable
{
public:
    std::unordered_map<int, int> agentsCount;

    void addAgentPosition(int i, int j, int time, const Map& map);
    void removeAgentPosition(int i, int j, int time, const Map& map);
    void addAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map);
    void removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end, const Map& map);
    int getAgentsCount(int i, int j, int time, const Map& map) const;
};

#endif // CONFLICTAVOIDANCETABLE_H
