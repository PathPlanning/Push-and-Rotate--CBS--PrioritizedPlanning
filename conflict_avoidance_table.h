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
    void addAgentPosition(const Node &node);
    void removeAgentPosition(const Node &node);
    void addAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end);
    void removeAgentPath(std::list<Node>::iterator& start, std::list<Node>::iterator& end);
    int getAgentsCount(const Node &node) const;
    void getSoftConflictIntervals(std::vector<std::pair<int, int>> &res, const Node & node,
                                                          int startTime, int endTime, bool binary) const;

private:
    std::map<std::tuple<int, int, int>, int> agentsCount;
};

#endif // CONFLICTAVOIDANCETABLE_H
