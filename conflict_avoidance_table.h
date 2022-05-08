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
    void addNode(const Node &node);
    void addEdge(const Node &node, const Node &prev);
    void removeNode(const Node &node);
    void removeEdge(const Node &node, const Node &prev);
    void addAgentPath(const std::list<Node>::const_iterator& start,
                      const std::list<Node>::const_iterator& end);
    void removeAgentPath(const std::list<Node>::const_iterator& start,
                         const std::list<Node>::const_iterator& end);
    int getAgentsCount(const Node &node, const Node &prev) const;
    int getFirstSoftConflict(const Node & node, int startTime, int endTime) const;
    int getFutureConflictsCount(const Node & node, int time) const;
    void getSoftConflictIntervals(std::vector<std::pair<int, int>> &res, const Node & node, const Node &prevNode,
                                                          int startTime, int endTime, bool binary) const;
    int getEdgeAgentsCount(const Node &node, const Node &prev) const;
    int getNodeAgentsCount(const Node &node) const;
    void addGoalNode(const Node &node);
    void removeGoalNode(const Node &node);

private:
    std::map<std::tuple<int, int, int>, int> nodeAgentsCount;
    std::map<std::tuple<int, int, int, int, int>, int> edgeAgentsCount;
    std::map<std::pair<int, int>, int> goalNodeAgentsCount;
};

#endif // CONFLICTAVOIDANCETABLE_H
