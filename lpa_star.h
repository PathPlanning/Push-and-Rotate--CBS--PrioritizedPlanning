/*#ifndef FOCALLPASTAR_H
#define FOCALLPASTAR_H

#include "searchresult.h"
#include "constraint.h"
#include "constraints_set.h"
#include "conflict_avoidance_table.h"
#include "search_queue.h"
#include "lpa_node.h"
#include "constraint.h"
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <set>
#include <map>
#include <algorithm>
#include <unordered_set>
#include <queue>


template <typename NodeType = LPANode>
class LPAStar
{
public:
    LPAStar();
    LPAStar(LPAStar& other) = default;
    LPAStar& operator=(LPAStar& other) = default;
    LPAStar(LPAStar&& other) = default;
    LPAStar& operator=(LPAStar&& other) = default;
    LPAStar(double FocalW = 1.0, double HW = 1.0, bool BT = true);
    SearchResult startSearch(const Map &map, const AgentSet &agentSet,
                             int start_i, int start_j, int goal_i = 0, int goal_j = 0,
                             bool (*isGoal)(const Node&, const Node&, const Map&, const AgentSet&) = nullptr,
                             bool freshStart = true, bool returnPath = true,
                             int startTime = 0, int goalTime = -1, int maxTime = -1,
                             const std::unordered_set<Node, NodeHash> &occupiedNodes =
                                   std::unordered_set<Node, NodeHash>(),
                             const ConstraintsSet &constraints = ConstraintsSet(),
                             bool withCAT = false, const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable(),
                             std::chrono::steady_clock::time_point globalBegin = std::chrono::steady_clock::time_point(),
                             int globalTimeLimit = -1);

    virtual std::list<NodeType> findSuccessors(const NodeType &curNode, const Map &map, int goal_i = 0, int goal_j = 0, int agentId = -1,
                                           const std::unordered_set<Node, NodeHash> &occupiedNodes =
                                                 std::unordered_set<Node, NodeHash>(),
                                           const ConstraintsSet &constraints = ConstraintsSet(),
                                           bool withCAT = false, const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

    virtual std::list<NodeType> findPredecessors(const NodeType &curNode, const Map &map, int goal_i = 0, int goal_j = 0, int agentId = -1,
                                           const std::unordered_set<Node, NodeHash> &occupiedNodes =
                                                 std::unordered_set<Node, NodeHash>(),
                                           const ConstraintsSet &constraints = ConstraintsSet(),
                                           bool withCAT = false, const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

    //static int convolution(int i, int j, const Map &map, int time = 0, bool withTime = false);

    // void getPerfectHeuristic(const Map &map, const AgentSet &agentSet);
    virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j);

    virtual void updateFocalW(double FocalW, const Map &map) {};

    virtual int getSize() {return open.size() + focal.size();};

    static int T;

    virtual void makePrimaryPath(Node &curNode, int endTime);//Makes path using back pointers
    virtual void makeSecondaryPath(const Map &map);//Makes another type of path(sections or points)
    //virtual void setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints);
    virtual void setHC(NodeType &neigh, const NodeType &cur,
                       const ConflictAvoidanceTable &CAT, bool isGoal);
    virtual void createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                          int agentId, const ConstraintsSet &constraints,
                                          const ConflictAvoidanceTable &CAT, bool isGoal);
    virtual bool checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints);
    virtual void addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT);
    virtual void addSuboptimalNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {}
    virtual bool checkOpenEmpty();
    virtual bool canStay() { return withTime; }
    virtual int getFocalSize() { return 0; }
    virtual NodeType getCur(const Map& map);
    //virtual void removeCur(const NodeType& cur, const Map& map);
    virtual void subtractFutureConflicts(NodeType &node) {}
    //virtual bool updateFocal(const NodeType& neigh, const Map& map);
    virtual double getMinFocalF();
    virtual void clearLists();
    void updateNode(NodeType &node, const Map &map, const NodeType &prevNode,
        int goal_i, int goal_j, int agentId,
        const std::unordered_set<Node, NodeHash> &occupiedNodes,
        const ConstraintsSet &constraints,
        bool withCAT, const ConflictAvoidanceTable &CAT, bool ignorePreds = false);
    void fillParents(NodeType &node, const Map &map,
        int goal_i, int goal_j, int agentId,
        const std::unordered_set<Node, NodeHash> &occupiedNodes,
        const ConstraintsSet &constraints,
        bool withCAT, const ConflictAvoidanceTable &CAT);
    void setPerfectHeuristic(std::unordered_map<std::pair<Node, Node>, int, NodePairHash>* PerfectHeuristic) {
        perfectHeuristic = PerfectHeuristic;
    }
    double manhattanDistance(int x1, int y1, int x2, int y2);
    double metric(int x1, int y1, int x2, int y2);
    void processConstraint(const Constraint& constraint, const Map &map,
        int goal_i, int goal_j, int agentId,
        const std::unordered_set<Node, NodeHash> &occupiedNodes,
        const ConstraintsSet &constraints,
        bool withCAT, const ConflictAvoidanceTable &CAT);

    SearchResult                        sresult;
    std::list<Node>                     lppath, hppath;
    double                              hweight;//weight of h-value
    bool                                breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal
    bool                                withTime = true;
    std::set<NodeType> open;
    std::set<NodeType, bool (*)(const NodeType&, const NodeType&)> focal;
    std::unordered_map<int, NodeType> sortByIndex;
    std::multiset<double> focalF;
    double focalW;
    bool secondPhase;
    std::unordered_map<std::pair<Node, Node>, int, NodePairHash>* perfectHeuristic = nullptr;
    int goalG;
    //std::unordered_map<int, std::set<int>> nodeTimes;
    //need to define open, close;
};

#endif // FOCALLPASTAR_H*/
