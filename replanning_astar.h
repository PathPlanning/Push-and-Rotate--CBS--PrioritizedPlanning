#ifndef REPLANNINGASTAR_H
#define REPLANNINGASTAR_H

#include "ilogger.h"
#include "searchresult.h"
#include "constraint.h"
#include "constraints_set.h"
#include "conflict_avoidance_table.h"
#include "search_queue.h"
#include "zero_scipp_node.h"
#include "replanning_astar_node.h"
#include "replanning_fs_node.h"
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
#include <cassert>

template <typename NodeType = Node>
class ReplanningAStar
{
    public:
        ReplanningAStar(bool WithTime = false);
        //virtual ~ISearch(void);
        ReplanningAStar(ReplanningAStar& other) = default;
        ReplanningAStar& operator=(ReplanningAStar& other) = default;
        ReplanningAStar(ReplanningAStar&& other) = default;
        ReplanningAStar& operator=(ReplanningAStar&& other) = default;
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

        //static int convolution(int i, int j, const Map &map, int time = 0, bool withTime = false);

        // void getPerfectHeuristic(const Map &map, const AgentSet &agentSet);
        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j);

        virtual void updateFocalW(double FocalW, const Map &map) {};

        virtual int getSize() {return sortByIndex.size();};

        static int T;
        static int P;

    //protected:
        //CODE HERE
        //Try to split class functionality to the methods that can be re-used in successors classes,
        //e.g. classes that realize A*, JPS, Theta* algorithms

        //Hint 1. You definetely need class variables for OPEN and CLOSE

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
        //will return it's sucessors, e.g. unordered list of nodes

        //Hint 4. It's a good idea to define a resetParent function that will be extensively used for Theta*
        //and for A*/Dijkstra/JPS will exhibit "default" behaviour

        //Hint 5. The last but not the least: working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement

        virtual void makePrimaryPath(Node &curNode, int endTime);//Makes path using back pointers
        virtual void makeSecondaryPath(const Map &map);//Makes another type of path(sections or points)
        //virtual void setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints);
        virtual void setHC(NodeType &neigh, const NodeType &cur,
                           const ConflictAvoidanceTable &CAT, bool isGoal) {}
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
        virtual void removeCur(const NodeType& cur, const Map& map);
        virtual void removeUnexpandedNode(const NodeType& node);
        virtual void subtractFutureConflicts(NodeType &node) {}
        virtual double getMinFocalF();
        virtual void clearLists();
        virtual std::list<NodeType> findPredecessors(const NodeType &curNode, const Map &map, int goal_i = 0, int goal_j = 0, int agentId = -1,
            const std::unordered_set<Node, NodeHash> &occupiedNodes =
                  std::unordered_set<Node, NodeHash>(),
            const ConstraintsSet &constraints = ConstraintsSet(),
            bool withCAT = false, const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());
        bool fillParents(NodeType &node, const Map &map,
            int goal_i, int goal_j, int agentId,
            const std::unordered_set<Node, NodeHash> &occupiedNodes,
            const ConstraintsSet &constraints,
            bool withCAT, const ConflictAvoidanceTable &CAT);
        NodeType* getBestParentPtr(NodeType &node, const Map &map,
            int goal_i, int goal_j, int agentId,
            const std::unordered_set<Node, NodeHash> &occupiedNodes,
            const ConstraintsSet &constraints,
            bool withCAT, const ConflictAvoidanceTable &CAT);
        virtual void updateNode(NodeType &node, const Map &map,
            int goal_i, int goal_j, int agentId,
            const std::unordered_set<Node, NodeHash> &occupiedNodes,
            const ConstraintsSet &constraints,
            bool withCAT, const ConflictAvoidanceTable &CAT,
            std::queue<NodeType>& queue, std::unordered_set<int>& addedNodesConv);
        virtual void processConstraint(const Constraint& constraint, const Map &map,
            int start_i, int start_j, int goal_i, int goal_j, int agentId,
            const std::unordered_set<Node, NodeHash> &occupiedNodes,
            const ConstraintsSet &constraints,
            bool withCAT, const ConflictAvoidanceTable &CAT);
        virtual int getPredConflictsCount(
            const NodeType& node, const NodeType& pred,
            const ConflictAvoidanceTable& CAT) const;

        virtual void checkMinFChange() {};

        void setPerfectHeuristic(std::unordered_map<std::pair<Node, Node>, int, NodePairHash>* PerfectHeuristic) {
            perfectHeuristic = PerfectHeuristic;
        }

        double manhattanDistance(int x1, int y1, int x2, int y2);
        double metric(int x1, int y1, int x2, int y2);

        SearchResult                        sresult;
        std::list<Node>                     lppath, hppath;
        double                              hweight;//weight of h-value
        bool                                breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal
        std::set<NodeType>                  open;
        std::unordered_map<int, NodeType>   sortByIndex;
        std::unordered_multiset<int>        closeConv;
        bool                                withTime;
        std::unordered_map<std::pair<Node, Node>, int, NodePairHash>* perfectHeuristic = nullptr;
        int                                 goalG = -1;
        int                                 goalI = -1;
        int                                 goalJ = -1;
        //std::unordered_set<int>             removedConvs;
        //std::unordered_map<int, int>        predsCount;
        //need to define open, close;

        virtual void checkFocal() {return;};
        virtual void addTime(int time) {return;};
};

#endif // REPLANNINGASTAR_H
