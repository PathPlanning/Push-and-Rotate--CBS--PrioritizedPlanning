#ifndef ISEARCH_H
#define ISEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "constraint.h"
#include "constraints_set.h"
#include "conflict_avoidance_table.h"
#include "search_queue.h"
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

class ISearch
{
    public:
        ISearch(bool WithTime = false);
        ~ISearch(void);
        SearchResult startSearch(const Map &map, const AgentSet &agentSet,
                                 int start_i, int start_j, int goal_i = 0, int goal_j = 0,
                                 bool (*isGoal)(const Node&, const Node&, const Map&, const AgentSet&) = nullptr,
                                 bool freshStart = true, bool returnPath = true,
                                 int startDepth = 0, int goalDepth = -1, int maxDepth = -1,
                                 bool withFocal = false, double focalW = 1.0,
                                 const std::unordered_set<Node> &occupiedNodes = std::unordered_set<Node>(),
                                 const ConstraintsSet &constraints = ConstraintsSet(),
                                 const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

        virtual std::list<Node> findSuccessors(const Node &curNode, const Map &map, int goal_i = 0, int goal_j = 0, int agentId = -1,
                                               const std::unordered_set<Node> &occupiedNodes = std::unordered_set<Node>(),
                                               const ConstraintsSet &constraints = ConstraintsSet(),
                                               const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

        //static int convolution(int i, int j, const Map &map, int time = 0, bool withTime = false);
        void getPerfectHeuristic(const Map &map, const AgentSet &agentSet);

    protected:
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
        //and only then begin enhancement!

        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j) {return 0;}
        virtual void makePrimaryPath(const Node &curNode);//Makes path using back pointers
        virtual void makeSecondaryPath(const Map &map);//Makes another type of path(sections or points)

        SearchResult                    sresult;
        std::list<Node>                 lppath, hppath;
        double                          hweight;//weight of h-value
        bool                            breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal
        SearchQueue                     open;
        std::unordered_map<int, Node>   close;
        bool                            withTime;
        std::unordered_map<std::pair<Node, Node>, int> perfectHeuristic;
        //need to define open, close;

};
#endif
