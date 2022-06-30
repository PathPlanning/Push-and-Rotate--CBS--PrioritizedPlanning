#include "replanning_astar.h"

template<typename NodeType>
ReplanningAStar<NodeType>::ReplanningAStar(bool WithTime)
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    withTime = WithTime;
}

template<typename NodeType>
int ReplanningAStar<NodeType>::T = 0;

template<typename NodeType>
SearchResult ReplanningAStar<NodeType>::startSearch(const Map &map, const AgentSet &agentSet,
                                  int start_i, int start_j, int goal_i, int goal_j,
                                  bool (*isGoal)(const Node&, const Node&, const Map&, const AgentSet&),
                                  bool freshStart, bool returnPath, int startTime, int goalTime, int maxTime,
                                  const std::unordered_set<Node, NodeHash> &occupiedNodes,
                                  const ConstraintsSet &constraints,
                                  bool withCAT, const ConflictAvoidanceTable &CAT,
                                  std::chrono::steady_clock::time_point globalBegin, int globalTimeLimit)
{
    sresult.pathfound = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (goalTime != -1) {
        maxTime = goalTime;
    }
    NodeType cur;
    int agentId = -1;
    if (agentSet.isOccupied(start_i, start_j)) {
        agentId = agentSet.getAgentId(start_i, start_j);
    }

    if (freshStart) {
        clearLists();
        sresult.numberofsteps = 0;
        sresult.nodesexpanded = 0;
        cur = NodeType(start_i, start_j, nullptr, startTime,
                 computeHFromCellToCell(start_i, start_j, goal_i, goal_j));
        //setEndTime(cur, start_i, start_j, startTime, agentId, constraints);
        addStartNode(cur, map, CAT);
        addSuboptimalNode(cur, map, CAT);
        goalG = -1;
        goalI = goal_i;
        goalJ = goal_j;
    }

    if (goalG != -1) {
        NodeType goalNode = NodeType(goal_i, goal_j, nullptr, goalG, 0);
        int goalConv = goalNode.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        if (sortByIndex.find(goalConv) != sortByIndex.end()) {
            cur = goalNode;
            sresult.pathfound = true;
        }
    }

    sresult.numberofsteps = 0;

    int removedCount = 0;

    while(!checkOpenEmpty() && !sresult.pathfound) {
        ++sresult.numberofsteps;
        if (sresult.numberofsteps % 100000 == 0) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            if (elapsedMilliseconds > 300000) {
                break;
            }
        }

        if (sresult.numberofsteps % 1000 == 0 && globalTimeLimit != -1) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - globalBegin).count();
            if (elapsedMilliseconds > globalTimeLimit) {
                break;
            }
        }

        cur = getCur(map);
        removeCur(cur, map);
        int curConv = cur.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        closeConv.insert(curConv);

        /*if (removedConvs.find(curConv) != removedConvs.end()) {
            ++removedCount;
            removedConvs.erase(curConv);
        }*/

        if (cur.i == goal_i && cur.j == goal_j)
        {
            if (!constraints.hasFutureConstraint(cur.i, cur.j, cur.g, agentId) &&
                checkGoal(cur, goalTime, agentId, constraints))
            {
                sresult.pathfound = true;
                break;
            }
        }

        std::list<NodeType> successors = findSuccessors(cur, map, goal_i, goal_j, agentId, occupiedNodes,
            constraints, withCAT, CAT);
        for (auto neigh : successors) {
            int neighConv = neigh.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
            //assert(++predsCount[neighConv] <= 5);
            if (closeConv.find(neighConv) == closeConv.end()) {
                auto it = sortByIndex.find(neighConv);
                if (it == sortByIndex.end() || neigh < it->second) {
                    if (it != sortByIndex.end()) {
                        removeUnexpandedNode(it->second);
                    }
                    open.insert(neigh);
                    sortByIndex[neighConv] = neigh;
                }
            }
        }
    }

    //std::cout << agentId << " " << sresult.numberofsteps << std::endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
    sresult.nodescreated = sortByIndex.size();
    //sresult.nodesexpanded = closeConv.size();

    /*std::cout << agentId << " " <<
        sresult.numberofsteps << " " <<
        sresult.nodesexpanded << " " <<
        removedConvs.size() << " " <<
        removedCount << std::endl;*/

    sresult.nodesexpanded += sresult.numberofsteps;

    if (sresult.pathfound) {
        if (!fillParents(cur, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT)) {
            sresult.pathfound = false;
            return sresult;
        }

        goalG = cur.g;
        sresult.pathlength = cur.g;
        sresult.minF = std::min(double(cur.F), getMinFocalF());
        sresult.lastNode = cur;
        if (returnPath) {
            lppath.clear();
            hppath.clear();
            makePrimaryPath(cur, goalTime == -1 ? -1 : goalTime + 1);
            makeSecondaryPath(map);
            sresult.hppath = &hppath; //Here is a constant pointer
            sresult.lppath = &lppath;
        }
    }
    return sresult;
}

template<typename NodeType>
void ReplanningAStar<NodeType>::removeUnexpandedNode(const NodeType& node) {
    open.erase(node);
}

template<typename NodeType>
double ReplanningAStar<NodeType>::manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

template<typename NodeType>
double ReplanningAStar<NodeType>::metric(int x1, int y1, int x2, int y2) {
    return manhattanDistance(x1, y1, x2, y2);
    //return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

template<typename NodeType>
double ReplanningAStar<NodeType>::computeHFromCellToCell(int i1, int j1, int i2, int j2)
{
    if (this->perfectHeuristic != nullptr) {
        auto it = this->perfectHeuristic->find(std::make_pair(NodeType(i1, j1), NodeType(i2, j2)));
        if (it != this->perfectHeuristic->end()) {
            return it->second;
        }
    }
    return metric(i1, j1, i2, j2) * this->hweight;
}

template<typename NodeType>
double ReplanningAStar<NodeType>::getMinFocalF() {
    return CN_INFINITY;
}

template<typename NodeType>
std::list<NodeType> ReplanningAStar<NodeType>::findSuccessors(const NodeType &curNode, const Map &map,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT)
{
    std::list<NodeType> successors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            if ((di == 0 || dj == 0) && (canStay() || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes)) {
                int newh = computeHFromCellToCell(newi, newj, goal_i, goal_j);
                NodeType neigh(newi, newj, nullptr, curNode.g + 1, newh);
                neigh.conflictsCount = CAT.getAgentsCount(neigh, curNode);
                createSuccessorsFromNode(curNode, neigh, successors, agentId, constraints, CAT,
                                         neigh.i == goal_i && neigh.j == goal_j);
            }
        }
    }
    return successors;
}

template<typename NodeType>
std::list<NodeType> ReplanningAStar<NodeType>::findPredecessors(const NodeType &curNode, const Map &map,
                                        int goal_i, int goal_j, int agentId,
                                        const std::unordered_set<Node, NodeHash> &occupiedNodes,
                                        const ConstraintsSet &constraints,
                                        bool withCAT, const ConflictAvoidanceTable &CAT)
{
    if (constraints.hasNodeConstraint(curNode.i, curNode.j, curNode.g, agentId)) {
        return {};
    }
    std::list<NodeType> predecessors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            if ((di == 0 || dj == 0) && (canStay() || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes)) {
                NodeType neigh(newi, newj, nullptr, curNode.g - 1, 0);
                if (!constraints.hasNodeConstraint(neigh.i, neigh.j, neigh.g, agentId) &&
                    !constraints.hasEdgeConstraint(curNode.i, curNode.j, curNode.g, agentId, neigh.i, neigh.j))
                {
                    predecessors.push_back(neigh);
                }
            }
        }
    }
    return predecessors;
}

template<typename NodeType>
void ReplanningAStar<NodeType>::updateNode(NodeType &node, const Map &map,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT,
    std::queue<NodeType>& queue, std::unordered_set<int>& addedNodesConv)
{
    int conv = node.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
    auto it = sortByIndex.find(conv);
    if (it == sortByIndex.end()) {
        return;
    }

    sresult.nodesexpanded++;

    //assert(predsCount[conv] >= 1);
    //bool checkPredecessors = (--predsCount[conv] > 0);
    auto predecessors = findPredecessors(node, map, goal_i, goal_j, agentId, occupiedNodes,
        constraints, withCAT, CAT);
    for (auto& pred : predecessors) {
        int prevConv = pred.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        if (closeConv.find(prevConv) != closeConv.end()) {
            return;
        }
    }

    bool processSuccessors = false;
    auto closeIt = closeConv.find(conv);
    if (closeIt == closeConv.end()) {
        auto openIt = open.find(it->second);
        assert(openIt != open.end());
        open.erase(openIt);
        //removedConvs.insert(conv);
    } else {
        closeConv.erase(closeIt);
        if (node.i == goal_i && node.j == goal_j && node.g == goalG) {
            goalG = -1;
        }
        processSuccessors = true;
    }
    sortByIndex.erase(it);

    /*if (checkPredecessors || processSuccessors) {
        sresult.nodesexpanded++;
    }*/

    if (!processSuccessors) {
        return;
    }

    std::list<NodeType> successors = findSuccessors(node, map,
        goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
    for (auto& neigh : successors) {
        int neighConv = neigh.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        if (addedNodesConv.find(neighConv) == addedNodesConv.end()) {
            queue.push(neigh);
            addedNodesConv.insert(neigh.convolution(map.getMapWidth(), map.getMapHeight(), withTime));
        }/* else {
            predsCount[neighConv]--;
        }*/
        //updateNode(neigh, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT, tempCloseConv);
    }
}

template<typename NodeType>
void ReplanningAStar<NodeType>::processConstraint(const Constraint& constraint, const Map &map,
    int start_i, int start_j, int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT)
{
    if (sortByIndex.empty()) {
        return;
    }

    sresult.nodesexpanded = 0;
    //removedConvs.clear();

    NodeType node(constraint.i, constraint.j, nullptr, constraint.time,
        computeHFromCellToCell(constraint.i, constraint.j, goal_i, goal_j));

    std::queue<NodeType> queue;
    std::unordered_set<int> addedNodesConv;

    if (constraint.prev_i == -1) {
        bool goalConstraint = (constraint.i == goal_i && constraint.j == goal_j);
        bool futureGoalConstraint = (goalConstraint && constraint.time > goalG);
        if (futureGoalConstraint) {
            auto futureGoalNode = node;
            node.g = goalG;
            int futureGoalNodeConv = futureGoalNode.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
            assert(closeConv.find(futureGoalNodeConv) == closeConv.end());
            auto furuteByIndexIt = sortByIndex.find(futureGoalNodeConv);
            if (furuteByIndexIt != sortByIndex.end()) {
                removeUnexpandedNode(furuteByIndexIt->second);
                sortByIndex.erase(furuteByIndexIt);
                //predsCount[futureGoalNodeConv] = 0;
            }
        }

        int nodeConv = node.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        auto closeIt = closeConv.find(nodeConv);
        assert(closeIt != closeConv.end());
        closeConv.erase(closeIt);
        auto byIndexIt = sortByIndex.find(nodeConv);
        assert(byIndexIt != sortByIndex.end());
        if (futureGoalConstraint) {
            open.insert(byIndexIt->second);
        } else {
            sortByIndex.erase(byIndexIt);
        }
        if (goalConstraint && constraint.time >= goalG) {
            goalG = -1;
        }

        std::list<NodeType> successors = findSuccessors(node, map,
            goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
        for (auto& neigh : successors) {
            queue.push(neigh);
            addedNodesConv.insert(neigh.convolution(map.getMapWidth(), map.getMapHeight(), withTime));
            //updateNode(neigh, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT, tempCloseConv);
        }
    } else {
        queue.push(node);
        addedNodesConv.insert(node.convolution(map.getMapWidth(), map.getMapHeight(), withTime));
        //updateNode(node, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT, tempCloseConv);
    }

    while (!queue.empty()) {
        NodeType cur = queue.front();
        updateNode(cur, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT, queue, addedNodesConv);
        queue.pop();
    }

    checkMinFChange();
}

template<typename NodeType>
void ReplanningAStar<NodeType>::clearLists() {
    open.clear();
    sortByIndex.clear();
    closeConv.clear();
    //removedConvs.clear();
    //predsCount.clear();
}

template<typename NodeType>
void ReplanningAStar<NodeType>::addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    open.insert(node);
    sortByIndex[node.convolution(map.getMapWidth(), map.getMapHeight(), withTime)] = node;
}

template<typename NodeType>
bool ReplanningAStar<NodeType>::checkOpenEmpty() {
    return open.empty();
}

template<typename NodeType>
NodeType ReplanningAStar<NodeType>::getCur(const Map& map) {
    return *open.begin();
}

template<typename NodeType>
void ReplanningAStar<NodeType>::removeCur(const NodeType& cur, const Map& map) {
    open.erase(cur);
}

template<typename NodeType>
bool ReplanningAStar<NodeType>::checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.g == goalTime;
}

template<typename NodeType>
void ReplanningAStar<NodeType>::createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                       int agentId, const ConstraintsSet &constraints,
                                       const ConflictAvoidanceTable &CAT, bool isGoal) {
    if (!constraints.hasNodeConstraint(neigh.i, neigh.j, neigh.g, agentId) &&
        !constraints.hasEdgeConstraint(neigh.i, neigh.j, neigh.g, agentId, cur.i, cur.j))
    {
        setHC(neigh, cur, CAT, isGoal);
        successors.push_back(neigh);
    }
}

template<typename NodeType>
int ReplanningAStar<NodeType>::getPredConflictsCount(
    const NodeType& node, const NodeType& pred,
    const ConflictAvoidanceTable& CAT) const
{
    return CAT.getNodeAgentsCount(pred) + CAT.getEdgeAgentsCount(node, pred);
}

template<typename NodeType>
NodeType* ReplanningAStar<NodeType>::getBestParentPtr(NodeType &node, const Map &map,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT)
{
    auto predecessors = findPredecessors(node, map, goal_i, goal_j, agentId, occupiedNodes,
        constraints, withCAT, CAT);
    NodeType* bestPtr = nullptr;
    int bestConflictsCount = CN_INFINITY;
    for (auto& pred : predecessors) {
        int prevConv = pred.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        if (closeConv.find(prevConv) != closeConv.end()) {
            auto it = sortByIndex.find(prevConv);
            assert(it != sortByIndex.end() && it->second.g + 1 == node.g);
            int newConflictsCount = getPredConflictsCount(node, it->second, CAT);
            if (newConflictsCount < bestConflictsCount) {
                bestConflictsCount = newConflictsCount;
                bestPtr = &it->second;
            }
        }
    }
    return bestPtr;
}

template<typename NodeType>
bool ReplanningAStar<NodeType>::fillParents(NodeType &node, const Map &map,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT)
{
    if (node.g == 0) {
        node.parent = nullptr;
        return true;
    }

    NodeType* parent = getBestParentPtr(node, map, goal_i, goal_j, agentId,
        occupiedNodes, constraints, withCAT, CAT);

    assert(parent != nullptr);

    if (parent == nullptr) {
        std::cout << agentId << " " << node.i << " " << node.j << " " << node.g << std::endl;
        return false;
    }

    node.parent = parent;
    return fillParents(*parent, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
}

template<typename NodeType>
void ReplanningAStar<NodeType>::makePrimaryPath(Node &curNode, int endTime)
{
    if (withTime && endTime != -1) {
        int startTime = curNode.g;
        for (curNode.g = endTime - 1; curNode.g > startTime; --curNode.g) {
            lppath.push_front(curNode);
        }
    }
    lppath.push_front(curNode);
    if (curNode.parent != nullptr) {
        makePrimaryPath(*(curNode.parent), curNode.g);
    }
}

template<typename NodeType>
void ReplanningAStar<NodeType>::makeSecondaryPath(const Map &map)
{
    auto it = lppath.begin();
    hppath.push_back(*it);
    ++it;
    for (it; it != lppath.end(); ++it) {
        auto prevIt = it;
        --prevIt;
        auto nextIt = it;
        ++nextIt;
        if (nextIt == lppath.end() ||
            (it->i - prevIt->i) * (nextIt->j - it->j) != (it->j - prevIt->j) * (nextIt->i - it->i)) {
            hppath.push_back(*it);
        }
    }
}

template class ReplanningAStar<ReplanningAstarNode>;
template class ReplanningAStar<ReplanningFSNode>;
