#include "focal_lpa_star.h"

template<typename NodeType>
FocalLPAStar<NodeType>::FocalLPAStar(double FocalW, double HW, bool BT)
{
    hweight = HW;
    breakingties = BT;
    bool (*focalCmp)(const NodeType&, const NodeType&) = [](const NodeType &lhs, const NodeType &rhs) {
        return lhs.hc < rhs.hc || (lhs.hc == rhs.hc && lhs < rhs);
    };
    focal = std::set<NodeType, decltype (focalCmp)>(focalCmp);
    focalW = FocalW;
}

template<typename NodeType>
int FocalLPAStar<NodeType>::T = 0;

template<typename NodeType>
SearchResult FocalLPAStar<NodeType>::startSearch(const Map &map, const AgentSet &agentSet,
                                  int start_i, int start_j, int goal_i, int goal_j,
                                  bool (*isGoal)(const Node&, const Node&, const Map&, const AgentSet&),
                                  bool freshStart, bool returnPath, int startTime, int goalTime, int maxTime,
                                  const std::unordered_set<Node, NodeHash> &occupiedNodes,
                                  const ConstraintsSet &constraints,
                                  bool withCAT, const ConflictAvoidanceTable &CAT,
                                  std::chrono::steady_clock::time_point globalBegin,
                                  int globalTimeLimit)
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
    secondPhase = false;

    /*if (withCAT && freshStart) {
        open = SearchQueue<NodeType>([](const NodeType &lhs, const NodeType &rhs) {
            return std::tuple<int, int, int, int, int>(lhs.F, lhs.conflictsCount, -lhs.g, lhs.i, lhs.j) <
                    std::tuple<int, int, int, int, int>(rhs.F, rhs.conflictsCount, -rhs.g, rhs.i, rhs.j);
        });
    }*/

    bool debug = agentId == 33 && this->lppath.size() == 19;

    if (freshStart) {
        clearLists();
        sresult.numberofsteps = 0;
        cur = NodeType(start_i, start_j, nullptr, CN_INFINITY,
                 computeHFromCellToCellNew(start_i, start_i, start_i, start_j, goal_i, goal_j));
        cur.time = startTime;
        //setEndTime(cur, start_i, start_j, startTime, agentId, constraints);
        addStartNode(cur, map, CAT);
        addSuboptimalNode(cur, map, CAT);
        goalG = CN_INFINITY;
        secondPhase = true;
        maxFocalF = CN_INFINITY;
    }

    FLPANode goalNode(goal_i, goal_j);

    sresult.numberofsteps = 0;

    while(!checkOpenEmpty()) {
        ++sresult.numberofsteps;
        if (sresult.numberofsteps % 100000 == 0) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            if (elapsedMilliseconds > 300000) {
                break;
            }
        }

        auto it = sortByIndex.find(19280);
        if (it != sortByIndex.end()) {
            auto val = it->second;
            int t = 0;
            ++t;
        }

        if (sresult.numberofsteps == 10576) {
            int t = 0;
            ++t;
        }

        cur = getCur(map);

        for (int time : goalTimes) {
            goalNode.time = time;
            auto it = sortByIndex.find(goalNode.convolution(map.getMapWidth(), map.getMapHeight(), withTime));
            if (it != sortByIndex.end() && it->second < cur &&
                it->second.g == it->second.rhs && it->second.rhs != CN_INFINITY &&
                !constraints.hasFutureConstraint(goal_i, goal_j, time, agentId))
            {
                goalG = it->second.g;
                sresult.pathfound = true;
                break;
            }
        }
        if (sresult.pathfound) {
            break;
        }

        if (focal.size() == 8) {
            NodeType n1 = *focal.begin();
            NodeType n2 = *(std::next(focal.begin()));
            if (focal.key_comp()(n1, n2)) {
                int t;
            }
        }

        if (agentId == 11) {
            int t = 0;
            ++t;
        }

        if (agentId == 11 && cur.i == 31 && cur.j == 15 && cur.time == 12) {
            int t = 0;
            ++t;
        }

        if (agentId == 11 && cur.i == 21 && cur.j == 13 && cur.time == 26) {
            int t = 0;
            ++t;
        }

        if (cur.parent == nullptr && (cur.i != start_i || cur.j != start_j)) {
            int t = 0;
            ++t;
        }

        //removeCur(cur, map);

        /*if (debug) {
            std::cout << cur.i << " " << cur.j << " " << cur.g << std::endl;

            if (cur.i == 12 && cur.j == 11 && cur.g == 1000000000) {
                int t = 0;
                ++t;
            }

        }*/

        if (maxTime == -1 || cur.g < maxTime) {
            std::list<NodeType> successors = findSuccessors(cur, map, start_i, start_j,
                goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);

            bool updateSuccessors = true;

            if (cur.g > cur.rhs) {
                cur.g = cur.rhs;
                sortByIndex[cur.convolution(map.getMapWidth(), map.getMapHeight(), withTime)].g = cur.rhs;
                focalF.erase(focalF.find(cur.F));
                focal.erase(cur);
                secondPhase = true;
            } else {
                if (secondPhase) {
                    std::cout << "u" << std::endl;
                }

                cur.g = CN_INFINITY;
                sortByIndex[cur.convolution(map.getMapWidth(), map.getMapHeight(), withTime)].g = CN_INFINITY;
                successors.push_back(cur);
                updateNode(cur, map, cur, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
                if (cur.rhs != CN_INFINITY) {
                    updateSuccessors = false;
                }
            }

            if (cur.i == goal_i && cur.j == goal_j &&
                !constraints.hasFutureConstraint(cur.i, cur.j, cur.time, agentId))
            {
                goalTimes.insert(cur.time);
                goalG = cur.rhs;
                if (cur.rhs != CN_INFINITY) {
                    sresult.pathfound = true;
                    break;
                }
            }

            if (updateSuccessors) {
                for (auto& neigh : successors) {
                    updateNode(neigh, map, cur, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
                }
            }
        }
    }

    std::cout << agentId << " " << sresult.numberofsteps << std::endl;

    /*if (agentId == 9) {
        if (sortByIndex.find(2600) != sortByIndex.end() && sortByIndex[2600].hc > 0) {
            std::cout << "q1" << std::endl;
        } else {
            std::cout << "q2" << std::endl;
        }
    }*/

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    T += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
    //sresult.nodescreated = open.size() + close.size() + getFocalSize();
    //sresult.nodesexpanded = close.size();

    if (goalG == CN_INFINITY) {
        sresult.pathfound = false;
    }

    if (sresult.pathfound) {
        cur = NodeType(goal_i, goal_j, nullptr, goalG, 0);
        cur.time = goalG;
        sresult.pathlength = cur.g;
        sresult.minF = std::min(double(cur.F), getMinFocalF());
        sresult.lastNode = cur;
        if (returnPath) {
            lppath.clear();
            hppath.clear();
            fillParents(cur, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
            makePrimaryPath(cur, goalTime == -1 ? -1 : goalTime + 1);
            makeSecondaryPath(map);
            sresult.hppath = &hppath; //Here is a constant pointer
            sresult.lppath = &lppath;
        }
    }
    return sresult;
}

template<typename NodeType>
double FocalLPAStar<NodeType>::manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

template<typename NodeType>
double FocalLPAStar<NodeType>::metric(int x1, int y1, int x2, int y2) {
    return manhattanDistance(x1, y1, x2, y2);
    //return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

template<typename NodeType>
double FocalLPAStar<NodeType>::computeHFromCellToCell(int i1, int j1, int i2, int j2)
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
double FocalLPAStar<NodeType>::computeHFromCellToCellNew(int starti, int startj, int i1, int j1, int i2, int j2)
{
    if (this->perfectHeuristic != nullptr) {
        auto it = this->perfectHeuristic->find(std::make_pair(NodeType(i1, j1), NodeType(i2, j2)));
        if (it != this->perfectHeuristic->end()) {
            return it->second;
        }
    }
    return metric(i1, j1, i2, j2) * this->hweight;
        //0.001 / (std::min(std::abs(i1 - starti), std::abs(j1 - j2)) + 1);
}

int addWithInfinityCheck(int lhs, int rhs)
{
    return (lhs == CN_INFINITY) ? CN_INFINITY : lhs + rhs;
}

template<typename NodeType>
void FocalLPAStar<NodeType>::updateNode(NodeType &node, const Map &map, const NodeType &prevNode,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT, bool ignorePreds)
{
    int conv = node.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
    int bestG = CN_INFINITY;
    int bestHC = CN_INFINITY;
    int curG = CN_INFINITY;
    int curHC = CN_INFINITY;
    auto it = sortByIndex.find(conv);

    if (it == sortByIndex.end()) {
        bestG = addWithInfinityCheck(prevNode.g, 1);
        bestHC = addWithInfinityCheck(prevNode.hc, CAT.getAgentsCount(node, prevNode));
        //nodeTimes[node.convolution(map.getMapWidth(), map.getMapHeight(), false)].insert(node.time);
    } else {
        if (!ignorePreds) {
            auto predecessors = findPredecessors(node, map, goal_i, goal_j, agentId, occupiedNodes,
                constraints, withCAT, CAT);

            for (auto& pred : predecessors) {
                int prevConv = pred.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
                auto prevIt = sortByIndex.find(prevConv);
                if (prevIt != sortByIndex.end() && prevIt->second.g < bestG) {
                    bestG = addWithInfinityCheck(prevIt->second.g, 1);
                    bestHC = addWithInfinityCheck(prevIt->second.hc, CAT.getAgentsCount(node, pred));
                }
            }
        }
        auto& s = it->second;
        curG = it->second.g;
        curHC = it->second.hc;
        if (focal.find(it->second) != focal.end()) {
            focalF.erase(focalF.find(it->second.F));
            focal.erase(it->second);
        } if (open.find(it->second) != open.end()) {
            open.erase(it->second);
        }
        sortByIndex.erase(it);
    }
    node.rhs = bestG;
    if (secondPhase || curHC == CN_INFINITY) {
        node.hc = bestHC;
    } else {
        node.hc = curHC;
    }
    node.g = curG;
    node.setF();
    sortByIndex[conv] = node;
    if (node.rhs != node.g) {
        open.insert(node);
    }
}

template<typename NodeType>
void FocalLPAStar<NodeType>::processConstraint(const Constraint& constraint, const Map &map,
    int start_i, int start_j, int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT)
{
    if (sortByIndex.empty()) {
        return;
    }
    secondPhase = false;

    NodeType node(constraint.i, constraint.j, nullptr, constraint.time,
        computeHFromCellToCellNew(start_i, start_j, constraint.i, constraint.j, goal_i, goal_j));

    std::vector<int> times;

    updateNode(node, map, node, goal_i, goal_j, agentId, occupiedNodes,
        constraints, withCAT, CAT, constraint.prev_i == -1);

    if (!focalF.empty() && !this->open.empty() && open.begin()->F < *focalF.begin()) {
        auto it = focal.begin();
        while (it != focal.end()) {
            if (it->F > open.begin()->F * focalW) {
                open.insert(*it);
                focalF.erase(focalF.find(it->F));
                it = focal.erase(it);
            } else {
                ++it;
            }
        }
    }

    if (constraint.prev_i == -1) {
        std::list<NodeType> successors = findSuccessors(node, map, start_i, start_j,
            goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
        for (auto& neigh : successors) {
            updateNode(neigh, map, node, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
        }
    }
}

template<typename NodeType>
std::list<NodeType> FocalLPAStar<NodeType>::findSuccessors(const NodeType &curNode, const Map &map,
                                        int start_i, int start_j, int goal_i, int goal_j, int agentId,
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
                double newh = computeHFromCellToCellNew(start_i, start_j, newi, newj, goal_i, goal_j);
                NodeType neigh(newi, newj, nullptr, addWithInfinityCheck(curNode.g, 1), newh);
                //neigh.conflictsCount = CAT.getAgentsCount(neigh, curNode);
                neigh.time = curNode.time + 1;
                createSuccessorsFromNode(curNode, neigh, successors, agentId, constraints, CAT,
                                         neigh.i == goal_i && neigh.j == goal_j);
            }
        }
    }
    return successors;
}

template<typename NodeType>
std::list<NodeType> FocalLPAStar<NodeType>::findPredecessors(const NodeType &curNode, const Map &map,
                                        int goal_i, int goal_j, int agentId,
                                        const std::unordered_set<Node, NodeHash> &occupiedNodes,
                                        const ConstraintsSet &constraints,
                                        bool withCAT, const ConflictAvoidanceTable &CAT)
{
    if (constraints.hasNodeConstraint(curNode.i, curNode.j, curNode.time, agentId)) {
        return {};
    }
    std::list<NodeType> predecessors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            if ((di == 0 || dj == 0) && (canStay() || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes)) {
                NodeType neigh(newi, newj, nullptr, addWithInfinityCheck(curNode.g, -1), 0);
                neigh.time = curNode.time - 1;
                if (!constraints.hasNodeConstraint(neigh.i, neigh.j, neigh.time, agentId) &&
                    !constraints.hasEdgeConstraint(curNode.i, curNode.j, curNode.time, agentId, neigh.i, neigh.j))
                {
                    predecessors.push_back(neigh);
                }
            }
        }
    }
    return predecessors;
}

template<typename NodeType>
void FocalLPAStar<NodeType>::clearLists() {
    open.clear();
    focal.clear();
    focalF.clear();
    sortByIndex.clear();
}

template<typename NodeType>
void FocalLPAStar<NodeType>::addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    open.insert(node);
    sortByIndex[node.convolution(map.getMapWidth(), map.getMapHeight(), withTime)] = node;
}

template<typename NodeType>
bool FocalLPAStar<NodeType>::checkOpenEmpty() {
    return open.empty() && focal.empty();
}

template<typename NodeType>
NodeType FocalLPAStar<NodeType>::getCur(const Map& map) {
    if (!this->open.empty()) {
        double minF = open.begin()->F;
        if (!focalF.empty()) {
            minF = std::min(minF, *focalF.begin());
        }
        auto it = open.begin();
        while (it != open.end() && it->F <= minF * focalW) {
            focal.insert(*it);
            focalF.insert(it->F);
            it = open.erase(it);
        }
    }
    NodeType cur = *focal.begin();
    return cur;
}

/*template<typename NodeType>
void FocalLPAStar<NodeType>::removeCur(const NodeType& cur, const Map& map) {
    focal.erase(map, cur, this->withTime);
    focalF.erase(focalF.find(cur.F));
}*/

/*template<typename NodeType>
bool FocalLPAStar<NodeType>::updateFocal(const NodeType& neigh, const Map& map) {
    NodeType old = focal.getByIndex(map, neigh, this->withTime);
    if (old.i != -1) {
        if (focal.insert(map, neigh, this->withTime, true, old)) {
            focalF.erase(focalF.find(old.F));
            focalF.insert(neigh.F);
        }
        return true;
    }
    return false;
}*/

template<typename NodeType>
double FocalLPAStar<NodeType>::getMinFocalF() {
    if (focalF.empty()) {
        return CN_INFINITY;
    }
    return *focalF.begin();
}

template<typename NodeType>
bool FocalLPAStar<NodeType>::checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.g == goalTime;
}

template<typename NodeType>
void FocalLPAStar<NodeType>::setHC(NodeType &neigh, const NodeType &cur,
                                  const ConflictAvoidanceTable &CAT, bool isGoal) {
    neigh.hc = cur.hc + neigh.conflictsCount;
    /*if (isGoal) {
        addFutureConflicts(neigh, CAT);
    }*/
}

template<typename NodeType>
void FocalLPAStar<NodeType>::createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                       int agentId, const ConstraintsSet &constraints,
                                       const ConflictAvoidanceTable &CAT, bool isGoal) {
    if (!constraints.hasNodeConstraint(neigh.i, neigh.j, neigh.time, agentId) &&
        !constraints.hasEdgeConstraint(neigh.i, neigh.j, neigh.time, agentId, cur.i, cur.j))
    {
        //setHC(neigh, cur, CAT, isGoal);
        successors.push_back(neigh);
    }
}

template<typename NodeType>
void FocalLPAStar<NodeType>::fillParents(NodeType &node, const Map &map,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT)
{
    if (node.time == 0) {
        node.parent = nullptr;
        return;
    }

    if (agentId == 11 && node.i == 25 && node.j == 15) {
        int t = 0;
        ++t;
    }

    auto predecessors = findPredecessors(node, map, goal_i, goal_j, agentId, occupiedNodes,
        constraints, withCAT, CAT);
    NodeType* bestPtr;
    int bestHC = CN_INFINITY;
    for (auto& pred : predecessors) {
        int prevConv = pred.convolution(map.getMapWidth(), map.getMapHeight(), withTime);
        auto it = sortByIndex.find(prevConv);

        if (it != sortByIndex.end()) {
            auto& val = it->second;
            int t = 0;
            ++t;
        }

        if (it != sortByIndex.end() && it->second.g + 1 == node.g) {
            int newHC = std::max(it->second.hc, CAT.getNodeAgentsCount(it->second)) +
                CAT.getAgentsCount(node, pred);
            if (newHC < bestHC) {
                bestHC = newHC;
                bestPtr = &it->second;
            }
        }
    }
    node.parent = bestPtr;
    fillParents(*bestPtr, map, goal_i, goal_j, agentId, occupiedNodes,
        constraints, withCAT, CAT);
}

template<typename NodeType>
void FocalLPAStar<NodeType>::makePrimaryPath(Node &curNode, int endTime)
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
void FocalLPAStar<NodeType>::makeSecondaryPath(const Map &map)
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

template class FocalLPAStar<FLPANode>;

