#include "isearch.h"

ISearch::ISearch(bool WithTime)
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    withTime = WithTime;
}

ISearch::~ISearch(void) {}

/*int ISearch::convolution(int i, int j, const Map &map, int time, bool withTime) {
    int res = withTime ? map.getMapWidth() * map.getMapHeight() * time : 0;
    return res + i * map.getMapWidth() + j;
}*/

bool Node::breakingties;

SearchResult ISearch::startSearch(const Map &map, const AgentSet &agentSet,
                                  int start_i, int start_j, int goal_i, int goal_j,
                                  bool (*isGoal)(const Node&, const Node&, const Map&, const AgentSet&),
                                  bool freshStart, bool returnPath, int startDepth, int goalDepth, int maxDepth,
                                  bool withFocal, double focalW,
                                  const std::unordered_set<Node> &occupiedNodes,
                                  const ConstraintsSet &constraints,
                                  const ConflictAvoidanceTable &CAT)
{
    sresult.pathfound = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (goalDepth != -1) {
        maxDepth = goalDepth;
    }
    Node cur;
    int fin_i, fin_j, fin_depth;
    int agentId = -1;
    if (agentSet.isOccupied(start_i, start_j)) {
        agentId = agentSet.getAgentId(start_i, start_j);
    }
    auto focalCmp = [](const Node &lhs, const Node &rhs) {
        return lhs.hc < rhs.hc || lhs.hc == rhs.hc && lhs < rhs;
    };
    SearchQueue focal(focalCmp);
    std::multiset<double> focalF;

    if (freshStart) {
        open.clear();
        close.clear();

        Node::breakingties = breakingties;
        sresult.numberofsteps = 0;
        cur = Node(start_i, start_j, nullptr, 0,
                 computeHFromCellToCell(start_i, start_j, goal_i, goal_j), startDepth);

        open.insert(map, cur, withTime);
    }

    while(!open.empty() || !focal.empty()) {
        ++sresult.numberofsteps;
        if (sresult.numberofsteps % 100000 == 0) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            if (elapsedMilliseconds > 10000) {
                break;
            }
        }

        if (withFocal) {
            if (!open.empty()) {
                double minF = open.getFront().F;
                if (!focalF.empty()) {
                    minF = std::min(minF, *focalF.begin());
                }
                open.moveByThreshold(focal, minF * focalW, map, withTime, focalF);
            }
            cur = focal.getFront();
            focal.erase(map, cur, withTime);
            focalF.erase(cur.F);
        } else {
            cur = open.getFront();
            open.erase(map, cur, withTime);
        }
        close[SearchQueue::convolution(cur.i, cur.j, map, cur.depth, withTime)] = cur;

        Node *curPtr = &(close.find(SearchQueue::convolution(cur.i, cur.j, map, cur.depth, withTime))->second);
        if ((isGoal != nullptr && isGoal(Node(start_i, start_j), cur, map, agentSet)) ||
            (isGoal == nullptr && cur.i == goal_i && cur.j == goal_j) &&
             (goalDepth == -1 || cur.depth == goalDepth)) {
            if (!constraints.hasFutureConstraint(cur.i, cur.j, cur.depth, agentId)) {
                if (!freshStart) {
                    freshStart = true;
                } else {
                    sresult.pathfound = true;
                    fin_i = cur.i;
                    fin_j = cur.j;
                    fin_depth = cur.depth;
                    break;
                }
            }
        }

        if (maxDepth == -1 || cur.depth < maxDepth) {
            std::list<Node> successors = findSuccessors(cur, map, goal_i, goal_j, agentId, occupiedNodes,
                                                        constraints, CAT);
            for (auto neigh : successors) {
                if (close.find(SearchQueue::convolution(neigh.i, neigh.j, map, neigh.depth, withTime)) == close.end()) {
                    neigh.parent = curPtr;
                    if (withFocal) {
                        Node old = focal.getByIndex(map, neigh, withTime);
                        if (old.i != -1) {
                            focalF.erase(old.F);
                            focalF.insert(neigh.F);
                            focal.insert(map, neigh, withTime, true, old);
                            continue;
                        }
                    }
                    open.insert(map, neigh, withTime);
                }
            }
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
    sresult.nodescreated = open.size() + close.size();
    if (sresult.pathfound) {
        auto it = close.find(SearchQueue::convolution(fin_i, fin_j, map, fin_depth, withTime));
        sresult.pathlength = it->second.g;
        sresult.minF = it->second.F;
        if (!focalF.empty()) {
            sresult.minF = std::min(sresult.minF, *focalF.begin());
        }
        sresult.lastNode = it->second;
        if (returnPath) {
            lppath.clear();
            hppath.clear();
            makePrimaryPath(it->second);
            makeSecondaryPath(map);
            sresult.hppath = &hppath; //Here is a constant pointer
            sresult.lppath = &lppath;
        }
    }
    return sresult;
}

std::list<Node> ISearch::findSuccessors(const Node &curNode, const Map &map,
                                        int goal_i, int goal_j, int agentId,
                                        const std::unordered_set<Node> &occupiedNodes,
                                        const ConstraintsSet &constraints,
                                        const ConflictAvoidanceTable &CAT)
{
    std::list<Node> successors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            int depth = curNode.depth;
            if ((di == 0 || dj == 0) && (withTime || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes) &&
                    !constraints.hasNodeConstraint(newi, newj, curNode.depth + 1, agentId) &&
                    !constraints.hasEdgeConstraint(newi, newj, curNode.depth + 1, agentId, curNode.i, curNode.j)) {
                int newh = computeHFromCellToCell(newi, newj, goal_i, goal_j);
                int conflictsCount = CAT.getAgentsCount(newi, newj, curNode.depth + 1, map);
                Node neigh(newi, newj, nullptr, curNode.g + 1,
                           newh, curNode.depth + 1, conflictsCount, curNode.hc + conflictsCount);
                successors.push_back(neigh);
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(const Node &curNode)
{
    lppath.push_front(curNode);
    if (curNode.parent != nullptr) {
        makePrimaryPath(*(curNode.parent));
    }
}

void ISearch::makeSecondaryPath(const Map &map)
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

void ISearch::getPerfectHeuristic(const Map &map, const AgentSet &agentSet) {
    bool oldWithTime = withTime;
    withTime = false;
    perfectHeuristic.clear();
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        close.clear();
        Node goal = agentSet.getAgent(i).getGoalPosition();
        std::queue<Node> queue;
        queue.push(goal);

        while (!queue.empty()) {
            Node cur = queue.front();
            queue.pop();
            if (close.find(SearchQueue::convolution(cur.i, cur.j, map)) != close.end()) {
                continue;
            }
            close[SearchQueue::convolution(cur.i, cur.j, map)] = cur;
            perfectHeuristic[std::make_pair(cur, goal)] = cur.g;
            std::list<Node> successors = findSuccessors(cur, map);
            for (auto neigh : successors) {
                queue.push(neigh);
            }
        }
    }
    withTime = oldWithTime;
}
