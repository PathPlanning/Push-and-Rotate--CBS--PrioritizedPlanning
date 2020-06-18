#include "isearch.h"

ISearch::ISearch(bool WithTime)
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    withTime = WithTime;
}

ISearch::~ISearch(void) {}

bool Node::breakingties;
int ISearch::T = 0;

SearchResult ISearch::startSearch(const Map &map, const AgentSet &agentSet,
                                  int start_i, int start_j, int goal_i, int goal_j,
                                  bool (*isGoal)(const Node&, const Node&, const Map&, const AgentSet&),
                                  bool freshStart, bool returnPath, int startTime, int goalTime, int maxTime,
                                  bool withFocal, double focalW,
                                  const std::unordered_set<Node> &occupiedNodes,
                                  const ConstraintsSet &constraints,
                                  const ConflictAvoidanceTable &CAT)
{
    sresult.pathfound = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (goalTime != -1) {
        maxTime = goalTime;
    }
    Node cur;
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
                 computeHFromCellToCell(start_i, start_j, goal_i, goal_j), startTime);
        if (withIntervals) {
            cur.endTime = constraints.getFirstConstraintTime(start_i, start_j, startTime, agentId);
            if (cur.endTime < CN_INFINITY) {
                --cur.endTime;
            }
        }

        open.insert(map, cur, withTime, withIntervals);
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
                open.moveByThreshold(focal, minF * focalW, map, focalF, withTime, withIntervals);
            }
            cur = focal.getFront();
            focal.erase(map, cur, withTime, withIntervals);
            focalF.erase(cur.F);
        } else {
            cur = open.getFront();
            open.erase(map, cur, withTime, withIntervals);
        }
        close[SearchQueue::convolution(cur, map, withTime, withIntervals)] = cur;

        Node *curPtr = &(close.find(SearchQueue::convolution(cur, map, withTime, withIntervals))->second);
        if ((isGoal != nullptr && isGoal(Node(start_i, start_j), cur, map, agentSet)) ||
            (isGoal == nullptr && cur.i == goal_i && cur.j == goal_j) &&
             (goalTime == -1 || cur.time == goalTime || (cur.time <= goalTime && cur.endTime == CN_INFINITY))) {
            if (cur.endTime == CN_INFINITY || !constraints.hasFutureConstraint(cur.i, cur.j, cur.time, agentId)) {
                if (!freshStart) {
                    freshStart = true;
                } else {
                    sresult.pathfound = true;
                    break;
                }
            }
        }

        if (maxTime == -1 || cur.time < maxTime) {
            std::list<Node> successors = findSuccessors(cur, map, goal_i, goal_j, agentId, occupiedNodes,
                                                        constraints, CAT);
            for (auto neigh : successors) {
                if (close.find(SearchQueue::convolution(neigh, map, withTime, withIntervals)) == close.end()) {
                    neigh.parent = curPtr;
                    if (withFocal) {
                        Node old = focal.getByIndex(map, neigh, withTime, withIntervals);
                        if (old.i != -1) {
                            focalF.erase(old.F);
                            focalF.insert(neigh.F);
                            focal.insert(map, neigh, withTime, withIntervals, true, old);
                            continue;
                        }
                    }
                    open.insert(map, neigh, withTime, withIntervals);
                }
            }
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    T += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
    sresult.nodescreated = open.size() + close.size();

    //std::cout << sresult.nodescreated << std::endl;

    if (sresult.pathfound) {
        sresult.pathlength = cur.g;
        sresult.minF = cur.F;
        if (!focalF.empty()) {
            sresult.minF = std::min(sresult.minF, *focalF.begin());
        }
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
            int depth = curNode.time;
            if ((di == 0 || dj == 0) && ((withTime && !withIntervals) || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes)) {
                int newh = computeHFromCellToCell(newi, newj, goal_i, goal_j);
                Node neigh(newi, newj, nullptr, curNode.g + 1, newh, curNode.time + 1);
                int conflictsCount = CAT.getAgentsCount(neigh, map);
                neigh.conflictsCount = conflictsCount;
                neigh.hc = curNode.hc + conflictsCount;
                std::vector<std::pair<int, int>> safeIntervals;

                if (withIntervals) {
                    safeIntervals = constraints.getSafeIntervals(neigh.i, neigh.j, agentId, curNode.time + 1,
                                                                 curNode.endTime + (curNode.endTime != CN_INFINITY));
                } else if (!constraints.hasNodeConstraint(newi, newj, curNode.time + 1, agentId)) {
                    safeIntervals = {std::make_pair(neigh.time, neigh.time)};
                } else {
                    continue;
                }

                for (auto interval : safeIntervals) {
                    neigh.startTime = interval.first;
                    neigh.endTime = interval.second;

                    for (neigh.time = std::max(neigh.startTime, curNode.time + 1); neigh.time <= neigh.endTime; ++neigh.time) {
                        if (!constraints.hasEdgeConstraint(newi, newj, neigh.time, agentId, curNode.i, curNode.j)) {
                            break;
                        }
                    }
                    if (neigh.time <= neigh.endTime) {
                        neigh.g = neigh.time;
                        neigh.F = neigh.g + neigh.H;
                        successors.push_back(neigh);
                    }
                }
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node &curNode, int endTime)
{
    if (withTime && endTime != -1) {
        int startTime = curNode.time;
        for (curNode.time = endTime - 1; curNode.time > startTime; --curNode.time) {
            lppath.push_front(curNode);
        }
    }
    lppath.push_front(curNode);
    if (curNode.parent != nullptr) {
        makePrimaryPath(*(curNode.parent), curNode.time);
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
            if (close.find(SearchQueue::convolution(cur, map)) != close.end()) {
                continue;
            }
            close[SearchQueue::convolution(cur, map)] = cur;
            perfectHeuristic[std::make_pair(cur, goal)] = cur.g;
            std::list<Node> successors = findSuccessors(cur, map);
            for (auto neigh : successors) {
                queue.push(neigh);
            }
        }
    }
    withTime = oldWithTime;
}
