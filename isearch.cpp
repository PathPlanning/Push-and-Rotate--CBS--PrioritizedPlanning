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
                                  const std::unordered_set<Node> &occupiedNodes,
                                  const ConstraintsSet &constraints,
                                  bool withCAT, const ConflictAvoidanceTable &CAT)
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

    if (freshStart) {
        clearLists();

        Node::breakingties = breakingties;
        sresult.numberofsteps = 0;
        cur = Node(start_i, start_j, nullptr, 0,
                 computeHFromCellToCell(start_i, start_j, goal_i, goal_j), startTime);
        cur.endTime = getEndTime(start_i, start_j, startTime, agentId, constraints);
        open.insert(map, cur, withTime, withIntervals);
    }

    while(!checkOpenEmpty()) {
        ++sresult.numberofsteps;
        /*if (sresult.numberofsteps % 100000 == 0) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            if (elapsedMilliseconds > 10000) {
                break;
            }
        }*/

        cur = getCur(map);
        close[SearchQueue::convolution(cur, map, withTime, withIntervals)] = cur;

        Node *curPtr = &(close.find(SearchQueue::convolution(cur, map, withTime, withIntervals))->second);
        if (((isGoal != nullptr && isGoal(Node(start_i, start_j), cur, map, agentSet)) ||
            (isGoal == nullptr && cur.i == goal_i && cur.j == goal_j)) &&
            !constraints.hasFutureConstraint(cur.i, cur.j, cur.time, agentId) &&
            checkGoal(cur, goalTime, agentId, constraints)) {
            if (!freshStart) {
                freshStart = true;
            } else {
                sresult.pathfound = true;
                break;
            }
        }

        if (maxTime == -1 || cur.time < maxTime) {
            std::list<Node> successors = findSuccessors(cur, map, goal_i, goal_j, agentId, occupiedNodes,
                                                        constraints, withCAT, CAT);
            for (auto neigh : successors) {
                if (close.find(SearchQueue::convolution(neigh, map, withTime, withIntervals)) == close.end()) {
                    neigh.parent = curPtr;
                    if (!updateFocal(neigh, map)) {
                        open.insert(map, neigh, withTime, withIntervals);
                    }
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
        sresult.minF = std::min(cur.F, getMinFocalF());
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
                                        bool withCAT, const ConflictAvoidanceTable &CAT)
{
    std::list<Node> successors;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            int newi = curNode.i + di, newj = curNode.j + dj;
            if ((di == 0 || dj == 0) && ((withTime && !withIntervals) || di != 0 || dj != 0) && map.CellOnGrid(newi, newj) &&
                    map.CellIsTraversable(newi, newj, occupiedNodes)) {
                int newh = computeHFromCellToCell(newi, newj, goal_i, goal_j);
                Node neigh(newi, newj, nullptr, curNode.g + 1, newh, curNode.time + 1);
                int conflictsCount = CAT.getAgentsCount(neigh);
                if (withCAT) {
                    neigh.conflictsCount = conflictsCount;
                }
                neigh.hc = curNode.hc + conflictsCount;
                createSuccessorsFromNode(curNode, neigh, successors, agentId, constraints, CAT);
            }
        }
    }
    return successors;
}

void ISearch::clearLists() {
    open.clear();
    close.clear();
}

bool ISearch::checkOpenEmpty() {
    return open.empty();
}

Node ISearch::getCur(const Map& map) {
    Node cur = open.getFront();
    open.erase(map, cur, withTime, withIntervals);
    return cur;
}

bool ISearch::updateFocal(const Node& neigh, const Map& map) {
    return false;
}

double ISearch::getMinFocalF() {
    return CN_INFINITY;
}

int ISearch::getEndTime(int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) {
    return startTime;
}

bool ISearch::checkGoal(const Node &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.time == goalTime;
}

void ISearch::createSuccessorsFromNode(const Node &cur, Node &neigh, std::list<Node> &successors,
                                       int agentId, const ConstraintsSet &constraints,
                                       const ConflictAvoidanceTable &CAT) {
    if (!constraints.hasNodeConstraint(neigh.i, neigh.j, neigh.time, agentId) &&
        !constraints.hasEdgeConstraint(neigh.i, neigh.j, neigh.time, agentId, cur.i, cur.j)) {
        successors.push_back(neigh);
    }
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

bool ISearch::getWithIntervals() {
    return withIntervals;
}


bool ISearch::setWithIntervals(bool val) {
    withIntervals = val;
}
