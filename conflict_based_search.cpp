#include "conflict_based_search.h"

int CBSNode::curId = 0;

ConflictBasedSearch::ConflictBasedSearch()
{
    search = nullptr;
}

ConflictBasedSearch::ConflictBasedSearch (ISearch *Search)
{
    search = Search;
}

ConflictBasedSearch::~ConflictBasedSearch()
{
    if (search)
        delete search;
}

std::list<Node> ConflictBasedSearch::getNewPath(const Map &map, const AgentSet &agentSet, const Agent &agent,
                                                const Constraint &constraint, const ConstraintsSet &constraints,
                                                const std::list<Node>::iterator pathStart,
                                                const std::list<Node>::iterator pathEnd,
                                                const ConflictAvoidanceTable &CAT) {
    std::vector<Constraint> positiveConstraints = constraints.getPositiveConstraints();
    std::sort(positiveConstraints.begin(), positiveConstraints.end(),
            [](const Constraint &lhs, const Constraint &rhs){
        return lhs.time < rhs.time || lhs.time == rhs.time && lhs.prev_i != -1 && rhs.prev_i == -1;
    });
    int startTime = 0, endTime = -1;
    Node start = agent.getStartPosition(), end = agent.getGoalPosition();
    int i;
    for (i = 0; i < positiveConstraints.size(); ++i) {
        if (positiveConstraints[i].time >= constraint.time) {
            if (positiveConstraints[i].prev_i == -1) {
                end = Node(positiveConstraints[i].i, positiveConstraints[i].j);
                endTime = positiveConstraints[i].time;
            } else {
                if (positiveConstraints[i].time == constraint.time &&
                    positiveConstraints[i].i == constraint.i && positiveConstraints[i].j == constraint.j &&
                    (constraint.prev_i == -1 || constraint.prev_i == positiveConstraints[i].prev_i &&
                     constraint.prev_j == positiveConstraints[i].prev_j)) {
                    return std::list<Node>();
                }
                end = Node(positiveConstraints[i].prev_i, positiveConstraints[i].prev_j);
                endTime = positiveConstraints[i].time - 1;
            }
            if (i > 0) {
                start = Node(positiveConstraints[i - 1].i, positiveConstraints[i - 1].j);
                startTime = positiveConstraints[i - 1].time;
            }
            break;
        }
    }
    if (i == positiveConstraints.size() && i > 0) {
        start = Node(positiveConstraints[i - 1].i, positiveConstraints[i - 1].j);
        startTime = positiveConstraints[i - 1].time;
    }

    SearchResult searchResult = search->startSearch(map, agentSet, start.i, start.j, end.i, end.j, nullptr,
                                                    true, true, startTime, endTime, -1, {}, constraints, CAT);
    if (!searchResult.pathfound) {
        return std::list<Node>();
    }
    std::list<Node> res;
    auto it1 = searchResult.lppath->begin();
    int time = 0;
    for (auto it2 = pathStart; it2 != pathEnd; ++it2) {
        if (time < startTime || (endTime != -1 && time > endTime)) {
            res.push_back(*it2);
        } else {
            res.push_back(*it1);
            ++it1;
        }
        ++time;
    }
    for (; it1 != searchResult.lppath->end(); ++it1) {
        res.push_back(*it1);
    }
    return res;
}

CBSNode ConflictBasedSearch::createNode(const Map &map, const AgentSet &agentSet, const Config &config,
                                        const Conflict &conflict, const std::vector<int> &costs,
                                        ConstraintsSet &constraints, int id1, int id2,
                                        const Node &pos1, const Node &pos2,
                                        std::vector<std::list<Node>::iterator> &starts,
                                        std::vector<std::list<Node>::iterator> &ends,
                                        ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
                                        std::vector<MDD> &mdds,
                                        CBSNode *parentPtr) {
    Constraint constraint(pos1.i, pos1.j, conflict.time, id1);
    if (conflict.edgeConflict) {
        constraint.prev_i = pos2.i;
        constraint.prev_j = pos2.j;
    }

    ConstraintsSet agentConstraints = constraints.getAgentConstraints(id1);
    agentConstraints.addConstraint(constraint);

    Agent agent = agentSet.getAgent(id1);
    if (config.withCAT) {
        CAT.removeAgentPath(starts[id1], ends[id1], map);
    }

    std::list<Node> newPath = getNewPath(map, agentSet, agent, constraint, agentConstraints,
                                         starts[id1], ends[id1], CAT);

    if (config.withCAT) {
        CAT.addAgentPath(starts[id1], ends[id1], map);
    }
    if (newPath.empty()) {
        return CBSNode(false);
    }

    CBSNode node;
    node.paths[id1] = newPath;
    for (int i = 0; i < costs.size(); ++i) {
        if (i == id1) {
            node.cost += newPath.size() - 1;
        } else {
            node.cost += costs[i];
        }
    }
    node.constraint = constraint;
    node.parent = parentPtr;

    MDD oldMDD;
    if (config.withCardinalConflicts) {
        node.mdds[id1] = MDD(map, agentSet, search, id1, newPath.size() - 1, agentConstraints);
        oldMDD = mdds[id1];
        mdds[id1] = node.mdds[id1];
    }

    auto oldStart = starts[id1], oldEnd = ends[id1];
    starts[id1] = node.paths[id1].begin();
    ends[id1] = node.paths[id1].end();
    if (config.storeConflicts) {
        ConflictSet agentConflicts = findConflict<std::list<Node>::iterator>(starts, ends, id1, true, true, mdds);
        node.conflictSet = conflictSet;
        node.conflictSet.replaceAgentConflicts(id1, agentConflicts);
    }

    if (config.withMatchingHeuristic) {
        node.H = node.conflictSet.getMatchingHeuristic();
    }

    starts[id1] = oldStart;
    ends[id1] = oldEnd;
    if (config.withCardinalConflicts) {
        mdds[id1] = oldMDD;
    }

    node.G = node.H + node.cost;
    return node;
}

MultiagentSearchResult ConflictBasedSearch::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    // std::cout << agentSet.getAgentCount() << std::endl;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (config.withPerfectHeuristic) {
        search->getPerfectHeuristic(map, agentSet);
    }

    CBSNode root;
    int agentCount = agentSet.getAgentCount();
    std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
    std::vector<MDD> mdds;
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        Agent agent = agentSet.getAgent(i);
        Astar astar(false);
        SearchResult searchResult = astar.startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                        agent.getGoal_i(), agent.getGoal_j());
        if (!searchResult.pathfound) {
            std::cout << "fail" << std::endl;
        }
        root.cost += searchResult.pathlength;
        root.paths[i] = *searchResult.lppath;
        if (config.withCardinalConflicts) {
            root.mdds[i] = MDD(map, agentSet, search, i, searchResult.pathlength);
        }
        starts[i] = root.paths[i].begin();
        ends[i] = root.paths[i].end();
        mdds.push_back(root.mdds[i]);
    }
    if (config.storeConflicts) {
        root.conflictSet = findConflict<std::list<Node>::iterator>(starts, ends, -1, true, true, mdds);
    }

    MultiagentSearchResult result(false);
    std::set<CBSNode> open = {root};
    std::list<CBSNode> close;

    while (!open.empty()) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            result.pathfound = false;
            break;
        }

        CBSNode cur = *open.begin();
        open.erase(cur);
        close.push_back(cur);

        std::vector<int> costs(agentCount, 0);
        std::vector<bool> agentFound(agentCount, false);
        std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
        ConstraintsSet constraints;
        ConflictAvoidanceTable CAT;
        std::vector<MDD> mdds(agentCount);
        for (CBSNode *ptr = &cur; ptr != nullptr; ptr = ptr->parent) {
            for (auto it = ptr->paths.begin(); it != ptr->paths.end(); ++it) {
                if (!agentFound[it->first]) {
                    starts[it->first] = it->second.begin();
                    ends[it->first] = it->second.end();
                    costs[it->first] = it->second.size() - 1;
                    agentFound[it->first] = true;

                    if (config.withCAT) {
                        CAT.addAgentPath(starts[it->first], ends[it->first], map);
                    }

                    if (config.withCardinalConflicts) {
                        mdds[it->first] = ptr->mdds.find(it->first)->second;
                    }
                }
            }
            if (ptr->id != 0) {
                constraints.addConstraint(ptr->constraint);
                if (ptr->hasPositiveConstraint) {
                    constraints.addConstraint(ptr->positiveConstraint);
                }
            }
        }

        ConflictSet conflictSet;
        if (!config.storeConflicts) {
            conflictSet = findConflict<std::list<Node>::iterator>(starts, ends, -1, false,
                                                                  config.withCardinalConflicts, mdds);
        } else {
            conflictSet = cur.conflictSet;
        }
        if (conflictSet.empty()) {
            agentsPaths.resize(agentCount);
            for (int i = 0; i < agentCount; ++i) {
                for (auto it = starts[i]; it != ends[i]; ++it) {
                    agentsPaths[i].push_back(*it);
                }
            }
            result.agentsPaths = &agentsPaths;
            result.pathfound = true;

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            result.time = static_cast<double>(elapsedMilliseconds) / 1000;
            break;
        }

        Conflict conflict = conflictSet.getBestConflict();
        if (config.withDisjointSplitting &&
                mdds[conflict.id1].getLayerSize(conflict.time) < mdds[conflict.id2].getLayerSize(conflict.time)) {
            std::swap(conflict.id1, conflict.id2);
            std::swap(conflict.pos1, conflict.pos2);
        }

        std::vector<CBSNode> children;
        CBSNode child1 = createNode(map, agentSet, config, conflict, costs, constraints,
                                   conflict.id1, conflict.id2, conflict.pos2, conflict.pos1,
                                   starts, ends, CAT, conflictSet, mdds, &close.back());
        if (child1.pathFound) {
            children.push_back(child1);
        }
        CBSNode child2 = createNode(map, agentSet, config, conflict, costs, constraints,
                           conflict.id2, conflict.id1, conflict.pos1, conflict.pos2,
                           starts, ends, CAT, conflictSet, mdds, &close.back());
        if (child2.pathFound) {
            children.push_back(child2);
        }

        bool bypass = false;
        if (children.size() == 2) {
            for (auto child : children) {
                if (child.pathFound && config.withBypassing && cur.cost == child.cost &&
                        conflictSet.getConflictCount() > child.conflictSet.getConflictCount()) {
                    open.insert(child);
                    bypass = true;
                    break;
                }
            }
        }
        if (!bypass && !children.empty()) {
            if (config.withDisjointSplitting) {
                children[0].hasPositiveConstraint = true;
                int id1 = children[0].paths.begin()->first;
                int id2 = conflict.id1 == id1 ? conflict.id2 : conflict.id1;
                Constraint positiveConstraint;
                if (conflict.edgeConflict) {
                    positiveConstraint = Constraint(children[0].constraint.prev_i, children[0].constraint.prev_j,
                            conflict.time, id2, children[0].constraint.i, children[0].constraint.j);
                } else {
                    positiveConstraint = Constraint(children[0].constraint.i, children[0].constraint.j, conflict.time, id2);
                }
                positiveConstraint.positive = true;
                children[0].positiveConstraint = positiveConstraint;
            }

            for (auto child : children) {
                open.insert(child);
            }
        }
    }
    // std::cout << close.size() + open.size() << std::endl;
    return result;
}

void ConflictBasedSearch::clear() {
    agentsPaths.clear();
}

