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

CBSNode ConflictBasedSearch::createNode(const Map &map, const AgentSet &agentSet, const Conflict &conflict,
                                        const std::vector<int> &costs, ConstraintsSet &constraints, int agentId,
                                        const Node &pos1, const Node &pos2,
                                        std::list<Node>::iterator pathStart, std::list<Node>::iterator pathEnd,
                                        ConflictAvoidanceTable& CAT, bool withCAT,
                                        CBSNode *parentPtr) {
    Constraint constraint(pos1.i, pos1.j, conflict.time, agentId);
    if (conflict.edgeConflict) {
        constraint.prev_i = pos2.i;
        constraint.prev_j = pos2.j;
    }

    ConstraintsSet agentConstraints = constraints.getAgentConstraints(agentId);
    agentConstraints.addConstraint(constraint);
    Agent agent = agentSet.getAgent(agentId);
    if (withCAT) {
        CAT.removeAgentPath(pathStart, pathEnd, map);
    }
    SearchResult searchResult = search->startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                    agent.getGoal_i(), agent.getGoal_j(), nullptr,
                                                    true, true, -1, {}, agentConstraints, CAT);
    if (withCAT) {
        CAT.addAgentPath(pathStart, pathEnd, map);
    }

    CBSNode node;
    node.paths[agentId] = *searchResult.lppath;
    for (int i = 0; i < costs.size(); ++i) {
        if (i == agentId) {
            node.cost += searchResult.lppath->size() - 1;
        } else {
            node.cost += costs[i];
        }
    }
    node.constraint = constraint;
    node.parent = parentPtr;
    return node;
}

MultiagentSearchResult ConflictBasedSearch::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    // std::cout << agentSet.getAgentCount() << std::endl;

    CBSNode root;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (config.withPerfectHeuristic) {
        search->getPerfectHeuristic(map, agentSet);
    }
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

        int agentCount = agentSet.getAgentCount();
        std::vector<int> costs(agentCount, 0);
        std::vector<bool> agentFound(agentCount, false);
        std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
        ConstraintsSet constraints;
        ConflictAvoidanceTable CAT;
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
                }
            }
            if (ptr->id != 0) {
                constraints.addConstraint(ptr->constraint);
            }
        }

        Conflict conflict = Conflict::findConflict<std::list<Node>::iterator>(starts, ends);
        if (!conflict.conflictFound) {
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

        open.insert(createNode(map, agentSet, conflict, costs, constraints, conflict.id1, conflict.pos2, conflict.pos1,
                               starts[conflict.id1], ends[conflict.id1], CAT, config.withCAT, &close.back()));
        open.insert(createNode(map, agentSet, conflict, costs, constraints, conflict.id2, conflict.pos1, conflict.pos2,
                               starts[conflict.id2], ends[conflict.id2], CAT, config.withCAT, &close.back()));
    }
    return result;
}

void ConflictBasedSearch::clear() {
    agentsPaths.clear();
}

