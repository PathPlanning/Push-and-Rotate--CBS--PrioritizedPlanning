#include "conflict_based_search.h"

template<typename SearchType>
int CBSNode<SearchType>::curId = 0;

template<typename SearchType>
ConflictBasedSearch<SearchType>::ConflictBasedSearch()
{
    search = nullptr;
}

template<typename SearchType>
ConflictBasedSearch<SearchType>::ConflictBasedSearch (SearchType *Search)
{
    search = Search;
    auto focalCmp = [](const CBSNode<SearchType> &lhs, const CBSNode<SearchType> &rhs) {
        return lhs.hc < rhs.hc || lhs.hc == rhs.hc && lhs < rhs;
    };
    focal = std::set<CBSNode<SearchType>, bool (*)(const CBSNode<SearchType>&, const CBSNode<SearchType>&)>(focalCmp);
}

template<typename SearchType>
ConflictBasedSearch<SearchType>::~ConflictBasedSearch()
{
    if (search)
        delete search;
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::clear()
{
    MultiagentSearchInterface::clear();
    open.clear();
    close.clear();
    focal.clear();
    sumLb.clear();
    rootSearches.clear();
}

template<typename SearchType>
std::list<Node> ConflictBasedSearch<SearchType>::getNewPath(const Map &map, const AgentSet &agentSet, const Agent &agent,
                                                const Constraint &constraint, const ConstraintsSet &constraints,
                                                const std::list<Node>::iterator pathStart,
                                                const std::list<Node>::iterator pathEnd,
                                                bool withCAT, const ConflictAvoidanceTable &CAT,
                                                std::vector<double> &lb,
                                                std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                                                SearchType *search, bool freshStart,
                                                std::chrono::steady_clock::time_point globalBegin,
                                                int globalTimeLimit)
{
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
        freshStart, true, startTime, endTime, -1, {}, constraints, withCAT, CAT, globalBegin, globalTimeLimit);

    LLNodes.push_back(searchResult.nodescreated);
    LLExpansions.push_back(searchResult.nodesexpanded);
    if (!searchResult.pathfound) {
        return std::list<Node>();
    }

    double newLb = searchResult.minF;
    std::list<Node> res;
    auto it1 = searchResult.lppath->begin();
    int time = 0;
    for (auto it2 = pathStart; it2 != pathEnd && (endTime != -1 || it1 != searchResult.lppath->end()); ++it2) {
        if (time < startTime || (endTime != -1 && time > endTime)) {
            res.push_back(*it2);
            ++newLb;
        } else {
            res.push_back(*it1);
            ++it1;
        }
        ++time;
    }
    for (; time < startTime; ++time) {
        res.push_back(*std::prev(pathEnd));
    }
    for (; it1 != searchResult.lppath->end(); ++it1) {
        res.push_back(*it1);
    }
    lb[agent.getId()] = newLb;
    return res;
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::getState(
    const std::vector<int> &costs, int &oldCost,
    const std::vector<std::list<Node>::iterator> &starts, std::list<Node>::iterator& oldStart,
    const std::vector<std::list<Node>::iterator> &ends, std::list<Node>::iterator& oldEnd,
    const std::vector<MDD> &mdds, MDD& oldMDD,
    const std::vector<double> &lb, double& oldLb,
    int agentId, bool withMDD, bool withLb)
{
    oldCost = costs[agentId];
    oldStart = starts[agentId];
    oldEnd = ends[agentId];
    if (withMDD) {
        oldMDD = mdds[agentId];
    }
    if (withLb) {
        oldLb = lb[agentId];
    }
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::setState(
    std::vector<int> &costs, int oldCost,
    std::vector<std::list<Node>::iterator> &starts, std::list<Node>::iterator oldStart,
    std::vector<std::list<Node>::iterator> &ends, std::list<Node>::iterator oldEnd,
    std::vector<MDD> &mdds, const MDD& oldMDD,
    std::vector<double> &lb, double oldLb,
    int agentId, bool withMDD, bool withLb)
{
    costs[agentId] = oldCost;
    starts[agentId] = oldStart;
    ends[agentId] = oldEnd;
    if (withMDD) {
        mdds[agentId] = oldMDD;
    }
    if (withLb) {
        lb[agentId] = oldLb;
    }
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::createNode(
    const Map &map, const AgentSet &agentSet, const Config &config,
    const Conflict &conflict, std::vector<int> &costs,
    ConstraintsSet &constraints, int id1,
    const Node &pos1, const Node &pos2,
    std::vector<std::list<Node>::iterator> &starts,
    std::vector<std::list<Node>::iterator> &ends,
    ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
    std::vector<MDD> &mdds, std::vector<double> &lb,
    std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
    CBSNode<SearchType> *parentPtr,
    CBSNode<SearchType> &node,
    SearchType *search, bool updateNode,
    std::chrono::steady_clock::time_point globalBegin,
    int globalTimeLimit)
{
    Constraint constraint(pos1.i, pos1.j, conflict.time, id1);
    if (conflict.edgeConflict) {
        constraint.prev_i = pos2.i;
        constraint.prev_j = pos2.j;
    }

    ConstraintsSet agentConstraints = constraints.getAgentConstraints(id1);
    agentConstraints.addConstraint(constraint);

    Agent agent = agentSet.getAgent(id1);
    if (config.withCAT || config.withFocalSearch == true) {
        CAT.removeAgentPath(starts[id1], ends[id1]);
    }

    int oldCost;
    std::list<Node>::iterator oldStart;
    std::list<Node>::iterator oldEnd;
    MDD oldMDD;
    double oldLb;
    if (!updateNode) {
        getState(costs, oldCost, starts, oldStart, ends, oldEnd, mdds, oldMDD, lb, oldLb,
            id1, config.withCardinalConflicts, config.withFocalSearch);
    }

    search->processConstraint(constraint, map, agent.getGoal_i(), agent.getGoal_j(), id1,
        {}, agentConstraints, config.withCAT, CAT);

    std::list<Node> newPath = getNewPath(map, agentSet, agent, constraint, agentConstraints,
        starts[id1], ends[id1], config.withCAT, CAT, lb, LLExpansions,
        LLNodes, search, !updateNode, globalBegin, globalTimeLimit);

    if (config.withCAT || config.withFocalSearch) {
        if (updateNode && !newPath.empty()) {
            CAT.addAgentPath(newPath.begin(), newPath.end());
        } else {
            CAT.addAgentPath(starts[id1], ends[id1]);
        }
    }

    if (newPath.empty()) {
        node.pathFound = false;
        return;
    }

    if (config.searchType == CN_ST_AECBS && node.search == nullptr) {
        node.search.reset(new SearchType(std::move(*search)));
    }
    node.conflict = conflict;
    if (node.conflict.id1 != id1) {
        std::swap(node.conflict.id1, node.conflict.id2);
        std::swap(node.conflict.pos1, node.conflict.pos2);
    }

    node.paths[id1] = newPath;
    node.cost = 0;
    for (int i = 0; i < costs.size(); ++i) {
        if (i == id1) {
            node.cost += newPath.size() - 1;
            costs[i] = newPath.size() - 1;
        } else {
            node.cost += costs[i];
        }
    }
    node.constraint = constraint;
    node.parent = parentPtr;

    if (config.withCardinalConflicts) {
        node.mdds[id1] = MDD(map, agentSet, search, id1, newPath.size() - 1, agentConstraints);
        mdds[id1] = node.mdds[id1];
    }

    starts[id1] = node.paths[id1].begin();
    ends[id1] = node.paths[id1].end();
    if (config.storeConflicts) {
        ConflictSet agentConflicts = findConflict<std::list<Node>::iterator>(starts, ends, id1, true,
                                                                             config.withCardinalConflicts, mdds);
        node.conflictSet = conflictSet;
        node.conflictSet.replaceAgentConflicts(id1, agentConflicts);
    }
    if (config.withFocalSearch) {
        node.hc = node.conflictSet.getConflictingPairsCount();
        node.sumLb = 0;
        for (auto x : lb) {
            node.sumLb += x;
        }
        node.lb[id1] = lb[id1];
    }

    if (config.withMatchingHeuristic) {
        node.H = node.conflictSet.getMatchingHeuristic();
    }

    if (!updateNode) {
        setState(costs, oldCost, starts, oldStart, ends, oldEnd, mdds, oldMDD, lb, oldLb,
            id1, config.withCardinalConflicts, config.withFocalSearch);
    }

    node.G = node.H + node.cost;
}

template<typename SearchType>
MultiagentSearchResult ConflictBasedSearch<SearchType>::startSearch(const Map &map,
    const Config &config, AgentSet &agentSet,
    std::chrono::steady_clock::time_point globalBegin,
    int globalTimeLimit)
{
    // std::cout << agentSet.getAgentCount() << std::endl;

    //ISearch<>::T = 0;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (config.withPerfectHeuristic) {
        getPerfectHeuristic(map, agentSet);
        search->setPerfectHeuristic(&perfectHeuristic);
    }

    CBSNode<SearchType> root;
    int agentCount = agentSet.getAgentCount();
    MultiagentSearchResult result(false);

    if (open.empty() && focal.empty()) {
        std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
        std::vector<MDD> mdds;
        ConflictAvoidanceTable CAT;
        for (int i = 0; i < agentSet.getAgentCount(); ++i) {
            //std::cout << i << std::endl;

            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
                result.pathfound = false;
                return result;
            }

            if (globalTimeLimit != -1 &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - globalBegin).count() > config.maxTime)
            {
                result.pathfound = false;
                return result;
            }

            Astar<> astar(false, false);
            Agent agent = agentSet.getAgent(i);
            SearchResult searchResult;

            if (config.withCAT && config.useCatAtRoot)
            {
                SearchType* searchPtr = search;
                if (config.searchType == CN_ST_AECBS) {
                    SearchType tempSearch = *search;
                    rootSearches.push_back(std::move(tempSearch));
                    searchPtr = &rootSearches.back();
                }
                searchResult = searchPtr->startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                   agent.getGoal_i(), agent.getGoal_j(), nullptr,
                                                   true, true, 0, -1, -1, {}, {}, true, CAT, globalBegin, globalTimeLimit);
                if (searchResult.pathfound) {
                    CAT.addAgentPath(searchResult.lppath->begin(), searchResult.lppath->end());
                }
            } else {
                astar.setPerfectHeuristic(&perfectHeuristic);
                searchResult = astar.startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                    agent.getGoal_i(), agent.getGoal_j(),
                    nullptr, true, true, 0, -1, -1, {}, {}, false, {}, globalBegin, globalTimeLimit);
            }
            if (!searchResult.pathfound) {
                //std::cout << "fail" << std::endl;
                result.pathfound = false;
                return result;
            }
            root.cost += searchResult.pathlength;
            root.paths[i] = *searchResult.lppath;

            if (config.withCardinalConflicts) {
                root.mdds[i] = MDD(map, agentSet, search, i, searchResult.pathlength);
            }

            starts[i] = root.paths[i].begin();
            ends[i] = root.paths[i].end();
            mdds.push_back(root.mdds[i]);
            if (config.withFocalSearch) {
                root.lb[i] = searchResult.minF;
                root.sumLb += searchResult.minF;
            }
        }

        if (config.storeConflicts) {
            root.conflictSet = findConflict<std::list<Node>::iterator>(starts, ends, -1, true,
                                                                       config.withCardinalConflicts, mdds);
            if (config.withFocalSearch) {
                root.hc = root.conflictSet.getConflictingPairsCount();
                sumLb.insert(root.sumLb);
            }
        }

        open.insert(root);
    }

    std::vector<int> LLExpansions, LLNodes;

    int t = 0;
    while (!open.empty() || !focal.empty()) {
        //std::cout << t << std::endl;

        ++t;
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            result.pathfound = false;
            break;
        }

        if (globalTimeLimit != -1 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - globalBegin).count() > config.maxTime)
        {
            result.pathfound = false;
            return result;
        }

        CBSNode<SearchType> cur;
        if (config.withFocalSearch) {
            double threshold = *sumLb.begin() * config.focalW;
            auto it = open.begin();
            for (it; it != open.end() && it->cost <= threshold; ++it) {
                focal.insert(*it);
            }
            open.erase(open.begin(), it);
            cur = *focal.begin();
        } else {
            cur = *open.begin();
        }

        std::vector<int> costs(agentCount, 0);
        std::vector<bool> agentFound(agentCount, false);
        std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
        ConstraintsSet constraints;
        ConflictAvoidanceTable CAT;
        std::vector<MDD> mdds(agentCount);
        std::vector<double> lb(agentCount);
        std::vector<SearchType*> agentSearches(agentCount, nullptr);
        for (CBSNode<SearchType> *ptr = &cur; ptr != nullptr; ptr = ptr->parent) {
            for (auto it = ptr->paths.begin(); it != ptr->paths.end(); ++it) {
                if (!agentFound[it->first]) {
                    starts[it->first] = it->second.begin();
                    ends[it->first] = it->second.end();
                    costs[it->first] = it->second.size() - 1;
                    agentFound[it->first] = true;

                    if (config.withCAT || config.withFocalSearch == true) {
                        CAT.addAgentPath(starts[it->first], ends[it->first]);
                    }
                    if (config.withCardinalConflicts) {
                        mdds[it->first] = ptr->mdds.find(it->first)->second;
                    }
                    if (config.withFocalSearch) {
                        lb[it->first] = ptr->lb[it->first];
                    }
                }
            }
            if (ptr->parent == nullptr) {
                for (int agentId = 0; agentId < agentSearches.size(); ++agentId) {
                    if (agentSearches[agentId] == nullptr) {
                        agentSearches[agentId] = search;
                    }
                }
            } else {
                int agentId = ptr->paths.begin()->first;
                constraints.addConstraint(ptr->constraint);
                if (ptr->hasPositiveConstraint) {
                    constraints.addConstraint(ptr->positiveConstraint);
                }
                if (config.lowLevel == CN_SP_ST_FLPASTAR && agentSearches[agentId] == nullptr) {
                    agentSearches[agentId] = ptr->search.get();
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
            result.HLExpansions = {double(close.size())};
            result.HLNodes = {double(open.size() + close.size() + focal.size())};
            if (LLExpansions.empty()) {
                result.AvgLLExpansions = {0};
                result.AvgLLNodes = {0};
            } else {
                result.AvgLLExpansions = {(double)std::accumulate(LLExpansions.begin(), LLExpansions.end(), 0) / LLExpansions.size()};
                result.AvgLLNodes = {(double)std::accumulate(LLNodes.begin(), LLNodes.end(), 0) / LLNodes.size()};
            }

            if (config.searchType == CN_ST_AECBS) {
                double totalNodes = 0;
                for (auto& node : open) {
                    totalNodes += node.search.get()->getSize();
                }
                for (auto& node : close) {
                    if (node.search != nullptr) {
                        totalNodes += node.search.get()->getSize();
                    }
                }
                for (auto& node : focal) {
                    if (node.search != nullptr) {
                        totalNodes += node.search.get()->getSize();
                    }
                }
                result.totalNodes = {totalNodes};
            } else {
                result.totalNodes = {0};
            }

            result.cost = {cur.cost};
            result.focalW = {config.focalW};

            std::pair<int, int> costs = result.getCosts();
            result.makespan = {double(costs.first)};
            result.flowtime = {double(costs.second)};

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            result.time = {static_cast<double>(elapsedMilliseconds) / 1000};
            break;
        }

        if (config.withFocalSearch) {
            focal.erase(cur);
            sumLb.erase(sumLb.find(cur.sumLb));
        } else {
            open.erase(cur);
        }
        close.push_back(cur);

        Conflict conflict = conflictSet.getBestConflict();
        if (config.withDisjointSplitting &&
                mdds[conflict.id1].getLayerSize(conflict.time) < mdds[conflict.id2].getLayerSize(conflict.time)) {
            std::swap(conflict.id1, conflict.id2);
            std::swap(conflict.pos1, conflict.pos2);
        }

        /*std::cout << t << " " << cur.paths.begin()->first << " " <<
            conflict.id1 << " " << conflict.id2 << " " << conflict.pos1.i << " " << conflict.pos1.j << " " <<
            conflict.pos2.i << " " << conflict.pos2.j << " " << conflict.time << " " <<
            open.size() + close.size() + focal.size() << " " <<
            conflictSet.nonCardinal.size() << " " << cur.cost << " " << cur.id << " ";

        if (cur.search != nullptr) {
            std::cout << cur.search.get()->getSize() << std::endl;
        } else {
            std::cout << std::endl;
        }*/

        CBSNode<SearchType> child1, child2;
        if (config.lowLevel == CN_SP_ST_FLPASTAR) {
            child1.search.reset(new SearchType(*agentSearches[conflict.id1]));
            child2.search.reset(new SearchType(*agentSearches[conflict.id2]));
        }

        std::vector<CBSNode<SearchType>> children;
        createNode(map, agentSet, config, conflict, costs, constraints,
            conflict.id1, conflict.pos2, conflict.pos1, starts, ends,
            CAT, conflictSet, mdds, lb, LLExpansions, LLNodes, &close.back(), child1,
            (config.lowLevel == CN_SP_ST_FLPASTAR) ? child1.search.get() : search, false, globalBegin, globalTimeLimit);
        if (child1.pathFound) {
            children.push_back(child1);
        }
        createNode(map, agentSet, config, conflict, costs, constraints,
            conflict.id2, conflict.pos1, conflict.pos2, starts, ends,
            CAT, conflictSet, mdds, lb, LLExpansions, LLNodes, &close.back(), child2,
            (config.lowLevel == CN_SP_ST_FLPASTAR) ? child2.search.get() : search, false, globalBegin, globalTimeLimit);
        if (child2.pathFound) {
            children.push_back(child2);
        }

        /*for (auto child : children) {
            for (auto c : constraints.nodeConstraints) {
                if (c == child.constraint && c.agentId == child.constraint.agentId) {
                    std::cout << t << " q" << std::endl;
                }
            }
        }

        for (auto child : children) {
            auto newStarts = starts;
            auto newEnds = ends;
            newStarts[child.constraint.agentId] = child.paths.begin()->second.begin();
            newEnds[child.constraint.agentId] = child.paths.begin()->second.end();

            conflictSet = findConflict<std::list<Node>::iterator>(newStarts, newStarts, -1, false,
                config.withCardinalConflicts, mdds);

            for (auto c : conflictSet.nonCardinal) {
                if (c.id1 == conflict.id1 && c.id2 == conflict.id2 &&
                        c.pos1 == conflict.pos1 && c.time == conflict.time &&
                        c.edgeConflict == conflict.edgeConflict && c.pos2 == conflict.pos2) {
                    std::cout << t << " q\n";
                }
            }
        }*/

        /*for (auto child : children) {
            for (auto constraint : constraints.nodeConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    for (auto it = child.paths.begin()->second.begin(); it != child.paths.begin()->second.end(); ++it) {
                        if (std::next(it) != child.paths.begin()->second.end()) {
                            auto pr = this->mp->getPrimitive(std::next(it)->primitiveId);
                            for (auto cell : pr.cells) {
                                if (it->i + cell.i == constraint.i && it->j + cell.j == constraint.j) {
                                    int start = std::next(it)->g - pr.intDuration + cell.interval.first;
                                    int end = std::next(it)->g - pr.intDuration + cell.interval.second;
                                    if (start <= constraint.time && constraint.time <= end) {
                                        std::cout << "1 " << t << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            for (auto constraint : constraints.nodeConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    auto it = child.paths.begin()->second.begin();
                    std::advance(it, std::min(constraint.time, int(child.paths.begin()->second.size()) - 1));
                    if (it->i == constraint.i && it->j == constraint.j) {
                        std::cout << "1 " << t << std::endl;
                    }
                }
            }
            for (auto constraint : constraints.positiveConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    auto it = child.paths.begin()->second.begin();
                    std::advance(it, std::min(constraint.time, int(child.paths.begin()->second.size()) - 1));
                    if ((it->i != constraint.i || it->j != constraint.j)) {
                        std::cout << "3 " << t << std::endl;
                    }
                }
            }
        }*/

        bool bypass = false;
        if (children.size() == 2) {
            for (auto& child : children) {
                if (child.pathFound && config.withBypassing && cur.cost == child.cost &&
                        conflictSet.getConflictCount() > child.conflictSet.getConflictCount()) {
                    open.insert(child);
                    if (config.withFocalSearch) {
                        sumLb.insert(child.sumLb);
                    }
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

            for (auto& child : children) {
                open.insert(child);
                if (config.withFocalSearch) {
                    sumLb.insert(child.sumLb);
                }
            }
        }
    }
    //std::cout << close.size() + open.size() << std::endl;
    //std::cout << ISearch<>::T << std::endl;
    return result;
}

template class CBSNode<Astar<>>;
template class CBSNode<SIPP<>>;
template class CBSNode<ZeroSCIPP<>>;
template class CBSNode<FocalSearch<>>;
template class CBSNode<SCIPP<>>;
template class CBSNode<FocalLPAStar<FLPANode>>;

template class ConflictBasedSearch<Astar<>>;
template class ConflictBasedSearch<SIPP<>>;
template class ConflictBasedSearch<ZeroSCIPP<>>;
template class ConflictBasedSearch<FocalSearch<>>;
template class ConflictBasedSearch<SCIPP<>>;
template class ConflictBasedSearch<FocalLPAStar<FLPANode>>;
