#include "anytime_cbs.h"

template<typename SearchType>
AnytimeCBS<SearchType>::AnytimeCBS() {}

template<typename SearchType>
AnytimeCBS<SearchType>::AnytimeCBS(ConflictBasedSearch<SearchType>* Search)
{
    search = Search;
}

template<typename SearchType>
AnytimeCBS<SearchType>::~AnytimeCBS()
{
    if (search)
        delete search;
}

template<typename SearchType>
void AnytimeCBS<SearchType>::clear()
{
    search->clear();
}

template<typename SearchType>
void AnytimeCBS<SearchType>::setChildren(std::list<CBSNode<SearchType>>& nodeSet) {
    for (auto& node : nodeSet) {
        node.children.clear();
    }
    for (auto& node : nodeSet) {
        if (node.parent != nullptr) {
            node.parent->children.push_back(&node);
        }
    }
}

template<typename SearchType>
void AnytimeCBS<SearchType>::setRemoveSubtree(CBSNode<SearchType>* root) {
    root->remove = true;
    for (CBSNode<SearchType>* child : root->children) {
        setRemoveSubtree(child);
    }
}

template<typename SearchType>
void AnytimeCBS<SearchType>::updateNode(const Map &map, const AgentSet &agentSet, const Config &config,
    std::vector<int> &costs,
    ConstraintsSet &constraints,
    std::vector<std::list<Node>::iterator> &starts,
    std::vector<std::list<Node>::iterator> &ends,
    ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
    std::vector<MDD> &mdds, std::vector<double> &lb,
    std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
    std::list<CBSNode<SearchType>>& open,
    CBSNode<SearchType>* node, double focalW,
    std::chrono::steady_clock::time_point globalBegin, int globalTimeLimit)
{
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (globalTimeLimit != -1 && std::chrono::duration_cast<std::chrono::milliseconds>(now - globalBegin).count() > globalTimeLimit) {
        return;
    }

    int agentId;
    int oldCost;
    std::list<Node>::iterator oldStart;
    std::list<Node>::iterator oldEnd;
    MDD oldMDD;
    double oldLb;
    double oldSumLb = node->sumLb;
    bool isRoot = (node->paths.size() > 1);
    bool pathFound = true;
    if (!isRoot) {
        agentId = node->conflict.id1;
        this->search->getState(costs, oldCost, starts, oldStart, ends, oldEnd, mdds, oldMDD, lb, oldLb,
            agentId, false, config.withFocalSearch);
        constraints.addConstraint(node->constraint);
        if (node->hasPositiveConstraint) {
            constraints.addConstraint(node->positiveConstraint);
        }
        node->search.get()->updateFocalW(focalW, map);
        this->search->createNode(map, agentSet, config, node->conflict, costs, constraints,
            agentId, node->conflict.pos2, node->conflict.pos1, starts, ends,
            CAT, conflictSet, mdds, lb, LLExpansions, LLNodes, node->parent, *node,
            node->search.get(), true, false, globalBegin, globalTimeLimit);
        pathFound = node->pathFound;
    }

    bool removeSubtree = false;
    if (pathFound) {
        if (node->children.empty()) {
            search->sumLb.erase(search->sumLb.find(oldSumLb));
            search->sumLb.insert(node->sumLb);
        }

        if (config.cutIrrelevantConflicts && !node->children.empty() && !isRoot) {
            auto& conflict = node->newConflict;

            int time;
            std::list<Node>::iterator it1;
            for (time = 0, it1 = starts[conflict.id1];
                 time < conflict.time && std::next(it1) != ends[conflict.id1];
                 ++time, ++it1) {}
            std::list<Node>::iterator it2;
            for (time = 0, it2 = starts[conflict.id2];
                 time < conflict.time && std::next(it2) != ends[conflict.id2];
                 ++time, ++it2) {}
            Node realPos1 = *it1;
            Node realPos2 = *it2;
            removeSubtree = !conflict.edgeConflict && (*it1 != conflict.pos1 || *it2 != conflict.pos2) ||
                conflict.edgeConflict && (*it1 != conflict.pos2 || *it2 != conflict.pos1 ||
                    *std::prev(it1) != conflict.pos1 || *std::prev(it2) != conflict.pos2);
        }

        if (!removeSubtree) {
            for (CBSNode<SearchType>* child : node->children) {
                updateNode(map, agentSet, config, costs, constraints, starts, ends,
                    CAT, node->conflictSet, mdds, lb, LLExpansions, LLNodes, open,
                    child, focalW, globalBegin, globalTimeLimit);
            }
        }
    }

    if (!isRoot) {
        CAT.removeAgentPath(starts[agentId], ends[agentId]);
        CAT.addAgentPath(oldStart, oldEnd);
        this->search->setState(costs, oldCost, starts, oldStart, ends, oldEnd, mdds, oldMDD, lb, oldLb,
            agentId, false, config.withFocalSearch);
        constraints.removeConstraint(node->constraint);
        if (node->hasPositiveConstraint) {
            constraints.removeLastPositiveConstraint();
        }
    }

    if (removeSubtree) {
        if (config.withFocalSearch) {
            this->search->sumLb.insert(node->sumLb);
        }
        open.push_back(*node);
        for (size_t i = 0; i < node->parent->children.size(); ++i) {
            if (node->parent->children[i] == node) {
                node->parent->children[i] = &open.back();
            }
        }
        setRemoveSubtree(node);
    }
}

template<typename SearchType>
MultiagentSearchResult AnytimeCBS<SearchType>::startSearch(const Map &map,
    const Config &config, AgentSet &agentSet,
    std::chrono::steady_clock::time_point globalBegin,
    int globalTimeLimit)
{
    CBSNode<SearchType>::curId = 0;
    int agentCount = agentSet.getAgentCount();
    MultiagentSearchResult result(false);
    Config curConfig = config;
    curConfig.LogParams = nullptr;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<std::vector<Node>> agentsPaths;

    if (config.withPerfectHeuristic) {
        search->getPerfectHeuristic(map, agentSet);
        search->search->setPerfectHeuristic(&search->perfectHeuristic);
    }
    search->bestKnownCost = CN_INFINITY;

    search->search->updateFocalW(config.focalW, map);
    int iter = 0;
    while (curConfig.focalW > 1.0) {
        /*if (iter == 2) {
            break;
        }*/

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            break;
        }

        std::cout << search->open.size() << " " << search->close.size() << std::endl;

        MultiagentSearchResult newResult = search->startSearch(map, curConfig, agentSet, begin, config.maxTime);
        if (!newResult.pathfound) {
            result.finalHLNodesStart = newResult.HLNodesStart.back();
            result.finalHLNodes = newResult.HLNodes.back();
            result.finalHLExpansionsStart = newResult.HLExpansionsStart.back();
            result.finalHLExpansions = newResult.HLExpansions.back();
            break;
        }

        result += newResult;
        now = std::chrono::steady_clock::now();
        result.time.back() = (double)std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() / 1000;

        double minF = std::min(*search->sumLb.begin(), (double)result.cost.back());
        result.focalW.back() = result.cost.back() / minF;

        agentsPaths = *(result.agentsPaths);
        search->agentsPaths.clear();

        if (search->open.empty() && search->focal.size() == 1) {
            result.focalW.back() = 1.0;
            break;
        }

        curConfig.focalW = result.cost.back() / minF - 0.0001;
        if (curConfig.focalW < 1.0) {
            curConfig.focalW = 1.0;
        }

        search->bestKnownCost = std::min(result.cost.back(), search->bestKnownCost);

        std::cout << newResult.cost[0] << " " << curConfig.focalW << std::endl;

        search->search->updateFocalW(curConfig.focalW, map);

        if ((++iter) % config.restartFrequency == 0) {
            search->clear();
            search->search->clearLists();
            continue;
        }

        if (config.searchType == CN_ST_AECBS) {
            std::list<CBSNode<SearchType>> open;
            std::copy(search->open.begin(), search->open.end(), std::back_inserter(open));
            std::copy(search->focal.begin(), search->focal.end(), std::back_inserter(open));
            search->open.clear();
            search->focal.clear();

            setChildren(search->close);
            setChildren(open);

            std::vector<int> costs(agentCount, 0);
            ConstraintsSet constraints;
            std::vector<std::list<Node>::iterator> starts(agentCount);
            std::vector<std::list<Node>::iterator> ends(agentCount);
            ConflictAvoidanceTable CAT;
            std::vector<MDD> mdds(agentCount);
            std::vector<double> lb(agentCount, 0.0);
            std::vector<int> LLExpansions;
            std::vector<int> LLNodes;
            CBSNode<SearchType>* root = &(*search->close.begin());
            int newRootCost = 0;

            auto it = root->paths.begin();
            for (it = root->paths.begin(); it != root->paths.end(); ++it) {
                int agentId = it->first;
                if (config.useCatAtRoot) {
                    const Agent& agent = agentSet.getAgent(agentId);
                    SearchType& agentSearch = search->rootSearches[agentId];
                    agentSearch.updateFocalW(curConfig.focalW, map);
                    SearchResult searchResult = agentSearch.startSearch(map, agentSet,
                        agent.getStart_i(), agent.getStart_j(), agent.getGoal_i(), agent.getGoal_j(),
                        nullptr, false, true, 0, -1, -1, {}, {}, config.withCAT, CAT, begin, globalTimeLimit);

                    if (!searchResult.pathfound) {
                        break;
                    }

                    it->second = *searchResult.lppath;
                    newRootCost += searchResult.pathlength;

                    if (config.withCardinalConflicts) {
                        root->mdds[agentId] = MDD(map, agentSet, search->search, agentId, searchResult.pathlength);
                    }

                    root->lb[agentId] = searchResult.minF;
                }

                CAT.addAgentPath(it->second.begin(), it->second.end());
                search->setState(costs, it->second.size() - 1, starts, it->second.begin(),
                    ends, it->second.end(), mdds, root->mdds[agentId], lb, root->lb[agentId],
                    agentId, config.withCardinalConflicts, config.withFocalSearch);
            }

            if (it != root->paths.end()) {
                break;
            }

            if (config.useCatAtRoot) {
                root->cost = newRootCost;
                root->conflictSet = ConflictBasedSearch<>::findConflict<std::list<Node>::iterator>(
                    starts, ends, -1, true, config.withCardinalConflicts, mdds);
                root->hc = root->conflictSet.getConflictingPairsCount();
            }

            updateNode(map, agentSet, curConfig, costs, constraints, starts, ends,
                CAT, root->conflictSet, mdds, lb, LLExpansions, LLNodes, open, root, curConfig.focalW,
                begin, config.maxTime);

            /*this->search->close.erase(std::remove_if(this->search->close.begin(), this->search->close.end(), [](const CBSNode<SearchType>& node) {
                return node.remove;
            }), this->search->close.end());*/

            auto closeIt = this->search->close.begin();
            while (closeIt != this->search->close.end()) {
                if (closeIt->remove) {
                    closeIt = this->search->close.erase(closeIt);
                } else {
                    ++closeIt;
                }
            }

            /*open.erase(std::remove_if(open.begin(), open.end(), [&](const CBSNode<SearchType>& node) {
                if (node.remove) {
                    this->search->sumLb.erase(this->search->sumLb.find(node.sumLb));
                    return true;
                }
                return false;
            }), open.end());*/

            auto openIt = open.begin();
            while (openIt != open.end()) {
                if (openIt->remove) {
                    this->search->sumLb.erase(this->search->sumLb.find(openIt->sumLb));
                    openIt = open.erase(openIt);
                } else {
                    ++openIt;
                }
            }

            for (const auto& node : open) {
                if (node.pathFound) {
                    minF = std::min(minF, double(node.cost));
                    search->focal.insert(node);
                }
            }
        }

        auto it = search->focal.begin();
        while (it != search->focal.end()) {
            if (it->cost > minF * curConfig.focalW) {
                search->open.insert(*it);
                it = search->focal.erase(it);
            } else {
                ++it;
            }
        }
    }

    //std::cout << FocalSearch<>::Time << std::endl;

    search->agentsPaths = agentsPaths;
    result.agentsPaths = &search->agentsPaths;
    if (config.searchType == CN_ST_AECBS) {
        result.finalTotalNodes = 0;
        for (auto& node : search->open) {
            if (node.search != nullptr) {
                result.finalTotalNodes += node.search.get()->getSize();
            }
        }
        for (auto& node : search->close) {
            if (node.search != nullptr) {
                result.finalTotalNodes += node.search.get()->getSize();
            }
        }
        for (auto& node : search->focal) {
            if (node.search != nullptr) {
                result.finalTotalNodes += node.search.get()->getSize();
            }
        }
        if (config.useCatAtRoot) {
            for (auto& search : search->rootSearches) {
                result.finalTotalNodes += search.getSize();
            }
        }
    }
    return result;
}

template class AnytimeCBS<Astar<>>;
template class AnytimeCBS<FocalSearch<>>;
template class AnytimeCBS<SIPP<>>;
template class AnytimeCBS<SCIPP<>>;
template class AnytimeCBS<ZeroSCIPP<>>;
template class AnytimeCBS<ReplanningAStar<ReplanningAstarNode>>;
template class AnytimeCBS<ReplanningFocalSearch<>>;
