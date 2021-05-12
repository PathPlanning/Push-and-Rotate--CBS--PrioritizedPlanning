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
void AnytimeCBS<SearchType>::updateNode(const Map &map, const AgentSet &agentSet, const Config &config,
    std::vector<int> &costs,
    ConstraintsSet &constraints,
    std::vector<std::list<Node>::iterator> &starts,
    std::vector<std::list<Node>::iterator> &ends,
    ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
    std::vector<MDD> &mdds, std::vector<double> &lb,
    std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
    CBSNode<SearchType>* node, double focalW)
{
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
            CAT, conflictSet, mdds, lb, LLExpansions, LLNodes, node->parent, *node, node->search.get(), true);
        pathFound = node->pathFound;
    }

    if (pathFound) {
        if (node->children.empty()) {
            search->sumLb.erase(search->sumLb.find(oldSumLb));
            search->sumLb.insert(node->sumLb);
        }
        for (CBSNode<SearchType>* child : node->children) {
            updateNode(map, agentSet, config, costs, constraints, starts, ends,
                CAT, node->conflictSet, mdds, lb, LLExpansions, LLNodes, child, focalW);
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

}

template<typename SearchType>
MultiagentSearchResult AnytimeCBS<SearchType>::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
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

    search->search->updateFocalW(config.focalW, map);

    while (curConfig.focalW > 1.0) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            break;
        }

        MultiagentSearchResult newResult = search->startSearch(map, curConfig, agentSet);
        if (!newResult.pathfound) {
            break;
        }

        result += newResult;
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

        std::cout << newResult.cost[0] << " " << curConfig.focalW << std::endl;

        search->search->updateFocalW(curConfig.focalW, map);

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

            for (auto it = root->paths.begin(); it != root->paths.end(); ++it) {
                int agentId = it->first;
                CAT.addAgentPath(it->second.begin(), it->second.end());
                search->setState(costs, it->second.size() - 1, starts, it->second.begin(),
                    ends, it->second.end(), mdds, root->mdds[agentId], lb, root->lb[agentId],
                    agentId, config.withCardinalConflicts, config.withFocalSearch);
            }

            updateNode(map, agentSet, curConfig, costs, constraints, starts, ends,
                CAT, root->conflictSet, mdds, lb, LLExpansions, LLNodes, root, curConfig.focalW);

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
    search->agentsPaths = agentsPaths;
    result.agentsPaths = &search->agentsPaths;
    return result;
}

template class AnytimeCBS<Astar<>>;
template class AnytimeCBS<FocalSearch<>>;
template class AnytimeCBS<SIPP<>>;
template class AnytimeCBS<SCIPP<>>;
template class AnytimeCBS<ZeroSCIPP<>>;
