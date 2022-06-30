#include "replanning_focal_search.h"

template<typename NodeType>
ReplanningFocalSearch<NodeType>::ReplanningFocalSearch(bool WithTime, double FocalW, double HW, bool BT) :
    ReplanningAStar<NodeType>(WithTime)
{
    auto focalCmp = [](const NodeType &lhs, const NodeType &rhs) {
        return lhs.hc < rhs.hc || lhs.hc == rhs.hc && lhs < rhs;
    };
    focal = std::set<NodeType, bool (*)(const NodeType&, const NodeType&)>(focalCmp);
    focalW = FocalW;
}

template<typename NodeType>
bool ReplanningFocalSearch<NodeType>::checkOpenEmpty() {
    return this->open.empty() && focal.empty();
}

template<typename NodeType>
NodeType ReplanningFocalSearch<NodeType>::getCur(const Map& map) {
    if (!this->open.empty()) {
        double minF = this->open.begin()->F;
        if (!focalF.empty()) {
            minF = std::min(minF, *focalF.begin());
        }

        auto it = this->open.begin();
        while (it != this->open.end() && it->F <= minF * focalW) {
            focal.insert(*it);
            focalF.insert(it->F);
            it = this->open.erase(it);
        }
    }
    NodeType cur = *focal.begin();
    return cur;
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::removeCur(const NodeType& cur, const Map& map) {
    focal.erase(cur);
    focalF.erase(focalF.find(cur.F));
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::updateNode(NodeType &node, const Map &map,
    int goal_i, int goal_j, int agentId,
    const std::unordered_set<Node, NodeHash> &occupiedNodes,
    const ConstraintsSet &constraints,
    bool withCAT, const ConflictAvoidanceTable &CAT,
    std::queue<NodeType>& queue, std::unordered_set<int>& addedNodesConv)
{
    int conv = node.convolution(map.getMapWidth(), map.getMapHeight(), this->withTime);
    auto it = this->sortByIndex.find(conv);
    if (it == this->sortByIndex.end()) {
        return;
    }

    this->sresult.nodesexpanded++;

    bool addAgain = false;
    //assert(this->predsCount[conv] >= 1);
    //bool checkPredecessors = (--this->predsCount[conv] > 0);
    NodeType* parent = this->getBestParentPtr(node, map, goal_i, goal_j, agentId,
        occupiedNodes, constraints, withCAT, CAT);
    if (parent != nullptr) {
        int conflictsCount = CAT.getAgentsCount(node, *parent);
        int newHc = parent->hc + conflictsCount;
        if (it->second.hc == newHc) { // || it->second.F < getMinFocalF()) {
            return;
        } else {
            node.conflictsCount = conflictsCount;
            node.hc = newHc;
            addAgain = true;
        }
    }

    bool processSuccessors = false;
    auto closeIt = this->closeConv.find(conv);
    if (closeIt == this->closeConv.end()) {
        auto openIt = this->open.find(it->second);
        if (openIt == this->open.end()) {
            auto focalIt = this->focal.find(it->second);
            assert(focalIt != focal.end());
            focalF.erase(focalF.find(focalIt->F));
            focal.erase(focalIt);
        } else {
            this->open.erase(openIt);
        }
        /*if (!addAgain) {
            this->removedConvs.insert(conv);
        }*/

    } else {
        this->closeConv.erase(closeIt);
        if (node.i == goal_i && node.j == goal_j && node.g == this->goalG) {
            this->goalG = -1;
        }
        processSuccessors = true;
    }
    this->sortByIndex.erase(it);

    /*if (checkPredecessors || processSuccessors) {
        this->sresult.nodesexpanded++;
    }*/

    if (addAgain) {
        this->open.insert(node);
        this->sortByIndex[conv] = node;
    }

    if (!processSuccessors) {
        return;
    }

    std::list<NodeType> successors = this->findSuccessors(node, map,
        goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT);
    for (auto& neigh : successors) {
        int neighConv = neigh.convolution(map.getMapWidth(), map.getMapHeight(), this->withTime);
        if (addedNodesConv.find(neighConv) == addedNodesConv.end()) {
            queue.push(neigh);
            addedNodesConv.insert(neighConv);
        }/* else {
            this->predsCount[neighConv]--;
        }*/
        //updateNode(neigh, map, goal_i, goal_j, agentId, occupiedNodes, constraints, withCAT, CAT, tempCloseConv);
    }
}

template<typename NodeType>
double ReplanningFocalSearch<NodeType>::getMinFocalF() {
    if (focalF.empty()) {
        return CN_INFINITY;
    }
    return *focalF.begin();
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::clearLists() {
    ReplanningAStar<NodeType>::clearLists();
    focal.clear();
    focalF.clear();
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::setHC(NodeType &neigh, const NodeType &cur,
    const ConflictAvoidanceTable &CAT, bool isGoal)
{
    neigh.hc = cur.hc + neigh.conflictsCount;
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::removeUnexpandedNode(const NodeType& node) {
    auto focalIt = focal.find(node);
    if (focalIt != focal.end()) {
        focalF.erase(focalF.find(focalIt->F));
        focal.erase(focalIt);
    } else {
        this->open.erase(node);
    }
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::filterFocalNodes(double minF) {
    auto it = focal.begin();
    while (it != focal.end()) {
        if (it->F > minF * focalW) {
            this->open.insert(*it);
            focalF.erase(focalF.find(it->F));
            it = focal.erase(it);
        } else {
            ++it;
        }
    }
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::updateFocalW(double newFocalW, const Map& map) {
    focalW = newFocalW;
    if (focal.empty()) {
        return;
    }
    double minF = *focalF.begin();

    assert(this->goalG != -1);
    NodeType goalNode = NodeType(this->goalI, this->goalJ, nullptr, this->goalG, 0);
    int goalConv = goalNode.convolution(map.getMapWidth(), map.getMapHeight(), this->withTime);
    auto closeIt = this->closeConv.find(goalConv);
    assert(closeIt != this->closeConv.end());
    this->closeConv.erase(closeIt);
    auto byIndexIt = this->sortByIndex.find(goalConv);
    assert(byIndexIt != this->sortByIndex.end());
    this->open.insert(byIndexIt->second);
    this->goalG = -1;

    filterFocalNodes(minF);
}

template<typename NodeType>
void ReplanningFocalSearch<NodeType>::checkMinFChange() {
    if (focal.empty()) {
        return;
    }
    double minF = *focalF.begin();
    if (this->open.empty() || this->open.begin()->F >= minF) {
        return;
    }

    //std::cout << "Q " << focal.size() << std::endl;

    minF = this->open.begin()->F;
    filterFocalNodes(minF);
}

template class ReplanningFocalSearch<ReplanningFSNode>;
