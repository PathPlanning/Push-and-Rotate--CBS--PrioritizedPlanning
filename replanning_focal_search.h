#ifndef REPLANNINGFOCALSEARCH_H
#define REPLANNINGFOCALSEARCH_H

#include "replanning_astar.h"
#include "replanning_fs_node.h"

template <typename NodeType = ReplanningFSNode>
class ReplanningFocalSearch : virtual public ReplanningAStar<NodeType>
{
public:
    ReplanningFocalSearch(bool WithTime = false, double FocalW = 1.0, double HW = 1.0, bool BT = true);
    ReplanningFocalSearch(ReplanningFocalSearch& other) = default;
    ReplanningFocalSearch& operator=(ReplanningFocalSearch& other) = default;
    ReplanningFocalSearch(ReplanningFocalSearch&& other) = default;
    ReplanningFocalSearch& operator=(ReplanningFocalSearch&& other) = default;
    virtual ~ReplanningFocalSearch() {}
    //virtual void updateFocalW(double newFocalW, const Map& map) override;
    void clearLists() override;

    void updateNode(NodeType &node, const Map &map,
        int goal_i, int goal_j, int agentId,
        const std::unordered_set<Node, NodeHash> &occupiedNodes,
        const ConstraintsSet &constraints,
        bool withCAT, const ConflictAvoidanceTable &CAT,
        std::queue<NodeType>& queue, std::unordered_set<int>& addedNodesConv) override;

    void updateFocalW(double newFocalW, const Map& map) override;

    static int Time;

protected:
    bool checkOpenEmpty() override;
    NodeType getCur(const Map& map) override;
    virtual void removeCur(const NodeType& cur, const Map& map) override;
    double getMinFocalF() override;
    virtual void setHC(NodeType &neigh, const NodeType &cur,
        const ConflictAvoidanceTable &CAT, bool isGoal) override;
    virtual int getFocalSize() override { return focal.size(); }
    virtual void removeUnexpandedNode(const NodeType& node);
    int getPredConflictsCount(const NodeType& node, const NodeType& pred,
        const ConflictAvoidanceTable& CAT) const { return pred.hc + CAT.getEdgeAgentsCount(node, pred); };

    std::set<NodeType, bool (*)(const NodeType&, const NodeType&)> focal;
    std::multiset<double> focalF;
    double focalW;
};

#endif // REPLANNINGFOCALSEARCH_H
