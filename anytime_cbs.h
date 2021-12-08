#ifndef ANYTIMECBS_H
#define ANYTIMECBS_H

#include "conflict_based_search.h"
#include "astar.h"

template <typename SearchType = Astar<>>
class AnytimeCBS : public MultiagentSearchInterface
{
private:
    ConflictBasedSearch<SearchType>* search;

    void updateNode(const Map &map, const AgentSet &agentSet, const Config &config,
        std::vector<int> &costs,
        ConstraintsSet &constraints,
        std::vector<std::list<Node>::iterator> &starts,
        std::vector<std::list<Node>::iterator> &ends,
        ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
        std::vector<MDD> &mdds, std::vector<double> &lb,
        std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
        CBSNode<SearchType>* node, double focalW,
        std::chrono::steady_clock::time_point globalBegin = std::chrono::steady_clock::time_point(),
        int globalTimeLimit = -1);
public:
    AnytimeCBS();
    AnytimeCBS(ConflictBasedSearch<SearchType>* Search);
    ~AnytimeCBS();
    MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &agentSet,
        std::chrono::steady_clock::time_point globalBegin = std::chrono::steady_clock::time_point(),
        int globalTimeLimit = -1) override;
    void setChildren(std::list<CBSNode<SearchType>>& nodeSet);

    void clear() override;
};

#endif // ANYTIMECBS_H
