#ifndef ANYTIMECBS_H
#define ANYTIMECBS_H

#include "conflict_based_search.h"
#include "astar.h"

class AnytimeCBS : public MultiagentSearchInterface
{
private:
    ConflictBasedSearch<Astar<>>* search;
public:
    AnytimeCBS();
    AnytimeCBS(ConflictBasedSearch<Astar<>>* Search);
    ~AnytimeCBS();
    MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &agentSet) override;
    void clear() override;
};

#endif // ANYTIMECBS_H
