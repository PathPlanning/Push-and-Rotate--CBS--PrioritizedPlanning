#ifndef CONFLICTSET_H
#define CONFLICTSET_H

#include <vector>
#include <algorithm>
#include <unordered_set>
#include <set>
#include "conflict.h"
#include "node.h"

class ConflictSet
{
public:
    void addCardinalConflict(Conflict &conflict);
    void addSemiCardinalConflict(Conflict &conflict);
    void addNonCardinalConflict(Conflict &conflict);
    void replaceAgentConflicts(int agentId, ConflictSet &agentConflicts);
    bool empty();
    Conflict getBestConflict();
    int getConflictCount();
    int getCardinalConflictCount();
    std::vector<Conflict> getCardinalConflicts();
    int getMatchingHeuristic();
    int getConflictingPairsCount();
//private:
    std::vector<Conflict> cardinal, semiCardinal, nonCardinal;
};

#endif // CONFLICTSET_H
