#ifndef SCIPP_H
#define SCIPP_H

#include "focalsearch.h"
#include "sipp.h"

class SCIPP : public SIPP, FocalSearch
{
public:
    SCIPP(double FocalW = 1.0, double HW = 1.0, bool BT = true) :
        Astar(true, HW, BT), SIPP(HW, BT), FocalSearch(true, FocalW, HW, BT) {}
    virtual ~SCIPP();

protected:
    std::vector<std::pair<int, int>> splitBySoftConflicts(const Node & node, std::pair<int, int> interval,
                                                                  const ConflictAvoidanceTable &CAT) override;
};

#endif // SCIPP_H
