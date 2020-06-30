#ifndef FOCALSEARCH_H
#define FOCALSEARCH_H

#include "astar.h"

class FocalSearch : virtual public Astar
{
public:
    FocalSearch(bool WithTime = false, double FocalW = 1.0, double HW = 1.0, bool BT = true);
    virtual ~FocalSearch() {}
protected:
    bool checkOpenEmpty() override;
    Node getCur(const Map& map) override;
    bool updateFocal(const Node& neigh, const Map& map) override;
    double getMinFocalF() override;
    void clearLists() override;

    SearchQueue focal;
    std::multiset<double> focalF;
    double focalW;
};

#endif // FOCALSEARCH_H
