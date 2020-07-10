#ifndef FOCALSEARCH_H
#define FOCALSEARCH_H

#include "astar.h"
#include "fs_node.h"

template <typename NodeType = FSNode>
class FocalSearch : virtual public Astar<NodeType>
{
public:
    FocalSearch(bool WithTime = false, double FocalW = 1.0, double HW = 1.0, bool BT = true);
    virtual ~FocalSearch() {}
protected:
    bool checkOpenEmpty() override;
    NodeType getCur(const Map& map) override;
    bool updateFocal(const NodeType& neigh, const Map& map) override;
    double getMinFocalF() override;
    void clearLists() override;
    virtual void setHC(NodeType &neigh, const NodeType &cur) override;

    SearchQueue<NodeType> focal;
    std::multiset<double> focalF;
    double focalW;
};

#endif // FOCALSEARCH_H
