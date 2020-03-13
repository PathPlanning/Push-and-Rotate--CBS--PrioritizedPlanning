#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include <vector>
#include "gl_const.h"
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <unordered_set>
#include <algorithm>
#include "tinyxml2.h"
#include "agent_set.h"

//That's the class that stores BOTH grid map data AND start-goal locations.
//getValue reads the input XML and fills the Map object.
//Some naive checks are already included in getValue but still on some corrupted XMLs it may fail,
//so be sure to invoke it passing the correct XML (e.g. with correctly filled map tag, e.g. <map>.

//Map is NOT to be modified during the search. It should be passed as a const pointer.
//Think of it as an "independent" piece of data that is managed by outer (non-search related) proccesses.
//Search algorithm should create it own object/structures needed to run the search on that map.

class Map
{
    private:
        int     height, width;
        int     emptyCellCount;
        double  cellSize;
        int**   Grid;

    public:
        Map();
        Map(const Map& orig);
        ~Map();

        bool getMap(const char *FileName);
        bool CellIsTraversable (int i, int j, const std::unordered_set<Node> &occupiedNodes) const;
        bool CellOnGrid (int i, int j) const;
        bool CellIsObstacle(int i, int j) const;
        int  getValue(int i, int j) const;
        int getMapHeight() const;
        int getMapWidth() const;
        int getEmptyCellCount() const;
        int getCellDegree(int i, int j) const;
        double getCellSize() const;
};

#endif

