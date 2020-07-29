#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H
#include <list>
#include "node.h"

//That's the output structure for the search algorithms.
//The task of the pathfinder is to fill it in a right way.

//Practically, after SearchResult is filled by the path finding algorithm,
//one may invoke XMLogger methods like writeToLogSummary etc.
//(passing appropriate fields of SearchResult as parameters)
//and these methods will flush the data to the XML.

//lppath is a path that is a sequence of nodes, corresponding to adjacent cells on the grid.
//It's a natural path for Dijkstra, A*, JPS (e.g. these algorithms find this sort of path).

//hppath is a path that is a sequence of nodes NOT necessarily adjacent on the grid. It's like a path composed of the sections.
//It's a natural path for Theta*.

//In general one might want all algorithms to fill BOTH sorts of path (so both lppath and hppath will be flushed to output log).

//Theta* naturally generates hppath and one needs to post-process it to additionally create lppath.
//One might realize this post-proccessing step by sequentially invoking Line-Of-Sight function that was used during the search.
//NB: In such case the length of the lppath will be formally different form the length of hppath (which is stored as 'SearchResult.pathlength'

// A*-JPS-Dijkstra-etc. naturally generate lppath. One might post-process it by skipping the nodes at which
//the movement direction does not change to form sections of hppath. In this case length of the hppath always equals
//the length of the lppath.

struct SearchResult
{
        bool pathfound;
        float pathlength; //if path not found, then pathlength=0
        const std::list<Node>* lppath; //path as the sequence of adjacent nodes (see above)
                                       //This is a pointer to the list of nodes that is actually created and hadled by ISearch class,
                                       //so no need to re-create them, delete them etc. It's just a trick to save some memory
        const std::list<Node>* hppath; //path as the sequence of non-adjacent nodes: "sections" (see above)
                                        //This is a pointer to the list of nodes that is actually created and hadled by ISearch class,
                                        //so no need to re-create them, delete them etc. It's just a trick to save some memory
        unsigned int nodescreated; //|OPEN| + |CLOSE| = total number of nodes saved in memory during search process.
        unsigned int nodesexpanded;
        unsigned int numberofsteps; //number of iterations (expansions) made by algorithm to find a solution
        double time; //runtime of the search algorithm (expanding nodes + reconstructing the path)
        Node lastNode;
        double minF;
        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
            lppath = nullptr;
            hppath = nullptr;
            nodescreated = 0;
            numberofsteps = 0;
            time = 0;
        }

};

#endif
