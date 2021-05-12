#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "isearch.h"

template <typename NodeType = Node>
class Dijkstra : public ISearch<NodeType>
{
public:
    virtual ~Dijkstra() {}
    Dijkstra() = default;
    Dijkstra(Dijkstra& other) = default;
    Dijkstra& operator=(Dijkstra& other) = default;
    Dijkstra(Dijkstra&& other) = default;
    Dijkstra& operator=(Dijkstra&& other) = default;
};
#endif
