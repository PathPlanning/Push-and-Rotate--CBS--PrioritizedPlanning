#ifndef AGENT_SET_H
#define AGENT_SET_H

#include <vector>
#include <map>
#include <list>
#include <set>
#include <iostream>
#include "agent.h"
#include "tinyxml2.h"
#include "gl_const.h"
#include "node.h"
#include "agent_move.h"

class AgentSet
{
    private:
        int                                     width;
        int                                     height;
        std::map<std::pair<int, int>, int>      occupiedNodes;
        std::map<std::pair<int, int>, int>      connectivityComponents;
        std::vector<int>                        componentSizes;
        std::multimap<std::pair<int, int>, int> subgraphNodes;
        std::set<std::pair<int, int>>           subgraphPriorities;
        std::vector<Agent>                      agents;

    public:
        bool readAgents(const char *FileName);

        void clear();
        void addAgent(int start_i, int start_j, int goal_i, int goal_j);
        void moveAgent(Node& from, Node& to, std::vector<AgentMove>& result);
        void setAgentPosition(int agentId, Node pos);
        void setPriority(int first, int second);
        void setAgentSubgraph(int agentId, int subgraphNum);
        void setConnectedComponent(int i, int j, int compNum);
        void addComponentSize(int compSize);
        void setNodeSubgraph(int i, int j, int subgraphNum);
        void removeSubgraphs(int i, int j);

        int getAgentCount() const;
        Agent getAgent(int id) const;
        int getAgentId(int i, int j) const;
        bool isOccupied(int i, int j) const;
        bool hasPriority(int first, int second) const;
        std::vector<int> getSubgraphs(int i, int j) const;
        int getConnectedComponentsCount() const;
        int getConnectedComponent(int i, int j);
        int getComponentSize(int i, int j);
};

#endif // AGENT_SET_H
