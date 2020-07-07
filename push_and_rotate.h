#ifndef PUSH_AND_ROTATE_H
#define PUSH_AND_ROTATE_H

#include <list>
#include <vector>
#include <algorithm>
#include "isearch.h"
#include "dijkstra.h"
#include "astar.h"
#include "agent_set.h"
#include "agent_move.h"
#include "config.h"
#include "multiagent_search_result.h"
#include "multiagent_search_inteface.h"

class PushAndRotate : public MultiagentSearchInterface
{
    public:
        PushAndRotate();
        PushAndRotate(ISearch<>* Search);
        ~PushAndRotate(void);
        MultiagentSearchResult startSearch(const Map &Map, const Config &config, AgentSet &AgentSet) override;
        void clear() override;
    private:
        bool solve(const Map &map, const Config &config, AgentSet &AgentSet, std::chrono::steady_clock::time_point start);
        bool clearNode(const Map &map, AgentSet &agentSet, Node &nodeToClear,
                       const std::unordered_set<Node, NodeHash>& occupiedNodes);
        bool push(const Map &map, AgentSet &agentSet, Node& from, Node& to,
                  std::unordered_set<Node, NodeHash>& occupiedNodes);
        bool multipush(const Map &map, AgentSet &agentSet, Node first, Node second, Node& to, std::list<Node>& path);
        bool clear(const Map &map, AgentSet &agentSet, Node& first, Node& second);
        void exchange(const Map &map, AgentSet &agentSet, Node& first, Node& second);
        void reverse(int begSize, int endSize,
                     int firstAgentId, int secondAgentId, AgentSet &agentSet);
        bool swap(const Map &map, AgentSet &agentSet, Node& first, Node& second);
        bool rotate(const Map &map, AgentSet &agentSet, std::vector<Node> &qPath, int cycleBeg);
        void getPaths(AgentSet &agentSet);
        void getParallelPaths(AgentSet &agentSet, const Config &config);
        void getComponent(AgentSet &agentSet, std::pair<Node, Node> &startEdge,
                                         std::vector<std::pair<Node, Node>> &edgeStack,
                                         std::vector<std::unordered_set<Node, NodeHash>>& components);
        void combineNodeSubgraphs(AgentSet &agentSet, std::vector<std::unordered_set<Node, NodeHash>>& components,
                                                 Node &subgraphNode, int subgraphNum);
        void getSubgraphs(const Map &map, AgentSet &agentSet);
        int getReachableNodesCount(const Map &map, AgentSet &agentSet, Node &start,
                                                 bool (*condition)(const Node&, const Node&, const Map&, const AgentSet&),
                                                 const std::unordered_set<Node, NodeHash> &occupiedNodes);
        void assignToSubgraphs(const Map &map, AgentSet &agentSet);
        void getPriorities(const Map &map, AgentSet &agentSet);


        ISearch<>*                      search;
        std::vector<AgentMove>          agentsMoves;
        MultiagentSearchResult          result;
};

#endif // PUSH_AND_ROTATE_H
