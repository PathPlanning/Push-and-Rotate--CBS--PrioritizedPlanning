#ifndef MULTIAGENT_SEARCH_INTEFACE_H
#define MULTIAGENT_SEARCH_INTEFACE_H

#include "config.h"
#include "agent_set.h"
#include "map.h"
#include "multiagent_search_result.h"
#include "isearch.h"
#include <vector>
#include <unordered_map>

class MultiagentSearchInterface
{
public:
    virtual ~MultiagentSearchInterface(void) {}
    virtual MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &AgentSet) = 0;
    virtual void clear() {
        agentsPaths.clear();
        perfectHeuristic.clear();
    }

    void getPerfectHeuristic(const Map &map, const AgentSet &agentSet) {
        if (!perfectHeuristic.empty()) {
            return;
        }

        std::unordered_set<int> visited;
        ISearch<> search(false);
        for (int i = 0; i < agentSet.getAgentCount(); ++i) {
            visited.clear();
            Node goal = Node(agentSet.getAgent(i).getGoal_i(), agentSet.getAgent(i).getGoal_j());
            std::queue<Node> queue;
            queue.push(goal);
            while (!queue.empty()) {
                Node cur = queue.front();
                queue.pop();
                if (visited.find(cur.convolution(map.getMapWidth(), map.getMapHeight())) != visited.end()) {
                    continue;
                }
                perfectHeuristic[std::make_pair(cur, goal)] = cur.g;
                visited.insert(cur.convolution(map.getMapWidth(), map.getMapHeight()));
                std::list<Node> successors = search.findSuccessors(cur, map);
                for (auto neigh : successors) {
                    queue.push(neigh);
                }
            }
        }
    }

//protected:
    std::vector<std::vector<Node>>  agentsPaths;
    std::unordered_map<std::pair<Node, Node>, int, NodePairHash> perfectHeuristic;
};

#endif // MULTIAGENT_SEARCH_INTEFACE_H
