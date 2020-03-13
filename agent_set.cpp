#include "agent_set.h"

void AgentSet::clear() {
    occupiedNodes.clear();
    agents.clear();
}

void AgentSet::addAgent(int start_i, int start_j, int goal_i, int goal_j) {
    occupiedNodes[std::make_pair(start_i, start_j)] = agents.size();
    agents.push_back(Agent(start_i, start_j, goal_i, goal_j, agents.size()));
}

void AgentSet::setAgentPosition(int agentId, Node pos) {
    agents[agentId].setCurPosition(pos.i, pos.j);
}

void AgentSet::setPriority(int first, int second) {
    subgraphPriorities.insert(std::make_pair(first, second));
}

void AgentSet::setConnectedComponent(int i, int j, int compNum) {
    connectivityComponents[std::make_pair(i, j)] = compNum;
}

void AgentSet::addComponentSize(int compSize) {
    componentSizes.push_back(compSize);
}

int AgentSet::getAgentCount() const {
    return agents.size();
}

Agent AgentSet::getAgent(int i) const {
    return agents[i];
}

bool AgentSet::isOccupied(int i, int j) const {
    return occupiedNodes.find(std::make_pair(i, j)) != occupiedNodes.end();
}

int AgentSet::getAgentId(int i, int j) const {
    return occupiedNodes.at(std::make_pair(i, j));
}

void AgentSet::moveAgent(Node &from, Node &to, std::vector<AgentMove>& result) {
    int id = occupiedNodes.at(std::make_pair(from.i, from.j));
    occupiedNodes[std::make_pair(to.i, to.j)] = id;
    occupiedNodes.erase(std::make_pair(from.i, from.j));
    agents[id].setCurPosition(to.i, to.j);
    result.push_back(AgentMove(to.i - from.i, to.j - from.j, id));
}

bool AgentSet::readAgents(const char *FileName)
{
    tinyxml2::XMLElement *root = 0, *node;
    tinyxml2::XMLDocument doc;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (node = root->FirstChildElement(); node; node = node->NextSiblingElement()) {
        int id, start_i, start_j, goal_i, goal_j;
        node->QueryIntAttribute("id", &id);
        node->QueryIntAttribute("start_i", &start_i);
        node->QueryIntAttribute("start_j", &start_j);
        node->QueryIntAttribute("goal_i", &goal_i);
        node->QueryIntAttribute("goal_j", &goal_j);
        addAgent(start_i, start_j, goal_i, goal_j);
    }

    return true;
}

void AgentSet::setNodeSubgraph(int i, int j, int subgraphNum) {
    subgraphNodes.insert(std::make_pair(std::make_pair(i, j), subgraphNum));
}

void AgentSet::setAgentSubgraph(int agentId, int subgraphNum) {
    agents[agentId].setSubgraph(subgraphNum);
}

void AgentSet::removeSubgraphs(int i, int j) {
    subgraphNodes.erase(std::make_pair(i, j));
}

std::vector<int> AgentSet::getSubgraphs(int i, int j) const {
    std::vector<int> res;
    std::pair<int, int> pair = std::make_pair(i, j);
    for (auto it = subgraphNodes.lower_bound(pair); it != subgraphNodes.upper_bound(pair); ++it) {
        res.push_back(it->second);
    }
    return res;
}

bool AgentSet::hasPriority(int first, int second) const {
    return subgraphPriorities.find(std::make_pair(first, second)) != subgraphPriorities.end();
}

int AgentSet::getConnectedComponentsCount() const {
    return componentSizes.size();
}

int AgentSet::getComponentSize(int i, int j) {
    return componentSizes[connectivityComponents[std::make_pair(i, j)]];
}

int AgentSet::getConnectedComponent(int i, int j) {
    return connectivityComponents[std::make_pair(i, j)];
}
