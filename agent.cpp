#include "agent.h"


int Agent::getStart_i() const
{
    return start_i;
}

int Agent::getStart_j() const
{
    return start_j;
}

int Agent::getGoal_i() const
{
    return goal_i;
}

int Agent::getGoal_j() const
{
    return goal_j;
}

int Agent::getCur_i() const
{
    return cur_i;
}

int Agent::getCur_j() const
{
    return cur_j;
}


int Agent::getId() const
{
    return id;
}

int Agent::getSubgraph() const {
    return subgraph;
}

Node Agent::getStartPosition() const {
    return Node(start_i, start_j);
}

Node Agent::getGoalPosition() const {
    return Node(goal_i, goal_j);
}

Node Agent::getCurPosition() const {
    return Node(cur_i, cur_j);
}

void Agent::setCurPosition(int i, int j) {
    cur_i = i;
    cur_j = j;
}

void Agent::setSubgraph(int Subgraph) {
    subgraph = Subgraph;
}


