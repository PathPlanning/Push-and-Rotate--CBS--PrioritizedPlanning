#ifndef AGENT_MOVE_H
#define AGENT_MOVE_H

struct AgentMove
{
    int     di, dj;
    int     id;

    AgentMove(int x, int y, int Id) {
        di = x;
        dj = y;
        id = Id;
    }
};

#endif // AGENT_MOVE_H
