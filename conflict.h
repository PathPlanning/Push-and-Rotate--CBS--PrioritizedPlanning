#include "node.h"
#include <vector>
#include <unordered_map>

struct Conflict {
    int id1, id2;
    Node pos1, pos2;
    int time;
    bool edgeConflict;
    bool conflictFound;

    Conflict(bool ConflictFound = false) {
        conflictFound = ConflictFound;
    }

    Conflict(int Id1, int Id2, Node Pos1, Node Pos2, int Time, bool EdgeConflict) {
        id1 = Id1;
        id2 = Id2;
        pos1 = Pos1;
        pos2 = Pos2;
        time = Time;
        edgeConflict = EdgeConflict;
        conflictFound = true;
    }
};



