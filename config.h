#ifndef CONFIG_H
#define	CONFIG_H
#include <string>

class Config
{
    public:
        Config();
        Config(const Config& orig);
        ~Config();
        bool getConfig(const char *FileName);

    public:
        double*         SearchParams;
        std::string*    LogParams;
        unsigned int    N;
        int             searchType;
        int             minAgents;
        int             maxAgents;
        int             maxTime;
        std::string     agentsFile;
        int             tasksCount;
        bool            withCAT;
        bool            withPerfectHeuristic;
};

#endif

