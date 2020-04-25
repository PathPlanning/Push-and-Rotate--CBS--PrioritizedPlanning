#ifndef ILOGGER_H
#define	ILOGGER_H
#include "map.h"
#include "node.h"
#include <unordered_map>
#include <list>

class ILogger
{
    public:
        ILogger(std::string loglevel) {this->loglevel = loglevel;}
        virtual bool getLog(const char *FileName, const std::string *LogParams) = 0;
        virtual void saveLog() = 0;
        virtual void writeToLogMap(const Map& map, const std::list<Node>& path) = 0;
        //virtual void writeToLogOpenClose(const typename &open, const typename &close) = 0;
        virtual void writeToLogPath(const std::list<Node>& path) = 0;
        virtual void writeToLogHPpath(const std::list<Node>& path) = 0;
        virtual void writeToLogAgentsPaths(const AgentSet& agentSet, const std::vector<std::vector<Node>>& agentsPaths,
                                   const std::string &agentsFile, int makespan, int flowtime) = 0;
        virtual void writeToLogNotFound() = 0;
        virtual void writeToLogAggregatedResults(std::map<int, int> successCount,
                                                  std::map<int, double> makespans,
                                                  std::map<int, double> timeflows,
                                                  std::map<int, double> times) = 0;
        virtual void writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize) = 0;
        virtual ~ILogger() {};
    protected:
        std::string loglevel;
};

#endif

