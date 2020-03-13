#ifndef XMLLOGGER_H
#define	XMLLOGGER_H
#include "tinyxml2.h"
#include "ilogger.h"


//That's the class that flushes the data to the output XML


class XmlLogger : public ILogger {

public:
    XmlLogger(std::string loglevel):ILogger(loglevel){}

    virtual ~XmlLogger() {};

    bool getLog(const char *FileName, const std::string *LogParams);

    void saveLog();

    void writeToLogMap(const Map &Map, const std::list<Node> &path);

    //void writeToLogOpenClose(const typename &open, const typename &close);

    void writeToLogPath(const std::list<Node> &path);

    void writeToLogHPpath(const std::list<Node> &hppath);

    void writeToLogAggregatedResults(std::map<int, int> successCount,
                                     std::map<int, double> makespans,
                                     std::map<int, double> timeflows,
                                     std::map<int, double> times);

    void writeToLogAgentsPaths(const std::vector<std::vector<Node>>& agentsPaths,
                               const std::string &agentsFile, int makespan, int flowtime);

    void writeToLogNotFound();

    void writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize);

private:
    std::string LogFileName;
    tinyxml2::XMLDocument doc;
};

#endif

