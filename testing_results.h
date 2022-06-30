#ifndef TESTING_RESULTS_H
#define TESTING_RESULTS_H

#include "gl_const.h"
#include <vector>
#include <map>
#include <string>

struct TestingResults {
    TestingResults() {
        data[CNS_TAG_ATTR_TIME] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_MAKESPAN] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_FLOWTIME] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_LLE] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_LLN] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_HLE] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_HLN] = std::map<int, std::vector<double>>();
        data[CNS_TAG_FOCAL_W] = std::map<int, std::vector<double>>();
        data[CNS_TAG_ATTR_TN] = std::map<int, std::vector<double>>();
    }

    std::vector<std::string> getKeys() {
        std::vector<std::string> keys;
        for (auto pair : data) {
            keys.push_back(pair.first);
        }
        return keys;
    }

    std::map<std::string, std::map<int, std::vector<double>>> data;
    std::map<int, int> finalTotalNodes;
    std::map<int, int> finalHLNodes;
    std::map<int, int> finalHLNodesStart;
    std::map<int, int> finalHLExpansions;
    std::map<int, int> finalHLExpansionsStart;
};

#endif // TESTING_RESULTS_H
