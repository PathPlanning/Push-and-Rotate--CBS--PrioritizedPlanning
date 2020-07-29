#ifndef TESTING_RESULTS_H
#define TESTING_RESULTS_H

#include "gl_const.h"
#include <vector>
#include <map>
#include <string>

struct TestingResults {
    TestingResults() {
        data[CNS_TAG_ATTR_TIME] = std::map<int, double>();
        data[CNS_TAG_ATTR_MAKESPAN] = std::map<int, double>();
        data[CNS_TAG_ATTR_FLOWTIME] = std::map<int, double>();
        data[CNS_TAG_ATTR_LLE] = std::map<int, double>();
        data[CNS_TAG_ATTR_LLN] = std::map<int, double>();
        data[CNS_TAG_ATTR_HLE] = std::map<int, double>();
        data[CNS_TAG_ATTR_HLN] = std::map<int, double>();
    }

    std::vector<std::string> getKeys() {
        std::vector<std::string> keys;
        for (auto pair : data) {
            keys.push_back(pair.first);
        }
        return keys;
    }

    std::map<std::string, std::map<int, double>> data;
};

#endif // TESTING_RESULTS_H
