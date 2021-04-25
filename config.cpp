#include "config.h"
#include "gl_const.h"
#include "tinyxml2.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <math.h>

Config::Config()
{
    LogParams = nullptr;
    SearchParams = nullptr;
}

Config::~Config()
{
    if (SearchParams) delete[] SearchParams;
    if (LogParams) delete[] LogParams;
}

bool Config::getConfig(const char *FileName)
{
    std::string value;
    std::stringstream stream;
    tinyxml2::XMLElement *root = 0, *algorithm = 0, *element = 0, *options = 0;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' element found in XML file!" << std::endl;
        return false;
    }

    options = getChild(root, CNS_TAG_OPT);
    if (!options) {
        return false;
    }

    LogParams = new std::string[3];
    LogParams[CN_LP_PATH] = "";
    LogParams[CN_LP_NAME] = "";

    element = options->FirstChildElement(CNS_TAG_LOGPATH);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_LOGPATH << "' tag found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_LOGPATH << "' tag was defined to 'current directory'." << std::endl;
    }
    else if (!element->GetText()) {
        std::cout << "Warning! Value of '" << CNS_TAG_LOGPATH << "' tag is missing!" << std::endl;
        std::cout << "Value of '" << CNS_TAG_LOGPATH << "' tag was defined to 'current directory'." << std::endl;
    }
    else {
        LogParams[CN_LP_PATH] = element->GetText();
    }


    element = options->FirstChildElement(CNS_TAG_LOGFN);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_LOGFN << "' tag found in XML file!" << std::endl;
        std::cout << "Value of '" << CNS_TAG_LOGFN
                  << "' tag was defined to default (original filename +'_log' + original file extension."
                  << std::endl;
    }
    else if (!element->GetText()) {
        std::cout << "Warning! Value of '" << CNS_TAG_LOGFN << "' tag is missing." << std::endl;
        std::cout << "Value of '" << CNS_TAG_LOGFN
                  << "' tag was defined to default (original filename +'_log' + original file extension."
                  << std::endl;
    }
    else
        LogParams[CN_LP_NAME] = element->GetText();

    bool success = true;
    success = success && getText(options, CNS_TAG_AGENTS_FILE, agentsFile);

    tinyxml2::XMLElement *range = getChild(options, CNS_TAG_AGENTS_RANGE, false);
    if (range) {
        getValueFromAttribute(range, CNS_TAG_AGENTS_RANGE, CNS_TAG_MIN, "int", &minAgents);
        getValueFromAttribute(range, CNS_TAG_AGENTS_RANGE, CNS_TAG_MAX, "int", &maxAgents);
    }
    if (minAgents < 1 || (maxAgents != -1 && maxAgents < minAgents)) {
        std::cout << "Error! '" << CNS_TAG_MIN << "' value or '" << CNS_TAG_MAX << "' value is incorrect." << std::endl;
        return false;
    }

    getValueFromText(options, CNS_TAG_TASKS_COUNT, "int", &tasksCount);
    getValueFromText(options, CNS_TAG_MAXTIME, "int", &maxTime);
    getValueFromText(options, CNS_TAG_SINGLE_EX, "bool", &singleExecution);
    getValueFromText(options, CNS_TAG_AR, "bool", &saveAggregatedResults);
    getValueFromText(options, CNS_TAG_AGENTS_STEP, "int", &agentsStep);


    algorithm = getChild(root, CNS_TAG_ALG);
    if (!algorithm) {
        return false;
    }

    success = success && getText(algorithm, CNS_TAG_PLANNER, value);
    if (value == CNS_ST_CBS || value == CNS_ST_ECBS) {
        searchType = CN_ST_CBS;
        if (value == CNS_ST_ECBS) {
            withFocalSearch = true;
        }
    } else if (value == CNS_ST_ACBS) {
        searchType = CN_ST_ACBS;
        withFocalSearch = true;
    } else if (value == CNS_ST_PR) {
        searchType = CN_ST_PR;
    } else if (value == CNS_ST_PP) {
        searchType = CN_ST_PP;
    } else {
        std::cout << "Error! Planner name '" << value << "' is unknown." << std::endl;
        return false;
    }

    std::string lowLevelSearch;
    success = success && getText(algorithm, CNS_TAG_LOW_LEVEL, lowLevelSearch);

    if (!success) {
        std::cout << "Error! Incorrect config: one or more required parameters are missing." << std::endl;
        return false;
    }

    if (lowLevelSearch == CNS_SP_ST_ASTAR) {
        lowLevel = CN_SP_ST_ASTAR;
    } else if (lowLevelSearch == CNS_SP_ST_SIPP) {
        lowLevel = CN_SP_ST_SIPP;
    } else if (lowLevelSearch == CNS_SP_ST_ZSCIPP) {
        lowLevel = CN_SP_ST_ZSCIPP;
    } else if (lowLevelSearch == CNS_SP_ST_FS) {
        lowLevel = CN_SP_ST_FS;
    } else if (lowLevelSearch == CNS_SP_ST_SCIPP) {
        lowLevel = CN_SP_ST_SCIPP;
    } else {
        std::cout << "Error! Low level search '" << lowLevelSearch << "' is unknown." << std::endl;
        return false;
    }

    if (searchType == CN_ST_CBS && !withFocalSearch && lowLevel != CN_SP_ST_ASTAR && lowLevel != CN_SP_ST_SIPP) {
        std::cout << "Warning! Specified low level search can not be used in this algorithm. Using A* instead." << std::endl;
        lowLevel = CN_SP_ST_ASTAR;
    }

    getValueFromText(algorithm, CNS_TAG_WITH_CAT, "bool", &withCAT);
    getValueFromText(algorithm, CNS_TAG_WITH_PH, "bool", &withPerfectHeuristic);
    getValueFromText(algorithm, CNS_TAG_PP_ORDER, "int", &ppOrder);
    getValueFromText(algorithm, CNS_TAG_PAR_PATHS_1, "bool", &parallelizePaths1);
    getValueFromText(algorithm, CNS_TAG_PAR_PATHS_2, "bool", &parallelizePaths2);
    getValueFromText(algorithm, CNS_TAG_CARD_CONF, "bool", &withCardinalConflicts);
    getValueFromText(algorithm, CNS_TAG_BYPASSING, "bool", &withBypassing);
    getValueFromText(algorithm, CNS_TAG_WITH_MH, "bool", &withMatchingHeuristic);
    getValueFromText(algorithm, CNS_TAG_WITH_DS, "bool", &withDisjointSplitting);
    getValueFromText(algorithm, CNS_TAG_FOCAL_W, "double", &focalW);
    getValueFromText(algorithm, CNS_TAG_SFO, "bool", &genSuboptFromOpt);

    parallelizePaths1 = parallelizePaths1 || parallelizePaths2;
    storeConflicts = withFocalSearch || withBypassing || withMatchingHeuristic || withDisjointSplitting;
    withCardinalConflicts = withCardinalConflicts || withMatchingHeuristic || withDisjointSplitting;
    return true;
}

bool Config::getValueFromText(tinyxml2::XMLElement *elem, const char *name, const char *typeName, void *field) {
    tinyxml2::XMLElement *child;
    child = elem->FirstChildElement(name);
    if (!child) {
        std::cout << "Warning! No '" << name << "' tag found in XML file! Using deafult value." << std::endl;
        return false;
    }

    tinyxml2::XMLError res;
    if (typeName == "int") {
        res = child->QueryIntText((int*)field);
    } else if (typeName == "bool") {
        res = child->QueryBoolText((bool*)field);
    } else if (typeName == "double") {
        res = child->QueryDoubleText((double*)field);
    }
    if (res != tinyxml2::XML_SUCCESS) {
        std::cout << "Warning! Couldn't get value from '" << name << "' tag! Using deafult value." << std::endl;
        return false;
    }
    return true;
}

bool Config::getValueFromAttribute(tinyxml2::XMLElement *elem, const char *elemName, const char *attrName, const char *typeName, void *field) {
    tinyxml2::XMLError res;
    if (typeName == "int") {
        res = elem->QueryIntAttribute(attrName, (int*)field);
    } else if (typeName == "bool") {
        res = elem->QueryBoolAttribute(attrName, (bool*)field);
    } else if (typeName == "double") {
        res = elem->QueryDoubleAttribute(attrName, (double*)field);
    }
    if (res != tinyxml2::XML_SUCCESS) {
        std::cout << "Warning! Couldn't get value from '" << attrName << "' attribute in '" <<
                    elemName << "' tag! Using deafult value." << std::endl;
        return false;
    }
    return true;
}

tinyxml2::XMLElement* Config::getChild(tinyxml2::XMLElement *elem, const char *name, bool printError) {
    tinyxml2::XMLElement *child = elem->FirstChildElement(name);
    if (!child) {
        std::cout << (printError ? "Error" : "Warning") << "! No '"
                  << name << "' tag found in XML file!";
        if (!printError) {
            std::cout << " Using default value.";
        }
        std::cout << std::endl;
        return 0;
    }
    return child;
}

bool Config::getText(tinyxml2::XMLElement *elem, const char *name, std::string &field) {
    tinyxml2::XMLElement *child = getChild(elem, name);
    if (!child) {
        return false;
    }
    field = child->GetText();
    return true;
}
