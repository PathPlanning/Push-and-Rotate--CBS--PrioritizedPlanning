#-------------------------------------------------
#
# Project created by QtCreator 2011-02-26T12:08:02
#
#-------------------------------------------------

TARGET = ASearch
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11 -Wall


#QMAKE_CXXFLAGS += -O0
#QMAKE_CXXFLAGS -= -O1
#QMAKE_CXXFLAGS -= -O2
#QMAKE_CXXFLAGS -= -O3

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}

SOURCES += \
    agent.cpp \
    agent_set.cpp \
    conflict_avoidance_table.cpp \
    conflict_based_search.cpp \
    conflict_set.cpp \
    constraints_set.cpp \
    mdd.cpp \
    prioritized_planning.cpp \
    push_and_rotate.cpp \
    tinyxml2.cpp \
    xmllogger.cpp \
    isearch.cpp \
    mission.cpp \
    map.cpp \
    dijkstra.cpp \
    config.cpp \
    astar.cpp \
    main.cpp \
    environmentoptions.cpp

HEADERS += \
    agent.h \
    agent_move.h \
    agent_set.h \
    cbs_node.h \
    conflict.h \
    conflict_avoidance_table.h \
    conflict_based_search.h \
    conflict_set.h \
    constraint.h \
    constraints_set.h \
    mdd.h \
    multiagent_search_result.h \
    prioritized_planning.h \
    push_and_rotate.h \
    tinyxml2.h \
    node.h \
    gl_const.h \
    xmllogger.h \
    isearch.h \
    mission.h \
    map.h \
    ilogger.h \
    dijkstra.h \
    config.h \
    astar.h \
    searchresult.h \
    environmentoptions.h

DISTFILES += \
    CMakeLists.txt \
    README-EN.md \
    README.md
