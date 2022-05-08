#-------------------------------------------------
#
# Project created by QtCreator 2011-02-26T12:08:02
#
#-------------------------------------------------

TARGET = MultiAgentSearch
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
    anytime_cbs.cpp \
    conflict_avoidance_table.cpp \
    conflict_based_search.cpp \
    conflict_set.cpp \
    constraints_set.cpp \
    focal_lpa_star.cpp \
    focalsearch.cpp \
    lpa_star.cpp \
    mdd.cpp \
    prioritized_planning.cpp \
    push_and_rotate.cpp \
    replanning_astar.cpp \
    replanning_focal_search.cpp \
    scipp.cpp \
    search_queue.cpp \
    sipp.cpp \
    tinyxml2.cpp \
    xmllogger.cpp \
    isearch.cpp \
    mission.cpp \
    map.cpp \
    dijkstra.cpp \
    config.cpp \
    astar.cpp \
    main.cpp \
    zero_scipp.cpp

HEADERS += \
    agent.h \
    agent_move.h \
    agent_set.h \
    anytime_cbs.h \
    cbs_node.h \
    conflict.h \
    conflict_avoidance_table.h \
    conflict_based_search.h \
    conflict_set.h \
    constraint.h \
    constraints_set.h \
    flpa_node.h \
    focal_lpa_star.h \
    focalsearch.h \
    fs_node.h \
    lpa_node.h \
    lpa_star.h \
    mdd.h \
    multiagent_search_interface.h \
    multiagent_search_result.h \
    prioritized_planning.h \
    push_and_rotate.h \
    replanning_astar.h \
    replanning_astar_node.h \
    replanning_focal_search.h \
    replanning_fs_node.h \
    scipp.h \
    scipp_node.h \
    search_queue.h \
    sipp.h \
    sipp_node.h \
    testing_results.h \
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
    zero_scipp.h \
    zero_scipp_node.h

DISTFILES += \
    CMakeLists.txt \
    Examples/empty-16-16-random-1.xml \
    Examples/empty-16-16-random-2.xml \
    Examples/empty.xml \
    Examples/empty_batch_aggregated.xml \
    Examples/empty_batch_aggregated_log.xml \
    Examples/empty_batch_full.xml \
    Examples/empty_batch_full_log.xml \
    Examples/empty_log.xml \
    Examples/empty_single.xml \
    Examples/empty_single_log.xml \
    README-EN.md \
    README-RU.md \
    README.md
