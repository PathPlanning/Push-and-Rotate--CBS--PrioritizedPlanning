#ifndef GL_CONST_H
#define	GL_CONST_H

#define CN_PI_CONSTANT 3.14159265359
#define CN_SQRT_TWO    1.41421356237
#define CN_INFINITY    1000000000

//XML tags
#define CNS_TAG_ROOT "root"

    #define CNS_TAG_MAP             "map"
        #define CNS_TAG_CELLSIZE    "cellsize"
        #define CNS_TAG_WIDTH       "width"
        #define CNS_TAG_HEIGHT      "height"
        #define CNS_TAG_STX         "startx"
        #define CNS_TAG_STY         "starty"
        #define CNS_TAG_FINX        "finishx"
        #define CNS_TAG_FINY        "finishy"
        #define CNS_TAG_GRID        "grid"
            #define CNS_TAG_ROW     "row"

    #define CNS_TAG_ALG             "algorithm"
        #define CNS_TAG_ST          "searchtype"
        #define CNS_TAG_HW          "hweight"
        #define CNS_TAG_MT          "metrictype"
        #define CNS_TAG_BT          "breakingties"
        #define CNS_TAG_AS          "allowsqueeze"
        #define CNS_TAG_AD          "allowdiagonal"
        #define CNS_TAG_CC          "cutcorners"

    #define CNS_TAG_OPT             "options"
        #define CNS_TAG_LOGLVL      "loglevel"
        #define CNS_TAG_LOGPATH     "logpath"
        #define CNS_TAG_LOGFN       "logfilename"
        #define CNS_TAG_PLANNER     "planner"
        #define CNS_TAG_MAXTIME     "maxtime"
        #define CNS_TAG_AGENTS_FILE "agents_file"
        #define CNS_TAG_WITH_CAT    "with_cat"
        #define CNS_TAG_WITH_PH     "with_perfect_h"
        #define CNS_TAG_CARD_CONF   "with_card_conf"
        #define CNS_TAG_BYPASSING   "with_bypassing"
        #define CNS_TAG_WITH_MH     "with_matching_h"
        #define CNS_TAG_WITH_DS     "with_disjoint_splitting"
        #define CNS_TAG_FOCAL_W     "focal_w"
        #define CNS_TAG_WEIGHT      "weight"
        #define CNS_TAG_SFO         "gen_subopt_from_opt"
        #define CNS_TAG_LOW_LEVEL   "low_level"
        #define CNS_TAG_PP_ORDER    "pp_order"
        #define CNS_TAG_PAR_PATHS_1 "parallelize_paths_1"
        #define CNS_TAG_PAR_PATHS_2 "parallelize_paths_2"
        #define CNS_TAG_SINGLE_EX   "single_execution"
        #define CNS_TAG_TASKS_COUNT "tasks_count"
        #define CNS_TAG_AGENTS_RANGE "agents_range"
            #define CNS_TAG_MIN     "min"
            #define CNS_TAG_MAX     "max"

    #define CNS_TAG_LOG             "log"
        #define CNS_TAG_MAPFN       "mapfilename"
        #define CNS_TAG_TASKFN      "taskfilename"
        #define CNS_TAG_SUM         "summary"
        #define CNS_TAG_PATH        "path"
        #define CNS_TAG_RESULTS     "aggregated_results"
        #define CNS_TAG_RESULT      "result"
        #define CNS_TAG_SUMMARY     "summary"
        #define CNS_TAG_AGENT       "agent"
        #define CNS_TAG_LPLEVEL     "lplevel"
        #define CNS_TAG_HPLEVEL     "hplevel"
            #define CNS_TAG_SECTION "section"
        #define CNS_TAG_LOWLEVEL    "lowlevel"
            #define CNS_TAG_STEP    "step"
            #define CNS_TAG_OPEN    "open"
            #define CNS_TAG_POINT   "node"
            #define CNS_TAG_CLOSE   "close"

//XML tags' attributes
    #define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
    #define CNS_TAG_ATTR_NODESCREATED   "nodescreated"
    #define CNS_TAG_ATTR_LENGTH         "length"
    #define CNS_TAG_ATTR_LENGTH_SCALED  "length_scaled"
    #define CNS_TAG_ATTR_TIME           "time"
    #define CNS_TAG_ATTR_NAME           "name"
    #define CNS_TAG_ATTR_X              "x"
    #define CNS_TAG_ATTR_Y              "y"
    #define CNS_TAG_ATTR_NUM            "number"
    #define CNS_TAG_ATTR_ID             "id"
    #define CNS_TAG_ATTR_STARTX         "start.x"
    #define CNS_TAG_ATTR_STARTY         "start.y"
    #define CNS_TAG_ATTR_GOALX          "goal.x"
    #define CNS_TAG_ATTR_GOALY          "goal.y"
    #define CNS_TAG_ATTR_DUR            "duration"
    #define CNS_TAG_ATTR_PATH_FOUND     "pathfound"
    #define CNS_TAG_ATTR_COUNT          "agents_count"
    #define CNS_TAG_ATTR_MAKESPAN       "makespan"
    #define CNS_TAG_ATTR_FLOWTIME       "flowtime"
    #define CNS_TAG_ATTR_SC             "success_count"
    #define CNS_TAG_ATTR_F              "F"
    #define CNS_TAG_ATTR_G              "g"
    #define CNS_TAG_ATTR_PARX           "parent_x"
    #define CNS_TAG_ATTR_PARY           "parent_y"
    #define CNS_TAG_ATTR_STX            "start.x"
    #define CNS_TAG_ATTR_STY            "start.y"
    #define CNS_TAG_ATTR_FINX           "finish.x"
    #define CNS_TAG_ATTR_FINY           "finish.y"


//Search Parameters
    #define CNS_ST_CBS                  "cbs"
    #define CNS_ST_PR                   "push_and_rotate"
    #define CNS_ST_PP                   "prioritized_planning"
    #define CNS_ST_ECBS                 "ecbs"
    #define CNS_ST_ECBS_CT              "ecbs-ct"

    #define CN_ST_CBS                   0
    #define CN_ST_PR                    1
    #define CN_ST_PP                    2
    #define CN_ST_ECBS_CT               3

    #define CN_SP_ST 0

        #define CNS_SP_ST_ASTAR         "astar"
        #define CNS_SP_ST_FS            "focal_search"
        #define CNS_SP_ST_SIPP          "sipp"
        #define CNS_SP_ST_WSIPP         "weighted_sipp"
        #define CNS_SP_ST_SCIPP         "scipp"

        #define CN_SP_ST_ASTAR          0
        #define CN_SP_ST_TIME_ASTAR     1
        #define CN_SP_ST_FS             2
        #define CN_SP_ST_SIPP           3
        #define CN_SP_ST_WSIPP          4
        #define CN_SP_ST_SCIPP          5


    #define CN_SP_AD 1 //AllowDiagonal

    #define CN_SP_CC 2 //CutCorners

    #define CN_SP_AS 3 //AllowSqueeze

    #define CN_SP_MT 4 //MetricType

        #define CNS_SP_MT_DIAG  "diagonal"
        #define CNS_SP_MT_MANH  "manhattan"
        #define CNS_SP_MT_EUCL  "euclidean"
        #define CNS_SP_MT_CHEB  "chebyshev"

        #define CN_SP_MT_DIAG   0
        #define CN_SP_MT_MANH   1
        #define CN_SP_MT_EUCL   2
        #define CN_SP_MT_CHEB   3

    #define CN_SP_HW 5 //HeuristicWeight

    #define CN_SP_BT 6 //BreakingTies

        #define CNS_SP_BT_GMIN "g-min"
        #define CNS_SP_BT_GMAX "g-max"

        #define CN_SP_BT_GMIN 0
        #define CN_SP_BT_GMAX 1



    //Log Configuration
    #define CN_LP_LEVEL 0

        #define CN_LP_LEVEL_NOPE_VALUE      "0"
        #define CN_LP_LEVEL_NOPE_WORD       "none"
        #define CN_LP_LEVEL_TINY_VALUE      "0.5"
        #define CN_LP_LEVEL_TINY_WORD       "tiny"
        #define CN_LP_LEVEL_SHORT_VALUE     "1"
        #define CN_LP_LEVEL_SHORT_WORD      "short"
        #define CN_LP_LEVEL_MEDIUM_VALUE    "1.5"
        #define CN_LP_LEVEL_MEDIUM_WORD     "medium"
        #define CN_LP_LEVEL_FULL_VALUE      "2"
        #define CN_LP_LEVEL_FULL_WORD       "full"

    #define CN_LP_PATH 1
    #define CN_LP_NAME 2


//Grid Cell
    #define CN_GC_NOOBS 0
    #define CN_GC_OBS   1

//Other
    #define CNS_OTHER_PATHSELECTION     "*"
    #define CNS_OTHER_MATRIXSEPARATOR   ' '
    #define CNS_OTHER_POSITIONSEPARATOR ','

#endif

