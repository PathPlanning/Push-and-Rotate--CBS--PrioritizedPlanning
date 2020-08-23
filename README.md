# [Документация на русском](README-RU.md)
# Push-and-Rotate--CBS--PrioritizedPlanning
This project contains implementations of different algorithms designed for the multiagent pathfinding problem. Namely, following algorithms with some of their modifications are implemented: [Conflict based search](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239), [Enhanced conflict based search](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875), [Push and rotate](https://pdfs.semanticscholar.org/0a84/5fa6530f84b5df50d652a5e4eecc38d77681.pdf), [Prioritized planning](https://arxiv.org/pdf/1409.2399.pdf).

## Build and run

The project can be built with cmake using CMakeLists.txt file provided in repository. Also the project can be opened and built in Qt Creator using PathPlanning.pro file.

Input of the programm consists of several XML files: one main file and one or more additional files. Name of the main input file is passed through the first command line argument. This file contains the description of an environement and an algorithm and links for the additional input files with agents’ description. Output of the program is also saved in the XML file. Examples of input and output files are provided in `Examples` directory.

## Input data

The main file contains two sections `map` and `options`:

### Section `map` - map definition. Contains following tags:
- grid - contains attributes `width` and `height` specifing dimensions of the map. Map description is provided in the tag body, 0 means a traversable cell and 1 means an obstacle. Each row is included into `row` tag, number of rows must be equal to the `height` value and number of digits in every row must be equal to the `width` value.

### Section `algorithm` - definition of the algorihm parameters. Contains following tags:
- algorithm - algorithm to be used. Can take following values:
    1. cbs - Conflict based search
    2. ecbs - Enhanced conflict based search. In the high level search secondary heuristic h3 from the [article](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875) is used. Low level search depends on the low_level option
    3. push_and_rotate - Push and rotate
    4. prioritized_planning - Prioritized planning
- low_level - algorithm, applied in the low level search in CBS, ECBS and Prioritized planning algorithms. Can take following values:
    1. astar - algorithm [A*](https://www.cs.auckland.ac.nz/courses/compsci709s2c/resources/Mike.d/astarNilsson.pdf)
    2. sipp - algorithm [SIPP](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875) (discrete version for 4-connected grid)
    3. weighted_sipp - suboptimal version of SIPP algorithm described [here](https://docs.google.com/document/d/16NjWHubNFczPDGi3QqWkMrWE8ewQtOBXruUBaa9hQnw/edit)
    4. focal_search - focal search algorithm, as in original ECBS [paper](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875). Secondary heuristic is defined as number of vertex conflicts on the partial path to the current vertex
    5. scipp - [SCIPP](https://www.aaai.org/ocs/index.php/SOCS/SOCS19/paper/viewFile/18327/17443) algoritm (discrete version for 4-connected grid)
- agents_file - common prefix for the input files with agents’ description
- tasks_count - number of input files with agents’ description: in testing files with names of the form `agents_file-n.xml` are used for all `n` from 1 to tasks_count. Optional parameter, default value is 1
- agents_range - `min` and `max` attributes specify minimal and maximal number of agents for testing. When single_execution=`false`, number of agents is  gradually increased from `min` to `max` and the algorithm is being run on corresponding subset of agent set. If algorithm fails to find the solution or runs longer then some fixed time limit, testing of current scenario terminates. Optional parameter, by default minimal number of agents is 1 and maximal is the same as number of agents in the agents file
- maxtime - maximal running time of the algorihm in milliseconds. Optional parameter, default value is 1000
- with_perfect_h - find the shortest paths from all cells to agents goal positions to compute perfect heuristic for A* method (`true` or `false`, considered for CBS and Prioritized planning algorithms). Optional parameter, default value is false
- with_cat - use Conflict avodance table (`true` or `false`, considered for CBS and ECBS algorithms). Optional parameter, default value is false
- with_card_conf - use cardinal conflicts (as described [here](https://pdfs.semanticscholar.org/c072/38579a95c424707dbe855efba189cce68650.pdf)). Can be `true` or `false`, considered for CBS and ECBS algorithms. Optional parameter, default value is false
- with_bypassing - use conflict bypassing (as described [here](https://pdfs.semanticscholar.org/c072/38579a95c424707dbe855efba189cce68650.pdf)). Can be `true` or `false`, considered for CBS and ECBS algorithms. Optional parameter, default value is false
- with_matching_h - compute heuristic on vertices of constraint tree in CBS, based on maximal matching in cardinal conflicts graph. Described [here](http://idm-lab.org/bib/abstracts/papers/icaps18a.pdf) as ICBS-h1, can be `true` or `false`, considered for CBS and ECBS algorithms. Optional parameter, default value is false
- with_disjoint_splitting - use disjoint splitting. Described [here](http://idm-lab.org/bib/abstracts/papers/icaps19a.pdf), can be `true` or `false`, considered for CBS and ECBS algorithms. When using this option, with_card_conf option is set to `true`. Optional parameter, default value is false
- focal_w - weight used in ECBS high level search and in Focal search and SCIPP low level searches for construction of the FOCAL list. Also f-values of optimal nodes in weighted_sipp algorithm are multiplied by this value. In any case it is garantied that cost of the found solution will not exceed the optimal cost more than in focal_w times. Optional parameter, default value is 1.0
- gen_subopt_from_opt - generate suboptimal successors from optimal nodes in weighted_sipp alogrithm. Can be `true` or `false`, considered for low_level = `weighted_sipp`. Optional parameter, default value is false

- pp_order - specifies a method of agents priorities definition for Prioritized Planning. Can take following values:
    - 0 - agents are considered in the same order as in the input file
    - 1 - agents are considered in increasing order of manhattan distances from start to goal node
    - 2 - agents are considered in decreasing order of manhattan distances from start to goal node

    Optional parameter, default value is 0
- parallelize_paths_1 - use path parallelization technique in Push and rotate algorithm (without this option only one agent is moving at every time step in the solution). Can be `true` or `false`, considered for Push and rotate algorithm. Optional parameter, default value is false
- parallelize_paths_2 - use additional parallelization technique in Push and rotate algorithm (allows to increase parallelization ratio, but significantly slows algorihm down). Can be `true` or `false`, considered for Push and rotate algorithm. When using this option, option parallelize_paths_1 is set to `true`. Optional parameter, default value is false
- single_execution - can be `true` or `false`. If option is set to `true`, the algorithm will be executed only once for the first agents file with the number of agents equal to `max` attribute in agents_range option. Output file format will also be different (see "Output data" section). Optional parameter, default value is false
- aggregated_results - save separate testing results for each agents file or aggregated results over all agents files. Optional parameter, default value is true
- logpath - path to the directory, where log will be stored (optional parameter, by default log is stored to the same directory where the input file is located)
- logfilename - name of the log file (optional parameter, by default name of the log file has the form `input_file_name_log.xml` where "input_file_name.xml" is a name of the main input file)

### Files with agents’ descriptions
For each agent its own tag `agent` is provided with following attributes:
- id - agent’s id
- start_i, start_j - coordinates of agent’s start position (cells are numbered from 0, cell (0, 0) is in the left upper corner of the map, the first coordinate corresponds to the row number and the second to the column number)
- goal_i, goal_j - coordinates of agent’s goal position

## Output data
Output file contains `map` and `options` sections, similar to the input file, and also the `log` section with results of the execution. This section contains the `mapfilename` tag and several other tags depending on the value of single_execution and aggregated_results parameters.

If single_execution = `false`, aggregated_results  = `false`, one or more `results` tags are created. There is one `results` tag for each agents file containing several `result` tags for each number of agents. Every such tag has following attributes, descibing execution results for current agents file and current number of agents:

    1. agents_count - number of agents
    2. success_count - number of tests (among tasks_count input files) for which algorihm was able to find the solution in fixed time.
    3. makespan - number of time steps until the last agent stops moving (average)
    4. flowtime - total number of actions performed by each of the agents until they reach their goal positions (average)
    5. time - running time of the algorithm  (average)
    6. HL_expansions - number of high level expansions in CBS and ECBS (the size of CLOSE list at the end of the search).
    7. HL_nodes - number of high level nodes created in CBS and ECBS (the size of CLOSE list at the end of the search)
    8. LL_avg_expansions - average number of low level expansions in CBS and ECBS among all low level search executions
    9. LL_avg_nodes - average number of low level nodes created in CBS and ECBS among all low level search executions

[Example](Examples/empty_batch_full_log.xml) of output file for this mode.

If single_execution = `false`, aggregated_results  = `true`, one `results` tag with aggeregated testing results is created. As in the previous case, this tag contains several `result` tags with the same attributes for each number of agents. Tag attributes contain average values among all agent files for which algorithm was able to find the solution with current number of agents. [Example](Examples/empty_batch_aggregated_log.xml) of output file for this mode.

If single_execution=`true`, following tags are created (tag aggregated_results is not considered):
- taskfilename - name of the file with agents’ description
- summary - properties of the found solution. Contains `agents_count`, `makespan`, `flowtime`, `time`, `HL_expansions`, `HL_nodes`, `LL_avg_expansions`, `LL_avg_nodes` which are defined in the same way as stated above
- for each agent its own tag `agent` is provided with following attributes:
    - id - agent’s id
    - start.x, start.y - coordinates of agent’s start position
    - goal.x, goal.y - coordinates of agent’s goal position

  **Coordinate x corresponds to j coordinate, and y cooresponds to i coordinate.**

  This tag also includes tag `path` with `pathfound` attribute, which can be `true` or `false` depending on whether the solution was found. Tag `path` includes several `section` tags each of which corresponds to one agent’s action and contains the following tags:
    - id - section id
    - start.x, start.y - agent position before taking action
    - goal.x, goal.y - agent position after taking action (this position is either the same as the start position either adjacent to it)
    - duration - duration of the action (always equals 1)

[Example](Examples/empty_single_log.xml) of output file for this mode.
