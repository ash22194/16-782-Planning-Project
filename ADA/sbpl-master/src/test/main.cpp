/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

#include <sbpl/headers.h>
#include <sbpl/utils/2Dgridsearch.h>

enum PlannerType
{
    INVALID_PLANNER_TYPE = -1,
    PLANNER_TYPE_ADSTAR,
    PLANNER_TYPE_ARASTAR,
    PLANNER_TYPE_PPCP,
    PLANNER_TYPE_RSTAR,
    PLANNER_TYPE_VI,
    PLANNER_TYPE_ANASTAR,

    NUM_PLANNER_TYPES
};

std::string PlannerTypeToStr(PlannerType plannerType)
{
    switch (plannerType) {
    case PLANNER_TYPE_ADSTAR:
        return std::string("adstar");
    case PLANNER_TYPE_ARASTAR:
        return std::string("arastar");
    case PLANNER_TYPE_PPCP:
        return std::string("ppcp");
    case PLANNER_TYPE_RSTAR:
        return std::string("rstar");
    case PLANNER_TYPE_VI:
        return std::string("vi");
    case PLANNER_TYPE_ANASTAR:
        return std::string("anastar");
    default:
        return std::string("invalid");
    }
}

PlannerType StrToPlannerType(const char* str)
{
    if (!strcmp(str, "adstar")) {
        return PLANNER_TYPE_ADSTAR;
    }
    else if (!strcmp(str, "arastar")) {
        return PLANNER_TYPE_ARASTAR;
    }
    else if (!strcmp(str, "ppcp")) {
        return PLANNER_TYPE_PPCP;
    }
    else if (!strcmp(str, "rstar")) {
        return PLANNER_TYPE_RSTAR;
    }
    else if (!strcmp(str, "vi")) {
        return PLANNER_TYPE_VI;
    }
    else if (!strcmp(str, "anastar")) {
        return PLANNER_TYPE_ANASTAR;
    }
    else {
        return INVALID_PLANNER_TYPE;
    }
}

enum EnvironmentType
{
    INVALID_ENV_TYPE = -1, ENV_TYPE_2D, ENV_TYPE_2DUU, ENV_TYPE_XYTHETA, ENV_TYPE_XYTHETAMLEV, ENV_TYPE_ROBARM,

    NUM_ENV_TYPES
};

std::string EnvironmentTypeToStr(EnvironmentType environmentType)
{
    switch (environmentType) {
    case ENV_TYPE_2D:
        return std::string("2d");
    case ENV_TYPE_2DUU:
        return std::string("2duu");
    case ENV_TYPE_XYTHETA:
        return std::string("xytheta");
    case ENV_TYPE_XYTHETAMLEV:
        return std::string("xythetamlev");
    case ENV_TYPE_ROBARM:
        return std::string("robarm");
    default:
        return std::string("invalid");
    }
}

EnvironmentType StrToEnvironmentType(const char* str)
{
    if (!strcmp(str, "2d")) {
        return ENV_TYPE_2D;
    }
    else if (!strcmp(str, "2duu")) {
        return ENV_TYPE_2DUU;
    }
    else if (!strcmp(str, "xytheta")) {
        return ENV_TYPE_XYTHETA;
    }
    else if (!strcmp(str, "xythetamlev")) {
        return ENV_TYPE_XYTHETAMLEV;
    }
    else if (!strcmp(str, "robarm")) {
        return ENV_TYPE_ROBARM;
    }
    else {
        return INVALID_ENV_TYPE;
    }
}

enum MainResultType
{
    INVALID_MAIN_RESULT = -1,

    MAIN_RESULT_SUCCESS = 0,
    MAIN_RESULT_FAILURE = 1,
    MAIN_RESULT_INSUFFICIENT_ARGS = 2,
    MAIN_RESULT_INCORRECT_OPTIONS = 3,
    MAIN_RESULT_UNSUPPORTED_ENV = 4,

    NUM_MAIN_RESULTS
};

/*******************************************************************************
 * PrintUsage - Prints the proper usage of the sbpl test executable.
 *
 * @param argv The command-line arguments; used to determine the name of the
 *             test executable.
 *******************************************************************************/
void PrintUsage(char *argv[])
{
    printf("USAGE: %s [-s] [--env=<env_t>] [--planner=<planner_t>] [--search-dir=<search_t>] <cfg file> [mot prims]\n",
           argv[0]);
    printf("See '%s -h' for help.\n", argv[0]);
}

/*******************************************************************************
 * PrintHelp - Prints a help prompt to the command line when the -h option is
 *             used.
 *
 * @param argv The command line arguments; used to determine the name of the
 *             test executable
 *******************************************************************************/
void PrintHelp(char** argv)
{
    printf("\n");
    printf("Search-Based Planning Library\n");
    printf("\n");
    printf("    %s -h\n", argv[0]);
    printf("    %s [-s] [--env=<env_t>] [--planner=<planner_t>] [--search-dir=<search_t>] <env cfg> [mot prim]\n",
           argv[0]);
    printf("\n");
    printf("[-s]                      (optional) Find a solution for an example navigation\n");
    printf("                          scenario where the robot only identifies obstacles as\n");
    printf("                          it approaches them.\n");
    printf("[--env=<env_t>]           (optional) Select an environment type to choose what\n");
    printf("                          example to run. The default is \"xytheta\".\n");
    printf("<env_t>                   One of 2d, xytheta, xythetamlev, robarm.\n");
    printf("[--planner=<planner_t>]   (optional) Select a planner to use for the example.\n");
    printf("                          The default is \"arastar\".\n");
    printf("<planner_t>               One of arastar, adstar, rstar, anastar.\n");
    printf("[--search-dir=<search_t>] (optional) Select the type of search to run. The default\n");
    printf("                          is \"backwards\".\n");
    printf("<search_t>                One of backward, forward.\n");
    printf("<env cfg>                 Config file representing the environment configuration.\n");
    printf("                          See sbpl/env_examples/ for examples.\n");
    printf("[mot prim]                (optional) Motion primitives file for x,y,theta lattice\n");
    printf("                          planning. See sbpl/matlab/mprim/ for examples.\n");
    printf("                          NOTE: resolution of motion primtives should match that\n");
    printf("                              of the config file.\n");
    printf("                          NOTE: optional use of these for x,y,theta planning is\n");
    printf("                              deprecated.\n");
    printf("\n");
}

/*******************************************************************************
 * CheckIsNavigating
 * @brief Returns whether the -s option is being used.
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return whether the -s option was passed in on the cmd line
 *******************************************************************************/
bool CheckIsNavigating(int numOptions, char** argv)
{
    for (int i = 1; i < numOptions + 1; i++) {
        if (strcmp(argv[i], "-s") == 0) {
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * CheckSearchDirection -
 * @brief Returns the search direction being used
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return A string representing the search direction; "backward" by default
 ******************************************************************************/
std::string CheckSearchDirection(int numOptions, char** argv)
{
    int optionLength = strlen("--search-dir=");
    for (int i = 1; i < numOptions + 1; i++) {
        if (strncmp("--search-dir=", argv[i], optionLength) == 0) {
            std::string s(&argv[i][optionLength]);
            return s;
        }
    }
    return std::string("backward");
}

/*******************************************************************************
 * CheckEnvironmentType
 * @brief Returns the environment being used
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return A string denoting the environment type; "xytheta" by default
 *******************************************************************************/
std::string CheckEnvironmentType(int numOptions, char** argv)
{
    int optionLength = strlen("--env=");
    for (int i = 1; i < numOptions + 1; i++) {
        if (strncmp("--env=", argv[i], optionLength) == 0) {
            std::string s(&argv[i][optionLength]);
            return s;
        }
    }
    return std::string("xytheta");
}

/*******************************************************************************
 * CheckPlannerType - Checks for a planner specifier passed in through the
 *                    command line. This determines what planner to run in
 *                    the example. If none is found, ARA* is assumed.
 *
 * @param numOptions The number of options passed through the command line
 * @param argv The command-line arguments
 * @return A string denoting the planner type; "arastar" by default
 ******************************************************************************/
std::string CheckPlannerType(int numOptions, char** argv)
{
    int optionLength = strlen("--planner=");
    for (int i = 1; i < numOptions + 1; i++) {
        if (strncmp("--planner=", argv[i], optionLength) == 0) {
            std::string s(&argv[i][optionLength]);
            return s;
        }
    }
    return std::string("arastar");
}


void run_planner(SBPLPlanner* & planner, EnvironmentNAV2D & environment_nav2D, EnvironmentNAV2D & trueenvironment_nav2D, FILE* & fSol, bool & bPrint, 
    int & size_x, int & size_y, int* & map, int & startx, int & starty, int & goalx, 
    int & goaly, double & allocated_time_secs_foreachplan, vector<int> & solution_stateIDs_V, 
    int & plantime_over1secs, int & plantime_over0p5secs, int & plantime_over0p1secs, int & plantime_over0p05secs, int & plantime_below0p05secs,
    int & obsthresh, vector<nav2dcell_t> & changedcellsV){    


    int x, y;
    bool bChanges = (changedcellsV.size() > 0);
    vector<int> preds_of_changededgesIDV;

    preds_of_changededgesIDV.clear();
    

    double TimeStarted = clock();

    if (bChanges) {
        //get the affected states
        environment_nav2D.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
        //let know the incremental planner about them
        ((ADPlanner*)planner)->update_preds_of_changededges(&preds_of_changededgesIDV);
    }
    //planner.force_planning_from_scratch();

    fprintf(fSol, "%d %d ", startx, starty);

    //plan a path
    bool bPlanExists = false;
    while (bPlanExists == false) {
        printf("new planning...\n");
        bPlanExists = (planner->replan(allocated_time_secs_foreachplan, &solution_stateIDs_V) == 1);
        printf("done with the solution of size=%d\n", (unsigned int)solution_stateIDs_V.size());
        environment_nav2D.PrintTimeStat(stdout);
        if (bPlanExists == false) {
            throw SBPL_Exception();
        }

        //for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        //environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
        //}
        //fprintf(fSol, "*********\n");
    }

    double plantime_secs = (clock() - TimeStarted) / ((double)CLOCKS_PER_SEC);
    fprintf(fSol, "%.5f %.5f\n", plantime_secs, planner->get_solution_eps());
    fflush(fSol);
    if (plantime_secs > 1.0)
        plantime_over1secs++;
    else if (plantime_secs > 0.5)
        plantime_over0p5secs++;
    else if (plantime_secs > 0.1)
        plantime_over0p1secs++;
    else if (plantime_secs > 0.05)
        plantime_over0p05secs++;
    else
        plantime_below0p05secs++;

    //print the map
    int startindex = startx + starty * size_x;
    int goalindex = goalx + goaly * size_x;
    for (y = 0; bPrint && y < size_y; y++) {
        for (x = 0; x < size_x; x++) {
            int index = x + y * size_x;

            //check to see if it is on the path
            bool bOnthePath = false;
            for (int j = 1; j < (int)solution_stateIDs_V.size(); j++) {
                int newx, newy;
                environment_nav2D.GetCoordFromState(solution_stateIDs_V[j], newx, newy);
                if (x == newx && y == newy) bOnthePath = true;
            }

            if (index != startindex && index != goalindex && !bOnthePath)
                printf("%3d ", map[index]);
            else if (index == startindex)
                printf("  R ");
            else if (index == goalindex)
                printf("  G ");
            else if (bOnthePath)
                printf("  * ");
            else
                printf("  ? ");
        }
        printf("\n");
    }
    if (bPrint) printf("System Pause (return=%d)\n", system("pause"));

    //move along the path
    if (bPlanExists && (int)solution_stateIDs_V.size() > 1) {
        //get coord of the successor
        int newx, newy;
        environment_nav2D.GetCoordFromState(solution_stateIDs_V[1], newx, newy);

        if (trueenvironment_nav2D.GetMapCost(newx, newy) >= obsthresh) {
            throw SBPL_Exception("ERROR: robot is commanded to move into an obstacle");
        }

        //move
        printf("moving from %d %d to %d %d\n", startx, starty, newx, newy);
        startx = newx;
        starty = newy;

        //update the environment
        environment_nav2D.SetStart(startx, starty);

        //update the planner
        if (planner->set_start(solution_stateIDs_V[1]) == 0) {
            throw SBPL_Exception("ERROR: failed to update robot pose in the planner");
        }
    }
}

void initialize_adplanning_2d(char* envCfgFilename, 
    SBPLPlanner* & planner, EnvironmentNAV2D & environment_nav2D, EnvironmentNAV2D & trueenvironment_nav2D, 
    bool bPrint, int & size_x, int & size_y, int* & map, int & startx, int & starty, int & goalx, int & goaly, 
    double & allocated_time_secs_foreachplan,
    int & plantime_over1secs, int & plantime_over0p5secs, int & plantime_over0p1secs, int & plantime_over0p05secs, 
    int & plantime_below0p05secs, int & obsthresh, int & is_16_connected){

    MDPConfig MDPCfg;

    //int dx[8] = {-1, -1, -1,  0,  0,  1,  1,  1};
    //int dy[8] = {-1,  0,  1, -1,  1, -1,  0,  1};

    int x, y;    
    srand(0);

    //set parameters - should be done before initialization
    if (!trueenvironment_nav2D.SetEnvParameter("is16connected", is_16_connected)) {
        throw SBPL_Exception("ERROR: failed to set parameters");
    }
    if (!environment_nav2D.SetEnvParameter("is16connected", is_16_connected)) {
        throw SBPL_Exception("ERROR: failed to set parameters");
    }

    //initialize true map and robot map
    if (!trueenvironment_nav2D.InitializeEnv(envCfgFilename)) {
        throw SBPL_Exception("ERROR: InitializeEnv failed");
    }


    trueenvironment_nav2D.GetEnvParms(&size_x, &size_y, &startx, &starty, &goalx, &goaly, &obsthresh);
    map = (int*)calloc(size_x * size_y, sizeof(int));

    //print the map
    if (bPrint) printf("true map:\n");
    for (y = 0; bPrint && y < size_y; y++) {
        for (x = 0; x < size_x; x++) {
            printf("%d ", (int)trueenvironment_nav2D.IsObstacle(x, y));
        }
        printf("\n");
    }
    if (bPrint) printf("System Pause (return=%d)\n", system("pause"));

    //Initialize Environment (should be called before initializing anything else)
    if (!environment_nav2D.InitializeEnv(size_x, size_y, map, startx, starty, goalx, goaly, obsthresh)) {
        throw SBPL_Exception("ERROR: InitializeEnv failed");
    }

    //Initialize MDP Info
    if (!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }

    //create a planner
    
    bool bforwardsearch = false;


    printf("Initializing ADPlanner...\n");
    planner = new ADPlanner(&environment_nav2D, bforwardsearch);
    

    planner->set_initialsolution_eps(2.0);

    //set the start and goal configurations
    if (planner->set_start(MDPCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw SBPL_Exception("ERROR: failed to set start state");
    }
    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw SBPL_Exception("ERROR: failed to set goal state");
    }

    //set search mode
    planner->set_search_mode(false);    
    printf("End Initializing ADPlanner\n");    

}

int main(int argc, char *argv[])
{
    // Check command line arguments to find environment type and whether or not to
    // use one of the navigating examples.

    bool forwardSearch = false;
    int is_16_connected = 1; // == 0 [8 connected] , == 1 [16 connected]

    // Launch the correct example given the planner and an environment file.
    EnvironmentNAV2D environment_nav2D;
    EnvironmentNAV2D trueenvironment_nav2D;
    vector<int> solution_stateIDs_V;
    SBPLPlanner* planner = NULL;
    double allocated_time_secs_foreachplan = 1.0; // 0.2; //in seconds
    int size_x = -1, size_y = -1;
    int startx = 0, starty = 0;
    int goalx = -1, goaly = -1;
    
    const char* sol = "sol.txt";
    FILE* fSol = fopen(sol, "w");
    if (fSol == NULL) {
        throw SBPL_Exception("ERROR: could not open solution file");
    }
    
    bool bPrint = false;   
    int obsthresh = 0;
    int plantime_over1secs = 0, plantime_over0p5secs = 0, plantime_over0p1secs = 0, plantime_over0p05secs = 0, plantime_below0p05secs = 0;
    int* map;

    initialize_adplanning_2d("../env_examples/nav2d/env1.cfg", planner, environment_nav2D, trueenvironment_nav2D, 
        bPrint, size_x, size_y, map, startx, starty, goalx, goaly, allocated_time_secs_foreachplan, 
        plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs, plantime_below0p05secs, obsthresh, is_16_connected);

    //now comes the main loop
    int goalthresh = 0;
    
    
    nav2dcell_t nav2dcell;
    vector<nav2dcell_t> changedcellsV;
    int solution_step = 0;

    while (abs(startx - goalx) > goalthresh || abs(starty - goaly) > goalthresh) {

        changedcellsV.clear();

        int dX = 0;
        int dY = 0;
        for (dX = -2; dX <= 2; dX++) {
            for (dY = -2; dY <= 2; dY++) {

                int x = startx + dX;
                int y = starty + dY;
                if (x < 0 || x >= size_x || y < 0 || y >= size_y) {
                    continue;
                }

                int index = x + y * size_x;      
                unsigned char truecost = trueenvironment_nav2D.GetMapCost(x, y); 

                if (map[index] != truecost) {                
                    map[index] = truecost;
                    environment_nav2D.UpdateCost(x, y, map[index]);
                    printf("setting cost[%d][%d] to %d\n", x, y, map[index]);
                    // store the changed cells
                    nav2dcell.x = x;
                    nav2dcell.y = y;
                    changedcellsV.push_back(nav2dcell);
                }

            }
        }

        run_planner(planner, environment_nav2D, trueenvironment_nav2D, fSol, bPrint, size_x, size_y, map, startx, starty, goalx, goaly, 
            allocated_time_secs_foreachplan, solution_stateIDs_V,
            plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs, plantime_below0p05secs, 
            obsthresh, changedcellsV);
        solution_step += 1;

    } 

    //print stats
    printf("stats: plantimes over 1 secs=%d; over 0.5 secs=%d; over 0.1 secs=%d; "
           "over 0.05 secs=%d; below 0.05 secs=%d; solution size=%d\n",
           plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs,
           plantime_below0p05secs, solution_step);
    fprintf(fSol, "stats: plantimes over 1 secs=%d; over 0.5; secs=%d; over 0.1 secs=%d; "
            "over 0.05 secs=%d; below 0.05 secs=%d; solution size=%d\n",
            plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs,
            plantime_below0p05secs, solution_step);

    if (bPrint) printf("System Pause (return=%d)\n", system("pause"));

    fflush(NULL);
    fclose(fSol);

    delete planner;


    //return plannerRes == 1 ? MAIN_RESULT_SUCCESS : MAIN_RESULT_FAILURE;
    return 0;
}
