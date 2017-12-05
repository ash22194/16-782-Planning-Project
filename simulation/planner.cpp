/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  ANAHITA
#include <sbpl/headers.h>
#include <sbpl/utils/2Dgridsearch.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  ANAHITA


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]
#define	EPSILON_IN  prhs[3]
#define	TIME_IN     prhs[4]

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]
#define	PLANCOST_OUT    plhs[2]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((X-1)*YSIZE + (YSIZE-Y+1-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

int temp = 0;


   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  ANAHITA
EnvironmentNAV2D environment_nav2D;
SBPLPlanner* planner_AD = NULL;
vector<int> solution_stateIDs_V;
int plantime_over1secs = 0, plantime_over0p5secs = 0, plantime_over0p1secs = 0, plantime_over0p05secs = 0, plantime_below0p05secs = 0;

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


void run_planner(SBPLPlanner* & planner_AD, EnvironmentNAV2D & environment_nav2D, FILE* & fSol, bool & bPrint, 
    int size_x, int size_y, double* & map, int startx, int starty, int goalx, 
    int goaly, double & allocated_time_secs_foreachplan, vector<int> & solution_stateIDs_V, 
    int & plantime_over1secs, int & plantime_over0p5secs, int & plantime_over0p1secs, int & plantime_over0p05secs, int & plantime_below0p05secs,
    int obsthresh, vector<nav2dcell_t> & changedcellsV, int* psolcost){    

    int x, y;
    bool bChanges = (changedcellsV.size() > 0);
    vector<int> preds_of_changededgesIDV;

    preds_of_changededgesIDV.clear();

    double TimeStarted = clock();

    if (bChanges) {
        cout << "MAP changed...!\n";
        //get the affected states
        environment_nav2D.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
        //let know the incremental planner about them
        ((ADPlanner*)planner_AD)->update_preds_of_changededges(&preds_of_changededgesIDV);
        cout << "MAP changed END!\n";
    }
    //planner_AD.force_planning_from_scratch();
    fprintf(fSol, "%d %d ", startx, starty);
    //plan a path
    bool bPlanExists = false;
    
    while (bPlanExists == false) {
        printf("new planning...\n");
        
        bPlanExists = (planner_AD->replan(allocated_time_secs_foreachplan, &solution_stateIDs_V, psolcost) == 1);
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
    fprintf(fSol, "%.5f %.5f\n", plantime_secs, planner_AD->get_solution_eps());
    
    
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
//             int index = GETMAPINDEX(x, y, size_x, size_y);
            //check to see if it is on the path
            bool bOnthePath = false;
            for (int j = 1; j < (int)solution_stateIDs_V.size(); j++) {
                int newx, newy;
                environment_nav2D.GetCoordFromState(solution_stateIDs_V[j], newx, newy);
                if (x == newx && y == newy) bOnthePath = true;
            }

            if (index != startindex && index != goalindex && !bOnthePath)
                fprintf(fSol, "%3d ", int(map[index]));
            else if (index == startindex)
                fprintf(fSol, "  R ");
            else if (index == goalindex)
                fprintf(fSol, "  G ");
            else if (bOnthePath)
                fprintf(fSol, "  * ");
            else
                fprintf(fSol, "  ? ");
        }
        fprintf(fSol, "\n");
    }
    if (bPrint) printf("System Pause (return=%d)\n", system("pause"));
    fflush(fSol);
    //move along the path

    if (bPlanExists && (int)solution_stateIDs_V.size() > 1) {
        //get coord of the successor
        int newx, newy;
        environment_nav2D.GetCoordFromState(solution_stateIDs_V[1], newx, newy);
        if (environment_nav2D.GetMapCost(newx, newy) >= obsthresh) {
            throw SBPL_Exception("ERROR: robot is commanded to move into an obstacle");
        }

        //move
        printf("moving from %d %d to %d %d\n", startx, starty, newx, newy);
//         startx = newx;
//         starty = newy;       
        
    }
    cout.flush();
}

void initialize_adplanning_2d(char* envCfgFilename, 
    SBPLPlanner* & planner_AD, EnvironmentNAV2D & environment_nav2D, 
    bool bPrint, int size_x, int size_y, int startx, int starty, int goalx, int goaly, 
    double & allocated_time_secs_foreachplan,
    int & plantime_over1secs, int & plantime_over0p5secs, int & plantime_over0p1secs, int & plantime_over0p05secs, 
    int & plantime_below0p05secs, int & obsthresh, int & is_16_connected, double* original_map){
    cout << "initialization began!" << endl;
//     addpath('/home/anahita/Courses/2017/Planning/Project/sbpl-master');
    MDPConfig MDPCfg;

    //int dx[8] = {-1, -1, -1,  0,  0,  1,  1,  1};
    //int dy[8] = {-1,  0,  1, -1,  1, -1,  0,  1};

    int x, y;    
    srand(0);
    
    //set parameters - should be done before initialization
    if (!environment_nav2D.EnvNAV2D.bInitialized && !environment_nav2D.SetEnvParameter("is16connected", is_16_connected)) {
        throw SBPL_Exception("ERROR2: failed to set parameters");
    }
   
    //Initialize Environment (should be called before initializing anything else)
    environment_nav2D.SetConfiguration(size_x, size_y, obsthresh, startx, starty, goalx, goaly, original_map);

    //Initialize MDP Info
    if (!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }

    cout << "environment initialized!" << endl;
    
    //initialize true map and robot map
//     trueenvironment_nav2D.SetConfiguration(size_x, size_y, obsthresh, startx, starty, goalx, goaly, original_map);    
// //     trueenvironment_nav2D.GetEnvParms(&size_x, &size_y, &startx, &starty, &goalx, &goaly, &obsthresh);   
//     trueenvironment_nav2D.EnvNAV2D.bInitialized = true; 
//     cout << "true environment initialized!" << endl; 
   
    //print the map
    if (bPrint) printf("true map:\n");
    for (y = 0; bPrint && y < size_y; y++) {
        for (x = 0; x < size_x; x++) {
            printf("%d ", (int)environment_nav2D.IsObstacle(x, y));
        }
        printf("\n");
    }
    if (bPrint) printf("System Pause (return=%d)\n", system("pause"));
  

    //create a planner
    
    bool bforwardsearch = false;
    
    cout << "Initializing ADPlanner...\n";
    planner_AD = new ADPlanner(&environment_nav2D, bforwardsearch);

    planner_AD->set_initialsolution_eps(2.0);

    cout << "ADPlanner set start ..." << MDPCfg.startstateid << "\n";
    //set the start and goal configurations
    if (planner_AD->set_start(MDPCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw SBPL_Exception("ERROR: failed to set start state");
    }
    cout << "ADPlanner set goal ..." << MDPCfg.goalstateid << "\n";
    if (planner_AD->set_goal(MDPCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw SBPL_Exception("ERROR: failed to set goal state");
    }
    
    //set search mode
    planner_AD->set_search_mode(false);    
    cout << "End Initializing ADPlanner\n";  
       
//     cout.flush();

}

static void planner(double*	original_map, int x_size, int y_size,
                    float robotposeX, float robotposeY, float robotposeTheta,
                    float goalposeX, float goalposeY, float goalposeTheta,
                    int numofDOFs, double*** plan, int* planlength, double* plancost ,
                    float epsilon, float time)
{  
    cout << "********************* planner.cpp *********************" << endl;
    
    
    // Check command line arguments to find environment type and whether or not to
    // use one of the navigating examples.

    bool forwardSearch = false;
    int is_16_connected = 1; // == 0 [8 connected] , == 1 [16 connected]

    // Launch the correct example given the planner and an environment file.
    
    

    
    double allocated_time_secs_foreachplan = time; // 0.2; //in seconds
//     int size_x = -1, size_y = -1;
//     int startx = 0, starty = 0;
//     int goalx = -1, goaly = -1;
    int size_x = x_size, size_y = y_size;
    int startx = (robotposeX-1), starty = (robotposeY-1);
    int goalx = (goalposeX-1), goaly = (goalposeY-1);
    
    cout << "start: " << startx << " " << starty << endl;
    cout << "goal: " << goalx << " " << goaly << endl;
    
    const char* sol = "sol.txt";
    FILE* fSol = fopen(sol, "a");
    if (fSol == NULL) {
        throw SBPL_Exception("ERROR: could not open solution file");
    }
    
    bool bPrint = true;   
    int obsthresh = 253;
        
    if (planner_AD == NULL){
        initialize_adplanning_2d("../sbpl-master/env_examples/nav2d/env1.cfg", planner_AD, environment_nav2D, 
            bPrint, size_x, size_y, startx, starty, goalx, goaly, allocated_time_secs_foreachplan, 
            plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs, plantime_below0p05secs, obsthresh, is_16_connected, original_map);
//         cout << "start: " << startx << " " << starty << endl;
        cout << "Initialization was successful!" << endl;
    }
    else{        
        environment_nav2D.SetStart(startx, starty);
        int stateID = environment_nav2D.GetStateFromCoord(startx, starty);
        if (planner_AD->set_start(stateID) == 0) {
            throw SBPL_Exception("ERROR: failed to update robot pose in the planner");
        }
//         cout << "start: " << startx << " " << starty << endl;
    }
    
    //now comes the main loop
    int goalthresh = 0;     
    
    nav2dcell_t nav2dcell;
    vector<nav2dcell_t> changedcellsV;
    int solution_step = 0;
    
//     while (abs(startx - goalx) > goalthresh || abs(starty - goaly) > goalthresh) {
    cout << "Update Map... !" << endl;
    changedcellsV.clear();

    int x = 0;
    int y = 0;
    for (x = 0; x < size_x; x++) {
        for (y = 0; y < size_y; y++) {
            int index = x + y * size_x;      
            int truecost = int(original_map[index]); 
            int prevcost = environment_nav2D.GetMapCost(x, y); 

            if (prevcost != truecost) {                
                environment_nav2D.UpdateCost(x, y, truecost);
                printf("setting cost[%d][%d] to %d\n", x, y, truecost);
                // store the changed cells
                nav2dcell.x = x;
                nav2dcell.y = y;
                changedcellsV.push_back(nav2dcell);
                
            }

        }
    }
    cout << "\n";
//     cout << "start: " << startx << " " << starty << endl;
    cout << "Update Map done!" << endl;
    int psolcost;
    run_planner(planner_AD, environment_nav2D, fSol, bPrint, size_x, size_y, original_map, startx, starty, goalx, goaly, 
        allocated_time_secs_foreachplan, solution_stateIDs_V,
        plantime_over1secs, plantime_over0p5secs, plantime_over0p1secs, plantime_over0p05secs, plantime_below0p05secs, 
        obsthresh, changedcellsV, &psolcost);
    solution_step += 1;
    cout << "start: " << startx << " " << starty << endl;
    cout << "compute solution... " << endl;
    // SOLUTION
    *planlength = solution_stateIDs_V.size();
    *plan = (double**) malloc((*planlength)*sizeof(double*));
    *plancost = psolcost;
    for (int i = 0; i < (*planlength); i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));      
        int next_x,next_y;
        environment_nav2D.GetCoordFromState(solution_stateIDs_V[i], next_x, next_y);
        
        (*plan)[i][0] = next_x+1;
        (*plan)[i][1] = next_y+1;
        //euclidean distance
        cout << "["<< (*plan)[i][0] << " " << (*plan)[i][1] << "]";
    }
    cout << "\n";
    cout << "solcost: " << *plancost << "\n";
    cout << "end compute solution... " << endl;
    // SOLUTION

//     } 

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

//     delete planner_AD;


    //return plannerRes == 1 ? MAIN_RESULT_SUCCESS : MAIN_RESULT_FAILURE;
    
    cout << "********************* planner.cpp ended *********************\n";
    
    cout.flush();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  ANAHITA



/*static void planner(double*	map, int x_size, int y_size,
                    float robotposeX, float robotposeY, float robotposeTheta,
                    float goalposeX, float goalposeY, float goalposeTheta,
                    int numofDOFs, double*** plan, int* planlength, double* plancost,
                    float epsilon, float time)
{   
    main_function();
    mexPrintf("temp = %d\n", temp);
    temp = temp+1;
    
    mexPrintf("robot: %.2f %.2f %.2f;\n", robotposeX, robotposeY, robotposeTheta);
    mexPrintf("goal: %.2f %.2f %.2f;\n", goalposeX, goalposeY, goalposeTheta);
    
    //interpolate between start and goal
	*planlength = 3;
    *plan = (double**) malloc((*planlength)*sizeof(double*));
    float dx = (goalposeX - robotposeX) / (*planlength - 1);
    float dy = (goalposeY - robotposeY) / (*planlength - 1);
    for (int i = 0; i < (*planlength); i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
        (*plan)[i][0] = robotposeX + dx*i;
        (*plan)[i][1] = robotposeY + dy*i;
        //euclidean distance
        *plancost += sqrt(dx*dx + dy*dy);
    }
    
    return;
}*/

void mexFunction(int nlhs, mxArray *plhs[], 
                 int nrhs, const mxArray*prhs[])
     
{

    /* Check for proper number of arguments */    
    if (nrhs != 5) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Five input arguments required."); 
    }
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    mexPrintf("x_size: %d, y_size: %d\n", x_size, y_size);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
    
//     mexPrintf("robotpose_M: %d, robotpose_N: %d\n", robotpose_M, robotpose_N);
    
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    mexPrintf("robotposeX: %.2f, robotposeY: %.2f, robotposeTheta: %.2f\n", 
              robotposeX, robotposeY, robotposeTheta);
    
    mexPrintf("cost index: %.2f\n", GETMAPINDEX(robotposeX, robotposeY, x_size, y_size));
    mexPrintf("cost index: %d\n", (int)GETMAPINDEX(robotposeX, robotposeY, x_size, y_size));
    mexPrintf("robotpose cost: %.2f\n", map[(int)GETMAPINDEX(robotposeX, robotposeY, x_size, y_size)]);
//     for (int i = 1; i <= x_size; i++) {
//         for (int j = 1; j <= y_size; j++) {
//             mexPrintf("x = %d, y = %d, cost = %.2f\n", i, j, map[(int)GETMAPINDEX(i, j, x_size, y_size)]);
//             
//         }
//     }
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
    
//     mexPrintf("goalpose_M: %d, goalpose_N: %d\n", goalpose_M, goalpose_N);
    
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
    float goalposeTheta = (float)goalposeV[2];
    
    mexPrintf("goalposeX: %.2f, goalposeY: %.2f, goalposeTheta: %.2f\n", 
              goalposeX, goalposeY, goalposeTheta);
    
    double* epsilonV = mxGetPr(EPSILON_IN);
    float epsilon = (float)epsilonV[0];
    mexPrintf("epsilon: %.2f\n", epsilon);
    
    double* timeV = mxGetPr(TIME_IN);
    float time = (float)timeV[0];
    mexPrintf("time: %.2f\n", time);
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    double plancost = 0;
    //waypoint DOF
    int numofDOFs = 2;
    
    planner(map, x_size, y_size, 
            robotposeX, robotposeY, robotposeTheta, 
            goalposeX, goalposeY, goalposeTheta,
            numofDOFs, &plan, &planlength, &plancost,
            epsilon, time);
    
    mexPrintf("planner returned plan of length = %d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0) {
        PLAN_OUT = mxCreateNumericMatrix((mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    } else {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
            plan_out[j] = robotposeV[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*)mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;
    
    PLANCOST_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    double* plancost_out = mxGetPr(PLANCOST_OUT);
    *plancost_out = plancost;
    
    return;
}





