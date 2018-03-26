/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include "addOn.hpp"

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
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y)*XSIZE + (X))
// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((X-1)*YSIZE + (YSIZE-Y+1-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

int temp = 0;

void convertCostMap (arma::mat& costMap, double* map, int x_size, int y_size) 
{
    costMap.zeros(y_size,x_size);
    for (int i=0; i<x_size; i++) {
        for (int j=0; j<y_size; j++) {
            costMap(j,i) = map[GETMAPINDEX(j,i,x_size,y_size)];
        }
    }
};

void printMap(double* map, int x_size, int y_size) 
{
    for (int i=0; i<x_size; i++) {
        for (int j=0; j<y_size; j++) {
            std::cout<<map[GETMAPINDEX(j,i,x_size,y_size)]<<" ";
        }
        std::cout<<std::endl;
    }
};

static void planner(double*	map, int x_size, int y_size,
                    float robotposeX, float robotposeY, float robotposeTheta,
                    float goalposeX, float goalposeY, float goalposeTheta,
                    int numofDOFs, double*** plan, int* planlength, double* plancost,
                    float epsilon, float timeLimit)
{   
    arma::mat costLoad;
    arma::vec o; o.zeros(2);
    convertCostMap(costLoad,map,x_size,y_size);
    CHOMP::map costMap(costLoad,epsilon,epsilon,o);
    // Convert start and goal to appropriate values
    arma::vec start; 
    start.zeros(2); 
    start(0) = robotposeX;
    start(1) = robotposeY;
    arma::vec goal; 
    goal.zeros(2); 
    goal(0) = goalposeX;
    goal(1) = goalposeY;
    
    if (start(0) < 0) {
        start(0) = 0;
    }
    else if (start(0) > (x_size-1)) {
        start(0) = (x_size-1);
    }

    if (start(1) < 0) {
        start(1) = 0;
    }
    else if (start(1) > (y_size-1)) {
        start(1) = (y_size-1);
    }

    if (goal(0) < 0) {
        goal(0) = 0;
    }
    else if (goal(0) > (x_size-1)) {
        goal(0) = (x_size-1);
    }

    if (goal(1) < 0) {
        goal(1) = 0;
    }
    else if (goal(1) > (y_size-1)) {
        goal(1) = (y_size-1);
    }

    int N = 100;
    float lambda = 100;
    int maxIter = 100000;
    int minIter = 1;
    CHOMP::chompOpt options(minIter, maxIter, lambda, timeLimit);

    arma::mat traj;
    arma::mat optimizedTraj;
    
    CHOMP::getInitialTrajectory(start,goal,N,traj);
    *plan = (double**) malloc(N*sizeof(double*));
    CHOMP::chompIterate(traj,optimizedTraj,plancost,costMap,options);
    
    for (int i=0; i<N; i++) {
        (*plan)[i] = (double*) malloc(sizeof(double)*2);
        (*plan)[i][0] = optimizedTraj(i,0);
        (*plan)[i][1] = optimizedTraj(i,1);
    }
    *planlength = N;
    return;
};

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
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
        
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
        
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
    float goalposeTheta = (float)goalposeV[2];
    
    double* epsilonV = mxGetPr(EPSILON_IN);
    float epsilon = (float)epsilonV[0];
    
    double* timeV = mxGetPr(TIME_IN);
    float time = (float)timeV[0];
    // mexPrintf("time: %.2f\n", time);
    
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
};





