/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "any_rrt_star.h"

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

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

int temp = 0;

static void planner(double*	map, int x_size, int y_size,
                    float robotposeX, float robotposeY, float robotposeTheta,
                    float goalposeX, float goalposeY, float goalposeTheta,
                    int numofDOFs, double*** plan, int* planlength, double* plancost,
                    float epsilon, float time)
{   
//     mexPrintf("temp = %d\n", temp);
    temp = temp+1;
    
//     mexPrintf("robot: %.2f %.2f %.2f;\n", robotposeX, robotposeY, robotposeTheta);
//     mexPrintf("goal: %.2f %.2f %.2f;\n", goalposeX, goalposeY, goalposeTheta);
    
    //interpolate between start and goal
    float goal_sample_rate = 0.2;

	Anytime_RRT_Star_Graph* G = new Anytime_RRT_Star_Graph(map,  robotposeX,  robotposeY,  goalposeX,  goalposeY,
    x_size, y_size, epsilon, goal_sample_rate, 10000, x_size, y_size);


    /******** Collision working properly? ********/
    // Anytime_RRT_Star_Node* near_node = new Anytime_RRT_Star_Node(15, 12);
    // G->no_collision_check_extend(near_node, 4.71, 10);
    
    // cout << map[GETMAPINDEX(30, 0, 30, 30)] << endl;
    // cout << map[GETMAPINDEX(0, 30, 30, 30)] << endl;
    /******** Collision working properly? ********/

    std::vector<std::pair<double, double> > path_vec (G->planning());

    // cout << "Final Path: " << endl;
    // for (vector<pair<double, double> >::iterator it = path_vec.begin(); it != path_vec.end(); ++it){
    //     cout << (*it).first << " " << (*it).second << endl;
    // }

    *planlength = path_vec.size();
    *plancost = G->total_cost;

    *plan = (double**) malloc(*planlength*sizeof(double*));
    for (int i = 0; i < *planlength; i++){
        (*plan)[i] = (double*) malloc(2*sizeof(double)); 
        (*plan)[i][0] = path_vec[*planlength - i - 1].first;
        (*plan)[i][1] = path_vec[*planlength - i - 1].second;
    }

    // cout << *plancost;
    return;
}

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
    
    // mexPrintf("x_size: %d, y_size: %d\n", x_size, y_size);
    
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
    
    // mexPrintf("robotposeX: %.2f, robotposeY: %.2f, robotposeTheta: %.2f\n", 
              // robotposeX, robotposeY, robotposeTheta);
    
//     mexPrintf("cost index: %.2f\n", GETMAPINDEX(robotposeX, robotposeY, x_size, y_size));
//     mexPrintf("cost index: %d\n", (int)GETMAPINDEX(robotposeX, robotposeY, x_size, y_size));
//     mexPrintf("robotpose cost: %.2f\n", map[(int)GETMAPINDEX(robotposeX, robotposeY, x_size, y_size)]);
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
    
    // mexPrintf("goalposeX: %.2f, goalposeY: %.2f, goalposeTheta: %.2f\n", 
    //           goalposeX, goalposeY, goalposeTheta);
    
    double* epsilonV = mxGetPr(EPSILON_IN);
    float epsilon = (float)epsilonV[0];
    // mexPrintf("epsilon: %.2f\n", epsilon);
    
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
    
    // mexPrintf("planner returned plan of length = %d\n", planlength); 
    
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





