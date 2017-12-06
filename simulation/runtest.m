%% Path Following for a Differential Drive Robot
%% Introduction
% This example demonstrates how to control a robot to follow a desired path
% using a Robot Simulator. The example uses the Pure Pursuit path following
% controller to drive a simulated robot along a predetermined path. A desired path is a
% set of waypoints defined explicitly or computed using a path planner (refer to
% <docid:robotics_examples.example-PathPlanningExample>). The Pure Pursuit
% path following controller for a simulated differential drive robot is created and
% computes the control commands to follow a given path. The computed control commands are
% used to drive the simulated robot along the desired trajectory to
% follow the desired path based on the Pure Pursuit controller.
%
% Note: Starting in R2016b, instead of using the step method to perform the
% operation defined by the System object, you can call the object with
% arguments, as if it were a function. For example, |y = step(obj,x)| and
% |y = obj(x)| perform equivalent operations.

% Copyright 2014-2016 The MathWorks, Inc
%% Initialize the Robot Simulator
% A simple robot simulator is used in this example that updates and
% returns the pose of the differential drive robot for given control inputs. An external
% simulator or a physical robot will require a localization mechanism to
% provide an updated pose of the robot.
% Initialize the robot simulator and assign an initial pose. The simulated
% robot has kinematic equations for the motion of a two-wheeled differential drive robot.
% The inputs to this simulated robot are linear and angular velocities. It also has plotting
% capabilities to display the robot's current location and draw the
% trajectory of the robot.

%% Start Robot Simulator with a simple map

close all;
envmap = load('data/map5.txt');
costmap = envmap;

envmap(envmap<255) = 0;
bogmap = robotics.BinaryOccupancyGrid(envmap);
bogmap2 = robotics.BinaryOccupancyGrid(envmap);

robotRadius = 1;
robot = RobotSimulator(bogmap);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

% Inflate costmap based on robotRadius
inflate(bogmap2, robotRadius);
bogmat = occupancyMatrix(bogmap2);
costmap(bogmat) = 255;

% f2 = figure('Name', 'Inflated Cost Map');
% image(255 - costmap);
% colormap(gray(256));
% axis image;
% movegui(f2,'northwest');

[sizeX, sizeY] = size(costmap);
curmap = zeros(sizeX, sizeY);
curmap(bogmat) = 255;

% f3 = figure('Name', 'Current Cost Map');
% image(255 - curmap);
% colormap(gray(256));
% axis image;
% movegui(f3,'northeast');

%%
% You can compute the |path| using the PRM path planning algorithm. See
% <docid:robotics_examples.example-PathPlanningExample> for details.
% mapInflated = copy(robot.Map);
% inflate(mapInflated,robotRadius);
% prm = robotics.PRM(mapInflated);
% prm.NumNodes = 5000;
% prm.ConnectionDistance = 2;

%%
% Find a path between the start and end location. Note that the |path| will
% be different due to the probabilistic nature of the PRM algorithm.
% startLocation = [50.0 50.0];
% endLocation = [200.0 200.0];
% startLocation = [5.0 25.0];
% endLocation = [29.0 15.0];
% path = findpath(prm, startLocation, endLocation);
% path = [startLocation; endLocation];

%%
% Set current location and the goal of the robot as defined by the path

% convert coordinates wrt. cost map axes to those wrt. what robot uses
% cost map axes: x to the bottom from 0 to sizeX
%                y to the right from 0 to sizeY
% robot axes:    x to the right from 0 to sizeX
%                y to the top from 0 to sizeY
RX = @(cy) (cy);
RY = @(cx) (sizeX+1-cx);
% convert coordinates wrt. what robot uses to those wrt. cost map axes
CX = @(ry) (sizeX+1-ry);
CY = @(rx) (rx);

% Assume an initial robot orientation
initialOrientation = 0;

% locations wrt. cost map axes
robotCurrentLocationC = [5 10 initialOrientation];
robotGoalC = [15.0 29.0 initialOrientation];

% Define the current pose for robot motion [x y theta]
robotCurrentPose = [RX(robotCurrentLocationC(2)) RY(robotCurrentLocationC(1)) initialOrientation];
robotGoalPose = [RX(robotGoalC(2)) RY(robotGoalC(1)) initialOrientation];

% Reset the current position of the simulated robot to the start of the path.
robot.setRobotPose(robotCurrentPose);

sr = 10;
C = round([robotCurrentLocationC(1) robotCurrentLocationC(2)]);

X = [C(1)-sr C(1)+sr];
X(X < 1) = 1;
X(X > sizeX) = sizeX;

Y = [C(2)-sr C(2)+sr];
Y(Y < 1) = 1;
Y(Y > sizeY) = sizeY;

for i=X(1):1:X(2)
    for j=Y(1):1:Y(2)
        r = norm([i j] - C);
        if (r <= sr)
            curmap(i, j) = costmap(i, j);
        end
    end
end

obsmap = (curmap == 255);
eps = 4;
curmap_ = create_costmap_sqdist(~obsmap,eps);
maxcurmap = max(max(curmap_));

%% Define the Path Following Controller
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
% following controller using the  |<docid:robotics_ref.buoofp1-1 robotics.PurePursuit>|  object.
controller = robotics.PurePursuit;
% Set the path following controller parameters. The desired linear
% velocity is set to 0.3 meters/second for this example.
controller.DesiredLinearVelocity = 0.3;
% The maximum angular velocity acts as a saturation limit for rotational velocity, which is
% set at 2 radians/second for this example.
controller.MaxAngularVelocity = deg2rad(114.592);
% As a general rule, the lookahead distance should be larger than the desired
% linear velocity for a smooth path. The robot might cut corners when the
% lookahead distance is large. In contrast, a small lookahead distance can
% result in an unstable path following behavior. A value of 0.5 m was chosen
% for this example.
controller.LookaheadDistance = 5;
% The |<docid:robotics_ref.buoofp1-1 controller>| object computes control commands for the robot.
% Drive the robot using these control commands until it reaches within the
% goal radius. If you are using an external simulator or a physical robot,
% then the controller outputs should be applied to the robot and a localization
% system may be required to update the pose of the robot. The controller runs at 10 Hz.
controlRate = robotics.Rate(10);
% You defined a path following controller above which you can re-use
% for computing the control commands of a robot on this map. To
% re-use the controller and redefine the waypoints while keeping the other
% information the same, use the |<docid:robotics_ref.bvl2d0a-1 release>| function.

%%
% Compute distance to the goal location
distanceToGoal = norm(robotCurrentLocationC(1:2) - robotGoalC(1:2));
% Define a goal radius
goalRadius = 0.1;

%%
% Drive the robot using the controller output on the given map until it
% reaches the goal. The controller runs at 10 Hz.
reset(controlRate);
fexec = figure('Name','Execution');
cm = imagesc(curmap); 
colorbar;
hold on;
while (distanceToGoal > goalRadius)

    % Run ADA
    [pathADA, pathlengthADA, ~] = plannerADA(curmap, round(robotCurrentLocationC), robotGoalC, 1.0, 0.5);
    pathcostADA = computeFinalCost(pathADA,curmap);
    fprintf('ADA* : %f\n',pathcostADA);
    
    % Run CHOMP
    varcostmap = zeros(size(curmap));
    varcostmap(~obsmap) = curmap(~obsmap);
    varcostmap = varcostmap/255*maxcurmap;
    curmapCHOMP = varcostmap + curmap_;
    [pathCHOMP, pathlengthCHOMP, ~] = plannerCHOMP(curmapCHOMP, robotCurrentLocationC, robotGoalC, 1.0, 0.5);
    pathcostCHOMP = computeFinalCost(pathCHOMP,curmap);
    fprintf('CHOMP : %f\n',pathcostCHOMP);
    
    % Run RRT
    [pathRRT, pathlengthRRT, ~] = plannerRRT(curmap, robotCurrentLocationC, robotGoalC, 1.0, 0.5);
    pathcostRRT = computeFinalCost(pathRRT,curmap);
    fprintf('RRT : %f\n',pathcostRRT);
    
    costs = [pathcostADA, pathcostCHOMP, pathcostRRT];
    [~,i] = min(costs);
    
    if (i==1)
        indexLookAhead = computeLookAheadPoint(robotCurrentLocationC,pathADA,controller.LookaheadDistance);
        pathC = pathADA(1:indexLookAhead,:);
        fada = plot(pathADA(:,2),pathADA(:,1),'g');
        fchomp = plot(pathCHOMP(:,2),pathCHOMP(:,1),'r');
        frrt = plot(pathRRT(:,2),pathRRT(:,1),'r');
    elseif (i==2)
        indexLookAhead = computeLookAheadPoint(robotCurrentLocationC,pathCHOMP,controller.LookaheadDistance);
        pathC = pathCHOMP(1:indexLookAhead,:);
        fada = plot(pathADA(:,2),pathADA(:,1),'r');
        fchomp = plot(pathCHOMP(:,2),pathCHOMP(:,1),'g');
        frrt = plot(pathRRT(:,2),pathRRT(:,1),'r');
    else
        indexLookAhead = computeLookAheadPoint(robotCurrentLocationC,pathRRT,controller.LookaheadDistance);        
        pathC = pathRRT(1:indexLookAhead,:);
        fada = plot(pathADA(:,2),pathADA(:,1),'r');
        fchomp = plot(pathCHOMP(:,2),pathCHOMP(:,1),'r');
        frrt = plot(pathRRT(:,2),pathRRT(:,1),'g');
    end
    rc = scatter(robotCurrentLocationC(2),robotCurrentLocationC(1),20,'cyan','filled');
    la = scatter(pathC(end,2),pathC(end,1),20,'black','filled');
    
    pathR = zeros(size(pathC));
    pathR(:,1) = RX(pathC(:,2));
    pathR(:,2) = RY(pathC(:,1));
    lookAheadPoint = pathR(end,:);
    robotCurrentPose = robot.getRobotPose;
    distanceToLookAhead = norm(robotCurrentPose(1:2) - lookAheadPoint);
    release(controller);
    controller.Waypoints = pathR;
    robot.setRobotPose(robotCurrentPose);
    
    while( distanceToLookAhead > goalRadius )

        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = controller(robot.getRobotPose);

        % Simulate the robot using the controller outputs
        drive(robot, v, omega);

        % Extract current location information from the current pose
        robotCurrentPose = robot.getRobotPose;

        sr = 10;
        robotCurrentLocationC = [CX(robotCurrentPose(1,2)) CY(robotCurrentPose(1,1)) robotCurrentPose(1,3)];
        delete(rc);
        rc = scatter(robotCurrentLocationC(2),robotCurrentLocationC(1),20,'cyan','filled');
        C = round([robotCurrentLocationC(1) robotCurrentLocationC(2)]);

        X = [C(1)-sr C(1)+sr];
        X(X < 1) = 1;
        X(X > sizeX) = sizeX;

        Y = [C(2)-sr C(2)+sr];
        Y(Y < 1) = 1;
        Y(Y > sizeY) = sizeY;

        for i=X(1):1:X(2)
            for j=Y(1):1:Y(2)
                r = norm([i j] - C);
                if (r <= sr)
                    curmap(i, j) = costmap(i, j);
                end
            end
        end
        delete(cm);
        cm = imagesc(curmap);
        
        % Re-compute the distance to the goal
        distanceToLookAhead = norm(robotCurrentPose(1:2) - lookAheadPoint);
        distanceToGoal = norm(robotCurrentLocationC(1:2) - robotGoalC(1:2));
        waitfor(controlRate);
        if (distanceToGoal < goalRadius)
            break;
        end
    end
    distanceToGoal = norm(robotCurrentLocationC(1:2) - robotGoalC(1:2));
        
end
%%
% The simulated robot has reached the goal location using the path following
% controller along the desired path. Stop the robot.
drive(robot, 0, 0);

%%
% Close Simulation.
% delete(robot);
