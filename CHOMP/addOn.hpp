#include <armadillo>
#include <iostream>
#include <math.h>
#include <chrono>

namespace CHOMP {

	typedef std::chrono::high_resolution_clock Time;
	typedef std::chrono::milliseconds ms;
	typedef std::chrono::duration<float> fsec;
	
class chompOpt
{
public:
	int eta;
	int minIter;
	int maxIter;
	float tolFun;
	float lambda;
	float etc;
	bool decreaseStepSize;
	bool freeEndPt;
	
	chompOpt(int et, int minI, int maxI, float tolF, float l, float tc, bool f, bool d)
	{
		etc = tc;
		eta = et;
		minIter = minI;
		maxIter = maxI;
		tolF = tolFun;
		freeEndPt = f;
		decreaseStepSize = d;
		lambda = l;
	};

	chompOpt()
	{
		eta = 5000;
		etc = 0.95;
		minIter = 1;
		maxIter = 100;
		tolFun = 0.001;
		// metricInverse.eye();
		// metric.eye();
		freeEndPt = false;
		decreaseStepSize = false;
		lambda = 100;
	};

	chompOpt(int minI, int maxI, float l, float tc)
	{
		eta = 5000;
		etc = tc;
		minIter = minI;
		maxIter = maxI;
		tolFun = 0.001;
		// metricInverse.eye();
		// metric.eye();
		freeEndPt = false;
		decreaseStepSize = false;
		lambda = l;
	};

	float stepSize(int i) 
	{
		if (decreaseStepSize) {
			return 1.0/((float) eta)*1.0/std::sqrt(i+1.0);
		}
		else {
			return 1.0/((float) eta);	
		}
		
	};

};

class map
{
public:
	arma::mat table;
	float resolution_x;
	float resolution_y;	
	arma::vec origin;
	map(arma::mat& t, float r_x, float r_y, arma::vec& o) 
	{
		resolution_x = r_x;
		resolution_y = r_y;
		table = t;
		origin = o;
	};

	map() 
	{
		table.reset();
		origin.zeros(2,1);
	};
};

void gradientMat(arma::mat& M, arma::mat& Mx, arma::mat& My) 
{
	Mx.zeros(M.n_rows,M.n_cols);
	My.zeros(M.n_rows,M.n_cols);
	Mx.row(0) = M.row(1) - M.row(0);
	for (int i=1;i<M.n_rows-1;i++) {
		Mx.row(i) = 0.5*(M.row(i+1)-M.row(i-1));	
	}
	Mx.row(M.n_rows-1) = M.row(M.n_rows-1) - M.row(M.n_rows-2);
	
	My.col(0) = M.col(1) - M.col(0);
	for (int j=1;j<M.n_cols-1;j++) {
		My.col(j) = 0.5*(M.col(j+1)-M.col(j-1));
	}
	My.col(M.n_cols-1) = M.col(M.n_cols-1) - M.col(M.n_cols-2);
};

void getCostMapDerivatives(map& costMap, map& costMapx, map& costMapy) 
{
	float resolution_x = costMap.resolution_x;
	float resolution_y = costMap.resolution_y;
	arma::vec origin = costMap.origin;
	gradientMat(costMap.table,costMapx.table,costMapy.table);
	costMapx.resolution_x = resolution_x; costMapx.resolution_y = resolution_y;
	costMapy.resolution_x = resolution_x; costMapy.resolution_y = resolution_y;
	costMapx.origin = origin;
	costMapy.origin = origin;
	costMapx.table = costMapx.table/resolution_x;
	costMapy.table = costMapy.table/resolution_y;
};

void getInitialTrajectory(arma::vec& start, arma::vec& goal, int n, arma::mat& trajectory) 
{
	if (start.size()!=goal.size()) {
		std::cout<<"Start and Goal dimensions do not match"<<std::endl;
	}
	float d = arma::norm(goal-start);
	arma::vec di = arma::linspace<arma::vec>(0,d,n);
	arma::vec X; X<<0<<d;

	trajectory.reset();
	trajectory.zeros(n,start.size());
	for (int i=0;i<start.size();i++) {
		arma::vec Y; Y << start(i) << goal(i);
		arma::vec YY;
		interp1(X,Y,di,YY);
		trajectory.col(i) = YY;
	}	
};

void createSmoothMatrices(arma::vec& start, arma::vec& goal, int n, 
						  arma::mat& A, arma::mat& b, arma::mat& c)
{
	arma::mat K;
	K.zeros(n+1,n);
	arma::vec dia1; dia1.ones(n);
    arma::vec dia2; dia2.ones(n-1);
    arma::mat d1 = diagmat(dia1);
    arma::mat d2 = diagmat(-dia2,-1);
    K.submat(0,0,n-1,n-1) = d1 + d2;
    K(n,n-1) = -1;

    arma::mat e; 
    e.zeros(n+1,start.size()); 			 e.submat(0,0,0,1) = -start.t(); 
    						   			 e.submat(n,0,n,1) = goal.t();
    A.zeros(n,n); 			   			 A = K.t()*K;
    b.zeros(n,start.size());   			 b = K.t()*e;
    c.zeros(start.size(),start.size());  c = 0.5*e.t()*e;
};

float trajectoryCost(arma::mat& traj, arma::mat& A, arma::mat& b, arma::mat& c)
{
	float cost = (traj.n_rows+1)*arma::trace(0.5*traj.t()*A*traj+traj.t()*b+c);
	return cost;
};

void trajectoryCostGradient(arma::mat& traj, arma::mat& A, arma::mat& b, arma::mat& grad)
{	
	grad.zeros(traj.n_rows,traj.n_cols);
	grad = (traj.n_rows)*(A*traj+b);
};

void worldTrajectoryToGrid(arma::mat& worldTraj, arma::mat& gridTraj, map& worldMap)
{
	gridTraj = worldTraj - arma::repmat(worldMap.origin.t(),worldTraj.n_rows,1);
	gridTraj.col(0) = arma::round(gridTraj.col(0)/worldMap.resolution_x)-1;
	gridTraj.col(1) = arma::round(gridTraj.col(1)/worldMap.resolution_y)-1;
	for (int i=0;i<gridTraj.n_rows;i++) {
		if (gridTraj(i,0) < 0) {
			gridTraj(i,0) = 0;
		}
		else if (gridTraj(i,0)>worldMap.table.n_cols-1) {
			gridTraj(i,0) = worldMap.table.n_cols-1;
		}

		if (gridTraj(i,1) < 0) {
			gridTraj(i,1) = 0;
		}
		else if (gridTraj(i,1)>worldMap.table.n_rows-1) {
			gridTraj(i,1) = worldMap.table.n_rows-1;
		}
	}
};

void trajectoryValue(arma::mat& traj, map& worldMap, arma::vec& value)
{
	if (traj.n_rows==0 || traj.n_cols==0) {
		value.reset();
		return;
	}

	arma::mat gridTraj;
	worldTrajectoryToGrid(traj,gridTraj,worldMap);
	arma::umat gT = arma::conv_to<arma::umat>::from(gridTraj);
	value = worldMap.table(arma::sub2ind(arma::size(worldMap.table),gT.t()));
};

float arcLengthValue(arma::mat& traj, arma::vec& pstart, map& worldMap) 
{
	arma::vec value;
	if (pstart.n_rows==0 || pstart.n_cols==0) {
		pstart = traj.row(0).t();
	}
	arma::mat traj_;
	traj_.zeros(traj.n_rows+1,traj.n_cols);
	traj_.submat(1,0,traj.n_rows,traj.n_cols-1) = traj;
	traj_.submat(0,0,0,traj.n_cols-1) = pstart.t();

	trajectoryValue(traj,worldMap,value);
	float v = arma::sum(arma::dot(arma::sqrt(arma::sum(arma::pow(arma::diff(traj_,1,0),2),1)),value));
	return v;
};

void gradientArcLengthValue(arma::mat& traj, arma::vec& pstart, map& costMap, map& costMapx, map& costMapy, arma::mat& grad)
{
	if (pstart.n_rows==0 || pstart.n_cols==0) {
		pstart = traj.row(0).t();
	}
	arma::mat traj_;
	traj_.zeros(traj.n_rows+1,traj.n_cols);
	traj_.submat(1,0,traj.n_rows,traj.n_cols-1) = traj;
	traj_.submat(0,0,0,traj.n_cols-1) = pstart.t();

	arma::vec c,cx,cy;
	arma::mat delC;
	trajectoryValue(traj,costMap,c);
	trajectoryValue(traj,costMapx,cx);
	trajectoryValue(traj,costMapy,cy);
	delC.zeros(cx.size(),2);
	delC.col(0) = cx;
	delC.col(1) = cy;
	float n = (float) traj.n_rows+1;
	arma::mat traj_d = n*arma::diff(traj_,1,0); 
	arma::mat traj_d_norm = arma::normalise(traj_d,2,1); 
	arma::mat traj_dd;
	traj_dd.zeros(traj_d.n_rows,traj_d.n_cols);
	traj_dd.submat(1,0,traj_d.n_rows-1,traj_d.n_cols-1) = n*arma::diff(traj_d,1,0); 
	
	arma:: mat kappa = (arma::repmat((1/arma::sum(arma::pow(traj_d,2),1)),1,2))
					   % (traj_dd - (traj_d_norm) % (arma::repmat(arma::sum(traj_dd % traj_d_norm,1),1,2)));
	
	grad = 1.0/n * ((arma::repmat(arma::sqrt(arma::sum(arma::pow(traj_d,2),1)),1,2))
					% (delC - (traj_d_norm) % (arma::repmat(arma::sum(delC % traj_d_norm,1),1,2))
		   - (arma::repmat(c,1,2)) % (kappa)));
	grad.transform( [](double val) { return (std::isnan(val) ? 0 : val); } );
};

float costFinal (arma::mat& traj, arma::vec& start, arma::mat& A, arma::mat& b, arma::mat& c, map& costMap, float lambda)
{
	float arc = arcLengthValue(traj,start,costMap);
	float trajcost = trajectoryCost(traj,A,b,c);
	return (trajectoryCost(traj,A,b,c)+lambda*arcLengthValue(traj,start,costMap));
};

void gradientFinal(arma::mat& traj, arma::vec& start, arma::mat& A, arma::mat& b, map& costMap, map& costMapx, map& costMapy, float lambda, arma::mat& grad)
{
	arma::mat trajGrad, arcValueGrad;
	trajectoryCostGradient(traj,A,b,trajGrad);
	gradientArcLengthValue(traj,start,costMap,costMapx,costMapy, arcValueGrad);
	grad.reset();
	grad = trajGrad + lambda*arcValueGrad;
};

void chompIterate(arma::mat traj, arma::mat& optTraj, double* finalCost, map& costMap, chompOpt& options)
{
	// std::cout<<"Starting CHOMP"<<std::endl;
	auto t0 = Time::now();
	int n = traj.n_rows;
	arma::vec start = traj.row(0).t();
	arma::vec goal = traj.row(traj.n_rows-1).t();
	traj = traj.submat(1,0,traj.n_rows-2,traj.n_cols-1);
	if (options.freeEndPt) {
		goal.reset();
	}
	arma::mat A;
	arma::mat b;
	arma::mat c;

	createSmoothMatrices(start,goal,n-2,A,b,c);
	
	map costMapx, costMapy;
	getCostMapDerivatives(costMap,costMapx,costMapy); 
	arma::mat Minv;
	// std::cout<<"Trying to invert A"<<std::endl;
	try {
		Minv = arma::inv(A);
	}
	catch (std::logic_error e) {
		std::cout<<"Can't invert A!!!"<<std::endl;
		std::cerr << e.what();
		return;
	}

	int count = 0;
	float cost, costPrev;
	cost = costFinal(traj,start,A,b,c,costMap,options.lambda);
	costPrev = 0;
	arma::cube trajectoryHistory;
	trajectoryHistory.zeros(traj.n_rows,traj.n_cols,1);
	trajectoryHistory.slice(0) = traj;
	arma::vec costHistory, costNew;
	costHistory.zeros(1);
	costHistory(0) = cost;

	arma::mat grad;

	while(count < options.maxIter) {
		if (count > options.minIter) {
			if (std::abs(cost-costPrev)/costPrev < options.tolFun) {
				break;
			}
		}
		auto t1 = Time::now();
		fsec fs = t1-t0;
		ms d = std::chrono::duration_cast<ms>(fs);
		if ((((float)d.count())/1000 > options.etc) && (count > 1)) {
			std::cout << "Time Out (CHOMP)!" << std::endl;
            break;
		}

		// std::cout<<"Iteratoin : "<<count<<", "<<cost<<std::endl;

		gradientFinal(traj,start,A,b,costMap,costMapx,costMapy,options.lambda,grad);
		traj = traj - options.stepSize(count)*Minv*grad;
		arma::cube trajInsert; trajInsert.zeros(traj.n_rows,traj.n_cols,1); trajInsert.slice(0) = traj;
		trajectoryHistory.insert_slices(trajectoryHistory.n_slices,trajInsert);

		costPrev = cost;
		cost = costFinal(traj,start,A,b,c,costMap,options.lambda);
		costNew.zeros(1); costNew(0) = cost;
		costHistory.insert_rows(costHistory.n_rows,costNew);
		count++;
	}
	
	int minCostIndx = costHistory.index_min();
	*finalCost = costHistory(minCostIndx);
	optTraj = trajectoryHistory.slice(minCostIndx);
	optTraj.insert_rows(0,start.t());
	optTraj.insert_rows(traj.n_rows+1,goal.t());
};


}