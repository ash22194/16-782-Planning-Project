#pragma once

#include <string>
#include <vector>

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define PI 3.14159265359

class Anytime_RRT_Star_Node;
class Anytime_RRT_Star_Graph;

class Anytime_RRT_Star_Node{
public:
	float x;
	float y;
	float cost;
	int parent;	//index o the parent

	Anytime_RRT_Star_Node(double, double);
	Anytime_RRT_Star_Node(Anytime_RRT_Star_Node*);
	~Anytime_RRT_Star_Node();

	string toString(char*);
};

class Anytime_RRT_Star_Graph{
	double* map;
	Anytime_RRT_Star_Node* start;
	Anytime_RRT_Star_Node* goal;
	float min_random_area;
	float max_random_area;
	double epsilon;
	float goal_sample_rate;
	int max_iterations;
	int x_size;
	int y_size;

	vector<pair<double, double> > path;
	vector<Anytime_RRT_Star_Node*> node_list;
public:
	double total_cost;
	bool found_goal;

	Anytime_RRT_Star_Graph(double*, float, float, float, float,
	float, float, float, float, int, int, int);
	// Anytime_RRT_Star_Graph(Anytime_RRT_Star_Node*);
	~Anytime_RRT_Star_Graph();

	vector<pair<double, double> > planning();
	pair<double, double> sample();
	int nearest_neighbor(vector<Anytime_RRT_Star_Node*>, pair<double, double>);
	Anytime_RRT_Star_Node* extend(pair<double, double>, int);
	bool no_collision_check(Anytime_RRT_Star_Node*);
	vector<int> find_near_nodes(Anytime_RRT_Star_Node*);
	Anytime_RRT_Star_Node* choose_parent(Anytime_RRT_Star_Node*, vector<int>);
	bool no_collision_check_extend(Anytime_RRT_Star_Node*, Anytime_RRT_Star_Node*);
	void rewire(Anytime_RRT_Star_Node*, vector<int>);
	int get_best_last_index();
	double calc_dist_to_goal(double, double);
	void gen_final_course(int);
	double dist(Anytime_RRT_Star_Node*, Anytime_RRT_Star_Node*);
};