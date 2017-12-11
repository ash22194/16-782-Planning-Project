#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <string>
#include "any_rrt_star.h"

/***********************    ANYTIME RRT STAR NODE   ***************************/

Anytime_RRT_Star_Node::Anytime_RRT_Star_Node(double _x, double _y){
	x = _x;
	y = _y;
	cost = 0.0;
	parent = -1;
}

Anytime_RRT_Star_Node::Anytime_RRT_Star_Node(Anytime_RRT_Star_Node* _node){
	x = _node->x;
	y = _node->y;
	cost = 0.0;
	parent = -1;
}

string Anytime_RRT_Star_Node::toString(char* p = NULL){
	static char buffer[1048];
	if (p == NULL) {
		p = buffer;
	}
	string str;
	
	sprintf(p, "(x: %lf, y: %lf, cost: %lf, parent: %d)", this->x, this->y, this->cost, this->parent);
	str.append(p);

	// str.append("\n");
	return str;
}
/***********************    ANYTIME RRT STAR NODE   ***************************/

Anytime_RRT_Star_Graph::Anytime_RRT_Star_Graph(double* _map, float start_x, float start_y, float goal_x, float goal_y,
	float _min_random_area, float _max_random_area, float _epsilon, float _goal_sample_rate, int _max_iterations,
	int _x_size, int _y_size){
	start = new Anytime_RRT_Star_Node(start_x, start_y);
	goal  = new Anytime_RRT_Star_Node(goal_x, goal_y);
	min_random_area = _min_random_area;
	max_random_area = _max_random_area;
	epsilon = _epsilon;
	goal_sample_rate = _goal_sample_rate;
	max_iterations = _max_iterations;
	x_size = _x_size;
	y_size = _y_size;
	map = _map;
	// for (int i=0; i<_map.size(); i++)
 //        map.push_back(_map[i]);
	// path = NULL;
	// node_list = NULL;
	srand (time(NULL));
}

vector<pair<double, double> > Anytime_RRT_Star_Graph::planning(){
	node_list.push_back(start);
	for (int i=0; i<max_iterations; i++){
		// cout << "i: " << i << " " << endl;
		std::pair<double, double> random_point = sample();
		// cout << random_point.first << " " << random_point.second << endl;

		int nearest_idx = nearest_neighbor(node_list, random_point);
		// cout << nearest_idx << endl;
		if (nearest_idx == -1) continue;

		// cout << nearest_idx << endl;
		Anytime_RRT_Star_Node* new_node = extend(random_point, nearest_idx);
		// cout << new_node->toString() << endl;
		// cout << "new node: " << new_node->toString() << endl;
		if (no_collision_check(new_node) == true){
			vector<int> near_ids = find_near_nodes(new_node);
			// cout << "near_ids: ";
			// for (vector<int>::iterator it = near_ids.begin(); it != near_ids.end(); ++it){
			// 	cout << (*it) << " ";
			// }
			// cout << endl;

			new_node = choose_parent(new_node, near_ids);
			// cout << "Parent node: " << new_node->toString() << endl;

			node_list.push_back(new_node);
			// cout << "Printing node list" << endl;
			// for (std::vector<Anytime_RRT_Star_Node*>::iterator it = node_list.begin(); it != node_list.end(); ++it){
			// 	cout << (*it)->toString() << endl;
			// }

			rewire(new_node, near_ids);
		}
		// cout << "---------------------" << endl;
	}

	// cout << "Printing node list" << endl;
	// for (std::vector<Anytime_RRT_Star_Node*>::iterator it = node_list.begin(); it != node_list.end(); ++it){
	// 	cout << (*it)->toString() << endl;
	// }

	int last_index = get_best_last_index();
	// cout << last_index << endl;
	
	gen_final_course(last_index);
	// cout << "Final Path: " << endl;
	// for (vector<pair<double, double> >::iterator it = path.begin(); it != path.end(); ++it){
	// 	cout << (*it).first << " " << (*it).second << endl;
	// }
	// int path_length = path.size();
	// for (int i = 0; i < path_length; i++){
	// 	cout << path[path_length - i - 1].first << " " << path[path_length - i - 1].second << endl;
	// }
	// cout << endl;
	return path;
}

std::pair<double, double> Anytime_RRT_Star_Graph::sample(){
	int random_sample = rand() % 100 + 1;
	// cout << random_sample << endl;
	std::pair<double, double> random_point;
	if (random_sample > 100*goal_sample_rate){
		float x = min_random_area + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_random_area - min_random_area)));
		float y = min_random_area + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_random_area - min_random_area)));
		random_point = std::make_pair(x,y);
	} else {
		// cout << "goal sampled" << endl;
		random_point = std::make_pair(goal->x,goal->y);
	}
	return random_point;
}

int Anytime_RRT_Star_Graph::nearest_neighbor(vector<Anytime_RRT_Star_Node*> node_list, std::pair<double, double> random_point){
	vector<Anytime_RRT_Star_Node*>::iterator it;
	double min_dist = INT_MAX;
	int min_index = -1;
	for (it = node_list.begin(); it != node_list.end(); ++it){
		double dist = ((*it)->x - random_point.first)*((*it)->x - random_point.first) + ((*it)->y - random_point.second)*((*it)->y - random_point.second);
		// cout << "dist: " << dist << endl;
		if (dist < min_dist){
			min_index = std::distance(node_list.begin(), it);
			min_dist = dist;
			// cout << "inside: min_dist: " << min_dist << endl;
			// cout << "inside: min_index:" << min_index << endl;		
		}
	}
	// cout << "min_dist: " << min_dist << endl;
	// cout << "min_index:" << min_index << endl;
	return min_index;
}

Anytime_RRT_Star_Node* Anytime_RRT_Star_Graph::extend(std::pair<double, double> random_point, int nearest_idx){
	Anytime_RRT_Star_Node* nearest_node = node_list.at(nearest_idx);
	// Anytime_RRT_Star_Node* nearest_node = new Anytime_RRT_Star_Node(25, 10);
	// double theta = atan((random_point.second - nearest_node->y)/(random_point.first - nearest_node->x));
	// cout << theta << endl;
	Anytime_RRT_Star_Node* new_node = new Anytime_RRT_Star_Node(nearest_node);
	// new_node->x += epsilon * cos(theta);
	// new_node->y += epsilon * sin(theta);
	new_node->x += epsilon * (random_point.first - nearest_node->x)/sqrt((random_point.second - nearest_node->y)*(random_point.second - nearest_node->y) + (random_point.first - nearest_node->x)*(random_point.first - nearest_node->x));
	new_node->y += epsilon * (random_point.second - nearest_node->y)/sqrt((random_point.second - nearest_node->y)*(random_point.second - nearest_node->y) + (random_point.first - nearest_node->x)*(random_point.first - nearest_node->x));
	// cout << new_node->x << " " << new_node->y << endl;
	
	int state_cost = (int)map[(int)GETMAPINDEX(new_node->x, new_node->y, x_size, y_size)];
	// cout << "state cost: " << state_cost << endl;
	new_node->cost += epsilon + state_cost;
	new_node->parent = nearest_idx;
	return new_node;
}

bool Anytime_RRT_Star_Graph::no_collision_check(Anytime_RRT_Star_Node* new_node){
	// int map_index = (int)GETMAPINDEX(new_node->x, new_node->y, x_size, y_size);
	// if (map_index < 0){
	// 	cout << new_node->x << endl;
	// 	cout << new_node->y << endl;
	// 	cout << x_size << endl;
	// 	cout << y_size << endl;
	// 	cout << GETMAPINDEX(new_node->x, new_node->y, x_size, y_size) << endl;
	// 	exit(0);
	// }
	// cout << map_index << endl;
	// cout << map.at(map_index) << endl;
	int map_val = (int)map[(int)GETMAPINDEX((int)new_node->x, (int)new_node->y, x_size, y_size)];
	// cout << new_node->x << " " << new_node->y << " "<< map_val << endl;
	if ( map_val == 255){
		return false;
	}
	return true;
}

vector<int> Anytime_RRT_Star_Graph::find_near_nodes(Anytime_RRT_Star_Node* new_node){
	int num_nodes = node_list.size();
	int radius = 50.0 * sqrt((log(num_nodes)+1 / num_nodes));
	vector<int> near_ids;
	
	// cout << radius << endl;
	vector<Anytime_RRT_Star_Node*>::iterator it;
	for (it = node_list.begin(); it != node_list.end(); ++it){
		// cout << (*it)->toString() << endl << new_node->toString() << endl;
		double dist = ((*it)->x - new_node->x)*((*it)->x - new_node->x) + ((*it)->y - new_node->y)*((*it)->y - new_node->y);
		// cout << "finding near nodes: " << dist << endl;
		if (dist < radius){
			int within_idx = std::distance(node_list.begin(), it);
			near_ids.push_back(within_idx);
		}
	}

	return near_ids;
}

Anytime_RRT_Star_Node* Anytime_RRT_Star_Graph::choose_parent(Anytime_RRT_Star_Node* new_node, vector<int> near_ids){
	if (near_ids.size()==0){
		return new_node;
	}

	vector<double> dlist;
	vector<int>::iterator it;
	for (it = near_ids.begin(); it != near_ids.end(); ++it){
		// double dx = new_node->x - node_list.at(*it)->x;
		// double dy = new_node->y - node_list.at(*it)->y;
		// double d = sqrt(dx * dx + dy * dy);
		// double theta = atan(dy/dx);
		if (no_collision_check_extend(node_list.at(*it), new_node) == true){
			double d = dist(node_list.at(*it), new_node);
			int state_cost = (int)map[(int)GETMAPINDEX(new_node->x, new_node->y, x_size, y_size)];
			dlist.push_back(node_list.at(*it)->cost + d + state_cost);
		} else {
			dlist.push_back(INT_MAX);
		}
	}

	double min_cost = *std::min_element(dlist.begin(), dlist.end());
	int index = find(dlist.begin(), dlist.end(), min_cost) - dlist.begin();
	int min_ind = near_ids.at(index);

	if (min_cost == INT_MAX){
		return new_node;
	}

	new_node->cost = min_cost;
	new_node->parent = min_ind;
	return new_node;
}

bool Anytime_RRT_Star_Graph::no_collision_check_extend(Anytime_RRT_Star_Node* near_node, Anytime_RRT_Star_Node* reach_node){
	Anytime_RRT_Star_Node* temp_node = new Anytime_RRT_Star_Node(near_node);
	double d = dist(near_node, reach_node);
	for (int i = 0; i < int(d / epsilon); i++){
		temp_node->x += epsilon * (reach_node->x - near_node->x)/sqrt((near_node->x - reach_node->x)*(near_node->x - reach_node->x) + (near_node->y - reach_node->y)*(near_node->y - reach_node->y));
		temp_node->y += epsilon * (reach_node->y - near_node->y)/sqrt((near_node->x - reach_node->x)*(near_node->x - reach_node->x) + (near_node->y - reach_node->y)*(near_node->y - reach_node->y));
		// cout << "temp node: " << temp_node->x << " " << temp_node->y << endl;
		if (no_collision_check(temp_node) == false){
			// cout << "false" << endl;
			return false;
		}
	}
	// cout << "true" << endl;
	return true;
}

void Anytime_RRT_Star_Graph::rewire(Anytime_RRT_Star_Node* new_node, vector<int> near_ids){
	int num_nodes = node_list.size();
	vector<int>::iterator it;
	for (it = near_ids.begin(); it != near_ids.end(); ++it){
		// nearNode = self.nodeList[i]
		Anytime_RRT_Star_Node* near_node = node_list.at(*it);
		// double dx = new_node->x - near_node->x;
		// double dy = new_node->y - near_node->y;
		double d = dist(new_node, near_node);

		double scost = new_node->cost + d;

		if (near_node->cost > scost){
			// double theta = atan(dy/dx);
			if (no_collision_check_extend(near_node, new_node) == true){
				near_node->parent = num_nodes - 1;
				near_node->cost = scost;
			}
		}
	}
}

int Anytime_RRT_Star_Graph::get_best_last_index(){
	
	vector<Anytime_RRT_Star_Node*>::iterator it;
	double min_cost = INT_MAX;
	int min_index = -1;
	int near_goal_idx = -1;
	for (it = node_list.begin(); it != node_list.end(); ++it){
		double temp_cost = calc_dist_to_goal((*it)->x, (*it)->y);
		// cout << temp_cost << endl;
		// if (temp_cost <= epsilon){

			//save index to goalinds
			// near_goal_idx = std::distance(it, node_list.begin());
			//compute min cost
		if (temp_cost < min_cost){
			min_index = std::distance(node_list.begin(), it);
			min_cost = temp_cost;
			// cout << "inside: min_dist: " << min_cost << endl;
			// cout << "inside: min_index:" << min_index << endl;
		}
		// }
	}
	if (min_cost > epsilon){
		return -1;
	}
	total_cost = node_list.at(min_index)->cost + min_cost;
	return min_index;
}

void Anytime_RRT_Star_Graph::gen_final_course(int index){
	if (index == -1){
		cout << "no path exists" << endl;
		// path.push_back(std::make_pair(start->x, start->y));
		return;	
	}
	path.push_back(std::make_pair(goal->x, goal->y));
	while (node_list.at(index)->parent != -1){
		Anytime_RRT_Star_Node* node = node_list.at(index);
		path.push_back(std::make_pair(node->x, node->y));
		index = node->parent;
	}
	path.push_back(std::make_pair(start->x, start->y));
}

double Anytime_RRT_Star_Graph::dist(Anytime_RRT_Star_Node* node1, Anytime_RRT_Star_Node* node2){
	return sqrt((node1->x - node2->x)*(node1->x - node2->x) + (node1->y - node2->y)*(node1->y - node2->y));
}

double Anytime_RRT_Star_Graph::calc_dist_to_goal(double x_val, double y_val){
	return sqrt((x_val - goal->x)*(x_val - goal->x) + (y_val - goal->y)*(y_val - goal->y));
}