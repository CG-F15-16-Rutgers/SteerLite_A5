//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib {
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
	//	traversal_cost = gSpatialDatabase->getTraversalCost ( id );
		
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX() - 1);

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ() - 1);


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );

			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		maze.resize(gSpatialDatabase->getNumCellsX());
		for (int i = 0; i < maze.size(); ++i) {
			maze[i].resize(gSpatialDatabase->getNumCellsZ());
			for ( int j = 0; j < maze[i].size(); ++j) {
				maze[i][j].cell.x = i;
				maze[i][j].cell.z = j;
				maze[i][j].cell.y = 0;
				maze[i][j].index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				gSpatialDatabase->getLocationFromIndex(maze[i][j].index, maze[i][j].point);
				maze[i][j].init();
			}
		}

		unsigned int start_index = gSpatialDatabase->getCellIndexFromLocation(start);
		unsigned int goal_index = gSpatialDatabase->getCellIndexFromLocation(goal);
		unsigned int start_x, start_z;
		unsigned int goal_x, goal_z;
		gSpatialDatabase->getGridCoordinatesFromIndex(start_index, start_x, start_z); 
		gSpatialDatabase->getGridCoordinatesFromIndex(goal_index, goal_x, goal_z); 
		AStarPlannerNode* startNode = &maze[start_x][start_z];//new AStarPlannerNode(start, 0, heuristicManhattan(start_id, goal_id), NULL);               //use euclidean as alternative 
		AStarPlannerNode* goalNode = &maze[goal_x][goal_z];//new AStarPlannerNode(goal, 0, 0, NULL);
		startNode->g = 0;
//		std::cout << "Manhattan!" << std::endl;
//		startNode->f = 2*heuristicManhattan(start_x, start_z, goal_x, goal_z);
//		std::cout << "Euclidean!" << std::endl;
		startNode->f = heuristicEuclidean(start_x, start_z, goal_x, goal_z);
		openSet.clear();
		closeSet.clear();
		agent_path.clear();
		openSet.push_back(startNode);

//		std::cout << "start is " << start_x << " " << start_z << std::endl;
//		std::cout << "end is " << goal_x << " " << goal_z << std::endl;

		while(!openSet.empty()){
/*			std::cout << "open set is :" << std::endl;
			for (int i = 0; i < openSet.size(); ++i) {
				std::cout << openSet[i]->point.x << " "<< openSet[i]->point.z << " " << openSet[i]->cell.x << " " << openSet[i]->cell.z << " " << openSet[i]->f << " " << openSet[i]->g << std::endl;
			}
			std::cout << std::endl;
*/

			AStarPlannerNode* currentNode = *(openSet.begin());
//			std::cout << "currentNode is " << currentNode->point.x << " " << currentNode->point.z << std::endl;
			openSet.erase(openSet.begin());
			closeSet.insert(currentNode);

			if(*currentNode == *goalNode){
				std::cout << "We found a path!!!!" << std::endl;
				std::cout << "path length is " << goalNode->f << " expanded Nodes are " << closeSet.size() << std:: endl;
				return constructPath(agent_path, currentNode);
			}

			std::vector<AStarPlannerNode*> neighbors = getNeighborNodes(currentNode);

//			std::cout << "Find " << neighbors.size() << " neighbors for node at " << currentNode->cell.x << " " << currentNode->cell.z << std::endl;

			for (std::vector<AStarPlannerNode*>::iterator it = neighbors.begin(); 
					it != neighbors.end(); ++it){
				if(closeSet.find(*it) != closeSet.end()){
//					std::cout << " find a neighbor in closeSet" << std::endl;
					continue;
				}

				// calculate distance from current node to this neighbor
				double delta = 1;
				delta = heuristicEuclidean(currentNode->cell.x, currentNode->cell.z, (*it)->cell.x, (*it)->cell.z);
				double new_g = currentNode->g + delta;

				if(new_g < (*it)->g) {
					(*it)->parent = currentNode;
					(*it)->g = new_g;
				//	(*it)->f = new_g +  2*heuristicManhattan((*it)->cell.x, (*it)->cell.z, goalNode->cell.x, goalNode->cell.z);
					(*it)->f = new_g +  heuristicEuclidean((*it)->cell.x, (*it)->cell.z, goalNode->cell.x, goalNode->cell.z);

					std::vector<AStarPlannerNode*>::iterator myit = std::find(openSet.begin(), openSet.end(), *it);
				        if (myit == openSet.end()) {
						openSet.push_back(*it);
//						std::cout << "insert into openSet, openSet size is " << openSet.size() << std::endl;
					} else {
//						std::cout << "find a smaller g: new_g = " << new_g << " previous->g = " << (*myit)->g << std::endl;
						(*myit)->g = (*it)->g;
						(*myit)->f = (*it)->f;
					}
				} else {
//					std::cout << "new_g is larger, drop it. new_g = " << new_g << " it->g = " << (*it)->g << std::endl;
				}
			}

			std::sort(openSet.begin(), openSet.end(), NodeCompare());
//			std::cout << std::endl;
//			outputStatue();
		}

		std::cout << "No path!!!!" << std::endl;
		return false;
	}

	void AStarPlanner::outputStatue() {
		std::cout << "Current maze is like this:" << std::endl;
		for (int i = 0; i < maze.size(); ++i) {
			for ( int j = 0; j < maze[i].size(); ++j) {
				std::vector<AStarPlannerNode*>::iterator myit = std::find(openSet.begin(), openSet.end(), &maze[i][j]);
				if (myit != openSet.end()) {
					std::cout << "1";
					continue;
				}

				if(closeSet.find(&maze[i][j]) != closeSet.end()){
					std::cout << "0";
					continue;
				}

				std::cout << ".";
			}
			std::cout << std::endl;
		}
	}

	std::vector<AStarPlannerNode*> AStarPlanner::getNeighborNodes(AStarPlannerNode* currentNode) { 
		unsigned int x = currentNode->cell.x;
		unsigned int z = currentNode->cell.z;

		std::vector<AStarPlannerNode*> neighbors;

		for (int i = x - 1; i <= x + 1; ++i) {
			for (int j = z - 1; j <= z + 1; ++j) {
				if (i == x && j == z) continue;
				if (i < 0 || i >= maze.size()) continue;
				if (j < 0 || j >= maze[i].size()) continue;

				double traversal_cost = gSpatialDatabase->getTraversalCost ( maze[i][j].index );
				std::set<SteerLib::SpatialDatabaseItemPtr> items;

				if (traversal_cost > COLLISION_COST) continue;
//				std::cout << "Found a neighbor!!!!!" << std::endl;
				neighbors.push_back(&maze[i][j]);
			}
		}

		return neighbors;
	}

	bool AStarPlanner::constructPath(std::vector<Util::Point>& agent_path, AStarPlannerNode* endNode)
	{
		if(!endNode){
			return false;
		}
		while(endNode->parent){
			agent_path.insert(agent_path.begin(), endNode->point);
			endNode = endNode->parent;
		}    
		agent_path.insert(agent_path.begin(), endNode->point);
		return true;
	}

	double AStarPlanner::heuristicManhattan(int x_1, int z_1, int x_2, int z_2) { 
		return double(abs(x_1 - x_2) + abs(z_1 - z_2));
	}

	double AStarPlanner::heuristicEuclidean(int x_1, int z_1, int x_2, int z_2) {
		return sqrt(pow(double((x_1 - x_2)), 2.0) + pow(double(z_1 - z_2), 2.0));   
	}

}
