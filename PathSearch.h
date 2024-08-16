#pragma once
#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../PriorityQueue.h"
#include "../Framework/TileSystem/TileMap.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
using namespace std;

namespace ufl_cap4053
{
	namespace searches
	{
		struct PlannerNode {
			const Tile* state;
			PlannerNode* parent;
			float givenCost;
			float heuristicCost;
			float finalCost;
			PlannerNode(const Tile* tile) : state(tile), parent(nullptr), givenCost(0), heuristicCost(0), finalCost(0) {}
		};

		class PathSearch
		{
		private:
			TileMap* tileMap;
			unordered_map<const Tile*, vector<const Tile*>> adjacencyList;
			int startRow, startCol, goalRow, goalCol;
			vector<Tile const*> solution;
			bool doneSearch;
			float estimate(const Tile* tile, const Tile* desTile);
			float edgeCost(const Tile* from, const Tile* to);
			float scaledWeight(const Tile* tile, const Tile* desTile);
			unordered_map<const Tile*, PlannerNode*> visited;
			PriorityQueue<PlannerNode*> open;
			void AstarIteration();
			
		// CLASS DECLARATION GOES HERE
			public:
				DLLEXPORT PathSearch();
				DLLEXPORT ~PathSearch();
				DLLEXPORT void load(TileMap* _tileMap);
				DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
				DLLEXPORT void update(long timeslice);
				DLLEXPORT void shutdown();
				DLLEXPORT void unload();
				DLLEXPORT bool isDone() const;
				DLLEXPORT vector<Tile const*> const getSolution() const;
		};
	}
} 
