#include "PathSearch.h"

namespace ufl_cap4053
{
	namespace searches
	{
		float PathSearch::scaledWeight(const Tile* tile, const Tile* desTile) {
			return 1.2f;
		}
		float PathSearch::estimate(const Tile* tile, const Tile* desTile) {
			double dx = tile->getXCoordinate() - desTile->getXCoordinate();
			double dy = tile->getYCoordinate() - desTile->getYCoordinate();
			return sqrt(dx * dx + dy * dy);
		}

		float PathSearch::edgeCost(const Tile* from, const Tile* to) {
			return 2 * tileMap->getTileRadius() *to->getWeight();
		}

		static bool isGreater(PlannerNode* const& lhs, PlannerNode* const& rhs) {
			return (lhs->finalCost > rhs->finalCost);
		}

		void PathSearch::AstarIteration(){
			PlannerNode* current = open.front();
			PlannerNode* tempcurrent = current;
			open.pop();
			if (current->state == tileMap->getGoalTile()) {
				solution.clear();
				doneSearch = true;
				while (current != nullptr) {
					solution.insert(solution.begin(), current->state);
					current = current->parent;
				}
				return;
			}
			else {
				for (const auto& tile : solution) {
					Tile* mutableTile = const_cast<Tile*>(tile);
					mutableTile->clearLines();
				}
				solution.clear();
				while (tempcurrent != nullptr) {
					solution.insert(solution.begin(), tempcurrent->state);
					tempcurrent = tempcurrent->parent;
				}
				vector<Tile const*> reversedSolution(solution.rbegin(), solution.rend());
				for (unsigned int i = 0; i < reversedSolution.size() - 1; ++i) {
					Tile* mutableTile = const_cast<Tile*>(reversedSolution.at(i));
					mutableTile->addLineTo(const_cast<Tile*>(reversedSolution.at(i + 1)), 0xFFFFA500);
				}
				Tile* lastTile = const_cast<Tile*>(solution.back());
				for (const Tile* neighbor : adjacencyList[lastTile]) {
					lastTile->addLineTo(const_cast<Tile*>(neighbor), 0xFFFFA500);
					Tile* mutableNeighbor = const_cast<Tile*>(neighbor);
				}
			}
			for (const Tile* neighbor : adjacencyList[current->state]) {
				float tempGivenCost = current->givenCost + edgeCost(current->state, neighbor);
				if (visited.find(neighbor) != visited.end()) {
					PlannerNode* node = visited[neighbor];
					if (tempGivenCost < node->givenCost) {
						open.remove(node);
						node->givenCost = tempGivenCost;
						node->finalCost = node->givenCost + node->heuristicCost * scaledWeight(node->state, tileMap->getGoalTile());
						node->parent = current;
						visited[neighbor] = node;
						Tile* mutableTile = const_cast<Tile*>(neighbor);
						mutableTile->setFill(0xFF005DFE);
						open.push(node);
					}
				}
				else {
					PlannerNode* node = new PlannerNode(neighbor);
					node->givenCost = tempGivenCost;
					node->heuristicCost = estimate(neighbor, tileMap->getGoalTile());
					node->finalCost = node->givenCost + node->heuristicCost * scaledWeight(node->state, tileMap->getGoalTile());
					node->parent = current;
					visited[neighbor] = node;
					Tile* mutableTile = const_cast<Tile*>(neighbor);
					mutableTile->setFill(0xFF005DFE);
					open.push(node);
				}
			}
		}
		PathSearch::PathSearch() : open(isGreater) {
			tileMap = nullptr;
			startRow = 0;
			startCol = 0;
			goalRow = 0;
			goalCol = 0;
			doneSearch = true;
		}

		PathSearch::~PathSearch() {
			unload();
			shutdown();
		}

		void PathSearch::load(TileMap* _tileMap) {
			tileMap = _tileMap;
			if (tileMap == nullptr) return;

			int rowCount = tileMap->getRowCount();
			int colCount = tileMap->getColumnCount();
			for (int row = 0; row < rowCount; ++row) {
				for (int col = 0; col < colCount; ++col) {
					Tile* currentTile = tileMap->getTile(row, col);
					if (currentTile->getWeight() != 0 && currentTile != nullptr) {
						vector<const Tile*> neighbors;
						for (int i = row - 1; i <= row + 1; ++i) {
							for (int j = col - 1; j <= col + 1; ++j) {
								if ((i == row && j == col)) {
									continue;
								}
								bool isAdj = false;
								int rowDiff = i - row;
								int colDiff = j - col;

								if (row % 2 == 0) {
									isAdj = (rowDiff == -1 && (colDiff == -1 || colDiff == 0)) || (rowDiff == 0 && (colDiff == 1 || colDiff == -1)) || (rowDiff == 1 && (colDiff == 0 || colDiff == -1));
								}
								else {
									isAdj = (rowDiff == -1 && (colDiff == 0 || colDiff == 1)) || (rowDiff == 0 && (colDiff == 1 || colDiff == -1)) || (rowDiff == 1 && (colDiff == 1 || colDiff == 0));
								}

								if (isAdj) {
									Tile* neighborTile = tileMap->getTile(i, j);

									if (neighborTile != nullptr && neighborTile->getWeight() != 0) {
										neighbors.push_back(neighborTile);
									}
								}
							}
						}
						adjacencyList[currentTile] = neighbors;
					}
				}
			}
		}

		void PathSearch::initialize(int _startRow, int _startCol, int _goalRow, int _goalCol) {
			startRow = _startRow;
			startCol = _startCol;
			goalRow = _goalRow;
			goalCol = _goalCol;
			doneSearch = false;
			tileMap->setStartTile(startRow, startCol);
			tileMap->setGoalTile(goalRow, goalCol);
			PlannerNode* startNode = new PlannerNode(tileMap->getStartTile());
			startNode->givenCost = 0;
			startNode->heuristicCost = estimate(tileMap->getStartTile(), tileMap->getGoalTile());
			startNode->finalCost = startNode->givenCost + startNode->heuristicCost * scaledWeight(startNode->state, tileMap->getGoalTile());
			open.push(startNode);
			visited[tileMap->getStartTile()] = startNode;
		}

		void PathSearch::update(long timeslice) {
			if (timeslice == 0) {
				AstarIteration();
			}
			auto start = std::chrono::steady_clock::now();
			auto end = start + std::chrono::milliseconds(timeslice);
			while (!open.empty() && timeslice != 0 && !doneSearch && std::chrono::steady_clock::now() < end) {
				AstarIteration();
			}
		}



		void PathSearch::shutdown() {
			for (auto& pair : visited) {
                if (pair.second != nullptr) {
                    delete pair.second;
                }
            }
			open.clear();
            visited.clear();
			solution.clear();
			startRow = 0;
			startCol = 0;
			goalRow = 0;
			goalCol = 0;
			doneSearch = true;
		}
		void PathSearch::unload() {
			adjacencyList.clear();
			tileMap = nullptr;
		}

		bool PathSearch::isDone() const {
			return doneSearch;
		}

		vector<Tile const*> const PathSearch::getSolution() const {
			for (const auto& tile : solution) {
				Tile* mutableTile = const_cast<Tile*>(tile);
				mutableTile->clearLines();
			}
			vector<Tile const*> reversedSolution(solution.rbegin(), solution.rend());
			for (unsigned int i = 0; i < reversedSolution.size() -1; ++i) {
				Tile* mutableTile = const_cast<Tile*>(reversedSolution.at(i));
				mutableTile->addLineTo(const_cast<Tile*>(reversedSolution.at(i+1)), 0xFFFF0000);
			}
			return reversedSolution;
		}
	}
}  // close namespace ufl_cap4053::searches