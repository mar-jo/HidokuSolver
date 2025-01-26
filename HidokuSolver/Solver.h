#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include "Grid.h"

using Path = std::vector<std::pair<int, int>>;
using Neighbor = std::vector<std::vector<std::vector<std::pair<int, int>>>>;

struct Position 
{
    int x;
    int y;

    Position(int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const Position& other) const 
    {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position& other) const 
    {
        return !(*this == other);
    }
};

struct pair_hash 
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const 
    {
        auto hash1 = std::hash<T1>()(pair.first);
        auto hash2 = std::hash<T2>()(pair.second);
        return hash1 ^ (hash2 << 1);
    }
};

class Solver 
{
public:
    Solver(const Grid& initialGrid, const Grid& solutionGrid);
    bool solve();
    void displaySolution() const;

private:
    Grid grid;
    Grid solution;

    std::vector<std::vector<int>> dofGrid;
    std::unordered_map<int, std::pair<int, int>> positions;
    Neighbor neighbors;

    std::unordered_map<int, std::pair<int, int>> findFixedCells(const Grid& grid);
    Neighbor computeNeighbors(const Grid& grid);
    std::vector<std::pair<int, Position>> getSortedMoves(Position current, Position opponent, int targetValue, bool isP1Turn);
    std::vector<int> getFixedNeighborValues(const Grid& grid, int x, int y);
    std::unordered_set<int> findUsedValues(const Grid& grid);

    std::vector<std::vector<int>> computeInitialDOF(const Grid& grid);
    void updateDOF(const Position& move, int value);

    bool checkNeighborConnectivity(int nx, int ny, int targetValue);
    bool isValidMove(int nx, int ny, int targetValue, int nextValue);

    bool solveRecursive(Position P1_head, Position P2_head, bool isP1Turn);
    std::pair<Position, bool> alphaBetaCSP(Position current, Position opponent, int alpha, int beta, int depth, bool isP1Turn, int targetValue);

    bool boardFilled() const;
    bool checkFixedValueProximity(Position pos, int value);

    int heuristicA(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY);
    int heuristicB(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY);
};