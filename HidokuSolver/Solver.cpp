#include "Solver.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <chrono>
#include <functional>
#include <queue>

Solver::Solver(const Grid& initialGrid, const Grid& solutionGrid) : grid(initialGrid), solution(solutionGrid)
{
    positions = findFixedCells(initialGrid);
    neighbors = computeNeighbors(initialGrid);
    dofGrid = computeInitialDOF(initialGrid);
}

std::unordered_map<int, std::pair<int, int>> Solver::findFixedCells(const Grid& grid)
{
    std::unordered_map<int, std::pair<int, int>> positions;

    for (int i = 0; i < grid.getSize(); ++i)
    {
        for (int j = 0; j < grid.getSize(); ++j)
        {
            int value = grid.getValue(i, j);
            if (value != 0)
            {
                positions[value] = { i, j };
            }
        }
    }

    return positions;
}

Neighbor Solver::computeNeighbors(const Grid& grid)
{
    int size = grid.getSize();
    Neighbor neighbors(size, std::vector<std::vector<std::pair<int, int>>>(size));

    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            std::vector<std::pair<int, int>> cellNeighbors;

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if ((dx != 0 || dy != 0) && grid.isWithinBounds(i + dx, j + dy))
                    {
                        cellNeighbors.emplace_back(i + dx, j + dy);
                    }
                }
            }

            neighbors[i][j] = std::move(cellNeighbors);
        }
    }

    return neighbors;
}

std::vector<int> Solver::getFixedNeighborValues(const Grid& grid, int x, int y) 
{
    std::vector<int> neighborValues;

    for (const auto& neighbor : neighbors[x][y]) 
    {
        int nx = neighbor.first;
        int ny = neighbor.second;

        int value = grid.getValue(nx, ny);

        if (value != 0) 
        {
            neighborValues.push_back(value);
        }
    }

    return neighborValues;
}

std::unordered_set<int> Solver::findUsedValues(const Grid& grid) 
{
    std::unordered_set<int> usedValues;

    for (int i = 0; i < grid.getSize(); ++i) 
    {
        for (int j = 0; j < grid.getSize(); ++j)
        {
            if (grid.getValue(i, j) != 0)
            {
                usedValues.insert(grid.getValue(i, j));
            }
        }
    }

    return usedValues;
}

std::vector<std::vector<int>> Solver::computeInitialDOF(const Grid& grid) 
{
    int size = grid.getSize();
    std::vector<std::vector<int>> dof(size, std::vector<int>(size, 0));
    std::unordered_set<int> usedValues = findUsedValues(grid);

    for (int i = 0; i < size; ++i) 
    {
        for (int j = 0; j < size; ++j) 
        {
            if (grid.getValue(i, j) == 0)
            {
                std::vector<int> neighborValues = getFixedNeighborValues(grid, i, j);
                int validCount = 0;

                for (int neighbor : neighborValues) 
                {
                    if ((neighbor + 1 <= size * size && !usedValues.count(neighbor + 1)) || (neighbor - 1 >= 1 && !usedValues.count(neighbor - 1))) 
                    {
                        ++validCount;
                    }
                }

                dof[i][j] = validCount;
            }
        }
    }

    return dof;
}

void Solver::updateDOF(const Position& move, int value) 
{
    int size = grid.getSize();
    int x = move.x;
    int y = move.y;

    dofGrid[x][y] = 0;

    for (const auto& neighbor : neighbors[x][y]) 
    {
        int nx = neighbor.first;
        int ny = neighbor.second;

        if (grid.getValue(nx, ny) == 0)
        {
            std::vector<int> neighborValues = getFixedNeighborValues(grid, nx, ny);
            int validCount = 0;

            for (int neighborValue : neighborValues) 
            {
                if ((neighborValue + 1 <= size * size && neighborValue + 1 != value) || (neighborValue - 1 >= 1 && neighborValue - 1 != value)) 
                {
                    ++validCount;
                }
            }
            dofGrid[nx][ny] = validCount;
        }
    }
}

bool Solver::checkNeighborConnectivity(int nx, int ny, int targetValue) 
{
    auto neighborValues = getFixedNeighborValues(grid, nx, ny);

    int connectedCount = std::count_if(neighborValues.begin(), neighborValues.end(), [&](int value) 
        { 
            return std::abs(value - targetValue) == 1; 
        });

    return connectedCount > 0;
}

bool Solver::isValidMove(int nx, int ny, int targetValue, int nextValue)
{
    if (!grid.isWithinBounds(nx, ny))
    {
        return false;
    }

    if (grid.getValue(nx, ny) != 0 && grid.getValue(nx, ny) != targetValue)
    {
        return false;
    }

    if (positions.find(targetValue) != positions.end() && positions[targetValue] != std::make_pair(nx, ny))
    {
        return false;
    }

    std::vector<int> neighborValues = getFixedNeighborValues(grid, nx, ny);

    if (positions.find(nextValue) != positions.end())
    {
       auto [fixedX, fixedY] = positions[nextValue];

       bool isBridgingFixed = std::any_of(neighborValues.begin(), neighborValues.end(), [&](int neighborValue)
          {
             return (neighborValue == nextValue || neighborValue == targetValue);
          });

       int distanceToFixed = abs(fixedX - nx) + abs(fixedY - ny);
       bool isAdjacent = (distanceToFixed == 1 || (abs(fixedX - nx) == 1 && abs(fixedY - ny) == 1));

       if (!isBridgingFixed || !isAdjacent)
       {
          return false;
       }
    }

    int validNeighborCount = std::count_if(neighborValues.begin(), neighborValues.end(), [&](int value)
        {
            return std::abs(value - targetValue) == 1;
        });

    if (validNeighborCount == 0)
    {
        return false;
    }

    bool hasValidNeighbor = std::any_of(neighborValues.begin(), neighborValues.end(), [&](int neighborValue)
        {
            return neighborValue == targetValue - 1 || neighborValue == targetValue + 1;
        });

    if (!hasValidNeighbor && !neighborValues.empty())
    {
        return false;
    }

    return true;
}

bool Solver::checkFixedValueProximity(Position pos, int value)
{
   auto [fixedX, fixedY] = positions[value];
   int manhattanDistance = abs(pos.x - fixedX) + abs(pos.y - fixedY);

   return manhattanDistance == 1 || (abs(pos.x - fixedX) == 1 && abs(pos.y - fixedY) == 1);
}

bool Solver::boardFilled() const
{
    for (int x = 0; x < grid.getSize(); ++x)
    {
        for (int y = 0; y < grid.getSize(); ++y)
        {
            if (grid.getValue(x, y) == 0)
            {
                return false;
            }
        }
    }
    return true;
}

std::vector<std::pair<int, Position>> Solver::getSortedMoves(Position current, Position opponent, int targetValue, bool isP1Turn) 
{
    std::vector<std::pair<int, Position>> moves;

    for (auto& neighbor : neighbors[current.x][current.y]) 
    {
        int nx = neighbor.first;
        int ny = neighbor.second;

        int nextTargetValue = isP1Turn ? targetValue + 1 : targetValue - 1;

        if (isValidMove(nx, ny, targetValue, nextTargetValue)) 
        {
            int heuristicScore = isP1Turn ? heuristicA(grid, nx, ny, opponent.x, opponent.y) : heuristicB(grid, nx, ny, opponent.x, opponent.y);

            moves.emplace_back(heuristicScore, Position(nx, ny));
        }
    }

    std::sort(moves.begin(), moves.end(), [&](const auto& a, const auto& b) 
        {
            return isP1Turn ? a.first > b.first : a.first < b.first;
        });

    return moves;
}

std::pair<bool, double> Solver::solve()
{
    auto start = std::chrono::high_resolution_clock::now();

    Position P1_head(positions[1].first, positions[1].second);
    Position P2_head(positions[grid.getSize() * grid.getSize()].first, positions[grid.getSize() * grid.getSize()].second);

    bool success = solveRecursive(P1_head, P2_head, true);

    auto end = std::chrono::high_resolution_clock::now();
    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;

    return { success, elapsedTime };
}

bool Solver::solveRecursive(Position P1_head, Position P2_head, bool isP1Turn)
{
    int currentValue = isP1Turn ? grid.getValue(P1_head.x, P1_head.y) : grid.getValue(P2_head.x, P2_head.y);
    int targetValue = isP1Turn ? currentValue + 1 : currentValue - 1;
    Position& currentHead = isP1Turn ? P1_head : P2_head;

    if (positions.find(targetValue) != positions.end() && checkFixedValueProximity(currentHead, targetValue))
    {
        auto [fx, fy] = positions[targetValue];
        currentHead = { fx, fy };

        targetValue = isP1Turn ? targetValue + 1 : targetValue - 1;
    }

    auto [bestMove, pathValid] = alphaBetaCSP(currentHead, isP1Turn ? P2_head : P1_head, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), grid.getSize() * grid.getSize(), isP1Turn, targetValue);

    if (grid.isComplete(solution))
    {
        return true;
    }

    if (!pathValid || bestMove.x == -1 || bestMove.y == -1)
    {
        return false;
    }

    grid.setValue(bestMove.x, bestMove.y, targetValue);
    updateDOF(bestMove, targetValue);

    positions[targetValue] = { bestMove.x, bestMove.y };
    currentHead = bestMove;

    return solveRecursive(P1_head, P2_head, !isP1Turn);
}

std::pair<Position, bool> Solver::alphaBetaCSP(Position current, Position opponent, int alpha, int beta, int depth, bool isP1Turn, int targetValue)
{
    if (checkFixedValueProximity(current, grid.getValue(opponent.x, opponent.y)) && boardFilled())
    {
        return { current, true };
    }

    if (depth == 0)
    {
        return { current, false };
    }

    Position bestMove(-1, -1);
    bool pathValid = false;

    if (positions.find(targetValue) != positions.end() && checkFixedValueProximity(current, targetValue))
    {
        auto [nx, ny] = positions[targetValue];
        int nextTargetValue = isP1Turn ? targetValue + 1 : targetValue - 1;

        auto [result, valid] = alphaBetaCSP({ nx, ny }, opponent, alpha, beta, depth - 1, isP1Turn, nextTargetValue);
        pathValid = valid;

        return { { nx, ny }, pathValid };
    }

    auto moves = getSortedMoves(current, opponent, targetValue, isP1Turn);

    for (const auto& move : moves)
    {
        int heuristicScore = move.first;
        Position nextMove = move.second;

        int nextTargetValue = isP1Turn ? targetValue + 1 : targetValue - 1;

        if (!isValidMove(nextMove.x, nextMove.y, targetValue, nextTargetValue))
        {
            continue;
        }

        int originalValue = grid.getValue(nextMove.x, nextMove.y);
        grid.setValue(nextMove.x, nextMove.y, targetValue);
        updateDOF(nextMove, targetValue);

        auto [result, valid] = alphaBetaCSP(nextMove, opponent, alpha, beta, depth - 1, isP1Turn, nextTargetValue);
        pathValid = pathValid || valid;

        grid.setValue(nextMove.x, nextMove.y, originalValue);
        updateDOF(nextMove, originalValue);

        if (valid)
        {
            return { nextMove, true };
        }

        if (isP1Turn)
        {
            alpha = std::max(alpha, heuristicScore);
            bestMove = nextMove;
        }
        else
        {
            beta = std::min(beta, heuristicScore);
            bestMove = nextMove;
        }

        if (beta <= alpha)
        {
            break;
        }
    }

    return { bestMove, pathValid };
}

int Solver::heuristicA(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY)
{
    int playerDOF = dofGrid[playerHeadX][playerHeadY];
    int opponentDOF = dofGrid[opponentHeadX][opponentHeadY];

    int conflictPenalty = 0;
    for (const auto& neighbor : neighbors[playerHeadX][playerHeadY])
    {
        int nx = neighbor.first;
        int ny = neighbor.second;
        if (grid.getValue(nx, ny) != 0)
        {
            conflictPenalty += 2;
        }
    }

    return (8 * playerDOF) - (5 * opponentDOF) - conflictPenalty;
}

int Solver::heuristicB(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY)
{
    int proximityScore = 0;
    int conflictPenalty = 0;

    for (const auto& neighbor : neighbors[playerHeadX][playerHeadY])
    {
        int nx = neighbor.first;
        int ny = neighbor.second;

        if (grid.getValue(nx, ny) == 0)
        {
            proximityScore += 5;
        }
        else
        {
            conflictPenalty += 2;
        }
    }

    int playerDOF = dofGrid[playerHeadX][playerHeadY];
    return proximityScore - conflictPenalty + (6 * playerDOF);
}

void Solver::displaySolution() const
{
    grid.display(std::cout);
}