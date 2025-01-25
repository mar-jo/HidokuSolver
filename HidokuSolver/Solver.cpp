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

    return true;
}

bool Solver::checkFixedValueProximity(Position pos, int value)
{
   auto [fixedX, fixedY] = positions[value];
   int manhattanDistance = abs(pos.x - fixedX) + abs(pos.y - fixedY);

   return manhattanDistance == 1 || (abs(pos.x - fixedX) == 1 && abs(pos.y - fixedY) == 1);
}

bool Solver::solveRecursive(Position P1_head, Position P2_head, bool isP1Turn)
{
   if (P1_head == P2_head)
   {
      return true;
   }

   bool isMaximizingPlayer = isP1Turn;

   int currentValue = isP1Turn ? grid.getValue(P1_head.x, P1_head.y) : grid.getValue(P2_head.x, P2_head.y);
   int targetValue = isP1Turn ? currentValue + 1 : currentValue - 1;
   Position& currentHead = isP1Turn ? P1_head : P2_head;
   Position& opponentHead = isP1Turn ? P2_head : P1_head;

   if (positions.find(targetValue) != positions.end())
   {
      currentHead = Position(positions[targetValue].first, positions[targetValue].second);
      grid.display(std::cout);
      return solveRecursive(P1_head, P2_head, !isP1Turn);
   }

   Position bestMove(-1, -1);
   int bestScore = isP1Turn ? std::numeric_limits<int>::min() : std::numeric_limits<int>::max();
   bool moveFound = false;

   for (const auto& neighbor : neighbors[currentHead.x][currentHead.y])
   {
      int nx = neighbor.first;
      int ny = neighbor.second;

      int nextTargetValue = isP1Turn ? targetValue + 1 : targetValue - 1;

      if (!isValidMove(nx, ny, targetValue, nextTargetValue))
         continue;

      moveFound = true;

      int originalValue = grid.getValue(nx, ny);
      grid.setValue(nx, ny, targetValue);
      updateDOF({ nx, ny }, targetValue);

      int score = minimax({ nx, ny }, opponentHead, grid.getSize() * grid.getSize(), std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), isMaximizingPlayer, isP1Turn, nextTargetValue);

      grid.setValue(nx, ny, originalValue);
      updateDOF({ nx, ny }, originalValue);

      if ((isMaximizingPlayer && score > bestScore) || (!isMaximizingPlayer && score < bestScore))
      {
          bestScore = score;
          bestMove = { nx, ny };
      }
   }

   if (!moveFound || bestMove.x == -1 || bestMove.y == -1)
   {
       return false;
   }

   grid.setValue(bestMove.x, bestMove.y, targetValue);
   updateDOF(bestMove, targetValue);
   currentHead = bestMove;

   grid.display(std::cout);

   return solveRecursive(P1_head, P2_head, !isP1Turn);
}

int Solver::minimax(Position current, Position opponent, int depth, int alpha, int beta, bool isMaximizingPlayer, bool isP1Turn, int targetValue)
{
   if (depth == 0 || current == opponent || grid.getValue(current.x, current.y) == targetValue) // stupid evaluation
   {
      return isMaximizingPlayer
         ? heuristicA(grid, current.x, current.y, opponent.x, opponent.y)
         : heuristicB(grid, current.x, current.y, opponent.x, opponent.y);
   }

   int bestScore = isMaximizingPlayer ? std::numeric_limits<int>::min() : std::numeric_limits<int>::max();

   if (positions.find(targetValue) != positions.end() && checkFixedValueProximity({ current.x, current.y }, targetValue))
   {
      auto [fixedX, fixedY] = positions[targetValue];
      current = Position(fixedX, fixedY);

      int nextTargetValue = isP1Turn ? targetValue + 1 : targetValue - 1;

      int score = minimax({ current.x, current.y }, opponent, depth - 1, alpha, beta, !isMaximizingPlayer, isP1Turn, nextTargetValue);

      int heuristicScore = isMaximizingPlayer
          ? heuristicA(grid, current.x, current.y, opponent.x, opponent.y)
          : heuristicB(grid, current.x, current.y, opponent.x, opponent.y);

      score += heuristicScore;

      if (isMaximizingPlayer)
      {
         bestScore = std::max(bestScore, score);
         alpha = std::max(alpha, bestScore);
      }
      else
      {
         bestScore = std::min(bestScore, score);
         beta = std::min(beta, bestScore);
      }

      //if (beta <= alpha)
      //    return bestScore;

      return bestScore;
   }

   for (const auto& neighbor : neighbors[current.x][current.y])
   {
      int nx = neighbor.first;
      int ny = neighbor.second;

      int nextTargetValue = isP1Turn ? targetValue + 1 : targetValue - 1;

      if (positions.find(targetValue) != positions.end() && positions[targetValue] == std::make_pair(nx, ny))
      {
         continue;
      }

      if (!isValidMove(nx, ny, targetValue, nextTargetValue))
      {
         continue;
      }

      int originalValue = grid.getValue(nx, ny);
      grid.setValue(nx, ny, targetValue);
      updateDOF({ nx, ny }, targetValue);

      int score = minimax({ nx, ny }, opponent, depth - 1, alpha, beta, !isMaximizingPlayer, isP1Turn, nextTargetValue);

      int heuristicScore = isMaximizingPlayer
          ? heuristicA(grid, nx, ny, opponent.x, opponent.y)
          : heuristicB(grid, nx, ny, opponent.x, opponent.y);

      score += heuristicScore;

      grid.setValue(nx, ny, originalValue);
      updateDOF({ nx, ny }, originalValue);

      if (isMaximizingPlayer)
      {
         bestScore = std::max(bestScore, score);
         alpha = std::max(alpha, bestScore);
      }
      else
      {
         bestScore = std::min(bestScore, score);
         beta = std::min(beta, bestScore);
      }

      if (beta <= alpha)
         break;
   }

   return bestScore;
}

bool Solver::solve() 
{
    auto start = std::chrono::high_resolution_clock::now();

    Position P1_head(positions[1].first, positions[1].second);
    Position P2_head(positions[grid.getSize() * grid.getSize()].first, positions[grid.getSize() * grid.getSize()].second);

    bool success = solveRecursive(P1_head, P2_head, true);

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Solution " << (success ? "found" : "not found") << " in "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms.\n";

    return success;
}

int Solver::heuristicA(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY)
{
    int conflictPenalty = 0;

    for (int x = 0; x < grid.getSize(); ++x)
    {
        for (int y = 0; y < grid.getSize(); ++y)
        {
            if (grid.getValue(x, y) != 0)
            {
                conflictPenalty += 10;
            }
        }
    }

    int playerDOF = dofGrid[playerHeadX][playerHeadY];
    int opponentDOF = dofGrid[opponentHeadX][opponentHeadY];

    int distanceToOpponent = abs(playerHeadX - opponentHeadX) + abs(playerHeadY - opponentHeadY);

    return -conflictPenalty + distanceToOpponent + (5 * playerDOF) - (3 * opponentDOF);
}

int Solver::heuristicB(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY)
{
    int proximityScore = 0;
    int conflictPenalty = 0;

    for (const auto& [num, pos] : positions)
    {
        auto [x, y] = pos;

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                int nx = x + dx;
                int ny = y + dy;

                if (grid.isWithinBounds(nx, ny))
                {
                    if (grid.getValue(nx, ny) == 0)
                    {
                        proximityScore += (10 - abs(dx) - abs(dy));
                    }
                    else
                    {
                        conflictPenalty += 5;
                    }
                }
            }
        }
    }

    int playerDOF = dofGrid[playerHeadX][playerHeadY];
    int opponentDOF = dofGrid[opponentHeadX][opponentHeadY];

    int distanceToOpponent = abs(playerHeadX - opponentHeadX) + abs(playerHeadY - opponentHeadY);
    int distanceFactor = -distanceToOpponent * 2;

    return proximityScore - conflictPenalty + distanceFactor + (5 * playerDOF) - (7 * opponentDOF);
}

void Solver::displaySolution() const
{
    std::cout << "FINAL GRID:" << std::endl;
    grid.display(std::cout);
}