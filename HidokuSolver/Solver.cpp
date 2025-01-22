#include "Solver.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <chrono>
#include <functional>

Solver::Solver(const Grid& initialGrid, const Grid& solutionGrid) : root(std::make_shared<Node>(initialGrid, -1)), solutionGrid(solutionGrid) 
{
    positions = mapGridValuesToPositions(initialGrid);
    pairs = findNonConsecutivePairs(positions);
}

std::unordered_map<int, std::pair<int, int>> Solver::mapGridValuesToPositions(const Grid& grid)
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

    for (const auto& [num, pos] : positions)
    {
        std::cout << "Value: " << num << ", Position: (" << pos.first << ", " << pos.second << ")\n";
    }

    return positions;
}

std::vector<std::pair<int, int>> Solver::findNonConsecutivePairs(const std::unordered_map<int, std::pair<int, int>>& positions)
{
    std::vector<int> keys;
    for (const auto& pair : positions)
    {
        keys.push_back(pair.first);
    }
    std::sort(keys.begin(), keys.end());

    std::vector<std::pair<int, int>> pairs;
    for (size_t i = 0; i < keys.size() - 1; ++i)
    {
        if (keys[i + 1] - keys[i] > 1)
        {
            pairs.emplace_back(keys[i], keys[i + 1]);
        }
    }

    std::cout << "Pairs: ";
    for (const auto& pair : pairs)
    {
        std::cout << "(" << pair.first << ", " << pair.second << ") ";
    }
    std::cout << std::endl;

    return pairs;
}

void Solver::buildTree(std::shared_ptr<Node> node, int depth, bool useHeuristicA)
{
    if (depth <= 0 || node->getGridState().isComplete(solutionGrid)) return;

    const Grid& grid = node->getGridState();

    for (const auto& [start, end] : pairs)
    {
        auto [startX, startY] = positions[start];
        auto [endX, endY] = positions[end];
        int totalNodes = abs(end - start) + 1;

        std::cout << "Building subtree for pair (" << start << ", " << end << ")\n";

        findAndAddChildren(node, grid, startX, startY, endX, endY, totalNodes, start,
            useHeuristicA, startX, startY, endX, endY);

        std::cout << "Node now has " << node->getChildren().size() << " children.\n";
    }
}

void Solver::findAndAddChildren(std::shared_ptr<Node> parentNode, const Grid& grid, int startX, int startY, int endX, int endY,
    int totalNodes, int startValue, bool useHeuristicA, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY)
{
    std::vector<std::vector<bool>> visited(grid.getSize(), std::vector<bool>(grid.getSize(), false));

    auto isValidMove = [&](int x, int y)
    {
        bool valid = grid.isWithinBounds(x, y) && !visited[x][y] && (grid.getValue(x, y) == 0 || (x == endX && y == endY));
        if (!valid)
        {
            std::cout << "Invalid move to: (" << x << ", " << y << ")\n";
        }
        return valid;
    };

    std::function<void(int, int, int, Grid)> explorePaths = [&](int x, int y, int remaining, Grid currentGrid)
    {
        if (remaining == 1 && x == endX && y == endY)
        {
            auto childNode = std::make_shared<Node>(currentGrid, startValue + totalNodes - 1);
            childNode->setScore(useHeuristicA
                ? heuristicA(currentGrid, playerHeadX, playerHeadY, opponentHeadX, opponentHeadY)
                : heuristicB(currentGrid, playerHeadX, playerHeadY, opponentHeadX, opponentHeadY));

            parentNode->addChild(childNode);

            std::cout << "Child added with score: " << childNode->getScore() << "\n";
            currentGrid.display(std::cout);
            return;
        }

        if (remaining <= 1) return;

        std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1} };

        for (const auto& direction : directions)
        {
            int nx = x + direction.first;
            int ny = y + direction.second;

            if (isValidMove(nx, ny))
            {
                visited[nx][ny] = true;
                currentGrid.setValue(nx, ny, startValue + totalNodes - remaining);

                std::cout << "Exploring move to: (" << nx << ", " << ny << "), Remaining: " << remaining - 1 << "\n";

                explorePaths(nx, ny, remaining - 1, currentGrid);

                visited[nx][ny] = false;
                currentGrid.setValue(nx, ny, 0);
            }
        }
    };

    visited[startX][startY] = true;
    Grid initialGrid = grid;
    initialGrid.setValue(startX, startY, startValue);

    explorePaths(startX, startY, totalNodes, initialGrid);
}

bool Solver::dfsBidirectional(std::shared_ptr<Node> node, int depth, int alpha, int beta,
    bool isPlayerOneTurn, int player1X, int player1Y, int player2X, int player2Y)
{
    if (node->getGridState().isComplete(solutionGrid))
    {
        root = node;
        return true;
    }

    if (depth <= 0) return false;

    if (isPlayerOneTurn)
    {
        for (const auto& child : node->getChildren())
        {
            int score = dfsBidirectional(child, depth - 1, alpha, beta, false, player1X, player1Y, player2X, player2Y);
            alpha = std::max(alpha, score);

            if (beta <= alpha) break;
        }

        return alpha >= beta;
    }
    else
    {
        for (const auto& child : node->getChildren())
        {
            int score = dfsBidirectional(child, depth - 1, alpha, beta, true, player1X, player1Y, player2X, player2Y);
            beta = std::min(beta, score);

            if (beta <= alpha) break;
        }
        return beta <= alpha;
    }
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

    int distanceToOpponent = abs(playerHeadX - opponentHeadX) + abs(playerHeadY - opponentHeadY);

    return -conflictPenalty + distanceToOpponent;
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

    int distanceToOpponent = abs(playerHeadX - opponentHeadX) + abs(playerHeadY - opponentHeadY);
    int distanceFactor = -distanceToOpponent * 2;

    return proximityScore - conflictPenalty + distanceFactor;
}

bool Solver::solve()
{
    auto start = std::chrono::high_resolution_clock::now();

    constexpr int alpha = std::numeric_limits<int>::min();
    constexpr int beta = std::numeric_limits<int>::max();

    auto [player1X, player1Y] = positions[1];
    auto [player2X, player2Y] = positions[solutionGrid.getMaxValue()];

    bool success = dfsBidirectional(root, pairs.size(), alpha, beta, true, player1X, player1Y, player2X, player2Y);

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Solution " << (success ? "found" : "not found") << " in "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;

    return success;
}

void Solver::displaySolution() const 
{
    std::cout << "FINAL GRID:" << std::endl;
    root->getGridState().display(std::cout);
}