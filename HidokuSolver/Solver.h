#pragma once

#include <vector>
#include <unordered_map>
#include <memory>
#include "Grid.h"
#include "Node.h"

class Solver 
{
public:
    Solver(const Grid& initialGrid, const Grid& solutionGrid);
    bool solve();
    void displaySolution() const;

private:
    std::shared_ptr<Node> root;
    Grid solutionGrid;

    std::unordered_map<int, std::pair<int, int>> mapGridValuesToPositions(const Grid& grid);
    std::vector<std::pair<int, int>> findNonConsecutivePairs(const std::unordered_map<int, std::pair<int, int>>& positions);

    void findAndAddChildren(std::shared_ptr<Node> parentNode, const Grid& grid, int startX, int startY, int endX, int endY,
        int totalNodes, int startValue, bool useHeuristicA, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY);

    std::unordered_map<int, std::pair<int, int>> positions;
    std::vector<std::pair<int, int>> pairs;

    void buildTree(std::shared_ptr<Node> node, int depth, bool useHeuristicA);
    bool dfsBidirectional(std::shared_ptr<Node> node, int depth, int alpha, int beta,
        bool isPlayerOneTurn, int player1X, int player1Y, int player2X, int player2Y);

    int heuristicA(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY);
    int heuristicB(const Grid& grid, int playerHeadX, int playerHeadY, int opponentHeadX, int opponentHeadY);
};