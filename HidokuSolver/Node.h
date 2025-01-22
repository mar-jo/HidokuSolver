#pragma once

#include <vector>
#include <memory>
#include "Grid.h"

class Node
{
public:
    Node(const Grid& grid, int move);
    ~Node();

    const Grid& getGridState() const;
    int getMove() const;
    int getScore() const;
    const std::vector<std::shared_ptr<Node>>& getChildren() const;

    void setScore(int score);

    void addChild(std::shared_ptr<Node> child);
    void clearChildren();

private:
    Grid gridState;
    int move;
    int score;
    std::vector<std::shared_ptr<Node>> children;
};

