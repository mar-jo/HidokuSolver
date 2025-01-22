#include "Node.h"

Node::Node(const Grid& grid, int move) : gridState(grid), move(move), score(0) {}

Node::~Node() 
{
    clearChildren();
}

const Grid& Node::getGridState() const 
{
    return gridState;
}

int Node::getMove() const 
{
    return move;
}

int Node::getScore() const 
{
    return score;
}

const std::vector<std::shared_ptr<Node>>& Node::getChildren() const
{
    return children;
}


void Node::setScore(int score) 
{
    this->score = score;
}

void Node::addChild(std::shared_ptr<Node> child)
{
    children.push_back(child);
}


void Node::clearChildren()
{
    children.clear();
}
