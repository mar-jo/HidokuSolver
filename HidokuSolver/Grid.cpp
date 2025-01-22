#include "Grid.h"

Grid::Grid(int size) : size(size), board(size, std::vector<int>(size, 0)) {}

Grid::Grid(int size, const std::vector<std::vector<int>>& boardData) : size(size), board(boardData) 
{
    if ((int)boardData.size() != size || (int)boardData[0].size() != size) 
    {
        throw std::runtime_error("Grid dimensions do not match specified size.");
    }
}

int Grid::getSize() const 
{
    return size;
}

int Grid::getValue(int x, int y) const 
{
    if (!isWithinBounds(x, y)) 
    {
        throw std::out_of_range("Coordinates out of bounds.");
    }

    return board[x][y];
}

int Grid::getMaxValue() const
{
    int maxValue = 0;

    for (const auto& row : board)
    {
        for (int cell : row)
        {
            if (cell > maxValue)
            {
                maxValue = cell;
            }
        }
    }

    return maxValue;
}


void Grid::setValue(int x, int y, int value) 
{
    if (!isWithinBounds(x, y)) 
    {
        throw std::out_of_range("Coordinates out of bounds.");
    }

    board[x][y] = value;
}

bool Grid::isWithinBounds(int x, int y) const 
{
    return x >= 0 && x < size && y >= 0 && y < size;
}

bool Grid::isComplete(const Grid& solutionGrid) const 
{
    if (size != solutionGrid.getSize()) 
    {
        throw std::runtime_error("Grid sizes do not match...");
    }

    for (int i = 0; i < size; ++i) 
    {
        for (int j = 0; j < size; ++j) 
        {
            if (board[i][j] != solutionGrid.getValue(i, j)) 
            {
                return false;
            }
        }
    }

    return true;
}

void Grid::display(std::ostream& out) const 
{
    for (const auto& row : board) 
    {
        for (int cell : row) 
        {
            if (cell == 0) 
            {
                out << "- ";
            }
            else 
            {
                out << cell << " ";
            }
        }

        out << "\n";
    }
}