#pragma once

#include <vector>
#include <iostream>
#include <stdexcept>

class Grid
{
public:
	Grid(int size);
	Grid(int size, const std::vector<std::vector<int>>& boardData);

	int getSize() const;
	int getValue(int x, int y) const;
	int getMaxValue() const;

	void setValue(int x, int y, int value);

	bool isWithinBounds(int x, int y) const;
	bool isComplete(const Grid& solutionGrid) const;
	void display(std::ostream& out) const;

private:
	int size;
	std::vector<std::vector<int>> board;
};

