#pragma once

#include <string>
#include "Grid.h"

class FileIO
{
public:
	FileIO() = default;
	~FileIO() = default;

	void loadPuzzleWithSolution(const std::string&, Grid&, Grid&);
};

