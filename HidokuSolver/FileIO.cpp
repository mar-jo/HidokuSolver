#include "FileIO.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>

void FileIO::loadPuzzleWithSolution(const std::string& fileName, Grid& grid, Grid& solutionGrid)
{
    std::ifstream file(fileName);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + fileName);
    }

    std::string line;
    bool readingProblem = false, readingSolution = false;
    int gridSize = 0;
    std::vector<std::vector<int>> problemBoard, solutionBoard;

    // Read grid size from the first line
    if (std::getline(file, line))
    {
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        size_t xPos = line.find('x');
        if (xPos == std::string::npos)
        {
            throw std::runtime_error("Invalid grid size format in file: " + fileName);
        }

        try
        {
            int rows = std::stoi(line.substr(0, xPos));
            int cols = std::stoi(line.substr(xPos + 1));
            if (rows != cols)
            {
                throw std::runtime_error("Only square grids are supported: " + fileName);
            }
            gridSize = rows;
        }
        catch (const std::exception&)
        {
            throw std::runtime_error("Invalid grid size in file: " + fileName);
        }
    }

    while (std::getline(file, line))
    {
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line == "[Problem]")
        {
            readingProblem = true;
            readingSolution = false;
            problemBoard.clear();
        }
        else if (line == "[Solution]")
        {
            readingProblem = false;
            readingSolution = true;
            solutionBoard.clear();
        }
        else if (!line.empty() && (readingProblem || readingSolution))
        {
            std::istringstream stream(line);
            std::vector<int> row;
            std::string token;

            while (stream >> token)
            {
                if (token == "-")
                {
                    row.push_back(0);
                }
                else
                {
                    try
                    {
                        row.push_back(std::stoi(token));
                    }
                    catch (const std::invalid_argument&)
                    {
                        throw std::runtime_error("Invalid character in grid file: " + token);
                    }
                }
            }

            if ((int)row.size() != gridSize)
            {
                throw std::runtime_error("Inconsistent row size in file: " + fileName);
            }

            if (readingProblem)
            {
                problemBoard.push_back(row);
            }
            else if (readingSolution)
            {
                solutionBoard.push_back(row);
            }
        }
    }

    if ((int)problemBoard.size() != gridSize || (int)solutionBoard.size() != gridSize)
    {
        throw std::runtime_error("Grid dimensions do not match specified size in file: " + fileName);
    }

    grid = Grid(gridSize, problemBoard);
    solutionGrid = Grid(gridSize, solutionBoard);
}
