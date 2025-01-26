#include <iostream>
#include <string>
#include <filesystem>

#include "FileIO.h"
#include "Grid.h"
#include "Solver.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[])
{
    std::string directory = "puzzles";

    if (argc > 1)
    {
        directory = argv[1];
    }

    if (!fs::exists(directory))
    {
        std::cerr << "Error: " << directory << " does not exist." << std::endl;
        return 1;
    }

    try
    {
        if (fs::is_regular_file(directory))
        {
            std::cout << "Processing file: " << directory << std::endl;

            FileIO fileIO;
            Grid problemGrid(0);
            Grid solutionGrid(0);

            fileIO.loadPuzzleWithSolution(directory, problemGrid, solutionGrid);

            Solver solver(problemGrid, solutionGrid);
            auto [success, elapsedSeconds] = solver.solve();

            if (success)
            {
                std::cout << "Solution for " << directory
                    << " found in " << elapsedSeconds << " seconds." << std::endl;
            }
            else
            {
                std::cout << "[WARNING!] No solution found for " << directory << "..." << std::endl;
            }
        }
        else if (fs::is_directory(directory))
        {
            for (const auto& entry : fs::directory_iterator(directory))
            {
                if (!entry.is_regular_file() || entry.path().extension() != ".txt")
                {
                    continue;
                }

                std::string filePath = entry.path().string();
                std::cout << "Processing: " << filePath << std::endl;

                FileIO fileIO;
                Grid problemGrid(0);
                Grid solutionGrid(0);

                fileIO.loadPuzzleWithSolution(filePath, problemGrid, solutionGrid);

                Solver solver(problemGrid, solutionGrid);
                auto [success, elapsedSeconds] = solver.solve();

                if (success)
                {
                    std::cout << "Solution for " << filePath
                        << " found in " << elapsedSeconds << " seconds." << std::endl;
                }
                else
                {
                    std::cout << "[WARNING!] No solution found for " << filePath << "..." << std::endl;
                }
            }
        }
        else
        {
            std::cerr << "Error: " << directory << " is neither a file nor a directory." << std::endl;
            return 1;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
