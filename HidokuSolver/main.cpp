#include <iostream>
#include <string>

#include "FileIO.h"
#include "Grid.h"
#include "Solver.h"

int main(int argc, char* argv[])
{
    std::string file;

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        file = "puzzles/test.txt";
    }
    else
    {
        file = argv[1];
    }

    try
    {
        FileIO fileIO;

        Grid problemGrid(0);
        Grid solutionGrid(0);
        fileIO.loadPuzzleWithSolution(file, problemGrid, solutionGrid);

        Solver solver(problemGrid, solutionGrid);

        std::cout << "\nStarting search..." << std::endl;
        bool success = solver.solve();

        if (success)
        {
            std::cout << "\nSolution found!" << std::endl;
            solver.displaySolution();
        }
        else
        {
            std::cout << "\nNo solution found." << std::endl;
            solver.displaySolution();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
