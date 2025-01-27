Team: Mark Youssef, Philipp Gschwendt

Approach:

The solver uses an alpha beta recursive approach. Additionally pruning is done by the usage of CSP -> multiple constraints were defined to limit steps that can be taken. The approach further includes a two player dynamic, where each player acts as one heuristic. Heuristics are used to sort the next moves neighbors by which one is most likely to be right. Heuristic 1 starts from the lowest number, Heuristic 2 starts from the highest number. All of which is rounded off by introducing "DOF" (=Degree of Freedom) which calculates a meta layer below the actual grid of how much freedom each cell has.

How to use:

1. Redirect into HidokuSolver, where the Makefile is included

2. Suite of following self explanatory commands you can use:

- make clean

- make 

- make test

Note: "make test" limits the solve time per puzzle to 1 minute, this will have three different outcomes:

- [PASSED]: Puzzle solved in under 1 minute.
- [SKIPPED]: Puzzle skipped, as it takes longer than 1 minute.
- [FAILED]: Puzzle failed to solve within 1 minute.

General Metrics: 459 puzzles are solvable in under 1 minute, the remaining 41 need longer than that.

