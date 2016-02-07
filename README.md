# AStar-Search
## Overall
A C++ implementation of A* search.
The transition model is deterministic and denoted by a `std::multimap`, with key being the current state and value a `std::pair` of next state and step cost. The heuristic should be a `std::map` from state to cost heuristic.
state and cost type can be specified as template arguments.
## Build
compile with `--std=c++11` option
## Implementation
Implemented as described in Chapter 3 of 'Artificial Intelligence: A Modern Approach(3rd edition)' by Russel and Norvig. The 'frontier' and 'visited' list are implemented by `std::priority_queue` and `std::set`, respectively.
