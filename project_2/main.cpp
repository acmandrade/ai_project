#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <chrono>

int nodes_generated = 0;
int nodes_expanded = 0;

struct PuzzleNode {
    std::vector<std::vector<int>> state;
    int g;
    int h;
    int f;
    PuzzleNode* parent;
};

int distance(const std::vector<std::vector<int>>& state, const std::vector<std::vector<int>>& goal) {
    int distance = 0;
    int n = state.size();
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            int value = state[i][j];
            if (value != 0) {
                int goal_i = (value - 1) / n;
                int goal_j = (value - 1) % n;
                distance += abs(i - goal_i) + abs(j - goal_j);
            }
        }
    }
    return distance;
}

//Implement unique heuristic here
int diagonal_conflict(const std::vector<std::vector<int>>& state, const std::vector<std::vector<int>>& goal) {
    int conflict_count = 0;
    int n = state.size();
    
    for (int i = 0; i < n; ++i){
        for (int j = 0; j < n; ++j){
            int value = state[i][j];
            if (value != 0) {
                int goal_i = (value - 1) / n;
                int goal_j = (value - 1) % n;
                
                if ((i - j == goal_i - goal_j || i + j == goal_i + goal_j) && (i != goal_i || j != goal_j)) {
                    conflict_count++;
                }
            }
        }
    }
    return  conflict_count * 2;
}

bool is_goal(const std::vector<std::vector<int>>& state, const std::vector<std::vector<int>>& goal) {
    return state == goal;
}

std::vector<std::vector<std::vector<int>>> generate_successors(const std::vector<std::vector<int>>& state) {
    int n = state.size();
    int i0, j0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (state[i][j] == 0) {
                i0 = i;
                j0 = j;
                break;
            }
        }
    }

    std::vector<std::vector<std::vector<int>>> successors;
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    
    for (const auto& direction : directions) {
        int new_i = i0 + direction.first;
        int new_j = j0 + direction.second;

        if (new_i >= 0 && new_i < n && new_j >= 0 && new_j < n) {
            std::vector<std::vector<int>> new_state = state;
            std::swap(new_state[i0][j0], new_state[new_i][new_j]);
            successors.push_back(new_state);
        }
    }

    return successors;
}

void print_puzzle(const std::vector<std::vector<int>>& puzzle) {
    for (const auto& row : puzzle) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << "\n";
    }
}

//PuzzleNode* create_puzzle_node(const std::vector<std::vector<int>>& state, const std::vector<std::vector<int>>& goal) {
//    PuzzleNode* node = new PuzzleNode();
//    node->state = state;
//    node->g = 0;
//    node->h = distance(state, goal);
//    node->f = node->g + node->h;
//    node->parent = nullptr;
//
//    //increments the node counter
//    nodes_generated++;
//
//    return node;
//}

PuzzleNode* create_puzzle_node(const std::vector<std::vector<int>>& state, const std::vector<std::vector<int>>& goal, int heuristic_choice) {
    PuzzleNode* node = new PuzzleNode();
    node->state = state;
    node->g = 0;
    //node->h = distance(state, goal);
    if (heuristic_choice == 1) {
        node->h = distance(state, goal);
    } else if (heuristic_choice == 2) {
        node->h = diagonal_conflict(state, goal);
    } else {
        std::cout << "Invalid choice. Defaulting to Manhattan distance." << std::endl;
        node->h = distance(state, goal);
    }
    node->f = node->g + node->h;
    node->parent = nullptr;
    
    //increments node counter
    nodes_generated++;
    
    return node;
}

std::vector<PuzzleNode*> extract_path(PuzzleNode* goal_node) {
    std::vector<PuzzleNode*> path;
    PuzzleNode* current_node = goal_node;

    while (current_node != nullptr) {
        path.push_back(current_node);
        current_node = current_node->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void print_path(const std::vector<PuzzleNode*>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "Step " << i << ":\n";
        print_puzzle(path[i]->state);
        std::cout << "\n";
    }
}

struct container_hash {
    std::size_t operator()(const std::vector<std::vector<int>>& c) const {
        std::size_t seed = 0;
        for (const auto& row : c) {
            for (const auto& elem : row) {
                seed ^= std::hash<int>{}(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
        }
        return seed;
    }
};

//void propagate_cost_downward(PuzzleNode* node, int g_propagated, const std::vector<std::vector<int>>& goal) {
//    if (g_propagated + node->h < node->f) {
//        node->g = g_propagated;
//        node->f = g_propagated + node->h;
//
//        for (const auto& successor_state : generate_successors(node->state)) {
//            PuzzleNode* successor = create_puzzle_node(successor_state, goal);
//            if (node->parent != nullptr && node->parent->state == successor->state) {
//                continue;
//            }
//            propagate_cost_downward(successor, g_propagated + 1, goal);
//            delete successor;
//        }
//    }
//}

std::vector<PuzzleNode*> a_star(PuzzleNode* initial_node, const std::vector<std::vector<int>>& goal, int heuristic_choice) {
    auto cmp = [](PuzzleNode* a, PuzzleNode* b) { return a->f > b->f; };
    
    //Stores nodes in the OPEN list and is ordered by f' value
    std::priority_queue<PuzzleNode*, std::vector<PuzzleNode*>, decltype(cmp)> open(cmp);
    //Stores nodes in the CLOSEDE list
    std::unordered_set<std::vector<std::vector<int>>, container_hash> closed;

    open.push(initial_node);
    //Iterates until a goal node is found or no solution
    while (!open.empty()) {
        
        //retrieving best_node and removed from OPEN list
        PuzzleNode* best_node = open.top();
        nodes_expanded++;
        open.pop();
        //CHECKS TO SEE I F BESTNODE IS GOAL NODE
        //checks current stat is the goal state
        if (is_goal(best_node->state, goal)) {
            return extract_path(best_node);
        }

        closed.insert(best_node->state);
        
        //generates successors of the current state
        auto successors = generate_successors(best_node->state);

        for (const auto& successor_state : successors) {
            if (closed.find(successor_state) != closed.end()) {
                continue;
            }

            PuzzleNode* successor_node = create_puzzle_node(successor_state, goal, heuristic_choice);
            
            successor_node->parent = best_node;
            //set best_nodde to successor
            successor_node->g = best_node->g + 1;
            //compute g & f'
            successor_node->f = successor_node->g + successor_node->h;
            
            open.push(successor_node);
        }
    }

    return {}; // Return an empty vector if no solution is found.
}

int get_nodes() {
    return nodes_generated;
}

int get_nodes_expanded() {
    return nodes_expanded;
}

int get_tree_depth(const std::vector<PuzzleNode*>& path) {
    return path.size() - 1;
}

double get_b(const std::vector<PuzzleNode*>& path){
    int ng = get_nodes();
    int d = get_tree_depth(path);

    if(d == 0){
        return 0.0;
    }

    return static_cast<double>(ng) / d;
}

int main() {
    
    int heuristic_choice;
    int choice;
    
    std::vector<std::vector<int>> initial_state1 = {
        {2, 8, 3},
        {1, 6, 4},
        {0, 7, 5}
    };
    
    std::vector<std::vector<int>> initial_state2 = {
        {2, 1, 6},
        {4, 0, 8},
        {7, 5, 3}
    };

    std::vector<std::vector<int>> goal_state = {
        {1, 2, 3},
        {8, 0, 4},
        {7, 6, 5}
    };
    
    std::cout << "Choose an initial state: " << std::endl;
    std::cout << "1: " << std::endl;
    print_puzzle(initial_state1);
    std::cout << std::endl << "2: " << std::endl;
    print_puzzle(initial_state2);
    std::cout << "Enter : ";
    std::cin >> choice;
    std::cout << std::endl;

    std::vector<std::vector<int>> initial_state;
    switch (choice) {
        case 1:
            initial_state = initial_state1;
            break;
        case 2:
            initial_state = initial_state2;
            break;
        default:
            std::cout << "Invalid choice. Using the first initial state.\n";
            initial_state = initial_state1;
            break;
    }
    
    std::cout << "Choose a heuristic: " << std::endl;
    std::cout << "1. A* Final" << std::endl;
    std::cout << "2. Diagonal conflict" << std::endl;
    std::cout << "Enter: ";
    std::cin >> heuristic_choice;
    std::cout << std::endl;
    
    if(heuristic_choice != 1 && heuristic_choice != 2) {
        std::cout << "Invalid choice. Using A* Final" << std::endl;
        heuristic_choice = 1;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();

    PuzzleNode* initial_node = create_puzzle_node(initial_state, goal_state, heuristic_choice);
    std::vector<PuzzleNode*> solution_path = a_star(initial_node, goal_state, heuristic_choice);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    if (!solution_path.empty()) {
        std::cout << "Solution found in " << duration << " ms:\n\n";
        print_path(solution_path);
        std::cout << "Total nodes generated: " << get_nodes() << std::endl;
        std::cout << "Total nodes expanded: " << get_nodes_expanded() << std::endl;
        std::cout << "Depth of Tree: " << get_tree_depth(solution_path) << std::endl;
        std::cout << "Effective branching factor b*: " << get_b(solution_path) << std::endl;
    } else {
        std::cout << "No solution found.\n";
    }

    return 0;
}
