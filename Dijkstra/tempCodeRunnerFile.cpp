#include <iostream>
#include <vector>
#include <limits>       // For infinity assignment
#include <utility>      // For std::pair
#include <thread>       // For sleep (animation)
#include <chrono>       // For time durations
#include <unordered_map>// For mapping node names to indices
#include <map>          // For ordered maps
#include <set>          // For storing unique node names
#include <cctype>       // For character functions like isspace, isdigit
#include <algorithm>    // For sorting

using namespace std;

// Define infinity as the maximum integer value
const int infinity = numeric_limits<int>::max();

// ANSI color codes for console output (for colored terminal output)
#define RESET           "\033[0m"
#define RED             "\033[31m"
#define GREEN           "\033[32m"
#define YELLOW          "\033[33m"
#define BLUE            "\033[34m"
#define MAGENTA         "\033[35m"
#define CYAN            "\033[36m"
#define WHITE           "\033[37m"
#define BRIGHT_BLACK    "\033[90m"
#define BRIGHT_RED      "\033[91m"
#define BRIGHT_GREEN    "\033[92m"
#define BRIGHT_YELLOW   "\033[93m"
#define BRIGHT_BLUE     "\033[94m"
#define BRIGHT_MAGENTA  "\033[95m"
#define BRIGHT_CYAN     "\033[96m"
#define BRIGHT_WHITE    "\033[97m"
#define CLEAR_SCREEN    "\033[2J\033[1;1H"

// Function prototypes
int select_next_node(const vector<int>& cost, const vector<bool>& visited);
void initialize(int source, vector<int>& cost, vector<int>& previous);
void explore_node(int u, int v, int weight, vector<int>& cost, vector<int>& previous);
void dijkstra(int source, int dest_index, const vector<vector<pair<int, int>>>& graph, int total_nodes,
              const vector<string>& node_names);
void display_state(const vector<int>& cost, const vector<int>& previous, const vector<bool>& visited,
                   int current_node, int step, const vector<string>& node_names);

// Input handling functions
bool get_node_input(const string& prompt, string& node_name);
bool get_weight_input(const string& prompt, int& weight);
string to_lower_trimmed(const string& str);

int main() {
    map<string, int> node_indices;             // Map node names to indices
    vector<string> node_names;                           // List of node names
    vector<vector<pair<int, int>>> graph;                // Adjacency list representation of the graph
    set<string> unique_node_names;     // Set to store unique node names
    map<pair<string, string>, int> connections;                // Stores the latest weight for each connection
    map<pair<string, string>, string> connection_summaries;    // Stores summaries of connections for display
    cout << CLEAR_SCREEN;   // Clear the console screen

    // Input loop for adding connections
    while (true) {
        string from_node, to_node;
        int weight;

        // Display input messages
        cout << BRIGHT_CYAN << "New connection: " << RESET;
        cout << YELLOW << "Type 'Done' in any input to finish input new connection.\n" << RESET;

        // Prompt for inputs using helper functions
        if (!get_node_input("From Node: ", from_node)) break; // Exit loop if 'Done' is entered
        if (!get_node_input("To Node: ", to_node))     break;

        // Prevent Self-loops (Connections from a Node to Itself)
        if (from_node == to_node) {
            cout << RED << "Invalid input! Cannot create a connection from a node to itself." << RESET << endl;
            continue;  // Skip this connection and prompt again
        }

        if (!get_weight_input("Weight of connection: ", weight)) break;

        // Add node names to the set of unique node names
        unique_node_names.insert(from_node);
        unique_node_names.insert(to_node);

        // Prepare the unordered key for the connection (order doesn't matter)
        string min_node = min(from_node, to_node);
        string max_node = max(from_node, to_node);
        pair<string, string> connection_key = make_pair(min_node, max_node);

        // Update the connection weight (overwrite if exists)
        connections[connection_key] = weight;

        // Update the connection summary
        string connection_summary = GREEN + min_node + RESET + " <---> " + GREEN + max_node + RESET +
                                    " with weight = " + BRIGHT_BLUE + to_string(weight) + RESET;
        connection_summaries[connection_key] = connection_summary;


        // Display the summary of connections so far
        cout << CLEAR_SCREEN;   // Clear the console screen
        cout << BRIGHT_CYAN << "Connections Summary:" << RESET << "\n";
        for (const auto& summary : connection_summaries)
            cout << summary.second << "\n";
        cout << "\n\n"; // Add spacing before the next input
    }



    // Sort the node names alphabetically and create node indices
    node_names.assign(unique_node_names.begin(), unique_node_names.end());

    // Build node_indices mapping
    for (size_t i = 0; i < node_names.size(); ++i) {
        node_indices[node_names[i]] = i;
    }

    int total_nodes = node_names.size();                 // Total number of nodes in the graph

    // Initialize the adjacency list for the graph
    graph.resize(total_nodes);

    // Build the graph from the connections map
    for (const auto& conn : connections) {
        int u = node_indices[conn.first.first];
        int v = node_indices[conn.first.second];
        int weight = conn.second;
        graph[u].push_back(make_pair(v, weight));
        graph[v].push_back(make_pair(u, weight));
    }



    // Display final summary when entering source node
    cout << CLEAR_SCREEN;   // Clear the console screen
    cout << BRIGHT_CYAN << "Final Connections Summary:" << RESET << "\n";
    for (const auto& summary : connection_summaries)
        cout << summary.second << "\n";
    cout << "\nA graph is created!\n";


// Prompt for source node
string source_node_name;
while (true) {
    cout << GREEN << "Enter the source node: " << RESET;
    getline(cin, source_node_name);

    // Trim whitespace from the source node name manually using isspace()
    string trimmed_source_node_name = "";
    for (char c : source_node_name) {
        if (!isspace(c)) {
            trimmed_source_node_name += c; // Append non-whitespace characters
        }
    }
    source_node_name = trimmed_source_node_name;

    // Check if the source node name is empty
    if (source_node_name.empty()) {
        cout << RED << "Invalid input! Source node cannot be empty." << RESET << endl;
        continue;
    }

    // Check if the source node exists in the graph
    if (node_indices.find(source_node_name) == node_indices.end()) {
        cout << RED << "Invalid input! Source node not found in the graph." << RESET << endl;
        continue;
    }
    break;  // Valid source node entered
}

// Prompt for destination node
string dest_node_name;
while (true) {
    cout << GREEN << "Enter the destination node: " << RESET;
    getline(cin, dest_node_name);

    // Trim whitespace from the destination node name manually using isspace()
    string trimmed_dest_node_name = "";
    for (char c : dest_node_name) {
        if (!isspace(c)) {
            trimmed_dest_node_name += c; // Append non-whitespace characters
        }
    }
    dest_node_name = trimmed_dest_node_name;

    // Check if the destination node name is empty
    if (dest_node_name.empty()) {
        cout << RED << "Invalid input! Destination node cannot be empty." << RESET << endl;
        continue;
    }

    // Check if the destination node exists in the graph
    if (node_indices.find(dest_node_name) == node_indices.end()) {
        cout << RED << "Invalid input! Destination node not found in the graph." << RESET << endl;
        continue;
    }
    break;  // Valid destination node entered
}


    int source_index = node_indices[source_node_name];   // Get the index of the source node
    int dest_index = node_indices[dest_node_name];       // Get the index of the destination node

    // Run Dijkstra's algorithm from the specified source node
    dijkstra(source_index, dest_index, graph, total_nodes, node_names);
    return 0;
}





// ---------------------------- Helper functions ------------------------------

// Function to get a valid node name from the user
bool get_node_input(const string& prompt, string& node_name) {
    cout << prompt;
    getline(cin, node_name);

    // Check if the user wants to finish input
    string node_name_trimmed = to_lower_trimmed(node_name);
    if (node_name_trimmed == "done") return false;

    // Remove whitespace characters manually
    string cleaned_node_name = "";
    for (char c : node_name) {
        if (!isspace(c)) {
            cleaned_node_name += c; // Append non-whitespace characters
        }
    }
    node_name = cleaned_node_name; // Update the original string


    // Check if the node name is empty
    if (node_name.empty()) {
        cout << RED << "Invalid input! Node name cannot be empty." << RESET << endl;
        return get_node_input(prompt, node_name);  // Prompt again if input is empty
    }
    return true;  // Valid node name entered
}
bool get_weight_input(const string& prompt, int& weight) {
    cout << prompt;
    string weight_str;
    getline(cin, weight_str);

    // Check if the user wants to finish input
    string weight_trimmed = to_lower_trimmed(weight_str);
    if (weight_trimmed == "done") return false;

    // Remove all whitespace characters manually
    string cleaned_weight_str = "";
    for (char c : weight_str) {
        if (!isspace(c)) {
            cleaned_weight_str += c; // Append non-whitespace characters
        }
    }

    // Check if the weight input is empty after removing whitespaces
    if (cleaned_weight_str.empty()) {
        cout << RED << "Invalid input! Weight cannot be empty." << RESET << endl;
        return get_weight_input(prompt, weight);  // Prompt again if input is empty
    }

    // Check if the cleaned weight is a valid integer
    try {
        weight = stoi(cleaned_weight_str);
    } catch (const invalid_argument&) {
        cout << RED << "Invalid input! Weight must be a number." << RESET << endl;
        return get_weight_input(prompt, weight);  // Prompt weight again
    }
    return true;  // Valid weight entered
}


// Helper function to remove whitespace and convert string to lowercase
string to_lower_trimmed(const string& str) {
    string result = "";
    for (char c : str) {
        if (!isspace(c)) {
            result += tolower(c);
        }
    }
    return result;
}

// ------------------------- Dijkstra's algorithm ------------------------

// Function to initialize the cost and previous node arrays
void initialize(int source, vector<int>& cost, vector<int>& previous) {
    for (size_t i = 0; i < cost.size(); i++) {
        cost[i] = infinity;   // Set all costs to infinity initially
        previous[i] = -1;     // No previous node
    }
    cost[source] = 0;         // Distance to source is 0
}

// Function to relax the edge between two nodes
void explore_node(int u, int v, int weight, vector<int>& cost, vector<int>& previous) {
    // If a shorter path to v is found through u
    if (cost[u] != infinity && cost[u] + weight < cost[v]) {
        cost[v] = cost[u] + weight;  // Update cost to reach v
        previous[v] = u;              // Update previous node for v
    }
}

// Function to select the next node with the minimum cost
int select_next_node(const vector<int>& cost, const vector<bool>& visited) {
    int min_cost = infinity;
    int next_node = -1;

    // Iterate through all nodes to find the unvisited node with the smallest cost
    for (size_t i = 0; i < cost.size(); i++) {
        if (!visited[i] && cost[i] < min_cost) {
            min_cost = cost[i];
            next_node = i;
        }
    }
    return next_node; // Returns -1 if there are no more reachable nodes
}


// Function to display the current state of the algorithm
void display_state(const vector<int>& cost, const vector<int>& previous, const vector<bool>& visited,
                   int current_node, int step, const vector<string>& node_names) {

    cout << CLEAR_SCREEN;    // Clear the console screen
    cout << CYAN << "Dijkstra's Algorithm - Step " << step << RESET << "\n\n"; // Display step number
    cout << "Node\tVisited\tCost\tPrevious\n";     // Display column names

    for (size_t i = 0; i < cost.size(); i++) {
        // Display information of the Current node (Highlight in green)
        if (i == current_node) {
            cout << GREEN << node_names[i] << "\t";
            cout << visited[i] << "\t";
            if (cost[i] == infinity)   cout << "INF\t";
            else                       cout << cost[i] << "\t";

            if (previous[i] == -1)     cout << "-" << RESET << "\n";
            else                       cout << node_names[previous[i]] << RESET << "\n";
        }
        // Display information of Visited nodes (Highlight in yellow)
        else if (visited[i]) {
            cout << YELLOW << node_names[i] << "\t";
            cout << visited[i] << "\t";
            if (cost[i] == infinity)   cout << "INF\t";
            else                       cout << cost[i] << "\t";

            if (previous[i] == -1)     cout << "-" << RESET << "\n";
            else                       cout << node_names[previous[i]] << RESET << "\n";
        }
        // Display information of Unvisited nodes (No highlight)
        else {
            cout << node_names[i] << "\t";
            cout << visited[i] << "\t";
            if (cost[i] == infinity)   cout << "INF\t";
            else                       cout << cost[i] << "\t";

            if (previous[i] == -1)     cout << "-" << "\n";
            else                       cout << node_names[previous[i]] << "\n";
        }
    }
    cout << "\n";
    this_thread::sleep_for(chrono::milliseconds(10000));    // Pause for 0.5 second to simulate animation
}



// Main Dijkstra's algorithm function
void dijkstra(int source, int dest_index, const vector<vector<pair<int, int>>>& graph, int total_nodes,
              const vector<string>& node_names) {

    vector<int>  cost(total_nodes, infinity);    // Store smallest cost to each node
    vector<bool> visited(total_nodes, false);    // Track visited nodes
    vector<int>  previous(total_nodes, -1);      // Track the previous node

    initialize(source, cost, previous);          // Initialize the cost and previous arrays
    int step = 0;                                // Step counter

    for (int i = 0; i < total_nodes; i++) {
        int current_node = select_next_node(cost, visited);
        if (current_node == -1) break;           // When no more reachable nodes

        visited[current_node] = true;            // Mark the current node as visited

        // Display the current state before exploring neighbors
        display_state(cost, previous, visited, current_node, ++step, node_names);

        // Relax all neighbors of the current node
        for (size_t j = 0; j < graph[current_node].size(); j++) {
            int neighbor_node = graph[current_node][j].first;
            int edge_cost     = graph[current_node][j].second;
            explore_node(current_node, neighbor_node, edge_cost, cost, previous);
        }
    }

    // Final state display
    display_state(cost, previous, visited, -1, ++step, node_names);


    // Print final results
    cout << CYAN << "Final Results:\n" << RESET;
    cout << "Node\tCost\tPath\n";
    for (int i = 0; i < total_nodes; i++) {
        // Highlight the source node in green and destination node in red
        if (i == dest_index)  cout << RED;    // Start red color for destination node
        else if (i == source) cout << GREEN;  // Start green color for source node
        cout << node_names[i] << "\t";

        if (cost[i] == infinity)   cout << "INF\t";
        else                       cout << cost[i] << "\t";

        // Reconstruct the path from source to current node
        vector<string> path;
        int prev = i;
        while (prev != -1) {
            path.push_back(node_names[prev]);
            prev = previous[prev];
        }
        if (path.size() == 1)       cout << "-";
        else {
            for (size_t k = path.size(); k > 0; k--) {
                cout << path[k - 1];
                if (k > 1) cout << " -> ";
            }
        }

        // Reset color after printing the node
        if (i == dest_index || i == source)    cout << RESET;
        cout << "\n";
    }
}
