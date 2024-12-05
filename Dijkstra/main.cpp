// Include necessary headers
#include <iostream>
#include <vector>
#include <limits>       // For infinity assignment
#include <utility>      // For pair
#include <thread>       // For sleep (animation)
#include <chrono>       // For time durations
#include <map>// For mapping node names to indices
#include <cctype>       // For character functions like isspace, isdigit
#include <cmath>        // For cos, sin functions
#include <SFML/Graphics.hpp> // For graphical display
#include <map>          // For ordered maps
#include <set>          // For storing unique node names
#include <algorithm>    // For sorting

using namespace std;

const int infinity = numeric_limits<int>::max();   // Define infinity as the maximum integer value

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

// SFML-related function prototypes
void calculate_node_positions(int window_width, int window_height, int total_nodes);
void draw_graph_with_shortest_path(sf::RenderWindow& window,
                                   const vector<vector<pair<int, int>>>& graph,
                                   const vector<string>& node_names,
                                   const map<int, sf::Vector2f>& node_positions,
                                   const vector<int>& previous,
                                   const vector<int>& cost,
                                   int source_index,
                                   int dest_index);
bool is_edge_in_shortest_path(int node1, int node2, const vector<int>& previous, int dest_index);
bool is_node_in_shortest_path(int node, const vector<int>& previous, int dest_index);

// New helper functions
void draw_node(sf::RenderWindow& window, int node_index, const string& node_name,
               const sf::Vector2f& position, const vector<int>& cost, int current_node,
               int source_index, int dest_index, const vector<bool>& visited, const vector<int>& previous,
               bool highlight_shortest_path, int dest_index_shortest_path);
void draw_edges(sf::RenderWindow& window, const vector<vector<pair<int, int>>>& graph,
                const map<int, sf::Vector2f>& node_positions, const vector<bool>& visited,
                const vector<int>& previous, int dest_index, bool highlight_shortest_path);

// Input handling functions
bool get_node_input(const string& prompt, string& node_name);
bool get_weight_input(const string& prompt, int& weight);
string to_lower_trimmed(const string& str);

// Global variables for SFML
map<int, sf::Vector2f> node_positions; // Map node indices to positions

int main() {
    // Data structures to store the graph
    map<string, int> node_indices;             // Map node names to indices
    vector<string> node_names;                 // List of node names
    vector<vector<pair<int, int>>> graph;      // Adjacency list representation of the graph

    // Maps to store connections and their summaries
    map<pair<string, string>, int> connections;                // Stores the latest weight for each connection
    map<pair<string, string>, string> connection_summaries;    // Stores summaries of connections for display

    // Set to store unique node names
    set<string> unique_node_names;

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

        // Prevent self-loops
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

        cout << CLEAR_SCREEN;   // Clear the console screen

        // Display the summary of connections so far
        cout << BRIGHT_CYAN << "Connections Summary:" << RESET << "\n";
        for (const auto& summary : connection_summaries)
            cout << summary.second << "\n";
        cout << "\n\n"; // Add spacing before the next input
    }

    // Sort the node names alphabetically and create node indices
    node_names.assign(unique_node_names.begin(), unique_node_names.end());
    sort(node_names.begin(), node_names.end());

    // Build node_indices mapping
    for (size_t i = 0; i < node_names.size(); ++i) {
        node_indices[node_names[i]] = i;
    }

    int total_nodes = node_names.size();                 // Total number of nodes in the graph

    // Initialize the adjacency list for the graph
    graph.resize(total_nodes);

    // Build the graph from the connections map
    for (const auto& conn : connections) {
        int from_index = node_indices[conn.first.first];
        int to_index = node_indices[conn.first.second];
        int weight = conn.second;
        graph[from_index].push_back(make_pair(to_index, weight));
        graph[to_index].push_back(make_pair(from_index, weight));
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

    // Calculate node positions for SFML
    int window_width = 800;
    int window_height = 600;
    calculate_node_positions(window_width, window_height, total_nodes);

    // Run Dijkstra's algorithm from the specified source node
    dijkstra(source_index, dest_index, graph, total_nodes, node_names);

    return 0;
}

// ---------------------------- Helper functions ------------------------------
bool get_node_input(const string& prompt, string& node_name) {
    cout << prompt;
    getline(cin, node_name);

    // Remove whitespace characters
    string cleaned_node_name = "";
    for (char c : node_name) {
        if (!isspace(c))    cleaned_node_name += c; // Append non-whitespace characters
    }

    // Check if the user wants to finish input
    string node_name_trimmed = to_lower_trimmed(cleaned_node_name);
    if (node_name_trimmed == "done") return false;

    // Check if the node name is empty
    if (cleaned_node_name.empty()) {
        cout << RED << "Invalid input! Node name cannot be empty." << RESET << endl;
        return get_node_input(prompt, node_name);  // Prompt again if input is empty
    }
    node_name = cleaned_node_name; // Update the original string
    return true;  // Valid node name entered
}

bool get_weight_input(const string& prompt, int& weight) {
    cout << prompt;
    string weight_str;
    getline(cin, weight_str);

    // Remove all whitespace characters manually
    string cleaned_weight_str = "";
    for (char c : weight_str) {
        if (!isspace(c)) cleaned_weight_str += c; // Append non-whitespace characters
    }

    // Check if the user wants to finish input
    string weight_trimmed = to_lower_trimmed(cleaned_weight_str);
    if (weight_trimmed == "done") return false;

    // Check if the weight input is empty after removing whitespaces
    if (weight_trimmed.empty()) {
        cout << RED << "Invalid input! Weight cannot be empty." << RESET << endl;
        return get_weight_input(prompt, weight);  // Prompt again if input is empty
    }

    // Validate if weight_trimmed contains only digits (valid integer)
    bool is_valid_number = true;
    for (char c : weight_trimmed) {
        if (!isdigit(c)) {
            is_valid_number = false;
            break;
        }
    }

    if (!is_valid_number) {
        cout << RED << "Invalid input! Weight must be a number." << RESET << endl;
        return get_weight_input(prompt, weight);  // Prompt weight again
    }

    // Convert to integer since input is validated as numeric
    weight = stoi(weight_trimmed);
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
        cost[i]     = infinity;   // Set all costs to infinity
        previous[i] = -1;         // No previous node
    }
    cost[source] = 0;             // Distance to source is 0
}

// Function to relax the edge between two nodes
void explore_node(int u, int v, int weight, vector<int>& cost, vector<int>& previous) {
    if (cost[u] != infinity && cost[u] + weight < cost[v]) {
        cost[v] = cost[u] + weight;
        previous[v] = u;  // Update cost and previous node
    }
}

// Function to select the next node with the minimum cost
int select_next_node(const vector<int>& cost, const vector<bool>& visited) {
    int min_cost = infinity;
    int next_node = -1;

    for (size_t i = 0; i < cost.size(); i++) {
        if (!visited[i] && cost[i] < min_cost) {
            min_cost = cost[i];
            next_node = i;
        }
    }
    return next_node; // -1 if there are no more reachable nodes
}

// Function to display the current state of the algorithm
void display_state(const vector<int>& cost, const vector<int>& previous, const vector<bool>& visited,
                   int current_node, int step, const vector<string>& node_names) {

    cout << CLEAR_SCREEN;    // Clear the console screen
    cout << CYAN << "Dijkstra's Algorithm - Step " << step << RESET << "\n\n";
    cout << "Node\tVisited\tCost\tPrevious\n";

    for (size_t i = 0; i < cost.size(); i++) {
        // Highlight the current node
        if (i == current_node) {
            cout << GREEN << node_names[i] << "\t";
            cout << visited[i] << "\t";
            if (cost[i] == infinity)  cout << "INF\t";
            else                      cout << cost[i] << "\t";

            if (previous[i] == -1)    cout << "-" << RESET << "\n";
            else                      cout << node_names[previous[i]] << RESET << "\n";
        }
        // Highlight Visited nodes
        else if (visited[i]) {
            cout << YELLOW << node_names[i] << "\t";
            cout << visited[i] << "\t";
            if (cost[i] == infinity)   cout << "INF\t";
            else                       cout << cost[i] << "\t";
            if (previous[i] == -1)     cout << "-" << RESET << "\n";
            else                       cout << node_names[previous[i]] << RESET << "\n";
        }
        // Not Highlight Unvisited nodes
        else {
            cout << node_names[i] << "\t";
            cout << visited[i] << "\t";
            if (cost[i] == infinity)    cout << "INF\t";
            else                        cout << cost[i] << "\t";
            if (previous[i] == -1)      cout << "-" << "\n";
            else                        cout << node_names[previous[i]] << "\n";
        }
    }
    cout << "\n";
    this_thread::sleep_for(chrono::milliseconds(500));    // Pause for a moment to simulate animation
}

// Main Dijkstra's algorithm
void dijkstra(int source, int dest_index, const vector<vector<pair<int, int>>>& graph, int total_nodes,
              const vector<string>& node_names) {
    vector<int>  cost(total_nodes, infinity);    // Store smallest cost to each node
    vector<bool> visited(total_nodes, false);    // Track visited nodes
    vector<int>  previous(total_nodes, -1);      // Track the previous node

    initialize(source, cost, previous);         // Initialize the cost and previous arrays
    int step = 0;                               // Step counter

    // SFML window setup
    int window_width = 800;
    int window_height = 600;
    sf::RenderWindow window(sf::VideoMode(window_width, window_height), "Dijkstra's Algorithm Visualization");

    for (int i = 0; i < total_nodes; i++) {
        int current_node = select_next_node(cost, visited);
        if (current_node == -1) break;          // When no more reachable nodes
        visited[current_node] = true;           // Mark the current node as visited

        // Display the current state before exploring neighbors
        display_state(cost, previous, visited, current_node, ++step, node_names);

        // Handle window events
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);

        // Draw edges
        draw_edges(window, graph, node_positions, visited, previous, dest_index, false);

        // Draw nodes
        for (int i = 0; i < node_names.size(); ++i) {
            draw_node(window, i, node_names[i], node_positions.at(i), cost, current_node, source, dest_index, visited, previous, false, dest_index);
        }

        window.display();

        // Sleep to control animation speed
        sf::sleep(sf::milliseconds(500));

        // Relax all neighbors of the current node
        for (size_t j = 0; j < graph[current_node].size(); j++) {
            int neighbor_node = graph[current_node][j].first;
            int edge_cost     = graph[current_node][j].second;
            explore_node(current_node, neighbor_node, edge_cost, cost, previous);
        }
    }

    // Final state display
    display_state(cost, previous, visited, -1, ++step, node_names);

    // Handle window events
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
    }

    window.clear(sf::Color::Black);

    // Draw edges
    draw_edges(window, graph, node_positions, visited, previous, dest_index, false);

    // Draw nodes
    for (int i = 0; i < node_names.size(); ++i) {
        draw_node(window, i, node_names[i], node_positions.at(i), cost, -1, source, dest_index, visited, previous, false, dest_index);
    }

    window.display();

    // Sleep to control animation speed
    sf::sleep(sf::milliseconds(500));

    // Print final results before opening the window
    cout << CYAN << "Final Results:\n" << RESET;
    cout << "Node\tCost\tPath\n";
    for (int i = 0; i < total_nodes; i++) {
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

    // Keep the window open until closed by the user
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        // Highlight the shortest path from source to destination
        window.clear(sf::Color::Black);
        draw_graph_with_shortest_path(window, graph, node_names, node_positions, previous, cost, source, dest_index);
        window.display();
    }
}

// --------------------------- SFML Functions -----------------------------

// Function to calculate node positions using circular layout
void calculate_node_positions(int window_width, int window_height, int total_nodes) {
    float center_x = window_width / 2.0f;
    float center_y = window_height / 2.0f;
    float radius = min(window_width, window_height) / 2.5f; // Adjust as needed
    float angle_increment = (2 * M_PI) / total_nodes;

    for (int i = 0; i < total_nodes; ++i) {
        float angle = angle_increment * i;
        float x = center_x + radius * cos(angle);
        float y = center_y + radius * sin(angle);
        node_positions[i] = sf::Vector2f(x, y);
    }
}

// Function to check if a node is part of the shortest path
bool is_node_in_shortest_path(int node, const vector<int> &previous, int dest_index) {
    int current = dest_index;
    while (current != -1) {
        if (current == node)
            return true;
        current = previous[current];
    }
    return false;
}

// Helper function to draw a node
void draw_node(sf::RenderWindow &window,
               int node_index,
               const string &node_name,
               const sf::Vector2f &position,
               const vector<int> &cost,
               int current_node, int source_index, int dest_index,
               const vector<bool> &visited,
               const vector<int>& previous,
               bool highlight_shortest_path,
               int dest_index_shortest_path) {

    // Load the font
    sf::Font font;
    font.loadFromFile("arial.ttf");

    sf::Color fill_color = sf::Color::White;        // Default Node color
    sf::Color text_color = sf::Color::Black;

    // Determine node color based on its state
    bool critical_node = is_node_in_shortest_path(node_index, previous, dest_index_shortest_path);

    if (node_index == source_index) {
        fill_color = sf::Color::Green;      // Source Node: Green color, Black text
        text_color = sf::Color::Black;
    }
    else if (node_index == dest_index) {
        fill_color = sf::Color::Red;        // Destination Node: Red color, White text
        text_color = sf::Color::White;
    }
    else if (highlight_shortest_path && critical_node) {
        fill_color = sf::Color::Cyan;       // Highlighted color for nodes in shortest path
        text_color = sf::Color::Black;
    }
    else if (node_index == current_node) {
        fill_color = sf::Color::Yellow;     // Current Node: Yellow color, Black text
        text_color = sf::Color::Black;
    }
    else if (visited[node_index]) {
        fill_color = sf::Color::Magenta;    // Visited Node: Magenta color, White text
        text_color = sf::Color::White;
    }
    else {
        fill_color = sf::Color::White;      // Unvisited Node: White color, Black text
        text_color = sf::Color::Black;
    }

    sf::CircleShape circle(30);             // Node size with radius of 30 pixels in a 60x60 box
    circle.setOrigin(30, 30);               // Center of the circle
    circle.setPosition(position);
    circle.setFillColor(fill_color);
    window.draw(circle);

    // Draw node name
    sf::Text text(node_name, font, 16);
    text.setFillColor(text_color);
    text.setStyle(sf::Text::Bold);
    text.setPosition(position.x - 10, position.y - 15);
    window.draw(text);

    // Display cost below the node
    if (cost[node_index] != infinity) {
        string cost_str = std::to_string(cost[node_index]);
        sf::Text cost_text(cost_str, font, 18);
        cost_text.setFillColor(sf::Color::Cyan);
        cost_text.setStyle(sf::Text::Bold);
        cost_text.setPosition(position.x - 15, position.y + 35);
        window.draw(cost_text);
    }
}

// Function to draw thick lines between two points
void drawThickLine(sf::RenderWindow& window, sf::Vector2f point1, sf::Vector2f point2, float thickness, sf::Color color) {
    sf::Vector2f direction = point2 - point1;
    float length = sqrt(direction.x * direction.x + direction.y * direction.y);
    sf::RectangleShape line(sf::Vector2f(length, thickness));
    line.setPosition(point1);
    line.setFillColor(color);
    line.setOrigin(0, thickness / 2);
    line.setRotation(atan2(direction.y, direction.x) * 180 / M_PI);
    window.draw(line);
}

// Function to check if an edge is part of the shortest path
bool is_edge_in_shortest_path(int node1, int node2, const vector<int> &previous, int dest_index) {
    vector<int> path_nodes;
    int current = dest_index;
    while (current != -1) {
        path_nodes.push_back(current);
        current = previous[current];
    }

    for (size_t i = 0; i < path_nodes.size() - 1; ++i) {
        int u = path_nodes[i];
        int v = path_nodes[i + 1];
        if ((u == node1 && v == node2) || (u == node2 && v == node1))
            return true;
    }
    return false;
}

// Helper function to draw edges
void draw_edges(sf::RenderWindow &window,
                const vector<vector<pair<int, int>>> &graph,
                const map<int, sf::Vector2f> &node_positions,
                const vector<bool> &visited,
                const vector<int> &previous,
                int dest_index,
                bool highlight_shortest_path) {

    sf::Font font;
    font.loadFromFile("arial.ttf");     // Load the arial font

    for (int i = 0; i < graph.size(); ++i) {
        for (const auto &neighbor : graph[i]) {
            int j = neighbor.first;   // Target Node index
            if (i < j) {              // Avoid duplicate edge: both A to B, and B to A
                sf::Color edge_color = sf::Color::White;

                // Check if edge should be highlighted
                bool critical_edge = is_edge_in_shortest_path(i, j, previous, dest_index);
                if (highlight_shortest_path && critical_edge)    edge_color = sf::Color::Cyan;
                else if (visited[i] && visited[j])               edge_color = sf::Color(255, 165, 0); // RGB: Orange

                // Draw the edge
                drawThickLine(window, node_positions.at(i), node_positions.at(j), 3.0f, edge_color);
            }
        }
    }
}

// Function to draw the graph highlighting the shortest path after the algorithm completes
void draw_graph_with_shortest_path(sf::RenderWindow &window,
                                   const vector<vector<pair<int, int>>> &graph,
                                   const vector<string> &node_names,
                                   const map<int, sf::Vector2f> &node_positions,
                                   const vector<int> &previous,
                                   const vector<int> &cost,
                                   int source_index,
                                   int dest_index) {

    // Load the font
    sf::Font font;
    font.loadFromFile("arial.ttf");

    // Draw edges, highlighting the shortest path
    vector<bool> dummy_visited(node_names.size(), false); // Not used here
    draw_edges(window, graph, node_positions, dummy_visited, previous, dest_index, true);

    // Draw nodes, highlighting nodes in the shortest path
    for (int i = 0; i < node_names.size(); ++i) {
        // We can pass dummy variables for current_node and visited since they are not relevant here
        draw_node(window, i, node_names[i], node_positions.at(i), cost, -1, source_index, dest_index, dummy_visited, previous, true, dest_index);
    }
}
