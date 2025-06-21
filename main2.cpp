#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <climits>
#include <tuple>
using namespace std;

// ===========================================
// STEP 1: BASIC DATA STRUCTURES
// ===========================================

struct Node {
    int id;
    string name;
    int type;  // 0=depot, 1=customer, 2=fuel_station
    
    // Constructor
    Node(int id = 0, string name = "", int type = 0) {
        this->id = id;
        this->name = name;
        this->type = type;
    }
};

struct Edge {
    int to;      // destination node id
    int weight;  // distance/cost
    
    // Constructor  
    Edge(int to = 0, int weight = 0) {
        this->to = to;
        this->weight = weight;
    }
};

// State for Dijkstra's algorithm
struct State {
    int location;
    int fuel;
    set<int> visited_customers;
    vector<int> path;
    int cost;
    
    // For priority queue (min-heap)
    bool operator>(const State& other) const {
        return cost > other.cost;
    }
};

// ===========================================
// STEP 2: GRAPH CLASS
// ===========================================
class Graph {
public:
    vector<Node> nodes;
    map<int, vector<Edge>> adj_list;  // node_id -> list of edges
    
    // Add a node to graph
    void add_node(int id, string name, int type) {
        Node new_node(id, name, type);
        nodes.push_back(new_node);
        adj_list[id] = vector<Edge>();
    }
    
    // Add edge between two nodes
    void add_edge(int from, int to, int weight) {
        adj_list[from].push_back(Edge(to, weight));
        adj_list[to].push_back(Edge(from, weight)); // undirected
    }
    
    // Get distance between two directly connected nodes
    int get_distance(int from, int to) {
        for (auto& edge : adj_list[from]) {
            if (edge.to == to) {
                return edge.weight;
            }
        }
        return -1; // not connected
    }
    
    // Print graph for debugging
    void print_graph() {
        for (auto& node : nodes) {
            cout << "- Node " << node.id << " (" << node.name << ") -> ";
            for (auto& edge : adj_list[node.id]) {
                cout << "Node " << edge.to << " (dist: " << edge.weight << ") ";
            }
            cout << endl;
        }
    }
};

// ===========================================
// STEP 3: VEHICLE CLASS  
// ===========================================
class Vehicle {
public:
    int fuel_capacity;
    int current_fuel;
    int current_location;
    vector<int> route;           // path taken
    set<int> visited_customers;  // delivered customers
    int total_distance;
    
    // Constructor
    Vehicle(int fuel_cap = 100, int start_location = 0) {
        fuel_capacity = fuel_cap;
        current_fuel = fuel_capacity;
        current_location = start_location;
        route.clear();
        total_distance = 0;
    }
    
    // Set the final solution
    void set_solution(vector<int> final_route, int final_cost) {
        route = final_route;
        total_distance = final_cost;
        current_location = (route.empty()) ? 0 : route.back();
    }
    
    // Print vehicle status
    void print_status() {
        cout << "- Fuel capacity: " << fuel_capacity << endl;
        cout << "- Total distance traveled: " << total_distance << endl;
        cout << endl;
    }
};

// ===========================================
// STEP 4: DIJKSTRA-BASED OPTIMIZER CLASS
// ===========================================
class Optimizer {
private:
    Graph* graph;
    Vehicle* vehicle;
    vector<int> customers;       // list of customer node ids
    vector<int> fuel_stations;   // list of fuel station node ids
    int depot;                   // depot node id
    
    // Convert set to string for map key
    string set_to_string(const set<int>& s) {
        string result = "";
        for (int x : s) {
            result += to_string(x) + ",";
        }
        return result;
    }
    
public:
    // Constructor
    Optimizer(Graph* g, Vehicle* v, int depot_id) {
        graph = g;
        vehicle = v;
        depot = depot_id;
    }
    
    // Set customer list
    void set_customers(vector<int> customer_list) {
        customers = customer_list;
    }
    
    // Set fuel station list  
    void set_fuel_stations(vector<int> station_list) {
        fuel_stations = station_list;
    }
    
    // MAIN ALGORITHM - Dijkstra on Augmented State Space
    bool solve_delivery_problem() {
        cout << "\n=== Starting Delivery Optimization with Dijkstra ===\n" << endl;
        
        // Priority queue for Dijkstra's algorithm
        priority_queue<State, vector<State>, greater<State>> pq;
        
        // Track best cost for each state (location, fuel, visited_customers)
        map<tuple<int, int, string>, int> best_cost;
        
        // Initial state: at depot, full fuel, no customers visited
        State start_state;
        start_state.location = depot;
        start_state.fuel = vehicle->fuel_capacity;
        start_state.visited_customers = set<int>();
        start_state.path = {depot};
        start_state.cost = 0;
        
        pq.push(start_state);
        
        cout << "Starting at depot: " << depot << " with fuel: " << vehicle->fuel_capacity << endl;
        
        while (!pq.empty()) {
            State current = pq.top();
            pq.pop();
            
            // Create state key for duplicate checking
            auto state_key = make_tuple(current.location, current.fuel, set_to_string(current.visited_customers));
            
            // Skip if we've seen a better version of this state
            if (best_cost.find(state_key) != best_cost.end() && best_cost[state_key] <= current.cost) {
                continue;
            }
            best_cost[state_key] = current.cost;
            
            // Check if goal reached: all customers visited AND back at depot
            if (current.visited_customers.size() == customers.size() && current.location == depot && current.path.size() > 1) {
                cout << endl << "========== Solution Found! ==========" << endl;
                cout << endl << "Optimal route: ";
                for (size_t i = 0; i < current.path.size(); ++i) {
                    cout << graph->nodes[current.path[i]].name;
                    if (i != current.path.size() - 1) cout << " -> ";
                }
                cout << endl;
                cout << "- Total cost: " << current.cost << endl;
                cout << "- Final Location: " << graph->nodes[current.location].name << endl;
                cout << "- Current Fuel: " << current.fuel << endl;
                
                // Set the solution in vehicle
                vehicle->set_solution(current.path, current.cost);
                return true;
            }
            
            // Explore all neighbors
            for (auto& edge : graph->adj_list[current.location]) {
                int next_location = edge.to;
                int travel_cost = edge.weight;
                
                // Check if vehicle has enough fuel to travel
                if (current.fuel >= travel_cost) {
                    State next_state = current;
                    next_state.location = next_location;
                    next_state.fuel = current.fuel - travel_cost;
                    next_state.cost = current.cost + travel_cost;
                    next_state.path.push_back(next_location);
                    
                    // Check what type of location this is
                    bool is_customer = find(customers.begin(), customers.end(), next_location) != customers.end();
                    bool is_fuel_station = find(fuel_stations.begin(), fuel_stations.end(), next_location) != fuel_stations.end();
                    
                    // If it's a customer, mark as visited
                    if (is_customer) {
                        next_state.visited_customers.insert(next_location);
                        cout << "Visiting customer: " << next_location << " (cost: " << travel_cost << ", fuel left: " << next_state.fuel << ")" << endl;
                    }
                    
                    // If it's a fuel station, refuel
                    if (is_fuel_station) {
                        next_state.fuel = vehicle->fuel_capacity;
                        cout << "Refueling at station: " << next_location << " (cost: " << travel_cost << ", fuel refilled to: " << next_state.fuel << ")" << endl;
                    }
                    
                    // If it's depot and we haven't visited all customers yet, just note the visit
                    if (next_location == depot && next_state.visited_customers.size() < customers.size()) {
                        cout << "Intermediate visit to depot: " << next_location << " (cost: " << travel_cost << ", fuel left: " << next_state.fuel << ")" << endl;
                    }
                    
                    // Add to priority queue
                    pq.push(next_state);
                }
            }
        }
        
        cout << endl;
        return false;
    }
};

// ===========================================
// STEP 5: MAIN FUNCTION - TEST YOUR CODE
// ===========================================
int main() {
    cout << "\n======== Dijkstra-Based Delivery Route Optimizer ========\n\n";
    cout << "=> Graph Structure: " << endl;
    // Create a simple test case
    Graph graph;
    
    // Add nodes: depot, customers, fuel station
    graph.add_node(0, "Depot", 0);
    graph.add_node(1, "Customer1", 1);
    graph.add_node(2, "Customer2", 1);
    graph.add_node(3, "FuelStation", 2);
    
    // Add edges with distances
    graph.add_edge(0, 1, 8);  // depot to customer1
    graph.add_edge(1, 2, 6);   // customer1 to customer2
    graph.add_edge(2, 3, 2);   // customer2 to fuel station
    graph.add_edge(0, 2, 5);   // depot to customer2
    graph.add_edge(1, 3, 5);   // customer1 to fuel station
    
    // Print graph
    graph.print_graph();
    
    // Create vehicle
    Vehicle vehicle(10, 0);  // fuel_capacity=11, start_at_depot=0
    
    // Create optimizer
    Optimizer optimizer(&graph, &vehicle, 0);
    
    // Set customers and fuel stations
    vector<int> customer_list = {1, 2};    // customer node ids
    vector<int> station_list = {3};        // fuel station node ids
    optimizer.set_customers(customer_list);
    optimizer.set_fuel_stations(station_list);
    
    // Solve the problem
    bool success = optimizer.solve_delivery_problem();
    
    if (success) {
        vehicle.print_status();
    } else {
        cout << "\n=== No solution found! ===" << endl;
    }
    
    return 0;
}
