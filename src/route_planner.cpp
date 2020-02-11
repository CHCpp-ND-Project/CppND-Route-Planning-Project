#include "route_planner.h"
#include <algorithm>

// Constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // start_node and end_node instantiated as pointer in route_planner.h, pass memory addresses back for storing memory address
    start_node = &m_Model.FindClosestNode(start_x, start_y);         
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // dereference node(memory address) and access method .distance by using '->'
    // Pass dereferenced end_node as 'other' to distance function.
    // node and end_node are mem addresses, distance accepts nodes as arguments
    // return distance from (dereferenced memory address of node) to (dereferenced memory address of end_node)
    return node->RouteModel::Node::distance(*end_node); 
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // takes in memory address of current node, need to dereference and call FindNeighbors using ->
    current_node->RouteModel::Node::FindNeighbors();  // populates neighbors vector of node pointers
    
    // need access to current_node.neighbors vector to iterate through neighbors vector:     - std::vector<Node *> neighbors
    for(RouteModel::Node *node :current_node->neighbors) {
        // node.visited is checked when populating neighbours list, no need to recheck
        node->parent = current_node;                         // set the parent to 'current_node': Node * parent = nullptr;
        node->h_value = RoutePlanner::CalculateHValue(node); // the h_value by calling CalculateHValue and...
        node->g_value = current_node->g_value + current_node->RouteModel::Node::distance(*node); // the g_value  by incrementing current_nodes' g plus the step
        node->visited = true;                                // set visited attribute to true
        open_list.emplace_back(node);                       // add each node to the open list and...;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // this method returns the next node to be checked in A*
    // Sort the open_list according to the sum of the h_value and g_value.
    // Reviewed example from https://www.tutorialspoint.com/Sorting-a-vector-of-custom-objects-using-Cplusplus-STL
    // Using lambda expressions in C++11
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* lhs, const RouteModel::Node* rhs) {
          return (lhs->g_value + lhs->h_value) < (rhs->g_value + rhs->h_value);
    });
    RouteModel::Node *nextNode = open_list.front();         // Create a pointer to the node in the list with the lowest sum.
    open_list.erase(open_list.begin());                     // Remove that node from the open_list.
    return nextNode;                                        // Return the pointer.
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    // initialize start from end_node
    std::vector<RouteModel::Node> path_found = *end_node;
    RouteModel::Node current = end_node;
    do
    {
        RouteModel::Node *next = current->parent;
        distance += next->RouteModel::Node::distance(*current); // distance += calculate distance
        path_found.insert(path_found.begin(), 1, next);
        current = next;
    } while (node != start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}
