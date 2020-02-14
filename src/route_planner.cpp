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

// Calculate Heuristic current node to end_node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // dereference node(memory address) and access method .distance by using '->'
    // Pass dereferenced end_node as 'other' to distance function.
    // node and end_node are mem addresses, distance accepts nodes as arguments
    // return distance from (dereferenced memory address of node) to (dereferenced memory address of end_node)
    return node->RouteModel::Node::distance(*end_node); 
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
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
        open_list.emplace_back(node);                        // add each node to the open list and...;
    }
}


// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // this method returns the next node to be checked in A*
    // Sort the open_list according to the sum of the h_value and g_value.
    // Reviewed example from https://www.tutorialspoint.com/Sorting-a-vector-of-custom-objects-using-Cplusplus-STL
    // Using lambda expressions in C++11
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* lhs, const RouteModel::Node* rhs) {
          return (lhs->g_value + lhs->h_value) > (rhs->g_value + rhs->h_value);
    });
    RouteModel::Node *nextNode = open_list.back();          // Create a pointer to the node in the list with the lowest sum (sorted to end of list)
    open_list.pop_back();                                   // Remove that node from the open_list using pop_back faster
    return nextNode;                                        // Return the pointer.
}


// 6: ConstructFinalPath method to return the final path found from A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector of nodes (dereferenced), in 1st to last order
    distance = 0.0f;
    // initialize path_found to have the end_node as an element.  Only call when end_node reached
    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);

    do
    {
        RouteModel::Node *currentParent = (current_node->parent);                   // Create pointer to current node's parent    
        distance += current_node->RouteModel::Node::distance(*currentParent);       // distance += calculate distance
        path_found.push_back(*currentParent);                                       // push the node (dereferenced address)
        current_node = currentParent;                                               // pass memory address back as new current_node
    } while (current_node->x != start_node->x && current_node->y != start_node->y); // while the start_node is not found
    std::reverse(path_found.begin(),path_found.end());                              // output 
    distance *= m_Model.MetricScale();                                              // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;  //
    do{ 
        current_node = RoutePlanner::NextNode();    // sort open_list and return reference to most likely best next node
        RoutePlanner::AddNeighbors(current_node);                // Add valid open neighbors to open_list using AddNeighbors method by dereferencing current node and passing
    } while (current_node->x != end_node->x && current_node->y != end_node->y);// || !open_list.empty());
    if (current_node->x != end_node->x && current_node->y != end_node->y) {
        std::cout<< "no viable path found\n";
    }
    m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
}
