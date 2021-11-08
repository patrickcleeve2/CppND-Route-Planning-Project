#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the start and end coordinates.
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}



// Calculate h-value using euclidean distance between nodes
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return node->distance(*end_node);

}



// Add all unvisited neighbors to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // FindNeighbors for the current node
    current_node->FindNeighbors();

    // For each of the current nodes neighbours, set the parent, h_value, g_value, 
    // visited attribute to true, and add it to the open_list.
    for (auto& node: current_node->neighbors) {

        node->parent = current_node;
        node->g_value = current_node->g_value + node->distance(*current_node);   // used hint from forums: question/720909 and /719494
        node->h_value = CalculateHValue(node);
        node->visited = true;
        this->open_list.emplace_back(node);
    }

}


// Return the next node to expand based on sum of h and g values.
#include <algorithm>

bool CompareNode(RouteModel::Node* node_1, RouteModel::Node* node_2){
    // Compare nodes according to the sum of the h_value and g_value (f_value)

    return (node_1->g_value + node_1->h_value) > (node_2->g_value + node_2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(this->open_list.begin(), this->open_list.end(), CompareNode);

    // create a pointer to the node with the lowest sum, and remove it from the open_list
    RouteModel::Node* next_node = this->open_list.back();
    this->open_list.pop_back();

    return next_node;
}



// Construct the final path from the goal to the start, and calculate the total distance
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // follow the chain of parents until the starting node is found    
    while (current_node != this->start_node){
        
        // add the distance from the node to its parent
        distance += current_node->distance(*current_node->parent);
        // add the current node to the found path
        path_found.push_back(*current_node);
        
        // follow the chain to the parent node
        current_node = current_node->parent;

    }

    // add the final start_node to the found path
    path_found.push_back(*current_node);

    // reverse the order of the found path
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// A* Search
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // initialise the start node
    this->start_node->g_value = 0;
    this->start_node->h_value = CalculateHValue(this->start_node);
    this->start_node->visited = true;  // used an answer from forums questions/720909 as hint
    this->open_list.push_back(this->start_node);
    
    while (this-open_list.size() > 0) {

        // sort the open list and return the next node
        current_node = this->NextNode();

        // stop when the search reaches the end node, store the final path in m_Model, return
        if (current_node->distance(*this->end_node) == 0) {
            this->m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // add all neighbours of the current node to the open list
        this->AddNeighbors(current_node);
    }




}