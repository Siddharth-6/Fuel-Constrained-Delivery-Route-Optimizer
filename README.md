# Fuel-Constrained-Delivery-Route-Optimizer
A delivery route optimizer that finds the path which takes minimum fuel for a vehicle to deliver parcels to multiple customers, considering fuel constraints and refueling stations. It uses Dijkstra's algorithm to ensure all deliveries are made efficiently.

## Overview
This solve an actual logistics problem in which delivery vehicles have to:
 - deliver parcels to all customers who ordered
 - use fuel stations for refueling when needed
 - minimize total travel cost/distance
 - use waypoints if it minimize the cost

## Features
 - Graph based network
 - Four node types: depot, customers, fuel stations, waypoints
 - Using Dijkstra's algorithm to find optimal route
 - Vehicles have fuel capacity

## Example Output
```
======== Dijkstra-Based Delivery Route Optimizer ========

=> Graph Structure: 
- Node 0 (Depot) -> Node 1 (dist: 8) Node 2 (dist: 5) 
- Node 1 (Customer1) -> Node 0 (dist: 8) Node 2 (dist: 6) Node 3 (dist: 5) 
- Node 2 (Customer2) -> Node 1 (dist: 6) Node 3 (dist: 2) Node 0 (dist: 5) 
- Node 3 (FuelStation) -> Node 2 (dist: 2) Node 1 (dist: 5) 

=== Starting Delivery Optimization with Dijkstra ===

Starting at depot: 0 with fuel: 10
Visiting customer: 1 (cost: 8, fuel left: 2)
Visiting customer: 2 (cost: 5, fuel left: 5)
Refueling at station: 3 (cost: 2, fuel refilled to: 10)
Intermediate visit to depot: 0 (cost: 5, fuel left: 0)
Visiting customer: 2 (cost: 2, fuel left: 8)
Visiting customer: 1 (cost: 5, fuel left: 5)
Visiting customer: 1 (cost: 6, fuel left: 2)
Refueling at station: 3 (cost: 2, fuel refilled to: 10)
Intermediate visit to depot: 0 (cost: 5, fuel left: 3)
Refueling at station: 3 (cost: 5, fuel refilled to: 10)
Visiting customer: 2 (cost: 2, fuel left: 8)
Visiting customer: 1 (cost: 5, fuel left: 5)
Visiting customer: 1 (cost: 6, fuel left: 2)
Refueling at station: 3 (cost: 2, fuel refilled to: 10)

========== Solution Found! ==========

Optimal route: Depot -> Customer2 -> FuelStation -> Customer1 -> FuelStation -> Customer2 -> Depot
- Total cost: 24
- Final Location: Depot
- Current Fuel: 3
- Fuel capacity: 10
- Total distance traveled: 24
```

**Time Complexity**: O(V^2 log V) V is no. of nodes
**Space Complexity**: O(V^2) 

## Future Improvements
 - Multiple Vehicle Support
 - Give priority to VIP customers
