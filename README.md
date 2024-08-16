# PathSearch-A-Algorithm-
Implementation of A* Algorithm in Path Planner Application.
The A* (A-star) algorithm is a popular and efficient pathfinding and graph traversal algorithm used in computer science, particularly in fields like artificial intelligence, robotics, and game development. It is designed to find the shortest path between a start node and a goal node in a graph.
![image](https://github.com/user-attachments/assets/c06dd261-f192-46ca-84c9-90085c7446aa)

### **Key Concepts:**

- **Heuristic:** A function that estimates the cost from the current node to the goal. It helps guide the search towards the goal more efficiently.
  
- **Cost Function (f(n)):** The function used to determine the order in which nodes are explored. It is defined as:
  \[
  f(n) = g(n) + h(n)
  \]
  Where:
  - **g(n):** The actual cost from the start node to the current node \(n\).
  - **h(n):** The estimated cost from the current node \(n\) to the goal node (heuristic).

- **Open List:** A priority queue that contains nodes that need to be explored. Nodes with the lowest \(f(n)\) value are explored first.

- **Closed List:** A set that contains nodes that have already been explored.

### **How It Works:**

1. **Initialization:** Start by adding the initial node to the open list.

2. **Exploration:**
   - While there are nodes to explore in the open list, select the node with the lowest \(f(n)\) value.
   - If this node is the goal, the algorithm ends, and the path is reconstructed.
   - Otherwise, move the node from the open list to the closed list.

3. **Expansion:**
   - For the current node, evaluate all its neighboring nodes.
   - If a neighbor is not in the closed list, calculate its \(f(n)\) value.
   - If this neighbor is not in the open list or the newly calculated \(f(n)\) is lower than a previously calculated value, add it to the open list or update its value.

4. **Termination:** The algorithm continues until the goal node is reached or the open list is empty, indicating that no path exists.

### **Advantages:**
- **Optimality:** A* is guaranteed to find the shortest path if the heuristic function \(h(n)\) is admissible (it never overestimates the cost).
- **Efficiency:** A* is more efficient than other algorithms like Dijkstra’s, especially when the heuristic is well-chosen.

### **Applications:**
- Pathfinding in games (e.g., NPC movement).
- Navigation systems (e.g., GPS).
- Solving puzzles (e.g., the 8-puzzle problem).

The A* algorithm is favored for its balance between speed and accuracy, making it a go-to choice for finding optimal paths in various computational problems.
