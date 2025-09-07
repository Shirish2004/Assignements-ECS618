# Path Planning Algorithms Implementation (Assignment-1 ECS- 618)

**Author:** Shirish
**Date:** 2025-09-07
**Description:**
This project implements and compares multiple path planning algorithms in a 2D environment with circular obstacles. The algorithms included are:

* Bug0
* Bug1
* Tangent Bug
* PRM (Probabilistic Roadmap Method)
* RRT (Rapidly-exploring Random Tree)

The goal is to find paths from a specified start to a goal position while avoiding obstacles, and to compare their performance based on path length, computation time, and success rate.

---

##  Project Structure

```
├── main.py            # Main script running all algorithms and displaying results
├── map.py             # Defines the 2D map, obstacles, collision checking, and visualization helpers
├── bug_alog.py        # Implementation of Bug0, Bug1, and Tangent Bug algorithms
├── prm.py             # Implementation of the Probabilistic Roadmap (PRM) algorithm
├── rrt.py             # Implementation of the Rapidly-exploring Random Tree (RRT) algorithm
├── results.csv        # Output file storing results of all algorithms
├── README.md          # This file
```

---

## Features

### Map Definition

* A 2D grid of size 30x30 units.
* Obstacles defined as circles with specified centers and radii.
* Start and goal positions are predefined but can be easily adjusted.

### Algorithms Implemented

1. **Bug Algorithms**

   * **Bug0**: Moves directly toward the goal and slides around obstacles.
   * **Bug1**: Follows obstacle boundaries fully before choosing an optimal exit point.
   * **Tangent Bug**: Uses local sensing to navigate around obstacles toward the goal.

2. **PRM (Probabilistic Roadmap Method)**

   * Samples random points avoiding obstacles.
   * Connects nearby points with collision-free edges.
   * Finds the shortest path using Dijkstra’s algorithm.

3. **RRT (Rapidly-exploring Random Tree)**

   * Builds a tree by extending from nearest nodes toward random samples.
   * Grows until the goal is reached or maximum iterations are exceeded.

---

## Results Visualization

* **Individual plots** for each algorithm showing obstacles, start and goal points, and the found path.
* **Comparative visualization** that overlays paths from all algorithms for side-by-side comparison.
* **Result table** displaying:

  * Algorithm name
  * Path length
  * Computation time
  * Success status
* Results are saved as `results.csv` for further analysis.
## Bug 0 Path
* ![Bug-0 Path](https://github.com/Shirish2004/Assignements-ECS618/blob/main/Assignment-1/assets/bug_0_path.png?raw=true)

## Bug 1 Path
* ![Bug-1 Path](https://github.com/Shirish2004/Assignements-ECS618/blob/main/Assignment-1/assets/bug_1_path.png?raw=true)

## Bug Tangent Path
* ![Bug-Tangent Path](https://github.com/Shirish2004/Assignements-ECS618/blob/main/Assignment-1/assets/bug_tangent_path.png?raw=true)

## PRM Path
* ![PRM Path](https://github.com/Shirish2004/Assignements-ECS618/blob/main/Assignment-1/assets/prm_path.png?raw=true)
  
## RRT Path
* ![RRT Path](https://github.com/Shirish2004/Assignements-ECS618/blob/main/Assignment-1/assets/rrt_path.png?raw=true)


---

## How to Run

1. Clone or download the project.
2. Ensure you have Python installed along with the required libraries:

   ```bash
   pip install numpy matplotlib pandas
   ```
3. Run the main script:

   ```bash
   python main.py
   ```
4. The program will:

   * Run all algorithms
   * Visualize the paths
   * Display the results in the console and save them in `results.csv`

---

## Dependencies

* Python 3.x
* numpy
* matplotlib
* pandas

---

## Notes

* The algorithms use fixed step sizes and parameters, but they can be modified easily for experimentation.
* All algos uses fixed seed for reproducibility.
* PRM provided shortest path but took longer to explore than RRT, and since RRT is based on PRM optimality was guaranteed
* Bug algorithms did provide paths but cannot be relied upon considering the obstacles were simple to navigate through.
* Viusalization help was taken from sources to provide effective visualizations of paths

---

