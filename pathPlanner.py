# The main path planning function using the A* algorithm.
# Additional functions, variables, or classes may be added,
# but this function signature must remain unchanged.
# It should return a list of (col, row) coordinates.

def do_a_star(grid, start, end, display_message):
    

    # Get grid dimensions
    ROWS, COLS = len(grid), len(grid[0])

    def heuristic(a, b):
        """Calculate the Euclidean distance heuristic h(n)."""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    # Allowed movements: Up, Down, Left, Right (no diagonal movement)
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    # Open set: Stores nodes to be explored (key: node, value: f-score)
    open_set = {start: heuristic(start, end)}

    # g-score: Cost from start node to the current node
    g_score = {start: 0}

    # f-score: Estimated total cost (g + h) for each node
    f_score = {start: heuristic(start, end)}

    # Parent map: Stores the previous node for path reconstruction
    came_from = {}

    while open_set:
        # Select node with the lowest f-score
        current = min(open_set, key=lambda k: f_score[k])

        # If goal is reached, reconstruct the path
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # Remove current node from open set
        del open_set[current]

        # Explore neighboring nodes
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            # Check if the neighbor is within grid bounds
            if not (0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS):
                continue

            # Check if the neighbor is a valid (walkable) cell
            if grid[neighbor[0]][neighbor[1]] == 0:
                continue

            # Calculate tentative g-score (cost to move to the neighbor)
            tentative_g_score = g_score[current] + 1  # Each move has a cost of 1

            # If a better path to the neighbor is found, update scores
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current  # Store path
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                open_set[neighbor] = f_score[neighbor]  # Add/update in open set

    # No valid path found, send a warning message
    display_message("No valid path found", "WARN")
    return []