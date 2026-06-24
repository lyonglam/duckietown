# map_graph_prototype.py

# 1. Represent directions as (row_delta, col_delta)
# North: (-1, 0), South: (1, 0), East: (0, 1), West: (0, -1)
DIRECTIONS = {
    'N': (-1, 0), #going up one row, so we subtract 1 from the row index
    'S': (1, 0),  
    'E': (0, 1),  #going right one column, so we add 1 to the column index
    'W': (0, -1)  
}

# 2. Map how tiles connect based on the visual road layout
# Format: (row, col): { 'incoming_direction': 'outgoing_direction(s)' }
# The incoming direction is the direction the robot is traveling as it enters the tile.
tile_connections = {
    # Top road section
    (0, 2): {'N': 'E', 'W': 'S'}, # Curve: coming from below going North turns East; coming from the right going West turns South
    (0, 3): {'E': 'E', 'W': 'W'}, # Straight horizontal: East continues East; West continues West
    (0, 4): {'E': 'E', 'W': 'W'}, # Straight horizontal: East continues East; West continues West
    (0, 5): {'N': 'W', 'E': 'S'}, # Curve: coming from below going North turns West; coming from the left going East turns South

    # Upper-left and upper-right connecting roads
    (1, 1): {'N': 'E', 'W': 'S'}, # Curve
    (1, 2): {'S': 'W', 'E': 'N'}, # Curve
    (1, 5): {'N': 'N', 'S': 'S'}, # Straight vertical

    # Row 2 roads and the upper-right 3-way intersection
    (2, 0): {'N': 'E', 'W': 'S'}, # Curve
    (2, 1): {'S': 'W', 'E': 'N'}, # Curve
    (2, 3): {'N': 'E', 'W': 'S'}, # Curve
    (2, 4): {'E': 'E', 'W': 'W'}, # Straight horizontal
    (2, 5): {
        'E': ['N', 'S'], # Entering from West going East: can turn Left (N) or Right (S)
        'S': ['S', 'W'], # Entering from North going South: can go Straight (S) or turn Right (W)
        'N': ['N', 'W']  # Entering from South going North: can go Straight (N) or turn Left (W)
    },

    # Vertical roads leading into the middle row
    (3, 0): {'N': 'N', 'S': 'S'}, # Straight vertical
    (3, 3): {'N': 'N', 'S': 'S'}, # Straight vertical
    (3, 5): {'N': 'N', 'S': 'S'}, # Straight vertical

    # Left 3-way intersection with stop lines on each approach
    (4, 0): {
        'S': ['S', 'E'], # Entering from North going South: can go Straight (S) or turn Left (E)
        'N': ['N', 'E'], # Entering from South going North: can go Straight (N) or turn Right (E)
        'W': ['N', 'S']  # Entering from East going West: can turn Right (N) or Left (S)
    },
    # The long straight horizontal road across the middle (Row 4)
    (4, 1): {'E': 'E', 'W': 'W'}, # Straight horizontal
    (4, 2): {'E': 'E', 'W': 'W'}, # Straight horizontal

    # The central 4-way intersection at (4,3) has red stop lines on all four approaches
    (4, 3): {
        'E': ['E', 'N', 'S'], # Entering from West going East: can go Straight (E), Left (N), Right (S)
        'W': ['W', 'N', 'S'], # Entering from East going West: can go Straight (W), Right (N), Left (S)
        'N': ['N', 'E', 'W'], # Entering from South going North: can go Straight (N), Right (E), Left (W)
        'S': ['S', 'E', 'W']  # Entering from North going South: can go Straight (S), Left (E), Right (W)
    },
    (4, 4): {'E': 'E', 'W': 'W'}, # Straight horizontal

    # Right 3-way intersection with stop lines on each approach
    (4, 5): {
        'E': ['N', 'S'], # Entering from West going East: can turn Left (N) or Right (S)
        'S': ['S', 'W'], # Entering from North going South: can go Straight (S) or turn Right (W)
        'N': ['N', 'W']  # Entering from South going North: can go Straight (N) or turn Left (W)
    },

    # Vertical roads leading to the bottom row
    (5, 0): {'N': 'N', 'S': 'S'}, # Straight vertical
    (5, 3): {'N': 'N', 'S': 'S'}, # Straight vertical
    (5, 5): {'N': 'N', 'S': 'S'}, # Straight vertical

    # Tile: bottom left corner
    (6, 0): {
        'S': 'E', # Coming down South curves to the East
        'W': 'N'  # Coming from the East traveling West curves up North
    },
    (6, 1): {'E': 'E', 'W': 'W'}, # Straight horizontal
    (6, 2): {'E': 'E', 'W': 'W'}, # Straight horizontal

    # Bottom 3-way intersection with stop lines on each approach
    (6, 3): {
        'S': ['E', 'W'], # Entering from North going South: can turn Left (E) or Right (W)
        'E': ['E', 'N'], # Entering from West going East: can go Straight (E) or turn Left (N)
        'W': ['W', 'N']  # Entering from East going West: can go Straight (W) or turn Right (N)
    },
    (6, 4): {'E': 'E', 'W': 'W'}, # Straight horizontal
    (6, 5): {'S': 'W', 'E': 'N'}  # Curve
}

def get_neighbors(row, col, facing):
    """
    Helper function for Person 2's A* algorithm.
    Returns a list of valid next steps: (next_row, next_col, next_facing)
    """
    if (row, col) not in tile_connections:
        return []
    
    moves = tile_connections[(row, col)].get(facing, [])
    if isinstance(moves, str): # Single exit path (straights/curves)
        moves = [moves]
        
    neighbors = []
    for out_dir in moves:
        dr, dc = DIRECTIONS[out_dir]
        neighbors.append((row + dr, col + dc, out_dir))
    return neighbors

# --- Quick Test ---
if __name__ == "__main__":
    print("--- Testing Map Graph Model ---")
    
    # Test 1: Standard straight movement
    print("\n1. Straight road at (4,2) facing East:")
    print("Options:", get_neighbors(4, 2, 'E')) 
    # Expect: [(4, 3, 'E')]
    
    # Test 2: Arriving at the central 4-way intersection
    print("\n2. Inside the 4-way hub at (4,3) facing East:")
    print("Options:", get_neighbors(4, 3, 'E'))
    # Expect: [(4, 4, 'E'), (3, 3, 'N'), (5, 3, 'S')] -> Straight, Left, Right!
    
    # Test 3: A tricky curve
    print("\n3. Bottom left curve at (6,0) facing South:")
    print("Options:", get_neighbors(6, 0, 'S'))
    # Expect: [(6, 1, 'E')] -> Curves the bot to the East