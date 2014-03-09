from heapq import heappush, heappop

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

cost = 1

current_goal_area = (None, 4, None, 5)

### Map constants ###
MAP_GOAL = 4 # coding MAP_GOAL
MAP_START_POSITION = 3
SQUARE_SIDE = 1.0
MAP_WALL = 1 # coding MAP_WALL
MAP_WHITE = 0 # coding MAP_WHITE
MAP_SPECIAL_DIRECTION = 11 # coding [MAP_SPECIAL_DIRECTION, DIRECTION]
DIRECTION_E = 2
DIRECTION_NE = 3
DIRECTION_N = 4
DIRECTION_NW = 5
DIRECTION_W = 6
DIRECTION_SW = 7
DIRECTION_S = 0
DIRECTION_SE = 1
MAP_SPECIAL_EUCLIDEAN_DISTANCE = 9 # coding [MAP_SPECIAL_EUCLIDEAN_DISTANCE, DISTANCE IN MAP UNITS]
MAP_SPECIAL_OPTIMAL = 10

def update_goal_area(area, position, direction):
    pos_x, pos_y = position
    x_lesser, x_greater, y_lesser, y_greater = area
    if direction == DIRECTION_E or direction == DIRECTION_NE or direction == DIRECTION_SE:
        if y_greater is None:
            y_greater = pos_y+1
        else:
            y_greater = max(pos_y+1, y_greater)
    if direction == DIRECTION_N or direction == DIRECTION_NW or direction == DIRECTION_NE:
        if x_lesser is None:
            x_lesser = pos_x-1
        else:
            x_lesser = min(pos_x-1, x_lesser)
    if direction == DIRECTION_W or direction == DIRECTION_NW or direction == DIRECTION_SW:
        if y_lesser is None:
            y_lesser = pos_y-1
        else:
            y_lesser = min(pos_y-1, y_lesser)
    if direction == DIRECTION_S or direction == DIRECTION_SW or direction == DIRECTION_SE:
        if x_greater is None:
            x_greater = pos_x+1
        else:
            x_greater = max(pos_x+1, x_greater)
    return x_lesser, x_greater, y_lesser, y_greater

def is_in_goal_area(area, position):
    x_lesser, x_greater, y_lesser, y_greater = area
    pos_x, pos_y = position
    if y_greater is not None:
        if pos_y < y_greater:
            return False

    if x_greater is not None:
        if pos_x < x_greater:
            return False

    if y_lesser is not None:
        if pos_y > y_lesser:
            return False

    if x_lesser is not None:
        if pos_x > x_lesser:
            return False

    return True

def goal_area_defined(area):
    return any((p is not None for p in area))

def calc_goal_area_heuristic(goal_area, position):
    if is_in_goal_area(goal_area, position):
        return 0

    x_lesser, x_greater, y_lesser, y_greater = goal_area
    pos_x, pos_y = position
    x = 0
    y = 0
    if y_greater is not None:
        if pos_y <= y_greater:
            y = y_greater - pos_y

    if x_greater is not None:
        if pos_x <= x_greater:
            x = x_greater - pos_x

    if y_lesser is not None:
        if pos_y >= y_lesser:
            y = pos_y - y_lesser

    if x_lesser is not None:
        if pos_x >= x_lesser:
            x = pos_x - x_lesser

    return x + y

# ----------------------------------------
# modify code below
# ----------------------------------------

def search():
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]


    x = init[0]
    y = init[1]
    g = 0
    h = calc_goal_area_heuristic(current_goal_area, (x, y))
    f = g + h

    open = []
    heappush(open, (f, g, h, x, y))

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            next = heappop(open)

            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
            count += 1

            if is_in_goal_area(current_goal_area, (x, y)):
                found = True
            else:
                #expand winning node
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = calc_goal_area_heuristic(current_goal_area, (x2,y2))
                            f2 = g2+h2
                            heappush(open, (f2, g2, h2, x2, y2))
                            closed[x2][y2] = 1
                            action[x2][y2] = i
    for i in range(len(expand)):
        print expand[i]
    return expand #Leave this line for grading purposes!

print "goal test: ", is_in_goal_area(current_goal_area, (4,5))
search()



