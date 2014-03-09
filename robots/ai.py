from heapq import heappush, heappop
import math

grid = {(0,0):(1,0), (0,1):(1,0), (0,2):(1,0), (0,3):(0,0), (0,4):(0,0), (0,5):(0,0),
        (1,0):(1,0), (1,1):(0,0), (1,2):(1,0), (1,3):(0,0), (1,4):(0,0), (1,5):(0,0),
        (2,0):(1,0), (2,1):(0,0), (2,2):(1,0), (2,3):(0,0), (2,4):(0,0), (2,5):(0,0),
        (3,0):(1,0), (3,1):(0,0), (3,2):(1,0), (3,3):(0,0), (3,4):(0,0), (3,5):(0,0),
        (4,0):(1,0), (4,1):(0,0), (4,2):(0,0), (4,3):(0,0), (4,4):(1,0), (4,5):(0,0)}

init = [1, 1]

current_goal_area = (None, 4, None, 5)

### Map constants ###
MAP_GOAL = 4 # coding MAP_GOAL
MAP_START_POSITION = 3
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

def get_front(pos, angle):
    dist = 1.0
    return int(pos[0] + 0.5 + dist * math.cos(angle)), int(pos[1] + 0.5 + dist * math.sin(angle))

def get_neighbour_pos(curr, angle):
    #""" returns [front, left, back, right] """"
    ret = []
    ret.append(get_front(curr, angle))
    for i in range(3):
        angle += math.pi * 0.5
        angle %= 2 * math.pi
        ret.append(get_front(curr, angle))
    return ret

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

def get_planned_cost(grid, from_where, rotation, to_where):
    if rotation == 0:
        return 1
    else:
        return 2

def path_can_enter(grid, pos):
    return pos not in grid or  grid[pos][0] != MAP_WALL

#returns a STACK with path
def actions_to_path(start, goal, action):
    path = []
    curr = goal
    last_vec = goal
    while (curr != start):
        info = action[curr]
        vec = info[1]
        curr = vec
        path.append((info[0], last_vec))
        last_vec = vec

    return path

def is_search_goal(grid, current_goal_area, position):
    if goal_area_defined(current_goal_area):
        return is_in_goal_area(current_goal_area, position) and (position not in grid)
    else:
        # no goal area, any unknown field will do
        return position not in grid


def search(pos, angle, current_goal_area, grid):
    closed = {}
    closed[pos] = 1

    expand = {}
    action = {}

    x = pos[0]
    y = pos[1]
    g = 0
    h = calc_goal_area_heuristic(current_goal_area, pos)
    f = g + h

    open = []
    heappush(open, (f, g, h, x, y, angle))

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0

    found_goal = None

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return None
        else:
            next = heappop(open)

            x = next[3]
            y = next[4]
            a = next[5]
            g = next[1]
            expand[(x,y)] = count
            count += 1

            if is_search_goal(grid, current_goal_area, (x, y)):
                found = True
                found_goal = (x, y)
            else:
                neighs = get_neighbour_pos((x,y), a)
                #expand winning node
                for i in range(4):
                    x2 = neighs[i][0]
                    y2 = neighs[i][1]
                    a2 = (a + i * (math.pi / 2)) % (2*math.pi)
                    if x2 > 0  and y2 > 0: #there're always borders on 0,0 coords
                        if (x2,y2) not in closed and path_can_enter(grid, (x2,y2)):
                            g2 = g + get_planned_cost(grid, (x, y), i, (x2, y2))
                            h2 = calc_goal_area_heuristic(current_goal_area, (x2,y2))
                            f2 = g2+h2
                            heappush(open, (f2, g2, h2, x2, y2, a2))
                            closed[(x2,y2)] = 1
                            action[(x2,y2)] = i, (x, y)

    return actions_to_path(pos, found_goal, action)

print search((init[0], init[1]), 0, current_goal_area, grid)



