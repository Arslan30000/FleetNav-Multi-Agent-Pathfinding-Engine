from collections import deque

def get_priority(robot_dict):
    return robot_dict["priority"]

def get_id(robot_dict):
    return robot_dict["id"]

def main():
    lines = []
    with open("input.txt", "r") as f:
        for line in f:
            clean_line = line.strip()
            if clean_line != "":
                lines.append(clean_line)

    if len(lines) == 0:
        return

    idx = 0
    first_line_parts = lines[idx].split()
    N = int(first_line_parts[0])
    M = int(first_line_parts[1])
    idx += 1

    grid = []
    for i in range(N):
        grid.append(lines[idx])
        idx += 1

    def get_cell(x, y):
        row = N - 1 - y
        return grid[row][x]

    R = int(lines[idx])
    idx += 1

    robots = []
    for i in range(R):
        parts = lines[idx].split()
        rid = int(parts[0])
        priority = int(parts[1])
        energy = int(parts[2])
        idx += 1
        
        start_parts = lines[idx].split()
        sx = int(start_parts[0])
        sy = int(start_parts[1])
        idx += 1
        
        goal_parts = lines[idx].split()
        gx = int(goal_parts[0])
        gy = int(goal_parts[1])
        idx += 1
        
        k = int(lines[idx])
        idx += 1
        
        checkpoints = []
        for j in range(k):
            cp_parts = lines[idx].split()
            cx = int(cp_parts[0])
            cy = int(cp_parts[1])
            checkpoints.append((cx, cy))
            idx += 1
            
        new_robot = {
            "id": rid,
            "priority": priority,
            "energy": energy,
            "start": (sx, sy),
            "goal": (gx, gy),
            "checkpoints": checkpoints
        }
        robots.append(new_robot)

    robots.sort(key=get_priority, reverse=True)

    reservations = set()
    results = {}

    for robot in robots:
        rid = robot["id"]
        start_x = robot["start"][0]
        start_y = robot["start"][1]
        goal_x = robot["goal"][0]
        goal_y = robot["goal"][1]
        checkpoints = robot["checkpoints"]
        energy_limit = robot["energy"]

        total_cps = len(checkpoints)

        initial_cp_index = 0
        if total_cps > 0:
            if start_x == checkpoints[0][0] and start_y == checkpoints[0][1]:
                initial_cp_index = 1
                
        if initial_cp_index == total_cps and start_x == goal_x and start_y == goal_y:
            results[rid] = {
                "path": [(start_x, start_y)],
                "time": 0,
                "energy": 0
            }
            reservations.add((start_x, start_y, 0))
            continue

        start_state = (start_x, start_y, initial_cp_index, 0)
        visited = set()
        visited.add(start_state)
        
        parent = {}
        parent[start_state] = None
        
        queue = deque()
        queue.append(start_state)

        found = False
        goal_state = None

        while len(queue) > 0:
            current_state = queue.popleft()
            current_x = current_state[0]
            current_y = current_state[1]
            cp_idx = current_state[2]
            t = current_state[3]

            if current_x == goal_x and current_y == goal_y and cp_idx == total_cps:
                found = True
                goal_state = current_state
                break

            if t >= energy_limit:
                continue

            next_time = t + 1
            cell = get_cell(current_x, current_y)

            possible_moves = []
            if cell == "^":
                possible_moves.append((current_x, current_y + 1))
            elif cell == "v":
                possible_moves.append((current_x, current_y - 1))
            elif cell == "<":
                possible_moves.append((current_x - 1, current_y))
            elif cell == ">":
                possible_moves.append((current_x + 1, current_y))
            else:
                possible_moves.append((current_x, current_y + 1)) 
                possible_moves.append((current_x, current_y - 1)) 
                possible_moves.append((current_x - 1, current_y)) 
                possible_moves.append((current_x + 1, current_y)) 
                possible_moves.append((current_x, current_y))     

            for move in possible_moves:
                next_x = move[0]
                next_y = move[1]

                if next_x < 0 or next_x >= M or next_y < 0 or next_y >= N:
                    continue
                    
                if get_cell(next_x, next_y) == "X":
                    continue
                    
                if (next_x, next_y, next_time) in reservations:
                    continue

                new_cp_idx = cp_idx
                if new_cp_idx < total_cps:
                    if next_x == checkpoints[new_cp_idx][0] and next_y == checkpoints[new_cp_idx][1]:
                        new_cp_idx += 1

                new_state = (next_x, next_y, new_cp_idx, next_time)
                if new_state not in visited:
                    visited.add(new_state)
                    parent[new_state] = current_state
                    queue.append(new_state)

        if found == True:
            path = []
            current_backtrack = goal_state
            while current_backtrack != None:
                path.append((current_backtrack[0], current_backtrack[1]))
                current_backtrack = parent[current_backtrack]
                
            path.reverse()
            total_time = len(path) - 1
            
            step = 0
            for point in path:
                px = point[0]
                py = point[1]
                reservations.add((px, py, step))
                step += 1
                
            results[rid] = {
                "path": path,
                "time": total_time,
                "energy": total_time
            }
        else:
            results[rid] = None

    robots.sort(key=get_id)

    with open("output.txt", "w") as f:
        first = True
        for robot in robots:
            rid = robot["id"]
            if first == False:
                f.write("\n")
            first = False
            
            if results[rid] == None:
                f.write("Error: No valid path found for Robot " + str(rid) + "\n")
            else:
                path = results[rid]["path"]
                t = results[rid]["time"]
                e = results[rid]["energy"]
                
                path_parts = []
                for point in path:
                    px = point[0]
                    py = point[1]
                    path_parts.append("(" + str(px) + "," + str(py) + ")")
                path_str = "->".join(path_parts)
                
                f.write("Robot " + str(rid) + ":\n")
                f.write("Path: " + path_str + "\n")
                f.write("Total Time: " + str(t) + "\n")
                f.write("Total Energy: " + str(e) + "\n")

if __name__ == '__main__':
    main()