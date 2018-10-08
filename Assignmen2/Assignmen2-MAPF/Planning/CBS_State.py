from Utils.constants import Action, NAV_ACTIONS
import Utils.constants as C
import copy

class CBS_State:

    def __init__(self, robots, g, constraint, predecessor, plans):
        self.robots = robots # All robots to be planned
        self.g = g  # The total travel time over all robots
        self.h = 0  # Don't attempt to find an admissible heuristic for CBS. Just leave it as zero.
        # setting a heuristic for CBS is not trivial (you can come talk with me about it)

        self.p = predecessor  # The state which generated this step, 'None' for root
        self.constraint = constraint  # The constraint generated from the conflict found in the predecessor.
        # In order to get all constraints for this node, go along the predecessors until the root and collect
        # all constraints

        self.plans = plans  # The plans affiliated with this node. The plan for one agent needs to be updated (the
        # agent for which the new constraint applies to). g value must be updated accordingly.
        self.no_conflicts = False

    def copy_child(self):
        cpy = CBS_State(self.robots, self.g, self.constraint, self.p, self.plans)
        return cpy

    def opposite(self, heading_1, heading_2):
        if heading_1 == 'N' and heading_2 == 'S':
            return True
        if heading_1 == 'S' and heading_2 == 'N':
            return True
        if heading_1 == 'W' and heading_2 == 'E':
            return True
        if heading_1 == 'E' and heading_2 == 'W':
            return True
        return False

    def is_colliding(self, r_i_area, r_j_area, orig_pos_i, is_opposite):
        constraint_list = []
        occupied = set(r_i_area)
        for pos in r_j_area:
            if pos in occupied:
                constraint_list.append(pos)
        if (orig_pos_i[0], orig_pos_i[1]) in r_j_area and is_opposite:
            constraint_list.append((orig_pos_i[0], orig_pos_i[1]))
        return constraint_list

    # generate the plans for each robot respecting all current conflicts
    def generate_individual_plans(self):
        temp_plans = []
        max_plan_len = -1
        child_robots = []
        for robot in self.robots:
            child_r = copy.deepcopy(robot)
            child_r.plan_path()
            self.g += len(child_r.plan) #assuming path length as total number of actions
            max_plan_len = max(max_plan_len, len(child_r.plan))
            child_robots.append(child_r)
        for child_r, robot in zip(child_robots, self.robots):
            if len(child_r.plan) < max_plan_len:
                b = [Action.no_op] * (max_plan_len - len(child_r.plan))
                robot.plan = b + child_r.plan
            else:
                robot.plan = child_r.plan
            temp_plans.append(robot.plan)

        self.plans = temp_plans

    # iterate over all robots and call "underlying algorithm" - single agent with conflict set - edited Robot.py to include a conflict dict
    # then do a "get path" for each agent using generate_individual_plans
    # collect the results into dict and update g value based on paths at end for loop
    def expand(self):
        if(self.plans == None):
            self.generate_individual_plans() #root node don't have plans initially'
        succesors = []
        local_time = 1
        plan_length = len(self.robots[0].plan)
        child_robots = []
        for robot in self.robots:
            child_r = copy.deepcopy(robot)
            child_robots.append(child_r)
        break_outer = False
        for time in range(plan_length-1, -1, -1): #todo:check 0 or -1 in range
            if break_outer:
                break
            grid_occupancy_dict = []
            orig_pos = []
            for robot in child_robots:
                occupies = ([], [])
                orig_pos.append((robot.position_x, robot.position_y))
                try:
                    occupies = robot.step_simulation(robot.plan[time])
                except ValueError:
                    continue  # Ignore illegal actions due to existing constraints and boundary conditions

                grid_occupancy_dict.append(occupies[0])

            for i,r_i_area in enumerate(grid_occupancy_dict):
                child_CBS =  copy.deepcopy(self)
                child_CBS.p = self
                conflict_found = False
                for j, r_j_area in enumerate(grid_occupancy_dict): #brute force n*n check for collisions, should improve it
                    if i == j:
                        continue
                    else:
                        is_opposite = self.opposite(child_robots[i].heading, child_robots[j].heading)
                        constraint_list = self.is_colliding(r_i_area, r_j_area, orig_pos[i], is_opposite)
                        if len(constraint_list) > 0:
                            conflict_found = True
                            if child_CBS.robots[j].constraints is None:
                                child_CBS.robots[j].constraints = {local_time : constraint_list}
                            elif local_time in child_CBS.robots[j].constraints:
                                child_CBS.robots[j].constraints[local_time].extend(constraint_list)
                            else:
                                child_CBS.robots[j].constraints[local_time] = constraint_list
                if conflict_found:
                    child_CBS.generate_individual_plans()
                    succesors.append(child_CBS)
                    break_outer = True
            local_time += 1
        if not succesors:
            self.no_conflicts = True
        return succesors

    # Return a list of list of actions
    # actions[i] points to a list of actions for robot i. actions[i][0] is the last action to be performed
    # actions[i][len(actions[i]) - 1] is the first (next) action to be performed.
    def get_plan(self, plans):
        for i, plan in enumerate(self.plans):
            plans.append([])
            for action in plan:
                plans[i].append(action)

    # Return True if this is the goal state
    def is_goal(self):
        # check conflict
        # if conflict add it to conflict to all agent except the first one
        # otherwise return true
        return self.no_conflicts

    def __eq__(self, other):
        # check if all paths are same
        for x in range(len(self.robots)):
            if self.robots[x].plan != other.robots[x].plan: # or self.robots[x].constraints != other.robots[x].constraints:
                return False
        return True

    def __hash__(self):
        ans = 0
        for robot in self.robots:
            ans += hash(robot) + hash(robot.plan.__str__()) # + hash(robot.constraints.__str__())
        return ans
        pass

    def __lt__(self, other):
         return self.g + self.h < other.g + other.h

    def __ge__(self, other):
        return not self < other

    def __str__(self):
        ans = ''
        for robot in self.robots:
            ans += "Robot[%d]-(%d,%d,%s,%d) " %(robot.index,robot.position_x, robot.position_y, robot.heading, robot.velocity)
        return ans
