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

    #todo: can we use functions in CollisionDetection.py instead of below function?
    def is_colliding(self, covered_area, check_area): #TODO: write colliding function
        for pos1 in covered_area:
            for pos2 in check_area:
                if(pos1[0] == pos2[0] and pos1[1] == pos2[1]):
                    return True
        return False

    # iterate over all robots and call "underlying algorithm" - single agent with conflict set - edit Robot.py to include a conflict set
    # then do a "get path" for each agent
    # collect the results into apt structure!! - TODO figure out what this apt structure is
    # also update g value based on paths at end for loop
    def generate_individual_pans(self):
        temp_plans = []
        max_plan_len = -1
        for r in self.robots:
            r.plan_path()
            self.g += len(r.plan) #assuming path length as total number of actions
            max_plan_len = max(max_plan_len, len(r.plan))
        for r in self.robots:
            if len(r.plan) < max_plan_len:
                b = [Action.no_op for i in range(max_plan_len - len(r.plan))]
                r.plan = b + r.plan
            temp_plans.append(r.plan)

        self.plans = temp_plans

    def expand(self):
        print("expand called")
        if(self.plans == None):
            self.generate_individual_pans() #root node don't have plans initially'
        succesors = []
        local_time = 1
        plan_length = len(self.robots[0].plan)
        for time in range(plan_length-1, -1, -1): #todo:check 0 or -1 in range
            grid_occupancy_dict = []
            for idx, r in enumerate(self.robots):
                try:
                    occupies = r.step_simulation(r.plan[time])
                    #print("  after performing action  ", r.plan[i])
                    #r.print_details()
                    #print("robot  ", idx , "occupies at time", local_time)
                    #for location in occupies[0]:
                    #    print("X: ", location[0], " Y: ", location[1], "  *** ", end="")
                    #print("   ")
                except ValueError:
                    continue  # Ignore illegal actions due to existing constraints and boundary conditions

                grid_occupancy_dict.append(occupies[0])

            for i,robot_occupied_area in enumerate(grid_occupancy_dict):
                conflict_found = False
                child_CBS =  copy.deepcopy(self)
                child_CBS.p = self
                for j, cross_check_area in enumerate(grid_occupancy_dict): #brute force n*n check for collisions, should improve it
                    if i == j:
                        continue
                    else:
                        collision = self.is_colliding(robot_occupied_area, cross_check_area)
                        if(collision):
                            # print("*******************************")
                            # print("collision occurs with robot ", i, "with robot", j)
                            # print("before adding constraint")
                            # for r in child_CBS.robots:
                            #     r.print_details()
                            conflict_found = True #robot has constraints dictionary with time as key
                            print("adding constraint to robot", j)
                            if local_time in child_CBS.robots[j].constraints:
                                child_CBS.robots[j].constraints[local_time].extend(robot_occupied_area)
                            else:
                                child_CBS.robots[j].constraints = {local_time: robot_occupied_area}
                            child_CBS.robots[j].plan = [Action.no_op]  ##invalidate the existing plan
                            print("after adding constraint")
                            for r in child_CBS.robots:
                                r.print_details()
                            print("*******************************")
                if conflict_found:
                    child_CBS.generate_individual_pans()
                    succesors.append(child_CBS)
            local_time += 1
        print("returning succesors ", len(succesors))
        if not succesors:
            self.no_conflicts = True
        return succesors

    # Return a list of list of actions
    # actions[i] points to a list of actions for robot i. actions[i][0] is the last action to be performed
    # actions[i][len(actions[i]) - 1] is the first (next) action to be performed.
    def get_plan(self, plans):
        for x in self.plans:
            plans.append(x)

    # Return True if this is the goal state
    def is_goal(self):
        # check conflict
        # if conflict add it to conflict to all agent except the first one
        # otherwise return true
        return self.no_conflicts

    def __eq__(self, other):
        # check if all paths are same
        pass

    def __hash__(self):
        ans = 0
        for r in self.robots:
            ans += hash(r)
        return ans
        pass

    def __lt__(self, other):
         return self.g + self.h < other.g + other.h

    def __ge__(self, other):
        return not self < other

    def __str__(self):
        # not sure if this is correct...
        ans = ''
        for r in self.robots:
            ans += "Robot[%d]-(%d,%d,%s,%d) " %(r.index,r.position_x, r.position_y, r.heading, r.velocity)
        return ans
