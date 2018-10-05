from Utils.constants import Action, NAV_ACTIONS


class SingleAgentState:

    def __init__(self, p, robot, g, action, time_stamp=0):
        # conflicts is a set of time steps starting at 0, where there is a conflict
        # add conflict will be done in is_goal, and conflict is found
        # if conflict is found, add the conflict to all robots except the first one
        self.robot = robot
        self.p = p
        self.g = g
        self.h = abs(robot.position_x - robot.goal_x) + abs(robot.position_y - robot.goal_y)# TODO: Your job - Set a better heuristic value
        self.action = action
        # self.conflicts = conflicts
        # self.current_depth = current_depth
        self.time_stamp = time_stamp

    def expand(self):
        successors = []
        for action in Action:
            if action.value not in NAV_ACTIONS:
                continue  # Lift, drop, and process are not part of the path planning
            child_robot = self.robot.copy()
            child_robot.plan = [action, action]
            try:
                occupies = child_robot.step()
            except ValueError:
                continue  # Ignore illegal actions
            constraints_obeyed = True
            for pos in occupies[0]: #before adding to succesor states check if they are in conflicting regions
                if self.time_stamp+1 in self.robot.constraints and pos in self.robot.constraints[self.time_stamp+1]:
                    constraints_obeyed = False
                    print("not respecting constraints")
                    break
            if child_robot.warehouse.are_open_cells(occupies[0], self.robot.carry) and constraints_obeyed:
                successors.append(SingleAgentState(self, child_robot, self.g + 1, action, self.time_stamp+1))
                #TODO:need to recheck if this time_stamp is correct
        return successors

    def get_plan(self, plan):
        if self.p is not None:
            plan.append(self.action)
            self.p.get_plan(plan)
        return

    def is_goal(self):
        return self.robot.at_goal()

    def __eq__(self, other):
        return self.robot == other.robot

    def __hash__(self):
        return hash(self.robot)

    def __lt__(self, other):
         return self.g + self.h < other.g + other.h

    def __ge__(self, other):
        return not self < other

    def __le__(self, other):
        return self.g + self.h <= other.g + other.h

    def __str__(self):
        return "%d,%d" %(self.robot.position_x, self.robot.position_y)