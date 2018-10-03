from Utils.constants import Action, NAV_ACTIONS


class SingleAgentState:

    def __init__(self, p, robot, g, action, conflicts=None, current_depth=None, consider_conflicts=False):
        #conflicts is a set of time steps starting at 0, where there is a conflict
        self.robot = robot
        self.p = p
        self.g = g
        self.h = abs(robot.position_x - robot.goal_x) + abs(robot.position_y - robot.goal_y)# TODO: Your job - Set a better heuristic value
        self.action = action

    def expand(self):
        successors = []
        for action in Action:
            if action.value not in NAV_ACTIONS:
                continue  # Lift, drop, and process are not part of the path planning
            child_robot = self.robot.copy()
            child_robot.plan = [action, action]
            # find and add contrain here
            if consider_conflicts and current_depth in conflict:
                continue
            try:
                occupies = child_robot.step()
            except ValueError:
                continue  # Ignore illegal actions
            if child_robot.warehouse.are_open_cells(occupies[0], self.robot.carry):
                successors.append(SingleAgentState(self, child_robot, self.g + 1, action))
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