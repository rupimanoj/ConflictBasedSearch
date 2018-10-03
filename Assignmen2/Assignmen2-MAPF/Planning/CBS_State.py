from Utils.constants import Action, NAV_ACTIONS
import Utils.constants as C


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

    # iterate over all robots and call "underlying algorithm" - single agent with conflict set - edit Robot.py to include a conflict set
    # then do a "get path" for each agent
    # collect the results into apt structure!! - TODO figure out what this apt structure is
    # also update g value based on paths at end for loop
    def expand(self):
        pass

    # Return a list of list of actions
    # actions[i] points to a list of actions for robot i. actions[i][0] is the last action to be performed
    # actions[i][len(actions[i]) - 1] is the first (next) action to be performed.
    def get_plan(self, plans):
        for x in range(len(self.robots)):
            plans.append([])
        self.get_plan_r(plans)

    def get_plan_r(self, plans):
        if self.p is not None:
            for x in range(len(self.actions)):
                plans[x].append(self.actions[x])
            self.p.get_plan_r(plans)
        return

    # Return True if this is the goal state
    def is_goal(self):
        # check conflict
        # if conflict add it to conflict to all agent except the first one
        # otherwise return true
        return True

    def __eq__(self, other):
        # check if all paths are same

    def __hash__(self):
        # hash paths?

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
