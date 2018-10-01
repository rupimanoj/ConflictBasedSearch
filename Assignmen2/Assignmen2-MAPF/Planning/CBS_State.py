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

    # Generate all valid successors
    def expand(self):
        # Todo: Your job
        raise NotImplementedError

    # Return a list of list of actions
    # actions[i] points to a list of actions for robot i. actions[i][0] is the last action to be performed
    # actions[i][len(actions[i]) - 1] is the first (next) action to be performed.
    def get_plan(self, plans):
        # Todo: Your job
        raise NotImplementedError

    # Return True if this is the goal state
    def is_goal(self):
         # Todo: Your job
        raise NotImplementedError


    # The following comparators are needed for the open and closed lists (used by Best-First Search)
    def __eq__(self, other):
        # Todo: Your job
        raise NotImplementedError

    def __lt__(self, other):
         # Todo: Your job
        raise NotImplementedError

    def __ge__(self, other):
        # Todo: Your job
        raise NotImplementedError

    # Hash function needed for the closed list. For efficiency try to produce unique values for unique states
    def __hash__(self):
        # Todo: Your job
        raise NotImplementedError

    # Name the state for easy debugging
    def __str__(self):
        # Todo: Your job
        raise NotImplementedError