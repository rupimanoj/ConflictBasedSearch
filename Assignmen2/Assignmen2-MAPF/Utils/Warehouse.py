from Utils.CollisionDetection import CollisionDetection, CollisionDetectionEdge
from Planning.MultiagentState import MultiagentState
from Planning.CBS_State import CBS_State
from Planning.BestFirstSearch import BestFirstSearch
from Utils.Agents.Robot import RobotNoCarry
from Utils.constants import Action
import random

class Warehouse:

    def __init__(self, map):
        self.map = map
        self.agents = map.get_agents(self)
        self.reservations_robots = CollisionDetection(self.map.height, self.map.width)
        self.reservations_pods = CollisionDetection(self.map.height, self.map.width)

    def display(self, screen, offset_x, offset_y):
        self.map.display(screen, offset_x, offset_y)
        for agent_type in self.agents:
            for agent in agent_type:
                agent.display(screen, offset_x, offset_y)

    def get_pod(self, at_x, at_y):
        for pod in self.agents[1]:
            if pod.position_x == at_x and pod.position_y == at_y:
                return pod
        return None

    def get_station(self, at_x, at_y):
        for station in self.agents[2]:
            if station.position_x == at_x and station.position_y == at_y:
                return station
        return None

    def step(self):
        self.step(self.agents[0])

    def step(self):
        cell_record_robots = CollisionDetection(self.map.height, self.map.width)
        edge_record_robots = CollisionDetectionEdge(self.map.height, self.map.width)
        cell_record_pods = CollisionDetection(self.map.height, self.map.width)
        for r in self.agents[0]:
            occupied = r.step()
            for coordinate in occupied[0]:
                cell_record_robots.add(coordinate[0],coordinate[1])
                if r.carry is not None:
                    cell_record_pods.add(coordinate[0],coordinate[1])
            for edge in occupied[1]:
                edge_record_robots.add(edge[0], edge[1], edge[2])
        for p in self.agents[1]:
            if p.assigned is None:
                cell_record_pods.add(p.position_x, p.position_y)
            elif p.assigned.carry is None:
                cell_record_pods.add(p.position_x, p.position_y)

    def are_open_cells(self,cells, mounted):
        for cell in cells:
            if cell[0] < 0 or cell[1] < 0:
                return False
            if len(self.map.grid) <= cell[1]:
                return False
            if len(self.map.grid[cell[1]]) <= cell[0]:
                return False
            if mounted:
                if mounted.original_x == cell[0] and mounted.original_y == cell[1]:
                    return True
                if self.map.grid[cell[1]][cell[0]] != '.':
                    return False
            elif self.map.grid[cell[1]][cell[0]] != 'P' and self.map.grid[cell[1]][cell[0]] != '.':
                return False
        return True

    def random_unassigned_pod(self):
        pod = random.choice(self.agents[1])
        while pod.assigned: # TODO check that unassigned pods exist
            pod = random.choice(self.agents[1])
        return pod

    def random_station(self):
        return random.choice(self.agents[2])

    def is_unique_goal(self,x,y):
        for r in self.agents[0]:
            if r.goal_x == x and r.goal_y == y:
                return False
        return True

    def multiagent_plan(self, planner):
        if planner == 'maA*':
            root = MultiagentState(None,self.agents[0],0,None)
        elif planner == 'CBS':
            root = CBS_State(self.agents[0])
        ma_plan = BestFirstSearch.plan(root)
        for x in range(len(ma_plan)):
            self.agents[0][x].plan = [Action.process,Action.process,Action.process,Action.process] + ma_plan[x]

    def assign_non_carying_robots(self):
        for rnc in self.agents[0]:
            if type(rnc) is RobotNoCarry:
                rnc.goal_x, rnc.goal_y = -1, -1
        for rnc in self.agents[0]:
            if type(rnc) is RobotNoCarry:
                rnc.assign_goal()
