import create2
import lab10_map
import math
from particle_filter import ParticleFilter as PF
from odometry import Odometry
import numpy as np
import pid_controller

class AStar:
    def __init__(self, m):
        self.m = m
        
    def adjacent(self, pos):
        """
        Computes the accessible neighbors assuming a 4-grid
        for a given map and position
        :param m: Matrix-encoded environment (0 free space; 1 obstacle)
        :param pos: Position (row, column)
        :return: List of neighbors [(row1, column1), (row2, column2), ...]
        """
        result = []
        if pos[0] > 0 and self.m[pos[0] - 1, pos[1]] == 0:
            result.append((pos[0] - 1, pos[1]))
        if pos[0] < self.m.shape[0] - 1 and self.m[pos[0] + 1, pos[1]] == 0:
            result.append((pos[0] + 1, pos[1]))
        if pos[1] > 0 and self.m[pos[0], pos[1] - 1] == 0:
            result.append((pos[0], pos[1] - 1))
        if pos[1] < self.m.shape[1] - 1 and self.m[pos[0], pos[1] + 1] == 0:
            result.append((pos[0], pos[1] + 1))
        return result


    def heuristic(self, a, b):
        """
        Computes the Manhatten distance between two points
        :param a: start position (row, column)
        :param b: goal position (row, column)
        :return: manhatten distance between a and b
        """
        return math.fabs(a[0] - b[0]) + math.fabs(a[1] - b[1])


    def valid(self, pos):
        """
        Returns if the given position is free space or not.
        :param m: Matrix-encoded environment (0 free space; 1 obstacle)
        :param pos: position to check (row, column)
        :return: True, if position is in free space
        """
        return self.m.shape[0] > pos[0] >= 0 and self.m.shape[1] > pos[1] >= 0 and self.m[pos] == 0


    def a_star(self, start_pos, goal_pos):
        """
        Computes the shortest path from start_pos to goal_pos using A*
        :param m: Matrix-encoded environment (0 free space; 1 obstacle)
        :param start_pos: start position (row, column)
        :param goal_pos: goal position (row, column)
        :return: Path (list of row/column tuples)
        """
        if not self.valid(start_pos) or not self.valid(goal_pos):
            return None

        open_set = set()
        closed_set = set()
        f_score = dict()
        g_score = dict()
        parent = dict()

        f_score[start_pos] = self.heuristic(start_pos, goal_pos)
        g_score[start_pos] = 0

        open_set.add(start_pos)
        n = None
        while len(open_set) > 0:
            # find item with lowest f-score
            n = min(open_set, key=lambda x: f_score[x])
            if n == goal_pos:
                break
            open_set.remove(n)
            closed_set.add(n)
            for a in self.adjacent(n):
                if a in closed_set:
                    continue
                tentative_g_score = g_score[n] + 1
                if a not in open_set or tentative_g_score < g_score[a]:
                    parent[a] = n
                    g_score[a] = tentative_g_score
                    f_score[a] = g_score[a] + self.heuristic(a, goal_pos)
                    if a not in open_set:
                        open_set.add(a)

        if n != goal_pos:
            return None

        # reconstruct path:
        p = goal_pos
        path = [p]
        while p != start_pos:
            p = parent[p]
            path.append(p)
        path.reverse()
        return path

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.map = np.loadtxt("map14.txt", delimiter=",")
        self.odometry = Odometry()
        self.search = AStar(self.map)
        self.location = (0,0)
        self.orientation = "east"
        self.pidTheta = pid_controller.PIDController(200, 25, 5, [-1, 1], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(100, 15, 5, [-10, 10], [-200, 200], is_angle=False)
        self.base_speed = 50
        
    def goToGoal(self, goal, state):
        goal_y = goal[1] / 30
        goal_x = goal[0] / 30
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
        theta = math.atan2(math.sin(self.odometry.x), math.cos(self.odometry.y))
        output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

        # improved version 2: fuse with velocity controller
        self.distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
        output_distance = self.pidDistance.update(0, self.distance, self.time.time())
        self.create.drive_direct(int(output_theta + output_distance)+self.base_speed, int(-output_theta + output_distance)+self.base_speed)
        return output_distance
        
    def shouldIContinueAlongMyPath(self):
        #First scan ahead
        distance = self.sonar.get_distance()
        if distance < 0.5:
            #Then add the obstacle to the map
            if self.orientation == "east":
                self.map[(self.location[0], self.location[1]+1)] = 1
            elif self.orientation == "west":
                self.map[(self.location[0], self.location[1]-1)] = 1
            elif self.orientation == "north":
                self.map[(self.location[0]-1, self.location[1])] = 1
            else:
                self.map[(self.location[0]+1, self.location[1])] = 1
            return False
        return True
    
    def moveOneSquare(self, goal):
        startX = self.odometry.x
        startY = self.odometry.y
        while True:
            print("Moving")
            #Get state and update the odometry
            self.state = self.create.update()
            while not self.state:
                self.state = self.create.update()
            if abs(self.goToGoal(goal, self.state)) < 10:
                break
        if self.orientation == "east":
            self.location = (self.location[0], self.location[1]+1)
        elif self.orientation == "west":
            self.location = (self.location[0], self.location[1]-1)
        elif self.orientation == "south":
            self.location = (self.location[0]+1, self.location[1])
        else:
            self.location = (self.location[0]-1, self.location[1]+1)
    
    def turnCreate(self, goalTheta, state, pos):
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        output_theta = self.pidTheta.update(self.odometry.theta, goalTheta, self.time.time())
        # improved version 2: fuse with velocity controller
        if pos:
            self.create.drive_direct(int(output_theta), int(-output_theta))
        else:
            self.create.drive_direct(-int(output_theta), int(output_theta))

        return output_theta
    
    def turnTowardsPoint(self, nextPoint):
        startTheta = self.odometry.theta
        if nextPoint[0] < self.location[0]:
            print("Turning north")
            #Then turn north
            goalTheta = math.pi / 2
            while True:
                #Get state
                self.state = self.create.update()
                while not self.state:
                    self.state = self.create.update()
                if abs(self.turnCreate(goalTheta, self.state, True)) < 10:
                    break
            print("Done turning")
            self.orientation = "north"
        elif nextPoint[0] > self.location[0]:
            print("Turning south")
            #Then turn south
            goalTheta = 3 * math.pi / 2
            while True:
                #Get state
                self.state = self.create.update()
                while not self.state:
                    self.state = self.create.update()
                if abs(self.turnCreate(goalTheta, self.state, True)) < 10:
                    break
            print("Done turning")
            self.orientation = "south"
        elif nextPoint[1] < self.location[1]:
            print("Turning west")
            #Then turn west
            goalTheta = math.pi
            while True:
                #Get state
                self.state = self.create.update()
                while not self.state:
                    self.state = self.create.update()
                if abs(self.turnCreate(goalTheta, self.state, True)) < 10:
                    break
            print("Done turning")
            self.orientation = "west"
        else:
            print("Turning east")
            #Then turn east
            goalTheta = 0
            while True:
                #Get state
                self.state = self.create.update()
                while not self.state:
                    self.state = self.create.update()
                if abs(self.turnCreate(goalTheta, self.state, True)) < 10:
                    break
            print("Done turning")
            self.orientation = "east"
        
    def goOnPath(self, path):
        for x, y in path:
            if (x, y) == self.location:
                pass
            #Turn towards the point
            print("Turning towards Point ", (x, y))
            self.turnTowardsPoint((x,y))
            #If point is now obstacle, go in a new path
            if self.shouldIContinueAlongMyPath():
                #Move towards point
                print("No obstacle. Moving toward point")
                self.moveOneSquare((x,y))
            else:
                return False
        return True
        
    def run(self):
        self.create.start()
        self.create.safe()
        
        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        
        self.state = self.create.update()
        while not self.state:
            self.state = self.create.update()
        self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
        
        self.location = (9,1)#(self.odometry.x, self.odometry.y)
        goal_pos = (1,9)
        
        path = self.search.a_star(self.location, goal_pos)
        
        print("Path is: ",path)
        
        #Now go on that path
        while not self.goOnPath(path):
            path = self.search.a_star(self.location, goal_pos)
            path.remove(self.location)
            print("New Path is: ", path)
        print("Done! I am at goal.")
        