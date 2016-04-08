import create2
import lab10_map
import math
from particle_filter import ParticleFilter as PF
from odometry import Odometry

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
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create("192.168.1.232")
        self.map = lab10_map.Map("lab10.map")
        self.odometry = Odometry()
        
    def showParticles(self, particles):
        self.virtual_create.set_point_cloud(particles)
        
    def visualizePose(self,x,y,z,theta):
        self.virtual_create.set_pose((x, y, z), theta)

    def run(self):
        self.create.start()
        self.create.safe()
        
        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        #self.visualizePose(0.5, 0.5, 0.1, math.pi)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        #data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        #self.showParticles(data)

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        #print(self.map.closest_distance((0.5,0.5), math.pi))
        
        self.state = self.create.update()
        while not self.state:
            self.state = self.create.update()
        self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
        
        #This is an example on how to use PF
        variances = [0.01,0.01,0.3]
        self.pf = PF([self.map.bottom_left[0], self.map.top_right[0]],[self.map.bottom_left[1], self.map.top_right[1]],[0,2*math.pi], variances, self.map)
        print("Drawing particles")
        for particle in self.pf.getParticles():
            self.showParticles([particle.x,particle.y,0.1,particle.theta])
        
        # This is an example on how to detect that a button was pressed in V-REP
        while True:
            self.create.drive_direct(0,0)
            
            self.state = self.create.update()
            if self.state:
                self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
                prevX = self.odometry.x
                prevY = self.odometry.y
                prevTheta = self.odometry.theta

                b = self.virtual_create.get_last_button()
                if b == self.virtual_create.Button.MoveForward:
                    print("Forward pressed!")
                    self.create.drive_direct(50,50)
                    self.time.sleep(1)
                    self.create.drive_direct(0,0)
                    self.state = self.create.update()
                    self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
                    self.pf.MoveForward()
                    print("Drawing particles")
                    for particle in self.pf.getParticles():
                        self.showParticles([particle.x,particle.y,0.1,particle.theta])
                elif b == self.virtual_create.Button.TurnLeft:
                    print("Turn Left pressed!")
                    self.create.drive_direct(50,-50)
                    self.time.sleep(1)
                    self.create.drive_direct(0,0)
                    self.state = self.create.update()
                    self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
                    self.pf.TurnLeft()
                    print("Drawing particles")
                    for particle in self.pf.getParticles():
                        self.showParticles([particle.x,particle.y,0.1,particle.theta])
                elif b == self.virtual_create.Button.TurnRight:
                    print("Turn Right pressed!")
                    self.create.drive_direct(-50,50)
                    self.time.sleep(1)
                    self.create.drive_direct(0,0)
                    self.state = self.create.update()
                    self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
                    self.pf.TurnRight()
                    print("Drawing particles")
                    for particle in self.pf.getParticles():
                        self.showParticles([particle.x,particle.y,0.1,particle.theta])
                elif b == self.virtual_create.Button.Sense:
                    print("Sense pressed!")
                    dist = self.sonar.get_distance()
                    if dist:
                        self.pf.Sensing(dist)
                    print("Drawing particles")
                    for particle in self.pf.getParticles():
                        self.showParticles([particle.x,particle.y,0.1,particle.theta])

                self.time.sleep(0.01)
            