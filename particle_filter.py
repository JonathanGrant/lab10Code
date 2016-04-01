import math
import numpy as np
import scipy.stats

class Particle:
    def __init__(self, x, y, theta, variances):
        #Fill in Location
        self.x = x
        self.y = y
        self.theta = theta
        self.varX = variances[0]
        self.varY = variances[1]
        self.varTheta = variances[2]

class ParticleFilter:
    def __init__(self, boundsX, boundsY, boundsTheta, variances, theMap):
        """Constructor.
        """
        #Here I should create a whole bunch of particles, right?
        self.theMap = theMap
        self.particles = self.createUniformParticles(boundsX, boundsY, boundsTheta, variances)
        print("Particles are done being created.")
        
    def removeOutOfBoundsParticles(self):
        toDelete = []
        for particle in self.particles:
            if not particle.x in range(self.theMap.bottom_left[0],self.theMap.top_right[0]):
                #delete particle and add new one randomly
                toDelete.append(particle)
            elif not particle.y in range(self.theMap.bottom_left[1], self.theMap.top_right[1]):
                toDelete.append(particle)
        for particle in toDelete:
            self.deleteParticleAndAddNewOne(particle)
                
    def deleteParticleAndAddNewOne(self, particle):
        self.particles.remove(particle)
        self.particles.append(self.createUniformParticle([self.theMap.bottom_left[0], self.theMap.top_right[0]],[self.theMap.bottom_left[1], self.theMap.top_right[1]], [0, 2*math.pi], [0.1,0.1,0.3]))
    
    def getParticles(self):
        return self.particles
    
    def MoveForward(self):
        self.Movement(0, 0.1, 0.052458669781640876, 0.01)
    
    def TurnLeft(self):
        self.Movement(0.4464567640990713, 0.1, 0, 0.01)
            
    def TurnRight(self):
        self.Movement(-0.4464567640990713, 0.1, 0, 0.01)
    
    def Movement(self, theta, varTheta, dist, varDist):
        print("Some shit moved brah")
        #Move particles
        for particle in self.particles:
            thetaPrime = theta + self.doNormal(0, varTheta)
            distPrime = dist + self.doNormal(0, varDist)
            particle.theta += thetaPrime
            particle.x += distPrime * math.cos(particle.theta)
            particle.y += distPrime * math.sin(particle.theta)
        print("Particles are done moving.")
        self.removeOutOfBoundsParticles()
    
    def Sensing(self, reading):
        prob = []
        #Find probability for each particle
        for particle in self.particles:
            dist = self.theMap.closest_distance((particle.x, particle.y), particle.theta)
            if dist:
                prob.append(scipy.stats.norm(reading, 0.1).pdf(dist))
        #Based on that, resample from that data
        newParticles = []
        for i in range(0,100):
            party = np.random.choice(self.particles,1,prob)[0]
            newParticles.append(self.createUniformParticle([party.x-0.1,party.x+0.1], [party.y-0.1,party.y+0.1], [party.theta-0.3,party.theta+0.3],[party.varX,party.varY,party.varTheta]))
        self.particles = newParticles
    
    def Estimation(self):
        pass
    
    def doNormal(self, mean, variance):
        return np.random.normal(mean, variance)
    
    def doUniform(self, minnie, maxie):
        return np.random.uniform(minnie,maxie)
    
    def createUniformParticle(self, boundsX, boundsY, boundsTheta, variances):
        return Particle(self.doUniform(boundsX[0], boundsX[1]), self.doUniform(boundsY[0], boundsY[1]), self.doUniform(boundsTheta[0], boundsTheta[1]), variances)
    
    def createUniformParticles(self, boundsX, boundsY, boundsTheta, variances):
        Particles = []
        for i in range(0, 100):
            Particles.append(Particle(self.doUniform(boundsX[0], boundsX[1]), self.doUniform(boundsY[0], boundsY[1]), self.doUniform(boundsTheta[0], boundsTheta[1]), variances))
        return Particles
    
    def moveNormalParticle(self, particle, dX, dY, dTheta):
        return [self.doNormal(dX, particle.varX), self.doNormal(dY, particle.varY), self.doNormal(dTheta, particle.varTheta)]
