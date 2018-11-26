import maya.cmds as cmds
import datetime
import math

# -----SETUP----------------

# Cleanup
if cmds.ls('ground'):
    cmds.delete('ground')

particleList = cmds.ls('myParticle*')

if len(particleList) > 0:
    cmds.delete(particleList)

# Create the ground plane
cmds.polyCube(sx=10, sy=15, sz=5, w=5, d=5, h=0.01, name='ground')

noParticles = 0
h = 0.1

# Create  a cube of particles
for i in range(5):
    for j in range(5):
        for k in range(5):
            particle = cmds.polySphere(n='myParticle#', sx=2, sy=2, r=0.05)
            cmds.move(i * h, 0.2 + j * h, k * h, particle)
            noParticles += 1

#particleList = cmds.ls('myParticle*')
print str(noParticles) + " particles created"

# -------SETUP DONE-----------------

particleList = cmds.ls('myParticle*', type='transform', showType=False)

# Select all particles
# cmds.select('myParticle*')
# print 'Particles selected'
# print str(len(particleList)) + " particles selected"

# Clear keyframes
cmds.cutKey()

class Particle:
    mass = 0.01

    def __init__(self):
        self.name = "unset"
        self.position = [0, 0, 0]
        self.density = 1
        self.pressure = 1
        self.acceleration = [0, 0, 0]
        self.velocity = [0, 0, 0]

# --------------- Functions -----------------

# Get all neighbors within the smoothing radius


def getNeighbors(particle, smoothingLength):

    neighbours = []

    index = 0

    for otherParticle in particles:

        deltadist = [0, 0, 0]

        deltadist[0] = particle.position[0] - otherParticle.position[0]
        deltadist[1] = particle.position[1] - otherParticle.position[1]
        deltadist[2] = particle.position[2] - otherParticle.position[2]

        distance = math.sqrt(math.pow( deltadist[0], 2) + math.pow(deltadist[1], 2) + math.pow(deltadist[2], 2))

        # if distance <= smoothingLength, put in array
        if distance <= smoothingLength and distance > 0:
            neighbours.append(particles[index])

        index += 1

    return neighbours


def applyPoly6Kernel(particleDistance):

    output = ((315/(64 * math.pi * math.pow(h, 9))) *
              math.pow((math.pow(h, 2) -
                        math.pow(h, 2)), 3)
              )

    return output

# Calculate the density based on all neighbors within the smoothingRadius
def calculateDensity(particle,  particleMass, neighbors):

    # print(neighbors)

    density = 40
    for neighbor in neighbors:
        distanceToNeighbor = calculateDistance(particle, neighbor)
        density += particleMass * applyPoly6Kernel(distanceToNeighbor)

    if density < 40:
        density = 40

    return density


def calculatePressure(particleDensity):
    p0 = 40
    K = 40
    pressure = K * (particleDensity - p0)

    # print(pressure)
    return pressure


def calculateDistance(p1, p2):

    distanceVector = getDistanceVector(p1, p2)

    distance = math.sqrt(math.pow(
        distanceVector[0], 2) + math.pow(distanceVector[1], 2) + math.pow(distanceVector[2], 2))

    # print(distance)

    return distance


# Input as strings expected
def getDistanceVector(p1, p2):

    distanceVector = [
        p1.position[0] - p2.position[0],
        p1.position[1] - p2.position[1],
        p1.position[2] - p2.position[2]]

    # print(distanceVector)

    return distanceVector


def calculateSpikyKernel(smoothingRadius, particle, neighbor):

    distanceVec = getDistanceVector(particle, neighbor)

    gX = (45 / (math.pi * math.pow(smoothingRadius, 6))) * \
        math.pow((smoothingRadius - distanceVec[0]), 2)
    gY = (45 / (math.pi * math.pow(smoothingRadius, 6))) * \
        math.pow((smoothingRadius - distanceVec[1]), 2)
    gZ = (45 / (math.pi * math.pow(smoothingRadius, 6))) * \
        math.pow((smoothingRadius - distanceVec[2]), 2)

    gradient = [gX, gY, gZ]
    return gradient


def calculateAcceleration(neighbors, particle, particleMass, particlePressure, particleDensity):
    acceleration = [0, 0, 0]

    # print(particlePressure)
    # print(particle.pressure)

    for neighbor in neighbors:
        spikyGradient = calculateSpikyKernel(h, particle, neighbor)

        vector = getDistanceVector(neighbor, particle)
        magnitude  = math.sqrt( math.pow(vector[0], 2)+ math.pow(vector[1], 2)+ math.pow(vector[2], 2))
        normalizedVector  = ( vector[0] / magnitude ,  vector[1] / magnitude,  vector[2] / magnitude )

        acceleration[0] += -(particleMass/particleMass) * ((neighbor.pressure +
                                                           particle.pressure) / (2*particleDensity*neighbor.density)) * spikyGradient[0] * normalizedVector[0]
        acceleration[1] += -(particleMass/particleMass) * ((neighbor.pressure +
                                                           particle.pressure) / (2*particleDensity*neighbor.density)) * spikyGradient[1] * normalizedVector[1]
        acceleration[2] += -(particleMass/particleMass) * ((neighbor.pressure +
                                                           particle.pressure) / (2*particleDensity*neighbor.density)) * spikyGradient[2] * normalizedVector[2]

    print(acceleration)
    return acceleration

# Make the particles collide with surfaces
def checkBoundaries(position, particle):
    if position[1] < 0.0:
        position[1] = 0.0
        particle.velocity[1] = 0.0


# Animation
startTime = cmds.playbackOptions(query=True, minTime=True)
endTime = cmds.playbackOptions(query=True, maxTime=True)
currentTime = startTime

noParticles = len(particleList)
spawnHeight = 3
gravity = [0, -9.82, 0]

# Create a progress bar
window = cmds.window()
cmds.columnLayout()
cmds.text(label='Simulation Progress')
progressControl = cmds.progressBar(
    status='"Simulation Progress', maxValue=noParticles*endTime, width=300)

# Start timer
start = cmds.timerX()
cmds.showWindow(window)

index = 0
particles = []

# Initialize particle array
for i in particleList:
    p = Particle()
    p.particleIndex = index
    p.name = particleList[index]
    particles.append(p)
    index += 1


def initialSetup():
    # Set initial positions
    loopindex = 1
    for p in particles:
        p.position = [
            cmds.getAttr('myParticle' + str(loopindex) + '.translateX'),
            cmds.getAttr('myParticle' + str(loopindex) + '.translateY'),
            cmds.getAttr('myParticle' + str(loopindex) + '.translateZ')]
        # print(p.position)
        loopindex += 1


initialSetup()


# Main simulaiton loop
while currentTime < endTime:

    # Loop all particles
    for p in range(noParticles):

        particleIndex = p + 1

        currentParticle = particles[p]
        # currentParticle.name = particleList[p]

        # print(currentParticle.position)

        # --------- Calculate! -------------

        particleNeighbors = getNeighbors(currentParticle, h)

        # print(particleNeighbors)

        currentParticle.density = calculateDensity(
            currentParticle, currentParticle.mass, particleNeighbors)

        # print(currentParticle.density)

        currentParticle.pressure = calculatePressure(currentParticle.density)

        # print(currentParticle.pressure)

        currentParticle.acceleration = calculateAcceleration(
            particleNeighbors, currentParticle, currentParticle.mass, currentParticle.pressure, currentParticle.density)

        # print(currentParticle.acceleration)

        totalAcceleration = currentParticle.acceleration
        totalAcceleration[1] += gravity[1]

        # print(totalAcceleration)

        # Apply gravity
        velocity = [i * (currentTime / 100) for i in totalAcceleration]
        currentParticle.velocity = velocity

        # print(velocity)

        nextPosition = currentParticle.position
        # print(nextPosition)
        nextPosition[0] = currentParticle.position[0] + \
            (velocity[0] * currentTime * mass)
        nextPosition[1] = currentParticle.position[1] + \
            (velocity[1] * currentTime * mass)
        nextPosition[2] = currentParticle.position[2] + \
            (velocity[2] * currentTime * mass)

        checkBoundaries(nextPosition, currentParticle)

        currentParticle.position = nextPosition

        
        # Set keyframes for animating the particles
        cmds.setKeyframe(currentParticle.name, attribute='translateX',
                         v=nextPosition[0], t=currentTime)
        cmds.setKeyframe(currentParticle.name, attribute='translateY',
                         v=nextPosition[1], t=currentTime)
        cmds.setKeyframe(currentParticle.name, attribute='translateZ',
                         v=nextPosition[2], t=currentTime)

        # Update progress bar
        cmds.progressBar(progressControl, edit=True, step=1)

    currentTime += 1

# Stop progress bar, we are done
cmds.progressBar(progressControl, edit=True, endProgress=True)
cmds.deleteUI(window, window=True)

# Display elapsed time
totalTime = cmds.timerX(startTime=start)
window = cmds.window(widthHeight=(400, 60))
cmds.columnLayout(adjustableColumn=True)
simulationTime = str(datetime.timedelta(seconds=totalTime)).split(".")[0]
cmds.text(label='Simulated in ' + simulationTime, align='center')
cmds.showWindow(window)

print 'Keyframes set'
