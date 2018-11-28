import maya.cmds as cmds
import datetime
import math

# -----SETUP----------------

# Cleanup
if cmds.ls('ground'):
    cmds.delete('ground')

walls = cmds.ls('wall*')
if len(walls) > 0:
    cmds.delete(walls)

particleList = cmds.ls('myParticle*')

if len(particleList) > 0:
    cmds.delete(particleList)

# Create the ground plane
cmds.polyCube(sx=10, sy=15, sz=5, w=5, d=5, h=0.01, name='ground')


xMax = 1
xMin = -xMax

yMin = 0.0

zMax = 1
zMin = -zMax 

# Create the walls
wall = cmds.polyCube(sx=10, sy=15, sz=15, w=2*zMax, d=0.01, name='wall1')
cmds.move(0, 0.5, zMin, wall)
wall = cmds.polyCube(sx=10, sy=15, sz=15, w=2*zMax, d=0.01, name='wall2')
cmds.move(0, 0.5, zMax, wall)

wall = cmds.polyCube(sx=10, sy=15, sz=15, w=0.01, d=2*xMax, name='wall3')
cmds.move(xMax, 0.5, 0, wall)
wall = cmds.polyCube(sx=10, sy=15, sz=15, w=0.01, d=2*xMax, name='wall4')
cmds.move(xMin, 0.5, 0, wall)


noParticles = 0
h = 0.1
initialDensity = 10

height = 5
width = 5
depth = 5

# Create  a cube of particles
for i in range(width):
    for j in range(height):
        for k in range(depth):
            particle = cmds.polySphere(n='myParticle#', sx=2, sy=2, r=h/2)
            cmds.move( i * h, h + j * h,  k * h, particle)
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
    mass = 0.000001

    def __init__(self):
        self.name = "unset"
        self.position = [0, 0, 0]
        self.density = initialDensity
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
                        math.pow(particleDistance, 2)), 3)
              )

    return output

# Calculate the density based on all neighbors within the smoothingRadius
def calculateDensity(particle,  particleMass, neighbors):

    # print(neighbors)

    density = initialDensity
    for neighbor in neighbors:
        distanceToNeighbor = calculateDistance(particle, neighbor)
        density += particleMass * applyPoly6Kernel(distanceToNeighbor)

    if density < initialDensity:
        density = initialDensity

    return density


def calculatePressure(particleDensity):
    p0 = initialDensity
    K = 20
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

    return acceleration

# Make the particles collide with surfaces
def checkBoundaries(particle):

    offset = h
    
    # X
    if particle.position[0] + offset > xMax or particle.position[0] - offset < xMin:
        particle.velocity[0] *= -0.0

        if particle.position[0] + offset > xMax:
            particle.position[0] = xMax - offset
        else:
            particle.position[0] = xMin + offset

     # Y
    if particle.position[1] - offset < yMin:
        particle.position[1] = yMin + offset
        particle.velocity[1] = 0.0

    # Z
    if particle.position[2] + offset > zMax or particle.position[2] - offset < zMin:
        particle.velocity[2] *= -0.0

        if particle.position[2] + offset > zMax:
            particle.position[2] = zMax - offset
        else:
            particle.position[2] = zMin + offset


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

        # C++ code
        # p.v += DT*p.f/p.rho;
        # p.x += DT*p.v;

        dt = 0.01

        currentParticle.velocity[0] += (dt * totalAcceleration[0] ) / currentParticle.density
        currentParticle.velocity[1] += (dt * totalAcceleration[1] ) / currentParticle.density
        currentParticle.velocity[2] += (dt * totalAcceleration[2] ) / currentParticle.density

        nextPosition = [0, 0, 0]

        currentParticle.position[0] += dt * currentParticle.velocity[0]
        currentParticle.position[1] += dt * currentParticle.velocity[1]
        currentParticle.position[2] += dt * currentParticle.velocity[2]


        # velocity = [i * (currentTime / 100) for i in totalAcceleration]
        # currentParticle.velocity = velocity

        # print(velocity)

        
        # checkBoundaries(currentParticle)
        # print(nextPosition)
        # nextPosition[0] = currentParticle.position[0] + \
        #     (currentParticle.velocity[0] * currentTime * currentParticle.mass)
        # nextPosition[1] = currentParticle.position[1] + \
        #     (currentParticle.velocity[1] * currentTime * currentParticle.mass)
        # nextPosition[2] = currentParticle.position[2] + \
        #     (currentParticle.velocity[2] * currentTime * currentParticle.mass)

       
        # checkBoundaries(nextPosition, currentParticle)
        # currentParticle.position = nextPosition

        
        # Set keyframes for animating the particles
        cmds.setKeyframe(currentParticle.name, attribute='translateX',
                         v=currentParticle.position[0], t=currentTime)
        cmds.setKeyframe(currentParticle.name, attribute='translateY',
                         v=currentParticle.position[1], t=currentTime)
        cmds.setKeyframe(currentParticle.name, attribute='translateZ',
                         v=currentParticle.position[2], t=currentTime)

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
