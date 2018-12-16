import maya.cmds as cmds
import datetime
import math
import random

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

# Number of particles

# height = 6
# width = 10
# depth = 10

height = 6
width = 14
depth = 14


noParticles = 0
h = 0.1
hsq = math.pow(h,2)
initialDensity = 1000

VISC = 40;


POLY6 = 315/(65*math.pi*math.pow(h, 9))
SPIKY_GRAD = -45/(math.pi*math.pow(h, 6))
VISC_LAP = 45/(math.pi*math.pow(h, 6))

# xMax = 0.6
# xMin = -xMax

# yMin = 0.0

# zMax = 0.6
# zMin = -zMax

xMax = width * h / 2 + h
xMin = -xMax

yMin = 0.0
yMax = height * h / 2

zMax = depth * h / 2 + h
zMin = -zMax  

# Create the ground plane
cmds.polyCube(sx=10, sy=15, sz=5, w=2*zMax, d=2*xMax, h=0.01, name='ground')

# Create the walls
wall = cmds.polyCube(sx=10, sy=15, sz=15, h=yMax, w=2*zMax, d=0.01, name='wall1')
cmds.move(0, yMax/2, zMin, wall)
wall = cmds.polyCube(sx=10, sy=15, sz=15, h=yMax, w=2*zMax, d=0.01, name='wall2')
cmds.move(0,yMax/2, zMax, wall)

wall = cmds.polyCube(sx=10, sy=15, h=yMax, sz=15, w=0.01, d=2*xMax, name='wall3')
cmds.move(xMax, yMax/2, 0, wall)
wall = cmds.polyCube(sx=10, sy=15, h=yMax, sz=15, w=0.01, d=2*xMax, name='wall4')
cmds.move(xMin, yMax/2, 0, wall)


level = 0
# Create  a wave  of particles
# for i in range(width):
#     for j in range(height):
#         for k in range(depth):
#             nudge = random.uniform(0.0, 0.05)
#             particle = cmds.polySphere(n='myParticle#', sx=2, sy=2, r=h/2)
#             cmds.move( i * h - width/2*h + nudge, h/2 + j * h,  k * h - depth/2 * h + nudge, particle)
#             noParticles += 1
#         level += 1

# Create  a box  of particles
for i in range(width):
    for j in range(height):
        for k in range(depth):
            nudge = random.uniform(0.0, 0.05)
            particle = cmds.polySphere(n='myParticle#', sx=2, sy=2, r=h/2)
            cmds.move( i * h - width/2*h + nudge, h/2 + j * h,  k * h - depth/2 * h + nudge, particle)
            noParticles += 1
        level += 1

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
    mass = 0.60

    def __init__(self):
        self.name = "unset"
        self.position = [0, 0, 0]
        self.density = initialDensity
        self.pressure = 1
        self.force = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.velocity = [-3, 0, 0]

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
        if distance <= h and distance > 0:
            neighbours.append(particles[index])

        index += 1

    return neighbours


# Calculate the density based on all neighbors within the smoothingRadius
def calculateDensity(particle, neighbors):

    density = 0
    for neighbor in particles:

        distanceToNeighbor = calculateDistance(neighbor, particle)
        squareDist = math.pow(distanceToNeighbor, 2)

        if squareDist < hsq:
            density += particle.mass* POLY6 * math.pow(hsq - squareDist, 3)

    return density


def calculatePressure(particle):
    K = 11
    pressure = K * (particle.density - initialDensity)
    return pressure


def calculateDistance(p1, p2):

    distanceVector = getDistanceVector(p1, p2)

    distance = math.sqrt(math.pow( distanceVector[0], 2) + math.pow(distanceVector[1], 2) + math.pow(distanceVector[2], 2))

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


def calculateForces(particle, neighbors):
    fPressure = [0, 0, 0]
    fGravity = [0, 0, 0]
    fVisc = [0, 0, 0]

    for neighbor in neighbors:
        vector = getDistanceVector(neighbor, particle)
        magnitude  = math.sqrt( math.pow(vector[0], 2)+ math.pow(vector[1], 2)+ math.pow(vector[2], 2))
        normalizedVector  = ( vector[0] / magnitude ,  vector[1] / magnitude,  vector[2] / magnitude ) 


        fPressure[0] += normalizedVector[0] * particle.mass * (particle.pressure + neighbor.pressure) / (2*neighbor.density) * SPIKY_GRAD*math.pow(h-magnitude, 2)
        fPressure[1] += normalizedVector[1] * particle.mass * (particle.pressure + neighbor.pressure) / (2*neighbor.density) * SPIKY_GRAD*math.pow(h-magnitude, 2)
        fPressure[2] += normalizedVector[2] * particle.mass * (particle.pressure + neighbor.pressure) / (2*neighbor.density) * SPIKY_GRAD*math.pow(h-magnitude, 2)

        fVisc[0] += VISC*particle.mass*(neighbor.velocity[0] - particle.velocity[0])/neighbor.density * VISC_LAP*(h-magnitude);
        fVisc[1] += VISC*particle.mass*(neighbor.velocity[1] - particle.velocity[1])/neighbor.density * VISC_LAP*(h-magnitude);
        fVisc[2] += VISC*particle.mass*(neighbor.velocity[2] - particle.velocity[2])/neighbor.density * VISC_LAP*(h-magnitude);


    fGravity[1] = gravity[1] * particle.density

    # print(fPressure)

    totalforce = [x + y for x, y in zip(fPressure, fGravity)]
    totalforce = [x + y for x, y in zip(totalforce, fVisc)]

    # print(fPressure)

    return totalforce


# Make the particles collide with surfaces
def checkBoundaries(particle):

    offset = h

    damping = -0.0

    atEdge = 0
    
    # X
    if particle.position[0] + offset > xMax or particle.position[0] - offset < xMin:
        particle.velocity[0] *= damping
        atEdge = 1

        if particle.position[0] + offset > xMax:
            particle.position[0] = xMax - offset
        else:
            particle.position[0] = xMin + offset

    # Z
    if particle.position[2] + offset > zMax or particle.position[2] - offset < zMin:
        particle.velocity[2] *= damping
        atEdge = 1

        if particle.position[2] + offset > zMax:
            particle.position[2] = zMax - offset
        else:
            particle.position[2] = zMin + offset

    # Y
    if particle.position[1] - offset < yMin:
        particle.position[1] = yMin + offset
        particle.velocity[1] *= damping
        particle.velocity[0] *= 0.95
        particle.velocity[2] *= 0.95

        # if atEdge == 1:
        #     particle.velocity[0] *= 0.94
        #     particle.velocity[1] *= 0.94



    # Check for tilted plane
    # if particle.position[1] < plane and particle.position[0] < plane:




# Integrate to find new position and velocity
def calculateNewPosition(particle):

    dt = 0.006

    particle.velocity[0] += dt * particle.force[0]/particle.density
    particle.velocity[1] += dt * particle.force[1]/particle.density
    particle.velocity[2] += dt * particle.force[2]/particle.density

    particle.position[0] += dt * particle.velocity[0]
    particle.position[1] += dt * particle.velocity[1]
    particle.position[2] += dt * particle.velocity[2]

    checkBoundaries(particle)


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

    # print("Frame " + str(currentTime))

    # Loop all particles
    for p in range(noParticles):

        particleIndex = p + 1

        currentParticle = particles[p]
        # currentParticle.name = particleList[p]

        # print(currentParticle.position)

        # --------- Calculate! -------------

        particleNeighbors = getNeighbors(currentParticle, h)

        # print(particleNeighbors)

        currentParticle.density = calculateDensity(currentParticle, particleNeighbors)

        # print(currentParticle.density)

        currentParticle.pressure = calculatePressure(currentParticle)

        # print(currentParticle.pressure)

        currentParticle.force = calculateForces(currentParticle, particleNeighbors)

        # print(currentParticle.force[1])

        nextPosition = [0, 0, 0]

        calculateNewPosition(currentParticle)

        # print(currentParticle.velocity)
        
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
