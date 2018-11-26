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

# Create particles
for i in range(5):
    for j in range(5):
        particle = cmds.polySphere(n='myParticle#', sx=6, sy=6, r=0.05)
        cmds.move(i * 0.1, 3, j * 0.1, particle)
        noParticles += 1

#particleList = cmds.ls('myParticle*')
print str(noParticles) + " particles created"

#-------SETUP DONE-----------------



particleList = cmds.ls('myParticle*', type='transform', showType=False)

# Print all particles in the scene
# for part in particleList:
#     print(part)

# Select all particles
cmds.select('myParticle*')
# print 'Particles selected'

# print str(len(particleList)) + " particles selected"

# Clear keyframes
cmds.cutKey()

class Particle:
    mass = 0.0001

    def __init__(self):
        self.name = "unset"
        self.position = [0, 0, 0]
        self.density = 1
        self.pressure = 1
        self.acceleration = [0, 0, 0]

# --------------- Functions -----------------

# Get all neighbors within the smoothing radius
def getNeighbors( particle, smoothingLength ):
    
    neighbours = []

    # currentPosition = [ cmds.getAttr( particle.name + '.translateX' ), cmds.getAttr( particle.name + '.translateY' ), cmds.getAttr( particle.name + '.translateZ' ) ]

    index = 0

    for otherParticle in particles:
        
        # particlePosition = [ cmds.getAttr( otherParticle + '.translateX' ), 
        # cmds.getAttr( otherParticle + '.translateY' ),
        # cmds.getAttr( otherParticle + '.translateZ' ) ]
        
        deltadist = [0, 0, 0]
        
        # Calculate distance 
        # deltadist[0] = particlePosition[0] - currentPosition[0]
        # deltadist[1] = particlePosition[1] - currentPosition[1]
        # deltadist[2] = particlePosition[2] - currentPosition[2]

        deltadist[0] = particle.position[0] - otherParticle.position[0]
        deltadist[1] = particle.position[1] - otherParticle.position[1]
        deltadist[2] = particle.position[2] - otherParticle.position[2]


        distance = math.sqrt( math.pow(deltadist[0], 2) + math.pow(deltadist[1], 2) + math.pow(deltadist[2], 2) )
    
        # if distance <= smoothingLength, put in array
        if distance <= smoothingLength and distance > 0: 
            #neighbours.append( otherParticle )
            # print(particles[index].name)
            neighbours.append( particles[index] )

        index += 1

    
    return neighbours

def applyPoly6Kernel(particleDistance):
    smoothingRadius = 0.2
    output = (315/(64 * math.pi * math.pow(smoothingRadius, 9))) * math.pow((math.pow(smoothingRadius, smoothingRadius) - math.pow(particleDistance, particleDistance)), 3)
    return output

# Calculate the density based on all neighbors within the smoothingRadius
def calculateDensity(particle,  particleMass, neighbors ):

    # print(neighbors)
    
    density = 1
    for neighbor in neighbors:
        distanceToNeighbor = calculateDistance(particle, neighbor)
        density += particleMass * applyPoly6Kernel(distanceToNeighbor)
    return density


def calculatePressure(particleDensity):
    p0 = 100
    K = 20
    pressure = K * (particleDensity - p0)
    return pressure


def calculateDistance(p1, p2):
    # print(p2.name)

    p1Pos = [ cmds.getAttr( p1.name + '.translateX' ), 
        cmds.getAttr( p1.name + '.translateY' ),
        cmds.getAttr( p1.name + '.translateZ' ) ]

    p2Pos = [ cmds.getAttr( p2.name + '.translateX' ), 
        cmds.getAttr( p2.name + '.translateY' ),
        cmds.getAttr( p2.name + '.translateZ' ) ]

    # print(cmds.getAttr( p2.name + '.translateX' ))
    

    distanceVector = [
            p1Pos[0] - p2Pos[0],
            p1Pos[1] - p2Pos[1],
            p1Pos[2] - p2Pos[2]]

    distance = math.sqrt( math.pow(distanceVector[0], 2) + math.pow(distanceVector[1], 2) + math.pow(distanceVector[2], 2) )

    return distance


# Input as strings expected
def getDistanceVector(p1, p2):
    p1Pos = [ cmds.getAttr( str(p1.name) + '.translateX' ), 
    cmds.getAttr( str(p1.name) + '.translateY' ),
    cmds.getAttr( str(p1.name) + '.translateZ' ) ]

    p2Pos = [ cmds.getAttr( str(p2.name) + '.translateX' ), 
        cmds.getAttr( str(p2.name) + '.translateY' ),
        cmds.getAttr( str(p2.name) + '.translateZ' ) ]

    distanceVector = [
            p1Pos[0] - p2Pos[0],
            p1Pos[1] - p2Pos[1],
            p1Pos[2] - p2Pos[2] ]

    return distanceVector

def calculateSpikyKernel(smoothingRadius, particle, neighbor):
    distanceVec = getDistanceVector(particle, neighbor)

    gX = -( 45 / (math.pi * math.pow(smoothingRadius, 6)) ) * math.pow((smoothingRadius - distanceVec[0]), 2)
    gY = -( 45 / (math.pi * math.pow(smoothingRadius, 6)) ) * math.pow((smoothingRadius - distanceVec[1]), 2)
    gZ = -( 45 / (math.pi * math.pow(smoothingRadius, 6)) ) * math.pow((smoothingRadius - distanceVec[2]), 2)

    gradient = [gX, gY, gZ]
    return gradient


def calculateAcceleration(neighbors, particle, particleMass, particlePressure, particleDensity):
    acceleration = [0, 0, 0]

    for neighbor in neighbors:
        #normalizedDirection = getDistanceVector(particle, neighbor)
        # print(neighbor)
        spikyGradient = calculateSpikyKernel(0.2, particle, neighbor)
        acceleration[0] = -(particleMass/particleMass) * ( (particlePressure + particle.pressure) / ( 2*particleDensity*neighbor.density ) ) * spikyGradient[0]
        acceleration[1] = -(particleMass/particleMass) * ( (particlePressure + particle.pressure) / ( 2*particleDensity*neighbor.density ) ) * spikyGradient[1]
        acceleration[2] = -(particleMass/particleMass) * ( (particlePressure + particle.pressure) / ( 2*particleDensity*neighbor.density ) ) * spikyGradient[2]

    return acceleration


# Animation
startTime = cmds.playbackOptions(query=True, minTime=True)
endTime = cmds.playbackOptions(query=True, maxTime=True)
currentTime = startTime

noParticles = len(particleList)
spawnHeight = 3
gravity = [0, -9.82, 0]
# mass = 0.00001
mass = 0.00001

# Create a progress bar
window = cmds.window()
cmds.columnLayout()
cmds.text(label='Simulation Progress')
progressControl = cmds.progressBar(status='"Simulation Progress', maxValue=noParticles*endTime, width=300)
# cmds.button( label='Simulation Progress!', command='cmds.progressBar(progressControl, edit=True, step=1)' )

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
    # cmds.setAttr( particleList[index] + '.translateX', 3 )
    cmds.setAttr( particleList[index] + '.translateY', 3 )
    # cmds.setAttr( particleList[index] + '.translateZ', 3 )
    particles.append(p)
    index += 1

def initialSetup():
    # Initial position
    loopindex = 1
    for p in particles:
        p.position = [ 
            cmds.getAttr('myParticle' + str(loopindex) + '.translateX'), 
            cmds.getAttr('myParticle' + str(loopindex) + '.translateY'), 
            cmds.getAttr('myParticle' + str(loopindex) + '.translateZ') ]       
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

        # --------- Calculate! -------------

        particleNeighbors = getNeighbors(currentParticle, 0.2)

        currentParticle.density = calculateDensity(currentParticle, mass, particleNeighbors)

        currentParticle.pressure = calculatePressure(currentParticle.density)

        # print(currentParticle.density)

        currentParticle.acceleration = calculateAcceleration(particleNeighbors, currentParticle, mass, currentParticle.pressure, currentParticle.density)

        # print(currentParticle.acceleration)

        totalAcceleration = currentParticle.acceleration
        totalAcceleration[1] += gravity[1]

        print(totalAcceleration)

        # Apply gravity
        velocity = [i * (currentTime / 100) for  i in totalAcceleration] 

        # print(velocity)
        
        # velocity = totalAcceleration * currentTime

        nextPosition = currentParticle.position
        # print(nextPosition)
        nextPosition[0] = currentParticle.position[0] + (velocity[0] * currentTime * mass)
        nextPosition[1] = currentParticle.position[1] + (velocity[1] * currentTime * mass)
        nextPosition[2] = currentParticle.position[2] + (velocity[2] * currentTime * mass)

        currentParticle.position = nextPosition

        # Set keyframes for animating the particles
        cmds.setKeyframe('myParticle*', attribute='translateX', v=nextPosition[0], t=currentTime)
        cmds.setKeyframe('myParticle*', attribute='translateY', v=nextPosition[1], t=currentTime)
        cmds.setKeyframe('myParticle*', attribute='translateZ', v=nextPosition[2], t=currentTime)

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
