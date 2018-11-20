import maya.cmds as cmds

particleList = cmds.ls('myParticle*')

# Select all particles
cmds.select('myParticle*')
print 'Particles selected'

# print str(len(particleList)) + " particles selected"

# Clear keyframes
cmds.cutKey()

# Animation
startTime = cmds.playbackOptions(query=True, minTime=True)
endTime = cmds.playbackOptions(query=True, maxTime=True)

currentTime = startTime

noParticles = 25

spawnHeight = 3

gravity = -9.82

mass = 0.00001


# Set keyframes for animating the particles
while currentTime < endTime:
    #newHeight = spawnHeight - currentTime * 0.01
    #cmds.setKeyframe('myParticle*', attribute='translateY', v=newHeight, t=currentTime)

    # Main simulaiton loop
    for p in range(noParticles):

        particleIndex = p + 1;

        # Get current position
        currentPosition = [ 
            cmds.getAttr('myParticle' + str(particleIndex) + '.translateX'), 
            cmds.getAttr('myParticle' + str(particleIndex) + '.translateY'), 
            cmds.getAttr('myParticle' + str(particleIndex) + '.translateZ')
        ]

        # Apply gravity
        velocity = gravity * currentTime

        nextPosition = currentPosition

        nextPosition[1] =  currentPosition[1] + (velocity * currentTime * mass)

        cmds.setKeyframe('myParticle*', attribute='translateY', v=nextPosition[1], t=currentTime)


    currentTime += 1

print 'Keyframes set'