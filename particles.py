import maya.cmds as cmds

particleList = cmds.ls('myParticle*')

# Select all particles
cmds.select('myParticle*')
print 'Particles selected'

# print str(len(particleList)) + " particles selected"

# Animation
startTime = cmds.playbackOptions(query=True, minTime=True)
endTime = cmds.playbackOptions(query=True, maxTime=True)

currentTime = startTime

noParticles = 100

spawnHeight = 5

# Set keyframes for animating the particles
while currentTime < endTime:
    newHeight = spawnHeight - i * 0.01
    cmds.setKeyframe('myParticle*', attribute='translateY', v=newHeight, t=currentTime)
    currentTime += 1

print 'Keyframes set'