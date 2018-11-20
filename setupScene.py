import maya.cmds as cmds

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