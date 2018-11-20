import maya.cmds as cmds
import datetime

particleList = cmds.ls('myParticle*')

# Select all particles
cmds.select('myParticle*')
# print 'Particles selected'

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

# Create a progress bar
window = cmds.window()
cmds.columnLayout()
cmds.text(label='Simulation Progress')
progressControl = cmds.progressBar(
    status='"Simulation Progress', maxValue=noParticles*endTime, width=300)
# cmds.button( label='Simulation Progress!', command='cmds.progressBar(progressControl, edit=True, step=1)' )

# Start timer
start = cmds.timerX()

cmds.showWindow(window)

# Main simulaiton loop
while currentTime < endTime:

    # Loop all particles
    for p in range(noParticles):

        particleIndex = p + 1

        # Get current position
        currentPosition = [
            cmds.getAttr('myParticle' + str(particleIndex) + '.translateX'),
            cmds.getAttr('myParticle' + str(particleIndex) + '.translateY'),
            cmds.getAttr('myParticle' + str(particleIndex) + '.translateZ')
        ]

        # Apply gravity
        velocity = gravity * currentTime

        nextPosition = currentPosition
        nextPosition[1] = currentPosition[1] + (velocity * currentTime * mass)

        # Set keyframes for animating the particles
        cmds.setKeyframe('myParticle*', attribute='translateY', v=nextPosition[1], t=currentTime)

        # Update progress bar
        cmds.progressBar(progressControl, edit=True, step=1)

    currentTime += 1

# Stop progress bar, we are done
cmds.progressBar(progressControl, edit=True, endProgress=True)
cmds.deleteUI(window, window=True)

totalTime = cmds.timerX(startTime=start)
window = cmds.window(widthHeight=(400, 60))
cmds.columnLayout(adjustableColumn=True)
simulationTime = str(datetime.timedelta(seconds=totalTime)).split(".")[0]
cmds.text(label='Simulated in ' + simulationTime, align='center')
cmds.showWindow(window)


print 'Keyframes set'
