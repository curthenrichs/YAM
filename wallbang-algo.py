import math
import matplotlib.pyplot as plt

# Need to figure out transform
sensorValues = [18,20,5]
sen1 = (sensorValues[0] * math.cos(math.radians(135)), sensorValues[0] * math.sin(math.radians(135)))
sen2 = (sensorValues[1] * math.cos(math.radians(90)), sensorValues[1] * math.sin(math.radians(90)))
sen3 = (sensorValues[2] * math.cos(math.radians(45)), sensorValues[2] * math.sin(math.radians(45)))

# Constant positions
origin = (0,0)
minA = (15  * math.cos(math.radians(180)), 0 * math.sin(math.radians(180)))
print(minA)
minB = (15 * math.cos(math.radians(0)), 0 * math.sin(math.radians(0)))
print(minB)

# Channel Bypass
# used when channel between s1 and s3 is not wide enough
channelBypass = (sen2[0] - sen1[0]) < 6

# Compute for all theta
maxTheta = 0
maxDistance = 0
for theta in range(0,181):

    # Select Sensor line
    if theta <= 45: # S3 to minB
        P0 = minB
        P1 = sen3
    elif theta <= 90: #S2 to S3
        if channelBypass:
            P0 = sen3
            P1 = sen1
        else:
            P0 = sen3
            P1 = sen2
    elif theta <= 135: #S1 to S2
        if channelBypass:
            P0 = sen3
            P1 = sen1
        else:
            P0 = sen2
            P1 = sen1
    else: #minA to S1
        P0 = sen1
        P1 = minA

    # Define Ray
    Q0 = origin
    Q1 = (1,math.tan(math.radians(theta)))

    # Compute intersection point
    u = (P1[0]-P0[0],P1[1]-P0[1]) #from P(s) = P0 + su
    v = (Q1[0]-Q0[0],Q1[1]-Q0[1]) # from Q(t) = Q0 + tv
    w = (P0[0]-Q0[0],P0[1]-Q0[1]) # from w = P0 - Q0
    ti = (u[0] * w[1] - u[1] * w[0]) / (u[0] * v[1] - u[1] * v[0])
    I = (ti * v[0],ti * v[1])

    # maximize on distance
    distance = math.sqrt(pow(I[0],2) + pow(I[1],2))
    if (distance >= maxDistance):
        maxTheta = theta
        maxDistance = distance

# convert into direction vector
print(maxTheta, maxDistance)
vector = (maxDistance * math.cos(math.radians(maxTheta)), maxDistance * math.sin(math.radians(maxTheta)))
print(vector)

# draw diagram of problem space
plt.plot([minA[0],sen1[0]],[minA[1],sen1[1]], color = 'b', marker = 'o')
plt.plot([sen1[0],sen2[0]],[sen1[1],sen2[1]], color = 'b', marker = 'o')
plt.plot([sen2[0],sen3[0]],[sen2[1],sen3[1]], color = 'b', marker = 'o')
plt.plot([sen3[0],minB[0]],[sen3[1],minB[1]], color = 'b', marker = 'o')
plt.plot([minA[0],minB[0]],[minA[1],minB[1]], color = 'b', marker = 'o')

if channelBypass:
    plt.plot([sen1[0],sen3[0]],[sen1[1],sen3[1]], color = 'g')

plt.plot([origin[0],vector[0]],[origin[1],vector[1]], color = 'r')

plt.show()


# Okay, so the simple algorithm is to scan the three sensors.
# If any are less than collision immediate threshold then
#   stop and reverse
# Else
#   create vector based on
#       - max distance for each of the points
#   each has a channel bypass to prevent robot from trying to squeeze itself into position
#       - bypass sen1
#           (minA,sen2)
#       - bypass sen2
#           (sen1,sen3)
#       - bypass sen3
#           (sen2,minB)
#   each adds neighbor values to itself

turnBounds = 10

sensorValues = [5,25,6]
rawSearchValues = [turnBounds,sensorValues[0],sensorValues[1],sensorValues[2],turnBounds]

if any([x < 5 for x in sensorValues]):
    pass # stop and reverse
else:

    # apply channel bypass as a zero mask
    channel = math.sqrt(pow(-0.707 * sensorValues[0] - 0.707 * sensorValues[2],2) + pow(0.707 * sensorValues[0] - 0.707 * sensorValues[2],2))
    if channel < 8:
        rawSearchValues[2] = 0

    # insert intermediate angles between raw
    searchValues = []
    for i in range(1,len(rawSearchValues)):
        searchValues.append(rawSearchValues[i-1])
        searchValues.append((rawSearchValues[i] + rawSearchValues[i-1])/2)
    searchValues.append(rawSearchValues[len(rawSearchValues)-1])

    # search for best direction
    maxIndex = 0
    maxValue = 0
    values = []
    for x in range(0,len(searchValues)):

        # create value (adding neighbor values)
        value = searchValues[x]
        if x > 0:
            value += searchValues[x-1]
        if x < len(searchValues)-1:
            value += searchValues[x+1]
        values.append(value)

        # check if it maximizes
        if value > maxValue:
            maxIndex = x
            maxValue = value

    # convert from index into angle and make drive vector
    theta = math.radians(180 - maxIndex * 22.5)
    vector = (100 * math.cos(theta), 100 * math.sin(theta))

    print("\n\n")
    print("values:",values)
    print("maxValue",maxValue,"maxIndex",maxIndex)
    print("Vector:",vector)
