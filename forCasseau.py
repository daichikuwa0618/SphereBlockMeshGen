#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np

# variables
convertToMeters = 1.0 # if 1, radiusSphere = 1 [m]
radiusSphere = 1.0 # radius of sphere
radiusFront = 3.0
wakeLength = 8.0
topHeight = 5.0

def writeHeader(f):
    f.write('/*--------------------------------*- C++ -*----------------------------------*\\\n')
    f.write('| =========                 |                                                 |\n')
    f.write('| \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox           |\n')
    f.write('|  \\\\    /   O peration     | Version:  2.1.0                                 |\n')
    f.write('|   \\\\  /    A nd           | Web:      www.OpenFOAM.org                      |\n')
    f.write('|    \\\\/     M anipulation  |                                                 |\n')
    f.write('\\*---------------------------------------------------------------------------*/\n')
    f.write('FoamFile\n')
    f.write('{\n')
    f.write('    version     2.0;\n')
    f.write('    format      ascii;\n')
    f.write('    class       dictionary;\n')
    f.write('    object      blockMeshDict;\n')
    f.write('}\n')
    f.write('// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n')
    f.write('\n')

def writeBoundary(f):
    f.write('    flow\n    {\n')
    f.write('        type patch;\n')
    f.write('        faces\n')
    f.write('        (\n')
    f.write('            (10 16 17 11)\n')
    f.write('            (16 20 21 17)\n')
    f.write('            (20 21 19 18)\n')
    f.write('        );\n')
    f.write('    }\n')
    f.write('    sphere\n    {\n')
    f.write('        type wall;\n')
    f.write('        faces\n')
    f.write('        (\n')
    f.write('            (6 12 13 7)\n')
    f.write('            (12 0 1 13)\n')
    f.write('        );\n')
    f.write('    }\n')
    f.write('    wedgeFront\n    {\n')
    f.write('        type symmetry;\n')
    f.write('        faces\n')
    f.write('        (\n')
    f.write('            (0 2 14 12)\n')
    f.write('            (12 14 8 6)\n')
    f.write('            (2 4 16 14)\n')
    f.write('            (14 16 10 8)\n')
    f.write('            (4 18 20 16)\n')
    f.write('        );\n')
    f.write('    }\n')
    f.write('    wedgeBack\n    {\n')
    f.write('        type symmetry;\n')
    f.write('        faces\n')
    f.write('        (\n')
    f.write('            (1 3 15 13)\n')
    f.write('            (13 15 9 7)\n')
    f.write('            (3 5 17 15)\n')
    f.write('            (15 17 11 9)\n')
    f.write('            (5 19 21 17)\n')
    f.write('        );\n')
    f.write('    }\n')

def rotateFront(x):
    c = np.cos(np.deg2rad(-2.5))
    s = np.sin(np.deg2rad(-2.5))
    Rx = np.array([[1, 0, 0],
                   [0, c,-s],
                   [0, s, c]])
    return np.dot(Rx, x)

def rotateBack(x):
    c = np.cos(np.deg2rad(2.5))
    s = np.sin(np.deg2rad(2.5))
    Rx = np.array([[1, 0, 0],
                   [0, c,-s],
                   [0, s, c]])
    return np.dot(Rx, x)


numPoints = 5
point = np.zeros((numPoints,3),float)
#point = np.insert(np.zeros((numPoints,3),float), 0, np.arange(numPoints), axis=1)

point[0,0] = -radiusFront
point[1,0] = wakeLength
point[1,1] = topHeight
point[2,0] = wakeLength
point[3,0] = radiusSphere
point[4,0] = -radiusSphere

print(point)

edge1 = np.array([np.cos(np.deg2rad(45)),np.sin(np.deg2rad(45)),0.])
edge2 = np.array([-np.cos(np.deg2rad(45)),np.sin(np.deg2rad(45)),0.])

with open('blockMeshDict',mode='w') as f:
    writeHeader(f)
    f.write('convertToMeters ')
    f.write(str(convertToMeters))
    f.write(';\n\n')
    f.write('vertices\n(\n')
    for i in range(numPoints):
        f.write('    (')
        for j in range(3):
            f.write(str(rotateFront(point[i])[j]) + ' ')
        f.write(')\n')
        f.write('    (')
        for j in range(3):
            f.write(str(rotateBack(point[i])[j]) + ' ')
        f.write(')\n')
    f.write(');\n\n')

    f.write('blocks\n(\n')
    f.write('    hex (0 2 14 12 1 3 15 13) (10 10 1) simpleGrading (1 1 1)\n')
    f.write('    hex (12 14 8 6 13 15 9 7) (10 10 1) simpleGrading (1 1 1)\n')
    f.write('    hex (2 4 16 14 3 5 17 15) (10 10 1) simpleGrading (1 1 1)\n')
    f.write('    hex (14 16 10 8 15 17 11 9) (10 10 1) simpleGrading (1 1 1)\n')
    f.write('    hex (4 18 20 16 5 19 21 17) (20 10 1) simpleGrading (1 1 1)\n')
    f.write(');\n\n')

    f.write('edges\n(\n')
    f.write('    arc 0 12 (')
    for j in range(3):
        f.write(str(rotateFront(radiusSphere*edge1)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 1 13 (')
    for j in range(3):
        f.write(str(rotateBack(radiusSphere*edge1)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 12 6 (')
    for j in range(3):
        f.write(str(rotateFront(radiusSphere*edge2)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 13 7 (')
    for j in range(3):
        f.write(str(rotateBack(radiusSphere*edge2)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 2 14 (')
    for j in range(3):
        f.write(str(rotateFront(radiusMiddle*edge1)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 3 15 (')
    for j in range(3):
        f.write(str(rotateBack(radiusMiddle*edge1)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 14 8 (')
    for j in range(3):
        f.write(str(rotateFront(radiusMiddle*edge2)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 15 9 (')
    for j in range(3):
        f.write(str(rotateBack(radiusMiddle*edge2)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 4 16 (')
    for j in range(3):
        f.write(str(rotateFront(radiusMesh*edge1)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 5 17 (')
    for j in range(3):
        f.write(str(rotateBack(radiusMesh*edge1)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 16 10 (')
    for j in range(3):
        f.write(str(rotateFront(radiusMesh*edge2)[j]) + ' ')
    f.write(')\n')
    f.write('    arc 17 11 (')
    for j in range(3):
        f.write(str(rotateBack(radiusMesh*edge2)[j]) + ' ')
    f.write(')\n')
    f.write(');\n\n')

    f.write('boundary\n(\n')
    writeBoundary(f)
    f.write(');\n\n')
    f.write('mergePatchPairs\n(\n')
    f.write(');\n\n')

with open('blockMeshDict') as f:
    print(f.read())

