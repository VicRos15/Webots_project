#!/usr/bin/env python

# Authors: Juan G Victores
# CopyPolicy: released under the terms of the LGPLv2.1
# URL: <https://github.com/roboticslab-uc3m/webots-tools>

#-- User variables

boxHeight = 1.0
inFileStr = 'assets/map1.csv'

resolution = 1.0  # Just to make similar to MATLAB [pixel/meter]
meterPerPixel = 1 / resolution  # [meter/pixel]

#-- Program

from numpy import genfromtxt
inFile = genfromtxt(inFileStr, delimiter=',')
print(inFile)

nX = inFile.shape[0]
nY = inFile.shape[1]
print("lines = X =", inFile.shape[0])
print("columns = Y =", inFile.shape[1])

Ez = boxHeight

Ex = meterPerPixel
Ey = meterPerPixel

myStr = '#VRML_SIM R2022b utf8\n\
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/backgrounds/protos/TexturedBackground.proto"\n\
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"\n\
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/floors/protos/Floor.proto"\n\
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/solids/protos/SolidBox.proto"\n\
WorldInfo {\n\
  coordinateSystem "ENU"\n\
}\n\
Viewpoint {\n\
  orientation -0.181 0.103 0.978 2.15\n\
  position 14.5 -11.1 7.31\n\
}\n\
TexturedBackground {\n\
}\n\
TexturedBackgroundLight {\n\
}\n'

#-- Create Floor with Grass Texture

floorEx = meterPerPixel * nX # x is x
floorEy = meterPerPixel * nY # y is y

floorStr = 'Floor {\n\
  translation $floorX $floorY 0\n\
  size $floorEx $floorEy\n\
  appearance Appearance {\n\
    texture ImageTexture {\n\
      url "textures/grass_.jpg"\n\
    }\n\
  }\n\
}\n'

floorStr = floorStr.replace('$floorX',str(floorEx/2.0))
floorStr = floorStr.replace('$floorY',str(floorEy/2.0))
floorStr = floorStr.replace('$floorEx',str(floorEx))
floorStr = floorStr.replace('$floorEy',str(floorEy))
myStr += floorStr

#-- Create Mountains (Obstacles) with Rock Texture

boxStr = 'SolidBox {\n\
  name "$name"\n\
  translation $x $y $z\n\
  size $Ex $Ey $Ez\n\
  appearance Appearance {\n\
    texture ImageTexture {\n\
      url "textures/rock_.jpg"\n\
    }\n\
  }\n\
}\n'

for iX in range(nX):
    for iY in range(nY):
        # Skip box if map indicates a 0
        if inFile[iX][iY] == 0:
            continue

        # Add E___/2.0 to each to force begin at 0,0,0 (centered by default)
        x = Ex/2.0 + iX*meterPerPixel
        y = Ey/2.0 + iY*meterPerPixel
        z = Ez/2.0  # Add this to raise to floor level (centered by default)

        # Create box for mountain-like obstacle
        name = 'mountain_'+str(iX)+'_'+str(iY)
        tmpBoxStr = boxStr.replace('$name',name)
        tmpBoxStr = tmpBoxStr.replace('$x',str(x))
        tmpBoxStr = tmpBoxStr.replace('$y',str(y))
        tmpBoxStr = tmpBoxStr.replace('$z',str(z))
        tmpBoxStr = tmpBoxStr.replace('$Ex',str(Ex))
        tmpBoxStr = tmpBoxStr.replace('$Ey',str(Ey))
        tmpBoxStr = tmpBoxStr.replace('$Ez',str(Ez))
        myStr += tmpBoxStr

# Output the final VRML simulation world
outFile = open('map_vrs_with_textures.wbt', 'w')
outFile.write(myStr)
outFile.close()