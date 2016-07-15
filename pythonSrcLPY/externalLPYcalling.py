# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 09:15:48 2016

@author: Ryan McCormick with code from Sandra Truong and the OpenAlea developers:
https://github.com/openalea/plantgl/blob/e07584f0c72e71e4aac9c3221305f800276691c0/src/plantgl/codec/obj.py
https://github.com/pradal/scanalea/blob/master/src/scanalea/vtk.py

The purpose of this script is to interpret a Lindenmeyer system using L-py, and 
write it as a mesh or point cloud to a file.
"""

# The anaconda readline seems to have a bug; reimporting readline seems to get around it.
import readline

# We're going to call meshlab to process the resulting mesh to a point cloud.
from subprocess import call

# The anaconda python doesn't seem to find most of the modules we need to Lpy, so we'll add them in.
import sys
# Manually specify most of the locations for the LPy libraries.
pathsToAppend =['/usr/bin', '/usr/lib/python2.7', 
                '/usr/lib/python2.7/plat-x86_64-linux-gnu', '/usr/lib/python2.7/lib-tk', 
                '/usr/lib/python2.7/lib-old', '/usr/lib/python2.7/lib-dynload', 
                '/home/rfm_node03/.local/lib/python2.7/site-packages', 
                '/usr/local/lib/python2.7/dist-packages', '/usr/lib/python2.7/dist-packages', 
                '/usr/lib/python2.7/dist-packages/PILcompat', '/usr/lib/python2.7/dist-packages/gtk-2.0', 
                '/usr/lib/pymodules/python2.7', '/usr/lib/python2.7/dist-packages/ubuntu-sso-client', 
                '/usr/lib/python2.7/dist-packages/IPython/extensions']      
for path in pathsToAppend:
    sys.path.append(path)

from openalea.lpy import *
from openalea.plantgl.all import get_shared_data, Viewer, Scene, SceneCodec
import openalea.plantgl.scenegraph as sg
import openalea.plantgl.algo as alg

# This is mostly a copy of the Faces class which is used by the Scanalea developers to generate different file formats from
#   a PlantGL Scene. A new member function has been added for PLY files.
# Originally obtained from: 
#   https://github.com/openalea/plantgl/blob/e07584f0c72e71e4aac9c3221305f800276691c0/src/plantgl/codec/obj.py
from itertools import izip_longest
class Faces(object):
    def __init__(self, name, offset, mesh):
        """ Create a temporary object to ease the writing of OBJ files.
        offset has to be greater than 1.
        """
        self.offset = offset
        self.vindex = mesh.indexList
        self.nindex = mesh.normalIndexList if mesh.normalPerVertex and mesh.normalIndexList else []
        self.tindex = mesh.texCoordIndexList if mesh.texCoordIndexList else []
        self.name = name

    def has_normal(self):
        return bool(self.nindex) 
    def has_texture(self):
        return bool(self.tindex) 

    def obj(self, output):
        """ Write the faces in an obj format. """
        gen = izip_longest(self.vindex, self.tindex,self.nindex, fillvalue=None)
        offset = self.offset
        output.write('g %s \n'%self.name)
        output.write('usemtl %s \n'%'red')
        for index, texture, normal in gen:
            s = ' '.join('/'.join((str(index[i]+offset),
                                   str(texture[i]+offset) if texture else '', 
                                   str(normal[i]+offset) if normal else '')).strip('/') for i in range(len(index)))
            line = 'f ' + s + '\n'
            output.write(line)
            
    def writePly(self, output):
        """ Write faces out in PLY format. Note that this is slighly unusual
                in that faces are written as squares, not triangles."""
        gen = izip_longest(self.vindex, self.tindex,self.nindex, fillvalue=None)
        offset = self.offset
        for index, texture, normal in gen:
            indices = []
            indices.append(str(index[0] + offset - 1))
            indices.append(str(index[1] + offset - 1))
            indices.append(str(index[2] + offset - 1))
            indices.append(str(index[3] + offset - 1))
            line = ' '.join(indices)
            line = "4 " + line + '\n'
            output.write(line)
            
            


# From https://github.com/pradal/scanalea/blob/master/src/scanalea/vtk.py
# See also the L-py manuscript Boudon, Pradal, Cokelaer, Prusinkiewicz, and Godin (2012)
inputFileName='175_3_mod_noColors.lpy'
print("Attempting to externally generate L-system using L-py from file: %s" % inputFileName )
# Need to get an AxialTree from the Lpy to be able to convert it to a scene.
lsys = Lsystem('175_3_mod_noColors.lpy')

lString = lsys.derive()
# interpretedString is an AxialTree
interpretedString= lsys.interpret(lString)
# The AxialTree can be converted to a Scene
scene = lsys.sceneInterpretation(interpretedString)

# Entities in the Scene can be discretized to generate polygons.
# Most of this code block was obtained from: https://github.com/pradal/scanalea/blob/master/src/scanalea/vtk.py
vertices = [] # List of point List
normals= [] # List of normal List
texcoords= [] # List of texture List
faces = [] # list  of tuple (offset,index List)
d = alg.Discretizer()
counter = 0
pointCounter = 0
faceCounter = 0
for i in scene:
    if i.apply(d):
        p = d.discretization
        pts = p.pointList
        ns = p.normalList
        ts = p.texCoordList
        indices = p.indexList
        n = len(p.pointList)
        if n > 0:
            vertices.append(pts)
            pointCounter += 1
            if ns:
                normals.append(ns)
            if ts:
                texcoords.append(ts)
            individualFace = Faces(i.name, counter+1, p)
            faces.append(individualFace)
            faceCounter += len(individualFace.vindex)
        counter += n

outputFileName = "testSceneOut.ply"
print("Writing to file: %s" % outputFileName)
outfile = file(outputFileName, 'w')

'''
ply
format ascii 1.0
comment author rfm_node03
comment File Generated with PlantGL 3D Viewer
element vertex 28323
property float x
property float y
property float z
property uchar diffuse_red
property uchar diffuse_green
property uchar diffuse_blue
element face 54144
property list uchar int vertex_indices 
end_header
'''

outfile.write('ply\n')
outfile.write('format ascii 1.0\n')
outfile.write('comment Generated by converting PlantGL scene\n')
outfile.write('element vertex %i\n'% (counter))
outfile.write('property float x\n')
outfile.write('property float y\n')
outfile.write('property float z\n')
outfile.write('element face %i\n'% (faceCounter))
outfile.write('property list uchar int vertex_indices\n')
outfile.write('end_header\n')

for pts in vertices:
    for x, y, z in pts:
        outfile.write('%f %f %f\n'%(x, y, z))

for face in faces:
    face.writePly(outfile)

outfile.close()

processedPLYname = "sampledLsystem.ply"
scriptName = "PoissonSamplingOfLsystem.mlx"
print("Invoking meshlabserver")
call(["meshlabserver", "-i", outputFileName, "-o", processedPLYname, "-s", scriptName])
