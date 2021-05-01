import trimesh
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity

from matplotlib.patches import PathPatch
from matplotlib.path import Path
from numpy import asarray, concatenate, ones

from includes.thirdParty.polyObjects import PolygonPath, PolygonPatch

class stlToContours():
    def __init__(self, file):
        self.mesh = trimesh.load_mesh(file)

    def getContourToolpath(self, depth,scalingFactor=1,offset=0, safetyDistance=5, layers=1):
        theta=np.pi/2.
        rotZ=[[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]]

        # if we wanted to take a bunch of parallel slices
        z_extents = self.mesh.bounds[:,2]
        z_levels  = np.arange(*z_extents, step=5)#TODO Make the step size a variable? or smaller
        # find a bunch of parallel cross sections
        sections = self.mesh.section_multiplane(plane_origin=self.mesh.bounds[0],
                                           plane_normal=[0,0,1],
                                           heights=z_levels)

        polygons=[]
        for section in sections:
            if section is not None:
                paths=section.discrete
                outlines=section.root
                holes=[]
                for i, shape in enumerate(paths):
                    if i in outlines:
                        outline=shape #TODO: what happens with more than 1 outline?
                    else:
                        holes.append(shape)
                polygons.append(Polygon(tuple(outline), holes))#TODO: what happens with more than 1 outline?
        if len(sections)==1:
            u=polygons[-1]
        else:
            u = unary_union(polygons)
        patch2b = PolygonPatch(u, fc="blue", ec="blue", alpha=0.5, zorder=2)

        paths=[]
        pathIndices=np.where(PolygonPath(u).codes==1)[0]

        for i in range(len(pathIndices)):
            if i!=len(pathIndices)-1:
                paths.append(PolygonPath(u).vertices[pathIndices[i]:pathIndices[i+1]])
            else:
                paths.append(PolygonPath(u).vertices[pathIndices[i]:])
        offsetPaths=[]
        offsetX=[]
        offsetY=[]
        for i,path in enumerate(paths):
            line=LineString(path)
            offsetPaths.append([line.parallel_offset(offset, 'right' if i==0 else'left', join_style=2)])
            for part in offsetPaths[-1]:#get offset paths
                try:
                    x, y = part.xy
                except:
                    continue
                offsetX.append(x)
                offsetY.append(y)
        #make toolpaths
        toolpath=np.array([[0,0,0]])
        for layer in range(1,layers+1):
            layerDepth=-depth*(layer/layers)
            for xpath, ypath in zip(offsetX,offsetY):
                toolpath=np.vstack((toolpath,np.array([[toolpath[-1][0],toolpath[-1][1],safetyDistance],[xpath[0]*scalingFactor,ypath[0]*scalingFactor,safetyDistance]])))#[toolpath[-1][0],toolpath[-1][1],safetyDistance],[xpath[0],ypath[0],safetyDistance]
                toolpath=np.vstack((toolpath,np.dstack((np.asarray(xpath)*scalingFactor,np.asarray(ypath)*scalingFactor,np.full(len(xpath),layerDepth)))[0]))
                toolpath=np.vstack((toolpath,np.array([xpath[0]*scalingFactor,ypath[0]*scalingFactor,layerDepth])))
                toolpath=np.vstack((toolpath,np.array([xpath[0]*scalingFactor,ypath[0]*scalingFactor,safetyDistance])))
        toolpath=np.vstack((toolpath,np.array([[0,0,safetyDistance],[0,0,0]])))
        return toolpath
