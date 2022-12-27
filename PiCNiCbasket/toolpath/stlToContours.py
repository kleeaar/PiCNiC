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

from PiCNiCbasket.thirdParty.polyObjects import PolygonPath, PolygonPatch

class stlToContours():
    def __init__(self, file):
        self.mesh = trimesh.load_mesh(file)
        self.offsetX=[]
        self.offsetY=[]

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

        layeredPolygons=[]
        outline=[]
        for section in sections:
            if section is not None:
                layeredPolygons.append([])
                paths=section.discrete
                outlines=section.root
                holes=[]
                for i, shape in enumerate(paths):
                    if i in outlines:
                        outline.append(shape) #TODO: what happens with more than 1 outline?
                    else:
                        holes.append(shape)
                for line in outline:
                    layeredPolygons[-1].append(Polygon(tuple(line), holes))#TODO: what happens with more than 1 outline?
        mergedLayeredPolys=layeredPolygons[0]
        for layer in range(1,len(layeredPolygons)):
            mergedPolys=[]
            for poly in layeredPolygons[layer]:
                mergedSubPolys=[]
                for poly2 in mergedLayeredPolys:
                    if poly.almost_equals(poly2) or poly.contains(poly2):
                        mergedSubPolys.append(poly)
                    elif poly2.contains(poly):
                        mergedSubPolys.append(poly2)
                    elif poly.intersects(poly2):
                        mergedSubPolys.append(unary_union([poly, poly2]))
                    mergedPolys.append(unary_union(mergedSubPolys))
            mergedLayeredPolys=mergedPolys
            #for i in range(len(mergedPolys)):
            #    for j in range(i):
            #        if mergedPolys[i].intersects(mergedPolys[j]):
            #            mergedPolys.append(unary_union([mergedPolys[i], mergedPolys[j]]))

        self.outlinesList=[]
        self.holesList=[]
        for poly in mergedLayeredPolys:
            sections=[poly]
            paths=section.discrete
            outlines=section.root
            for i, shape in enumerate(paths):
                if i in outlines:
                    self.outlinesList.append(Polygon(tuple(shape)))
                else:
                    self.holesList.append(Polygon(tuple(shape)))
        holes=[]
        contours=[]
        for poly in self.holesList:
            holes.append(self.getContour(poly, 'left',offset))

        for poly in self.outlinesList:
            contours.append(self.getContour(poly, 'right', offset))

        polys=[]
        for j in range(len(contours)):
            polys.append(Polygon(tuple(zip(contours[j][0][0], contours[j][1][0]))))
        mergedPoly=unary_union(polys)

        vertices=PolygonPath(mergedPoly).vertices
        outputX=[]
        outputY=[]

        for hole in holes:
            outputX.append(hole[0][0])
            outputY.append(hole[1][0])
        if False:
            #this method can generate disjunct areas. These must be split in the following in order to have proper toolpaths.
            out_array = [np.where(np.vstack([np.round(vertices[:,0],8),np.round(vertices[:,1],8)]).T == element)[0].tolist() for element in np.unique(np.vstack([np.round(vertices[:,0],8),np.round(vertices[:,1],8)]).T)]
            out_array=[elem for elem in out_array if len(elem)==2  and elem[1]-elem[0]>1]
            out_array.sort(key=lambda x: x[1])

            outIndices=[0]
            for i in range(len(out_array)-1):
                if out_array[i][1]!=0 and out_array[i][1]<len(vertices)-1 and np.sqrt((vertices[:,0][out_array[i][1]]-vertices[:,0][out_array[i][1]+1])**2+(vertices[:,1][out_array[i][1]]-vertices[:,1][out_array[i][1]+1])**2)>offset/2.:
                    outputX.append(vertices[:,0][outIndices[-1]:out_array[i][1]])
                    outputY.append(vertices[:,1][outIndices[-1]:out_array[i][1]])
                    outIndices.append(out_array[i][1]+1)
            outputX.append(vertices[:,0][outIndices[-1]:])
            outputY.append(vertices[:,1][outIndices[-1]:])
        else:
            outputX.append(vertices[:,0])
            outputY.append(vertices[:,1])
        velocity=np.array([1])
        toolpath=np.array([[0,0,0]])
        for layer in range(1,layers+1):
            layerDepth=-depth*(layer/layers)
            for xpath, ypath in zip(outputX,outputY):
                if len(xpath)==0:
                    continue
                velocity=np.append(velocity,np.array([1,1]))
                toolpath=np.vstack((toolpath,np.array([[toolpath[-1][0],toolpath[-1][1],safetyDistance],[xpath[0]*scalingFactor,ypath[0]*scalingFactor,safetyDistance]])))#[toolpath[-1][0],toolpath[-1][1],safetyDistance],[xpath[0],ypath[0],safetyDistance]
                velocity=np.append(velocity,np.zeros(len(xpath)))
                toolpath=np.vstack((toolpath,np.dstack((np.asarray(xpath)*scalingFactor,np.asarray(ypath)*scalingFactor,np.full(len(xpath),layerDepth)))[0]))
                velocity=np.append(velocity,0)
                toolpath=np.vstack((toolpath,np.array([xpath[0]*scalingFactor,ypath[0]*scalingFactor,layerDepth])))
                velocity=np.append(velocity,0)
                toolpath=np.vstack((toolpath,np.array([xpath[0]*scalingFactor,ypath[0]*scalingFactor,safetyDistance])))
        velocity=np.append(velocity,np.array([1,0]))
        toolpath=np.vstack((toolpath,np.array([[0,0,safetyDistance],[0,0,0]])))
        return toolpath,velocity

    def getContour(self, poly, side, offset):
        list=[[],[]]
        line=LineString(PolygonPath(poly).vertices)
        contour=[line.parallel_offset(offset, side, join_style=2)]
        for part in contour:#get offset paths
            x, y = part.xy
            list[0].append(x)
            list[1].append(y)
        return list
