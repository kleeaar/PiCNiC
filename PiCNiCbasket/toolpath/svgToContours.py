import trimesh
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.geometry import Polygon
from PiCNiCbasket.thirdParty.polyObjects import PolygonPath, PolygonPatch
import PiCNiCbasket.toolpath.constantZContour as constantZContour

class svgToContours():
    def __init__(self, file, origin=None):
        self.mesh = trimesh.load(file)
        invertY=True
        if invertY:
            theta=np.pi/2.
            rotZ=[[1,0,0],[0,-1,0],[0,0,0]]
            self.mesh.apply_transform(rotZ)
            if origin=='Centered':
                self.mesh.apply_obb()
            elif origin=='Bottom Left':
                self.mesh.rezero()
        self.outlinesList=[]
        self.holesList=[]
        self.offsetX=[]
        self.offsetY=[]

    def getContours(self):
        contours=[]
        for section in [self.mesh]:
            if section is not None:
                paths=section.discrete
                for i, shape in enumerate(paths):
                    vertices=[]
                    for elem in shape:
                        vertices.append([elem[0],elem[1],0])
                    contours.append(vertices)
        return contours

    def getContour(self, poly, side, offset):
        line=LineString(PolygonPath(poly).vertices)
        contour=[line.parallel_offset(offset, side, join_style=2)]
        try:
            for part in contour:#get offset paths
                x, y = part.xy
                self.offsetX.append(x)
                self.offsetY.append(y)
        except Exception as e:
            print(e)

    def getPaths(self, offset):
        sections=[self.mesh]
        for section in sections:
            if section is not None:
                paths=section.discrete
                outlines=section.root
                for i, shape in enumerate(paths):
                    if i in outlines:
                        self.outlinesList.append(Polygon(tuple(shape)))
                    else:
                        self.holesList.append(Polygon(tuple(shape)))

        for poly in self.holesList:
            self.getContour(poly, 'left',offset)

        for poly in self.outlinesList:
            self.getContour(poly, 'right', offset)

    def makeOuterOffsets(self, depth, offsetX, offsetY, scalingFactor=1, toolDimater=0, safetyDistance=5, layers=1):
        sections=[self.mesh]
        for section in sections:
            if section is not None:
                paths=section.discrete
                outlines=section.root
                for i, shape in enumerate(paths):
                    if i in outlines:
                        self.outlinesList.append(Polygon(tuple(shape)))
                    else:
                        self.holesList.append(Polygon(tuple(shape)))

        outerContours=constantZContour.constantZContour(offsetX, offsetY, toolDimater, self.mesh.extents[0], self.mesh.extents[1])
        outerContours.setShape(self.outlinesList,self.holesList)
        toolpath=outerContours.makeOuterContours(depth, scalingFactor, safetyDistance, layers)
        toolpath=np.vstack((toolpath,np.array([[-offsetX+toolDimater/2.,-offsetY+toolDimater/2.,safetyDistance],[-offsetX+toolDimater/2.,-offsetY+toolDimater/2.,-depth],[self.mesh.extents[0]*scalingFactor+offsetX-toolDimater/2.,-offsetY+toolDimater/2.,-depth],[self.mesh.extents[0]*scalingFactor+offsetX-toolDimater/2.,self.mesh.extents[1]*scalingFactor+offsetY-toolDimater/2.,-depth],[-offsetX+toolDimater/2.,self.mesh.extents[1]*scalingFactor+offsetY-toolDimater/2.,-depth],[-offsetX+toolDimater/2.,-offsetY+toolDimater/2.,-depth],[-offsetX+toolDimater/2.,-offsetY+toolDimater/2.,safetyDistance]])))
        return toolpath

    def getContourToolpath(self, depth,scalingFactor=1,offset=0, safetyDistance=5, layers=1):
        self.getPaths(offset/scalingFactor)
        velocity=np.array([1])
        toolpath=np.array([[0,0,0]])
        for layer in range(1,layers+1):
            layerDepth=-depth*(layer/layers)
            for xpath, ypath in zip(self.offsetX,self.offsetY):
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
