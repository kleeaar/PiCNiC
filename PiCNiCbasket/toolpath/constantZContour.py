import trimesh
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely import affinity
from shapely.ops import unary_union
from PiCNiCbasket.thirdParty.polyObjects import PolygonPath, PolygonPatch
import PiCNiCbasket.toolpath.svgToContours as svgToContours


class constantZContour():
    def __init__(self, xOffset, yOffset, toolDiameter, xMax, yMax):
        self.xOffset=xOffset
        self.yOffset=yOffset
        self.xMax=xMax
        self.yMax=yMax
        self.shape=None
        self.toolDiameter=toolDiameter
        self.offsetX=[]
        self.offsetY=[]

    def setShape(self, outlines, holes):
        self.outlines=outlines
        self.holes=holes

    def makeOuterContours(self, depth, scalingFactor=1, safetyDistance=5, layers=1):
        side='right'
        self.contoursX=[]
        self.contoursY=[]
        for outline in self.outlines:
            offset=self.toolDiameter/2.
            self.contoursX.append([])
            self.contoursY.append([])
            shape=outline #affinity.scale(outline,xfact=scalingFactor,yfact=scalingFactor)

            line=LineString(tuple(shape.exterior.coords))
            plt.plot(line.xy[0],line.xy[1])


            boxOutline=Polygon([((-self.xOffset+self.toolDiameter/2.)/scalingFactor, (-self.yOffset+self.toolDiameter/2.)/scalingFactor), (self.xMax+(self.xOffset-self.toolDiameter/2.)/scalingFactor, (-self.yOffset+self.toolDiameter/2.)/scalingFactor), (self.xMax+(self.xOffset-self.toolDiameter/2.)/scalingFactor, self.yMax+(self.yOffset-self.toolDiameter/2.)/scalingFactor), ((-self.xOffset+self.toolDiameter/2.)/scalingFactor, self.yMax+(self.yOffset-self.toolDiameter/2.)/scalingFactor)])
            stop=False
            while not stop:
                contour=[line.parallel_offset(offset/scalingFactor, side, join_style=1)]
                for part in contour:#get offset paths
                    subparts=[]
                    if part.type=='MultiLineString':
                        for subpart in list(part):
                            subparts.append(subpart)
                    else:
                        subparts.append(part)
                    for p, subpart in enumerate(subparts):
                        if subpart.is_empty:
                            continue
                        x, y = subpart.xy
                        x=np.asarray(x)
                        y=np.asarray(y)
                        try:
                            contourPoly=Polygon(tuple(zip(x, y)))
                            if boxOutline.within(contourPoly):
                                stop=True
                                break
                            intersection=contourPoly.intersection(boxOutline)
                            vertices=PolygonPath(intersection).vertices
                            self.contoursX[-1].append(vertices[:,0])
                            self.contoursY[-1].append(vertices[:,1])
                        except:
                            continue

                offset+=self.toolDiameter/2.

        outputX=[]
        outputY=[]

        self.contoursCounter=np.zeros(len(self.contoursX))

        lengths=[]
        for elem in self.contoursX:
            lengths.append(len(elem))
        for c in range(max(lengths)):
            self.exclusions=[]
            contourPoly=Polygon(tuple(zip(self.contoursX[0][min(c,len(self.contoursX[0])-1)], self.contoursY[0][min(c,len(self.contoursY[0])-1)])))
            outline=self.getUnion(contourPoly, max(lengths))
            if outline is not None:
                vertices=PolygonPath(outline).vertices
                #this method can generate disjunct areas. These must be split in the following in order to have proper toolpaths.
                out_array = [np.where(np.vstack([np.round(vertices[:,0],8),np.round(vertices[:,1],8)]).T == element)[0].tolist() for element in np.unique(np.vstack([np.round(vertices[:,0],8),np.round(vertices[:,1],8)]).T)]
                out_array=[elem for elem in out_array if len(elem)==2  and elem[1]-elem[0]>1]
                out_array.sort(key=lambda x: x[1])

                outIndices=[0]
                if False:
                    for i in range(len(out_array)-1):
                        if out_array[i][1]!=0 and out_array[i][1]<len(vertices)-1 and np.sqrt((vertices[:,0][out_array[i][1]]-vertices[:,0][out_array[i][1]+1])**2+(vertices[:,1][out_array[i][1]]-vertices[:,1][out_array[i][1]+1])**2)>self.toolDiameter/2.:
                            outputX.append(vertices[:,0][outIndices[-1]:out_array[i][1]])
                            outputY.append(vertices[:,1][outIndices[-1]:out_array[i][1]])
                            outIndices.append(out_array[i][1]+1)
                outputX.append(vertices[:,0][outIndices[-1]:])
                outputY.append(vertices[:,1][outIndices[-1]:])
                plt.plot(vertices[:,0][outIndices],vertices[:,1][outIndices], '.r')

        for hole in self.holes:
            offset=self.toolDiameter/2
            shape=hole #affinity.scale(hole,xfact=scalingFactor,yfact=scalingFactor)

            line=LineString(tuple(shape.exterior.coords))
            plt.plot(line.xy[0],line.xy[1])
            stop=False
            while not stop:
                contour=[line.parallel_offset(offset, 'left', join_style=1)]
                for part in contour:#get offset paths
                    subparts=[]
                    if part.type=='MultiLineString':
                        for subpart in list(part):
                            subparts.append(subpart)
                    else:
                        subparts.append(part)
                    for p, subpart in enumerate(subparts):
                        if subpart.is_empty:
                            stop=True
                            break
                        x, y = subpart.xy
                        x=np.asarray(x)
                        y=np.asarray(y)
                        outputX.append(x)
                        outputY.append(y)
                offset+=self.toolDiameter/2.
        velocity=np.array([1])
        toolpath=np.array([[0,0,0]])
        for layer in range(1,layers+1):
            layerDepth=-depth*(layer/layers)
            for xpath,ypath in zip(outputX,outputY):
                velocity=np.append(velocity,np.array([1,0]))
                toolpath=np.vstack((toolpath,np.array([[toolpath[-1][0],toolpath[-1][1],safetyDistance],[xpath[0]*scalingFactor,ypath[0]*scalingFactor,safetyDistance]])))#[toolpath[-1][0],toolpath[-1][1],safetyDistance],[xpath[0],ypath[0],safetyDistance]
                velocity=np.append(velocity,np.zeros(len(xpath)))
                toolpath=np.vstack((toolpath,np.dstack((np.asarray(xpath)*scalingFactor,np.asarray(ypath)*scalingFactor,np.full(len(xpath),layerDepth)))[0]))
                velocity=np.append(velocity,0)
                toolpath=np.vstack((toolpath,np.array([xpath[0]*scalingFactor,ypath[0]*scalingFactor,layerDepth])))
                velocity=np.append(velocity,0)
                toolpath=np.vstack((toolpath,np.array([xpath[0]*scalingFactor,ypath[0]*scalingFactor,safetyDistance])))
        return toolpath, velocity

    def getUnion(self, shape, maxLenth):
        numberOfContours=len(self.contoursX)
        if len(self.exclusions)<numberOfContours-1:
            for j in range(1,numberOfContours):
                if j not in self.exclusions:
                    if self.contoursCounter[j]<max(maxLenth,len(self.contoursX[j])):
                        #print(contoursCounter,j, len(contoursX[j]),len(contoursY[j]))
                        contourPolyOfOtherElement=Polygon(tuple(zip(self.contoursX[j][min(int(self.contoursCounter[j]),len(self.contoursX[j])-1)], self.contoursY[j][min(int(self.contoursCounter[j]),len(self.contoursY[j])-1)])))
                        if shape.intersects(contourPolyOfOtherElement):
                            contourPoly=unary_union([shape,contourPolyOfOtherElement])
                            self.exclusions.append(j)
                            self.contoursCounter[j]+=1
                            return self.getUnion(contourPoly, maxLenth)
        return shape
