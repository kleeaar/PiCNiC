import numpy as np
import sys
import concurrent.futures
from scipy import interpolate
import PiCNiCbasket.misc.utils as utils

#maybe use something better like trimesh?

class lineScan:
    def __init__(self, fFacets, fStepSize, fCoarseStepSize,toolDiameter):
        self.mFacets=fFacets
        self.mStepSize=fStepSize
        self.mCoarseStepSize=fCoarseStepSize
        self.mNumberOfFacets=len(self.mFacets)
        self.mCurrentDirection=1
        self.mToolDiameter=toolDiameter
        self.mCurrectFacetPointsToUse=np.zeros(2)
        self.mCurrentPosition=np.zeros(3)
        self.mStartPoint=np.zeros(3)
        self.mOutputPoints=np.empty((0,3))
        self.mSliceDistance=0
        self.mNumbersOfOutputPoints=0
        self.mGeometryProperties={"xMin":0,
                                "xMax":0,
                                "yMin":0,
                                "yMax":0,
                                "zMin":0,
                                "zMax":0,
                                "width":0,
                                "depth":0,
                                "height":0,
                                "diagonalXY":0}
        self.findPartDimensions()
        print("initialized with ", self.mNumberOfFacets, " facets")

    def findPartDimensions(self):
        self.mGeometryProperties["xMin"]=np.amin(self.mFacets[...,0][...,0])-self.mToolDiameter
        self.mGeometryProperties["xMax"]=np.amax(self.mFacets[...,0][...,0])+self.mToolDiameter
        self.mGeometryProperties["yMin"]=np.amin(self.mFacets[...,1][...,1])-self.mToolDiameter
        self.mGeometryProperties["yMax"]=np.amax(self.mFacets[...,1][...,1])+self.mToolDiameter
        self.mGeometryProperties["zMin"]=np.amin(self.mFacets[...,2][...,2])
        self.mGeometryProperties["zMax"]=np.amax(self.mFacets[...,2][...,2])
        self.mGeometryProperties["width"]=self.mGeometryProperties["xMax"]-self.mGeometryProperties["xMin"]
        self.mGeometryProperties["depth"]=self.mGeometryProperties["yMax"]-self.mGeometryProperties["yMin"]
        self.mGeometryProperties["height"]=self.mGeometryProperties["zMax"]-self.mGeometryProperties["zMin"]
        self.mGeometryProperties["diagonalXY"]=np.sqrt(self.mGeometryProperties["width"]**2+self.mGeometryProperties["depth"]**2)

    def interSectionWithFacet(self,facet, fCurrentPoint, axis="z"):
        #TODO: first Check if line is parallel
        if axis=="z":
            dX = fCurrentPoint[0]-facet[2][0]
            dY = fCurrentPoint[1]-facet[2][1]
            dX21 = facet[2][0]-facet[1][0]
            dY12 = facet[1][1]-facet[2][1]
            D = dY12*(facet[0][0]-facet[2][0]) + dX21*(facet[0][1]-facet[2][1])
            s = dY12*dX + dX21*dY
            t = (facet[2][1]-facet[0][1])*dX + (facet[0][0]-facet[2][0])*dY
            if D<0 :
                return s<=0 and t<=0 and s+t>=D
            return s>=0 and t>=0 and s+t<=D
        if axis=="y":
            dY = fCurrentPoint[1]-facet[2][1]
            dZ = fCurrentPoint[2]-facet[2][2]
            dY21 = facet[2][1]-facet[1][1]
            dZ12 = facet[1][2]-facet[2][2]
            D = dZ12*(facet[0][1]-facet[2][1]) + dY21*(facet[0][2]-facet[2][2])
            s = dZ12*dY + dY21*dZ
            t = (facet[2][2]-facet[0][2])*dY + (facet[0][1]-facet[2][1])*dZ
            if D<0 :
                return s<=0 and t<=0 and s+t>=D
            return s>=0 and t>=0 and s+t<=D

    def pol1Parameters(self, pt1, pt2):#calculates the parameters for the line equation of two points in the xy plane
        slope=(pt2[1]-pt1[1])/(pt2[0]-pt1[0]) if (pt2[0]-pt1[0])!=0 else np.nan
        return np.array([pt1[1]-slope*pt1[0],slope])

    def pol1ParametersXZPlane(self, pt1, pt2):#calculates the parameters for the line equation of two points in the xz plane
        slope=(pt2[2]-pt1[2])/(pt2[0]-pt1[0]) if (pt2[0]-pt1[0])!=0 else np.nan
        return np.array([pt1[2]-slope*pt1[0],slope])

    def findIntersectionPoints(self,facet, position): #return the two intersections of the facets boarders along direction vector
        fpermutationOrder=np.array([[0,1],[1,2],[2,0]])
        fDirectionVectors=np.empty((0,3))
        fIntersections=np.empty((0,3))
        for i, order in enumerate(fpermutationOrder):
            fDirectionVectors=np.vstack((fDirectionVectors,facet[order[1]]-facet[order[0]]))
            if fDirectionVectors[-1][1]!=0:
                fDistance=(position[1]-facet[order[0]][1])/fDirectionVectors[-1][1]
                if fDistance >=0 and  fDistance<=1:
                    fIntersections=np.vstack((fIntersections,facet[order[0]]+fDistance*fDirectionVectors[-1]))
        return fIntersections

    def normalVector(self,facet):#calculates normal Vector
        P01=facet[1]-facet[0]
        P02=facet[2]-facet[0]
        P01xP02=np.cross(P01,P02)
        norm=np.linalg.norm(P01xP02)
        return P01xP02/norm

    def dropEmptySlices(self,paths):
        idx=[]
        for layerId,layer in enumerate(paths):
            for i,slice in enumerate(layer):
                if len(slice[0])==0:
                    idx.append(i)
            for i,id in enumerate(idx):
                del paths[layerId][id-i]
        return paths

    def multiThreadedLineScan(self,spacing,fNumberOfLayers,fAlternateDirection=False):
        self.mSliceDistance=spacing
        self.mCurrentPosition=np.array([self.mGeometryProperties["xMin"]-self.mCoarseStepSize,self.mGeometryProperties["yMin"]-spacing,0])
        #number of slices
        fNumberOfSlices=int((self.mGeometryProperties["depth"]+spacing)//spacing+1)
        fLayers=np.flip(np.linspace(self.mGeometryProperties["zMin"],self.mGeometryProperties["zMax"],int(fNumberOfLayers+1))[:-1])
        output=[[np.empty((0,3))]*fNumberOfSlices]*fNumberOfLayers
        output=[]
        for layer, zPos in enumerate(fLayers):
            output.append([])
            with concurrent.futures.ProcessPoolExecutor() as executor:
                for slice,out1 in enumerate(executor.map(self.lineScanAtSlice, zip(np.arange(0,fNumberOfSlices,1),[zPos]*(fNumberOfSlices),[fAlternateDirection]*(fNumberOfSlices)))):
                    # put results into correct output list
                    output[-1].append(out1)
        output=self.dropEmptySlices(output)
        return output

    def hasIntersectingFacet(self,fFacetSubset,fCurrentPosition):
        for facet in self.mFacets[fFacetSubset==1]:
            if self.interSectionWithFacet(facet,fCurrentPosition) and self.getIntersectionPoint(facet,fCurrentPosition)[2]>=fCurrentPosition[2]:
                return True
        return False

    def findCorrectIntersectionPoint(self,fFacetSubset,fCurrentPosition,fCurrentDirection): #performs fine scan and returns intersection points with facets
        fCurrentPosition[0]-=fCurrentDirection*self.mCoarseStepSize
        fNoFacetFound=True
        fOutputPoints=np.empty((0,3))
        zPos=fCurrentPosition[2]
        while fNoFacetFound:
            for facet in self.mFacets[fFacetSubset==1]:
                if self.normalVector(facet)[2]<=0:
                    continue #continue if normal is not pointing upwards
                fIntersectionPoint=self.getIntersectionPoint(facet,fCurrentPosition)
                if self.interSectionWithFacet(facet,fCurrentPosition) and  fIntersectionPoint[2]>=zPos:
                    #find two intersection points with facet
                    fIntersections=self.findIntersectionPoints(facet,fCurrentPosition)
                    #jumpt to next intersection point; do not increase by stepsize
                    fCurrentPosition=fIntersections[0] if fCurrentDirection*fIntersections[0][0]>fCurrentDirection*fIntersections[1][0] else fIntersections[1]
                    fCurrentPosition[0]+=fCurrentDirection*self.mStepSize

                    if fCurrentDirection*fIntersections[0][0]>fCurrentDirection*fIntersections[1][0]: #check which point if first
                        fFacetFractionAboveZPos=(zPos-fIntersections[0][2])/(fIntersections[1][2]-fIntersections[0][2])if (fIntersections[1][2]-fIntersections[0][2]!=0 and fIntersections[1][2]<=zPos)  else 1 #determine fraction of facet that it is below the layer's z position
                        fIntersectionWithZPos=fIntersections[0]+fFacetFractionAboveZPos*(fIntersections[1]-fIntersections[0])
                        fOutputPoints=np.vstack((fOutputPoints,fIntersectionWithZPos))
                        fOutputPoints=np.vstack((fOutputPoints,fIntersections[0]))
                    else:
                        fFacetFractionAboveZPos=(zPos-fIntersections[1][2])/(fIntersections[0][2]-fIntersections[1][2])if (fIntersections[1][2]-fIntersections[0][2]!=0 and fIntersections[0][2]<=zPos)  else 1 #determine fraction of facet that it is below the layer's z position
                        fIntersectionWithZPos=fIntersections[1]+fFacetFractionAboveZPos*(fIntersections[0]-fIntersections[1])
                        fOutputPoints=np.vstack((fOutputPoints,fIntersectionWithZPos))
                        fOutputPoints=np.vstack((fOutputPoints,fIntersections[1]))
                    fNoFacetFound=False
                    return fOutputPoints
            fCurrentPosition[0]+=fCurrentDirection*self.mStepSize

    def getFacetSubset(self,fYPos):#returns index of facets that intersect with y-value
        indx=np.zeros(self.mNumberOfFacets, dtype="i")
        for i,facet in enumerate(self.mFacets):
            if facet[0][1]<=fYPos and (facet[1][1]>=fYPos or facet[2][1]>=fYPos) or facet[1][1]<=fYPos and (facet[0][1]>=fYPos or facet[2][1]>=fYPos) or facet[2][1]<=fYPos and (facet[0][1]>=fYPos or facet[1][1]>=fYPos):
                    indx[i]=1
        return indx

    def lineScanAtSlice(self,params):#performs a scan at given y-value
        slice=params[0]
        zPos=params[1]
        fAlternateDirection=params[2]
        fYPos=(self.mGeometryProperties["yMin"]-self.mSliceDistance/2.)+self.mSliceDistance*slice
        fCurrentPosition=np.array([self.mGeometryProperties["xMin"]-self.mCoarseStepSize if slice%2==0 or fAlternateDirection==False else self.mGeometryProperties["xMax"]+self.mCoarseStepSize, fYPos, zPos])
        fCurrentDirection=1 if slice%2==0 or fAlternateDirection==False else -1
        fOutputPoints=np.empty((0,3))
        noIntersectionAtLastStep=True
        fFacetSubset=self.getFacetSubset(fYPos) #find facet subset

        while (fCurrentPosition[0]<=self.mGeometryProperties["xMax"]+self.mCoarseStepSize and fCurrentDirection>0) or (fCurrentPosition[0]>=self.mGeometryProperties["xMin"]-self.mCoarseStepSize and fCurrentDirection<0):
            fNoFacetFound=True
            #find Intesection with facet
            if noIntersectionAtLastStep:
                if self.hasIntersectingFacet(fFacetSubset,fCurrentPosition):
                    #now go back one Step and perform fine Scan
                    fIntersections=self.findCorrectIntersectionPoint(fFacetSubset,fCurrentPosition,fCurrentDirection)
                    fOutputPoints=np.vstack((fOutputPoints[:-1],np.array([fIntersections[0][0],fIntersections[0][1],zPos])))
                    fOutputPoints=np.vstack((fOutputPoints,fIntersections))
                    noIntersectionAtLastStep=False
                    fNoFacetFound=False
                else:
                    #increase by coarse stepsize
                    fCurrentPosition[0]+=fCurrentDirection*self.mCoarseStepSize
            else:
                #Todo: make own function
                for facet in self.mFacets[fFacetSubset==1]:
                    if self.normalVector(facet)[2]<=0:
                        continue #continue if normal is not pointing upwards
                    fIntersectionPoint=self.getIntersectionPoint(facet,fCurrentPosition)
                    if self.interSectionWithFacet(facet,fCurrentPosition) and fIntersectionPoint[2]>=zPos:
                        #find two intersection points with facet
                        fIntersections=self.findIntersectionPoints(facet,fCurrentPosition)
                        #jumpt to next intersection point; do not increase by stepsize
                        fCurrentPosition=fIntersections[0] if fCurrentDirection*fIntersections[0][0]>fCurrentDirection*fIntersections[1][0] else fIntersections[1]
                        fCurrentPosition[0]+=fCurrentDirection*self.mStepSize
                        if fCurrentDirection*fIntersections[0][0]>fCurrentDirection*fIntersections[1][0]:
                            if np.linalg.norm(fOutputPoints[-1]-fIntersections[1])>self.mStepSize:
                                fOutputPoints=np.vstack((fOutputPoints,fIntersections[1]))
                            fFacetFractionAboveZPos=(zPos-fIntersections[1][2])/(fIntersections[0][2]-fIntersections[1][2]) if (fIntersections[1][2]-fIntersections[0][2]!=0 and fIntersections[0][2]<=zPos) else 1#determine fraction of facet that it is below the layer's z position
                            fIntersectionWithZPos=fIntersections[1]+fFacetFractionAboveZPos*(fIntersections[0]-fIntersections[1])
                            fOutputPoints=np.vstack((fOutputPoints,fIntersectionWithZPos))
                        else:
                            if np.linalg.norm(fOutputPoints[-1]-fIntersections[0])>self.mStepSize:
                                fOutputPoints=np.vstack((fOutputPoints,fIntersections[0]))
                            fFacetFractionAboveZPos=(zPos-fIntersections[0][2])/(fIntersections[1][2]-fIntersections[0][2])if (fIntersections[1][2]-fIntersections[0][2]!=0 and fIntersections[1][2]<=zPos)  else 1 #determine fraction of facet that it is below the layer's z position
                            fIntersectionWithZPos=fIntersections[0]+fFacetFractionAboveZPos*(fIntersections[1]-fIntersections[0])
                            fOutputPoints=np.vstack((fOutputPoints,fIntersectionWithZPos))
                        fNoFacetFound=False
                        break
            if fNoFacetFound:
                if noIntersectionAtLastStep==False:
                    fOutputPoints=np.vstack((fOutputPoints,np.array([fOutputPoints[-1][0],fOutputPoints[-1][1],zPos])))
                fOutputPoints=np.vstack((fOutputPoints,fCurrentPosition))
                noIntersectionAtLastStep=True

            fCurrentPosition[2]=zPos
            fCurrentPosition[0]+=fCurrentDirection*self.mStepSize
            self.mNumbersOfOutputPoints+=len(fOutputPoints)
        return fOutputPoints

    def performScan(self,spacing,fNumberOfLayers,fAlternateDirection=False):
        self.mSliceDistance=spacing
        self.mCurrentPosition=np.array([self.mGeometryProperties["xMin"]-self.mCoarseStepSize,self.mGeometryProperties["yMin"]-spacing,0])
        print("Starting at ",self.mCurrentPosition)
        #number of slices
        fOutput=np.empty((0,3))
        fNumberOfSlices=int((self.mGeometryProperties["depth"]+spacing)//spacing+1)
        fLayers=np.flip(np.linspace(self.mGeometryProperties["zMin"],self.mGeometryProperties["zMax"],int(fNumberOfLayers+1))[:-1])
        print(fLayers)
        output=[[np.empty((0,3))]*fNumberOfSlices]*fNumberOfLayers
        for layer,zPos in enumerate(fLayers):
            for slice in range(fNumberOfSlices):
                output[layer][slice]=np.vstack((output[layer][slice],self.lineScanAtSlice([slice,zPos,fAlternateDirection])))
        return output

    def dropPointsOnSameZlevel(self,points, flatten=False):
        fNumberOfLayers=len(points)
        fNumberOfSlices=len(points[0])
        print(fNumberOfLayers,fNumberOfSlices)
        output=[[np.empty((0,3))]*fNumberOfSlices]*fNumberOfLayers
        for layer in range(fNumberOfLayers):
            for slice in range(fNumberOfSlices):
                if len(points[layer][slice])==0:
                    continue
                output[layer][slice]=np.vstack((output[layer][slice],points[layer][slice][0]))
                for i in range(1,len(points[layer][slice])):
                    if np.abs(points[layer][slice][i][2]-output[layer][slice][-1][2])<1e-5:# and np.abs(points[layer][slice][i][1]-output[layer][slice][-1][1])<1e-5:
                        continue
                    if not np.all(output[layer][slice][-1]==points[layer][slice][i-1]):
                        output[layer][slice]=np.vstack((output[layer][slice],points[layer][slice][i-1]))
                    output[layer][slice]=np.vstack((output[layer][slice],points[layer][slice][i]))
                output[layer][slice]=np.vstack((output[layer][slice],points[layer][slice][-1]))
        if flatten: #flatten output to a array containing all points
            return self.flatten(output)
        return np.asarray(output)

    def flatten(self,path):
        fFlattenedOutput=np.empty((0,3))
        for layer in path:
            for slice in layer:
                fFlattenedOutput=np.vstack((fFlattenedOutput,slice))
        return fFlattenedOutput

    def getIntersectionPoint(self,facet,fCurrentPosition):#calculates Intersection Point with facet
        normVec=self.normalVector(facet)
        planeVal=np.dot(normVec,facet[0])
        tmp1=normVec[0]*fCurrentPosition[0]+normVec[1]*fCurrentPosition[1]+normVec[2]*fCurrentPosition[2]
        tmp2=normVec[2]
        l1=(planeVal-tmp1)/tmp2
        return np.array([fCurrentPosition[0],fCurrentPosition[1],fCurrentPosition[2]+l1])

    def interpolateContour(self,points,stepSize,diameter):
        fNumberOfLayers=len(points)
        fNumberOfSlices=len(points[0])
        output=[[np.empty((0,2))]*fNumberOfSlices]*fNumberOfLayers
        xInterpolated=np.linspace(self.mGeometryProperties['xMin']['mm']-diameter/2,self.mGeometryProperties['xMax']['mm']+diameter/2,stepSize)
        for layerIdx,layer in enumerate(points):
            for i, slice in enumerate(layer):
                f = interpolate.interp1d(slice[...,0], slice[...,2])
                zInterpolated=f(xInterpolated)
                output[layerIdx][slice]=np.columnStack(xInterpolated,zInterpolated)

    def interpolateSlice(self,points,stepSize,diameter,shift):
        xMin=self.mGeometryProperties['xMin']-diameter/2.-shift
        xMax=self.mGeometryProperties['xMax']+diameter/2.-shift
        xInterpolated=np.arange(xMin,xMax,stepSize)
        x=np.concatenate(([xMin],points[...,0],[xMax]))
        z=np.concatenate(([points[0][2]],points[...,2],[points[-1][2]]))
        f = interpolate.interp1d(x, z)
        zInterpolated=f(xInterpolated)

        return np.dstack((xInterpolated,zInterpolated))[0]

    def makeToolPath(self,fContour,safetyDistance=5):
        fSlicesToCheck=self.mToolDiameter/2.//self.mSliceDistance
        output=[]
        stepSize=0.01
        xMin=self.mGeometryProperties['xMin']-self.mToolDiameter/2.
        xMax=self.mGeometryProperties['xMax']+self.mToolDiameter/2.
        xInterpolated=np.arange(xMin,xMax,stepSize)
        xInterpolatedInverse=np.flip(xInterpolated)
        for layerIdx,layer in enumerate(fContour):
            output.append([])
            for i, slice in enumerate(layer):
                if len(slice)==0:
                    continue
                fFirstSliceIndx= int(max(i-fSlicesToCheck,0))
                fLastSliceIndx= int(min(len(layer-1),i+fSlicesToCheck)+1)
                fSliceYpos=slice[0][1]

                interpolations=[]
                for l,neighboringSlice in enumerate(fContour[layerIdx][fFirstSliceIndx:fLastSliceIndx]):
                    fNeighboringSliceYpos=neighboringSlice[0][1]
                    fToolOffset=np.sqrt((self.mToolDiameter/2.)**2-(fNeighboringSliceYpos-fSliceYpos)**2)#offset from drill diameter
                    for offset in np.arange(-fToolOffset,fToolOffset,0.01):
                        interpolations.append(np.interp(xInterpolated+offset, neighboringSlice[...,0], neighboringSlice[...,2]))
                        interpolations.append(np.interp(xInterpolated-offset, neighboringSlice[...,0], neighboringSlice[...,2]))
                zInterpolatedMax=np.amax(interpolations,axis=0)
                if i%2==1:
                    x=xInterpolatedInverse
                    zInterpolatedMax=np.flip(zInterpolatedMax)
                else:
                    x=xInterpolated
                output[-1].append(np.dstack((x,[fSliceYpos]*len(xInterpolated),zInterpolatedMax))[0])

            #add origin and safetydistances to path
            output[-1][0]=np.vstack((np.array([output[-1][0][0][0],output[-1][0][0][1],self.mGeometryProperties['zMax']+safetyDistance]),output[-1][0]))
            output[-1][-1]=np.vstack((output[-1][-1],np.array([output[-1][-1][-1][0],output[-1][-1][-1][1],self.mGeometryProperties['zMax']+safetyDistance])))
        output[0][0]=np.vstack((np.array([self.mGeometryProperties['xMin']+self.mToolDiameter,self.mGeometryProperties['yMin']+self.mToolDiameter,self.mGeometryProperties['zMax']]),output[0][0]))
        output[-1][-1]=np.vstack((output[-1][-1],np.array([self.mGeometryProperties['xMin']+self.mToolDiameter,self.mGeometryProperties['yMin']+self.mToolDiameter,self.mGeometryProperties['zMax']+safetyDistance]),np.array([self.mGeometryProperties['xMin']+self.mToolDiameter,self.mGeometryProperties['yMin']+self.mToolDiameter,self.mGeometryProperties['zMax']])))
        return output

    def makeToolPathTest(self,fContour):
        fSlicesToCheck=self.mToolDiameter/2.//self.mSliceDistance
        output=[[np.empty((0,3))]*len(fContour[0])]*len(fContour)
        stepSize=0.01
        xMin=self.mGeometryProperties['xMin']-self.mToolDiameter/2.
        xMax=self.mGeometryProperties['xMax']+self.mToolDiameter/2.
        xInterpolated=np.arange(xMin,xMax,stepSize)
        for layerIdx,layer in enumerate(fContour):
            for i, slice in enumerate(layer):
                zInterpolated=np.interp(xInterpolated, slice[...,0], slice[...,2])
                y=np.array([slice[0][1]]*len(xInterpolated))
                output[layerIdx][i]=np.dstack((xInterpolated,y,zInterpolated))
        return output
