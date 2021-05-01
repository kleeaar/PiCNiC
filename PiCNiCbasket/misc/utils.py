import numpy as np
def isClockwiseContour(path):
    sum=0
    for point in range(len(path)):
        sum+=(path[point][0]-path[point+1][0])*(path[point][1]-path[point+1][1]) if point<len(path)-1 else (path[point][0]-path[0][0])*(path[point][1]-path[0][1])
    return sum>0

def removeIntersections(path):
    for point in range(len(path)):
        nextPoint=point+1 if point<len(path)-1 else 0
        base1=path[point]
        dir1=np.array([(path[point][0]-path[nextPoint][0]),(path[point][1]-path[nextPoint][1]),0])

        a1=dir1[1]/dir1[0] if dir1[0]!=0 else np.inf
        b1=base1[1]-a1*base1[0]

        for point2 in range(nextPoint+1,len(path)):#two following lines cannot intersect, therefore go to next segment
            nextPoint2=point2+1 if point2<len(path)-1 else 0
            base2=path[point2]
            dir2=np.array([(path[point2][0]-path[nextPoint2][0]),(path[point2][1]-path[nextPoint2][1]),0])
            a2=dir2[1]/dir2[0] if dir2[0]!=0 else np.inf
            b2=base2[1]-a2*base2[0]
            if np.sum(np.cross(dir1,dir2)**2)<1e-12:
                continue
            if a1==np.inf:
                y=a2*base1[0]+b2
                if ((y<base1[1] and y>path[nextPoint][1]) or (y>base1[1] and y<path[nextPoint][1])) and ((y<base2[1] and y>path[nextPoint2][1]) or (y>base2[1] and y<path[nextPoint2][1])):
                    print("Case1",point,point2,y,base1[1],path[nextPoint][1],base2[1],path[nextPoint2][1])
            elif a2==np.inf:
                y=a1*base2[0]+b1
                if ((y<base1[1] and y>path[nextPoint][1]) or (y>base1[1] and y<path[nextPoint][1])) and ((y<base2[1] and y>path[nextPoint2][1]) or (y>base2[1] and y<path[nextPoint2][1])):
                    print("Case2",point,point2,y,base1[1],path[nextPoint][1],base2[1],path[nextPoint2][1])
            else:
                x=(b2-b1)/(a1-a2)
                if ((x<base1[0] and x>path[nextPoint][0]) or (x>base1[0] and x<path[nextPoint][0])) and ((x<base2[0] and x>path[nextPoint2][0]) or (x>base2[0] and x<path[nextPoint2][0])):
                    print("Case3",point,point2,x,base1[0],path[nextPoint][0],base2[0],path[nextPoint2][0])

def removeSameDirectionSteps(toolpath):
    finalToolPath=np.empty((0,3))
    finalToolPath=np.vstack((finalToolPath,toolpath[0]))
    step=1
    while step <len(toolpath)-1:
        directionVector=toolpath[step]-toolpath[step-1]
        while (np.abs(toolpath[step+1]-toolpath[step]-directionVector)<1e-9).all():
            step+=1
        finalToolPath=np.vstack((finalToolPath,toolpath[step]))
        step+=1
    finalToolPath=np.vstack((finalToolPath,toolpath[-1]))
    return finalToolPath

def shiftToolpath(toolpath,dir):
    return toolpath+np.array([dir]*len(toolpath))

def makeToolPathFromSvg(paths,toolDiameter,depth,safetyDistance=5,layers=1):
    isInnerContour=detectInnerContours(paths)
    points=np.empty((0,3))
    finalPath=np.empty((0,3))
    for path, isInner in zip(paths,isInnerContour):
        #addOffset
        if not isClockwiseContour(path):
            path=np.flip(path,0)
        isFirstPoint=True
        for point in range(len(path)):
            skipPoint=False
            nextPoint=point+1 if point<len(path)-1 else 0
            if path[point][0]==path[nextPoint][0] and path[point][1]==path[nextPoint][1]: #check if points are the same (sometimes happens for rirst an dlast point)
                continue
            dx=(path[point][0]-path[nextPoint][0])
            dy=(path[point][1]-path[nextPoint][1])
            #calculate perpendicular
            if dx==0:
                perpVec=np.array([1,0,0])
            elif dy==0:
                perpVec=np.array([0,1,0])
            else:
                alpha=np.arctan2(dy,dx)
                perpVec=np.array([np.cos(alpha+np.pi/2.),np.sin(alpha+np.pi/2.),0])

            corssProduct=np.cross(np.array([dx,dy,0]),perpVec)
            directionSign=np.sign(corssProduct[2]) if isInner==1 else np.sign(corssProduct[2])*-1
            offsetPoint=path[point]+directionSign*toolDiameter/2.*perpVec
            #check if point intersects with contour
            for point2 in range(len(path)):
                if np.sqrt(np.sum((offsetPoint-path[point2])**2))<toolDiameter/2.-0.001:
                    skipPoint=True
            if skipPoint: continue
            offsetPoint[2]=depth
            if isFirstPoint:
                points=np.vstack((points,np.array([offsetPoint[0],offsetPoint[1],safetyDistance])))
                isFirstPoint=False
            if (np.abs(points[-1]-offsetPoint)>1e-9).any(): #do not add point if similar to previous
                points=np.vstack((points,offsetPoint))

            offsetPoint=path[nextPoint]+directionSign*toolDiameter/2.*perpVec
            #check if point intersects with contour
            for point2 in range(len(path)):
                if np.sqrt(np.sum((offsetPoint-path[point2])**2))<toolDiameter/2.-0.001:
                    skipPoint=True
            if skipPoint: continue
            lastNonSkippedPoint=offsetPoint
            offsetPoint[2]=depth
            if (np.abs(points[-1]-offsetPoint)>1e-9).any(): #do not add point if similar to previous
                points=np.vstack((points,offsetPoint))
        points=np.vstack((points,np.array([lastNonSkippedPoint[0],lastNonSkippedPoint[1],safetyDistance])))
    points=removeSameDirectionSteps(points)
    #finalizePath: add path from/to origin and add layers
    finalPath=np.vstack((finalPath,np.array([0,0,safetyDistance])))
    finalPath=np.vstack((finalPath,np.array([points[0][0],points[0][1],safetyDistance])))
    for layerZ in np.linspace(depth/layers,depth,layers):
        print(layerZ)
        for point in points:
            if point[2]<0:
                point[2]=layerZ
                #print(point)
            finalPath=np.vstack((finalPath,point))
    finalPath=np.vstack((finalPath,np.array([0,0,safetyDistance])))
    finalPath=np.vstack((finalPath,np.array([0,0,0])))
    finalPath=removeSameDirectionSteps(finalPath)
    return finalPath

def detectInnerContours(paths):
    isInnerContour=np.empty(0)
    for i,path in enumerate(paths):
        if len(path)==0:
            isInnerContour=np.append(isInnerContour,-1)
            continue
        base1=path[0]
        numberOfIntersections=0
        for j,otherPath in enumerate(paths):
            if j==i:
                continue
            for point in range(len(otherPath)):
                base2=otherPath[point]
                direction=otherPath[point]-otherPath[point+1] if point<len(otherPath)-1 else otherPath[point]-otherPath[0]
                m=(base1[1]-base2[1])/direction[1] if direction[1]!=0 else np.inf
                intersectionPoint=base2+m*direction
                if m<1 and m>=0 and intersectionPoint[0]>base1[0]:
                    numberOfIntersections+=1
        isInnerContour=np.append(isInnerContour,numberOfIntersections%2!=0)
    return isInnerContour
