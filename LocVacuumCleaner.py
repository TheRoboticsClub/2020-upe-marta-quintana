import numpy as np
import math
import cv2

from HAL import HAL
from GUI import GUI

class MyAlgorithm:

    def __init__(self, pose3d, motors, laser, bumper, cv2, np, math):
        self.getPose3d = pose3d
        self.motors = motors
        self.laser = laser
        self.bumper = bumper
        self.cv2 = cv2
        self.np = np
        self.math = math
        PATH_TO_IMAGE = "RoboticsAcademy/exercises/vacuum_cleaner_loc/web-template/assets/img/map.png"
        
        #Image Processing Maps
        self.map_orig = self.cv2.imread(PATH_TO_IMAGE, self.cv2.IMREAD_GRAYSCALE)
        self.map_orig = self.cv2.resize(self.map_orig, (500, 500))
        kernel = self.np.ones((10, 10), self.np.uint8)
        self.mapE = self.cv2.erode(self.map_orig, kernel, iterations=1)
        self.mapEVis = self.cv2.erode(self.map_orig, kernel, iterations=2)
        self.mapECopy = self.mapE.copy()
        self.mapEVisCopy = self.mapEVis.copy()
        
        #Useful Constants
        self.SCALE = 50.00 #50 px = 1 m
        self.VACUUM_PX_SIZE = 12
        self.VACUUM_PX_HALF = self.VACUUM_PX_SIZE /2
        self.VACUUM_SIZE = 0.34
        self.COLOR_VIRTUAL_OBST = 128
        self.MIN_MAP = 24
        self.MAX_MAP = 476
        self.STEP = 0.10  
        self.NUM_RETURN = 0

		#Useful Variables
        self.x = None
        self.y = None
        self.yaw = None
        self.xPix = None
        self.yPix = None
        self.minDist = None
        self.goTo =  None
        
        self.goSouth = False
        self.goingReturnPoint = False
        self.endSearch = False

        self.currentCell = []
        self.nextCell = []
        self.returnPoints = []
        self.path = []
        self.returnPoint = []
        self.returnPath = []
        self.visPoints = []
 
    def parse_laser_data(self,laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = self.math.radians(i)
            laser += [(dist, angle)]
        return laser
    
    
    def laser_vector(self,laser_array):
        laser_vectorized = []
        for d,a in laser_array:
            x = d * self.math.cos(a) * -1
            y = d * self.math.sin(a) * -1 
            v = (x, y)
            laser_vectorized += [v]
        return laser_vectorized
         
    def sweep(self):
        if self.x == None and self.y == None:
            #First Position
            self.x = self.getPose3d().x
            self.y = self.getPose3d().y
            self.xPix, self.yPix = self.coord2pix(self.x, self.y)
            self.currentCell = [self.xPix, self.yPix]
            self.savePath(self.currentCell)
            self.nextCell = self.currentCell
            self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
        else:
            neighbors = self.calculateNeigh(self.currentCell)
            cells = self.checkNeigh(neighbors)
            self.isReturnPoint(cells)
            if self.goingReturnPoint == False:
                if self.isCriticalPoint(cells):
                    self.NUM_RETURN = self.NUM_RETURN + 1
                    self.stopVacuum()
                    self.goSouth = False
                    self.checkReturnPoints()
                    if len(self.returnPoints) > 0:
                        self.returnPoint = self.checkMinDist(self.returnPoints, self.currentCell)
                        self.goingReturnPoint = True
                    else:
                        exit()
                else:
                    self.driving(cells, neighbors)
            else:
                arrive = self.checkArriveCell(self.returnPoint)
                if arrive == False:
                    self.goToReturnPoint() 
                else:
                    self.goingReturnPoint = False
                    self.returnPath = []
                    self.currentCell = self.returnPoint
                    self.savePath(self.currentCell)
                    self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
        

    def RTy(self, angle, tx, ty, tz):
        RT = self.np.matrix([[self.math.cos(angle), 0, self.math.sin(angle), tx], [0, 1, 0, ty], [-self.math.sin(angle), 0, self.math.cos(angle), tz], [0,0,0,1]])
        return RT
        

    def RTVacuum(self):
        RTy = self.RTy(self.math.pi, 5.8, 4.2, 0)
        return RTy
        
    
    def coord2pix(self, coordX, coordY):
        RTVacuum = self.RTVacuum()
        origPoses = self.np.matrix([[coordX], [coordY], [1], [1]])
        finalPoses = RTVacuum * origPoses * self.SCALE
        xPix = finalPoses.flat[0]
        yPix = finalPoses.flat[1]
        return xPix, yPix
        
    
    def pix2coord(self, xPix, yPix):
        RTVacuum = self.RTVacuum()
        RTinv = self.np.linalg.inv(RTVacuum)
        origPoses = self.np.matrix([[xPix/(self.SCALE)], [yPix/(self.SCALE)], [1], [1]]) 
        finalPoses = RTinv * origPoses
        x = finalPoses.flat[0]
        y = finalPoses.flat[1]
        return x, y
    
    
    def paintCell(self, cell, color, img):
        cell = [int(cell[0]), int(cell[1])]
        for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
            for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                img[i][j] = color             
        
                        
    def paintMap(self): 
        if len(self.path) > 0:
            for cell in self.path:
                self.paintCell(cell, self.COLOR_VIRTUAL_OBST, self.mapECopy)
                        
        if len(self.returnPoints) > 0:
            for cell in self.returnPoints:
                self.paintCell(cell, 85, self.mapECopy)  
        
        if self.nextCell != []:
            self.paintCell(self.nextCell, 180, self.mapECopy)  
 
        if self.currentCell != []:
            self.paintCell(self.currentCell, 150, self.mapECopy)   
              
        if self.returnPoint != []:
            self.paintCell(self.returnPoint, 30, self.mapECopy)
            
        if self.nextCell != []:
            self.paintPoint(self.nextCell, 10, self.mapECopy)
        
        if self.returnPoint != []:
            self.paintPoint(self.returnPoint, 100, self.mapECopy)
        
        if len(self.returnPath) > 0:
            for cell in self.returnPath:
                self.paintPoint(cell, 70, self.mapECopy)
        
        if self.x != None and self.y != None:
            x,y = self.coord2pix(self.x,self.y)
            pose = [x, y]
            self.paintPoint(pose, 220, self.mapECopy)
        
    
    def paintPoint(self, point, color, img):
        point = [int(point[0]),int(point[1])]
        img[point[1]][point[0]] = color
        img[point[1]-1][point[0]] = color
        img[point[1]+1][point[0]] = color
        img[point[1]][point[0]-1] = color
        img[point[1]][point[0]+1] = color
        img[point[1]-1][point[0]-1] = color
        img[point[1]-1][point[0]+1] = color
        img[point[1]+1][point[0]+1] = color
        img[point[1]+1][point[0]-1] = color
        
    
    def showMap(self): 
        self.cv2.imshow("MAPE", self.mapE) 
            
    def zigzag(self, cells, neighbors):
        nCell = cells[0]
        eCell = cells[1]
        wCell = cells[2]
        sCell = cells[3]
        north = neighbors[0]
        east = neighbors[1]
        west = neighbors[2]
        south = neighbors[3]
        
        if self.goSouth == False:
            if nCell == 0: 
                self.nextCell = north
                self.goTo = 'north'
            else:
                if sCell == 0: 
                    self.nextCell = south
                    self.goTo = 'south'
                    self.goSouth = True 
                elif eCell == 0: 
                    self.nextCell = east
                    self.goTo = 'east'
                    self.goSouth = True 
                elif wCell == 0: 
                    self.nextCell = west
                    self.goTo = 'west'
                    self.goSouth = True                                          
        else:
            if sCell == 0: 
                self.nextCell = south
                self.goTo = 'south'      
            else:
                self.goSouth = False
          
                      
    def calculateNeigh(self, cell):
        if cell[0] != None and cell[1] != None:
            if cell[1] >= self.MIN_MAP:
                northCell = [cell[0], cell[1] - self.VACUUM_PX_SIZE] 
            else:
                northCell = [None, None]
                
            if cell[1] <= self.MAX_MAP:
                southCell = [cell[0], cell[1] + self.VACUUM_PX_SIZE] 
            else:
                southCell = [None, None]
               
            if cell[0] >= self.MIN_MAP:
                westCell = [cell[0] - self.VACUUM_PX_SIZE, cell[1]] 
            else:
                westCell = [None, None]
                
            if cell[0] <= self.MAX_MAP:
                eastCell = [cell[0] + self.VACUUM_PX_SIZE, cell[1]] 
            else:
                eastCell = [None, None]
        else:  
            northCell = [None, None]
            southCell = [None, None]
            westCell = [None, None]
            eastCell = [None, None]
        neighbors = [northCell, eastCell, westCell, southCell]   
        return neighbors
    
    
    def checkCell(self, cell): 
        obstacle = 0
        virtualObst = 0
        c = None
        if cell[0] != None and cell[1] != None:
            cell = [int(cell[0]), int(cell[1])]
            for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
                for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                    if self.mapE[i][j] == 0:
                        #obstacle
                        obstacle = 1
                    elif self.mapE[i][j] == self.COLOR_VIRTUAL_OBST:
                        #virtual obstacle
                        virtualObst = 1                                                
            if obstacle == 1:
                c = 1
            elif virtualObst == 1:
                c = 2
            else:
                c = 0                           
        return c
        
        
    def checkNeigh(self, neighbors):
        north = neighbors[0]
        northCell = self.checkCell(north) 
        
        east = neighbors[1]
        eastCell = self.checkCell(east) 

        west = neighbors[2]
        westCell = self.checkCell(west) 

        south = neighbors[3]
        southCell = self.checkCell(south) 
        
        cells = [northCell, eastCell, westCell, southCell] 
        return cells
        
        
    def isReturnPoint(self, cells):
        nCell = cells[0]
        eCell = cells[1]
        wCell = cells[2]
        sCell = cells[3]
        if nCell == 0 :
            self.savePoint(self.currentCell, self.returnPoints)
        if eCell == 0:
            self.savePoint(self.currentCell, self.returnPoints)
        if wCell == 0:
            self.savePoint(self.currentCell, self.returnPoints)
        if sCell == 0:
            self.savePoint(self.currentCell, self.returnPoints) 
             
             
    def savePoint(self, p, l):
        saved = False
        for i in range(len(l)): 
            if (l[i][0] == p[0]) and (l[i][1] == p[1]):
                saved = True
        if saved == False:
            l.append(p)
            
                  
    def checkReturnPoints(self):
        cont = 0
        for i in range(len(self.returnPoints)): 
            neighbors = self.calculateNeigh(self.returnPoints[i])
            cells = self.checkNeigh(neighbors)       
            nCell = cells[0]
            eCell = cells[1]
            wCell = cells[2]
            sCell = cells[3]
            if nCell == 0:
                cont += 1
            if eCell == 0:
                cont += 1
            if wCell == 0:
                cont += 1
            if sCell == 0:
                cont += 1
            
            if cont == 0:
                self.returnPoints.pop(i)
                self.checkReturnPoints()
                break
            cont = 0
         
             
    def euclideanDist(self, p1, p2):
        d = self.math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))     
        return d


    def checkMinDist(self, points, cell):
        minDist = None
        for i in points:
            d = self.euclideanDist(cell, i)
            if minDist == None or d < minDist:
                nextCell = i
                minDist = d
        return nextCell
           

    def isCriticalPoint(self, cells):
        nCell= cells[0]
        eCell= cells[1]
        wCell= cells[2]
        sCell= cells[3]
        critical = False
        if nCell > 0 or nCell == None:
            if eCell > 0 or eCell == None:
                if wCell > 0 or wCell == None:
                    if sCell > 0 or sCell == None:
                        critical = True      
               
        return critical
 
 
    def savePath(self, cell):
        self.path.append(cell)

    
    def driving(self, cells, neighbors):
        if self.nextCell == self.currentCell:
            self.zigzag(cells, neighbors)                  
        else:
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goToCell(self.nextCell)  
            else:
                self.currentCell = self.nextCell
                self.savePath(self.currentCell)
                self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
            
           
    def goToCell(self, cell):
        self.x = self.getPose3d().x
        self.y = self.getPose3d().y
        self.yaw = self.getPose3d().yaw
        poseVacuum = [round(self.x, 2), round(self.y, 2)]
        nextNCell = self.calculateNeigh(cell)[0]
        nextECell = self.calculateNeigh(cell)[1]
        nextWCell = self.calculateNeigh(cell)[2]
        nextSCell = self.calculateNeigh(cell)[3]
        xc, yc = self.pix2coord(cell[0], cell[1])
        cell = [round(xc, 2),round(yc, 2)]
        desviation = self.calculateDesv(poseVacuum, cell)
        self.controlDrive(desviation)
                 
            
    def calculateDesv(self, poseVacuum, cell):
        x, y = self.abs2rel(cell, poseVacuum, self.yaw)
        desv = self.math.degrees(self.math.atan2(y,x))
        desv = round(desv, 1)
        return desv


    def abs2rel(self, target, poseVacuum, yaw):
        dx = target[0] - poseVacuum[0]
        dy = target[1] - poseVacuum[1]
        x = dx*self.math.cos(-yaw) - dy*self.math.sin(-yaw)
        y = dx*self.math.sin(-yaw) + dy*self.math.cos(-yaw)
        return x,y
 
    
    def controlDrive(self, desv):
        wFast = 0.30
        wSlow = 0.15
        if desv <= 0:
        	#Turn Left
            self.controlDesv(desv, wFast, wSlow)
        else:
        	#Turn Right
            self.controlDesv(desv, -wFast, -wSlow)
       
       
    def controlDesv(self, desv, wFast, wSlow): 
        desv = abs(desv) 
        th1 = 4
        th2 = 13
        v = 0.1
        if desv >= th2:
            self.motors.sendV(0)
            self.motors.sendW(wFast)
        elif desv >= th1:
            self.motors.sendV(v)
            self.motors.sendW(wSlow)
        else:
            v = self.setV()
            self.motors.sendV(v)
            self.motors.sendW(0)
                        
    
    def setV(self):
        vMax = 0.75
        vFast = 0.50
        vMed = 0.25
        vSlow = 0.19
        
        v = vSlow
         
        dMax = 1.5
        dMed = 0.8
        dMin = 0.4
            
        if self.goingReturnPoint == False:
            if self.goTo == 'north' or self.goTo == 'south':
                cells = self.calculateNext4C(self.currentCell, self.goTo)
                c = self.checkNext4C(cells)
                c1 = c[0] 
                c2 = c[1]
                c3 = c[2]
                c4 = c[3]
                if c4 != None and (c1 + c2 + c3 + c4) == 0: 
                    v = vMax
                elif c3 != None and (c1 + c2 + c3) == 0:
                    v = vFast
                elif c2 != None and (c1 + c2) == 0:
                    v = vMed
                elif c1 != None and c1 == 0:
                    v = vSlow 
        else:
            pose = [self.x, self.y]
            xRet, yRet = self.pix2coord(self.nextCell[0], self.nextCell[1])
            returnPoint = [xRet, yRet]
            d = self.euclideanDist(pose, returnPoint) 
            if d <= dMin:
                v = vSlow 
            elif d <= dMed: 
                v = vMed
            elif d <= dMax: 
                v = vFast
            else:
                v = vMax 
        return v
        
        
    def calculateNext4C(self, cell, direction):
        if direction == 'north':
            cell1 = self.calculateNeigh(cell)[0]
            cell2 = self.calculateNeigh(cell1)[0]
            cell3 = self.calculateNeigh(cell2)[0]
            cell4 = self.calculateNeigh(cell3)[0]
        elif direction == 'south':
            cell1 = self.calculateNeigh(cell)[3]
            cell2 = self.calculateNeigh(cell1)[3]  
            cell3 = self.calculateNeigh(cell2)[3] 
            cell4 = self.calculateNeigh(cell3)[3]         
        cells = [cell1, cell2, cell3, cell4]      
        return cells
            
    
    def checkNext4C(self, cells):
        c1 = self.checkCell(cells[0])
        c2 = self.checkCell(cells[1])
        c3 = self.checkCell(cells[2])
        c4 = self.checkCell(cells[3])       
        c = [c1, c2, c3, c4]
        return c
        
                            
    def checkArriveCell(self, cell):
        north = self.checkCell(self.calculateNeigh(cell)[0])
        south = self.checkCell(self.calculateNeigh(cell)[3])
        dMin = 0.067
        dMax = 0.32
        dist = dMax
        if self.goTo == 'east' or self.goTo == 'west' or (north != 0 and self.goTo == 'north') or (south != 0 and self.goTo == 'south') or self.goingReturnPoint == True:
            dist = dMin
                    
        x = False
        y = False
        xc, yc = self.pix2coord(cell[0], cell[1])
        xdif = abs(xc - self.x)
        ydif = abs(yc - self.y)
        if xdif <= dist:
            x = True
        if ydif <= dist:
            y = True
        if x == True and y == True:
            arrive = True
            if dist == dMin:
                self.stopVacuum()
        else:
            arrive = False
        return arrive
        
    
    def stopVacuum(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        
   
    def goToReturnPoint(self):
        length = len(self.returnPath)
        if length > 0:
            self.nextCell = self.returnPath[length-1]
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goToCell(self.nextCell)  
            else:
                self.currentCell = self.nextCell
                self.returnPath.pop(length-1)
        else:
            self.savePoint(self.returnPoint, self.returnPath)
            visPoseRet = self.visibility(self.currentCell, self.returnPoint)   
            if visPoseRet == False:
                self.searchReturnPath(self.returnPoint)
            
            
            
    def searchReturnPath(self, cell):
        for i in range(len(self.path)-1, -1, -1):
            newCell = self.path[i]
            if cell != newCell:
                vis = self.visibility(cell, newCell)
                if vis == True:
                    self.savePoint(newCell, self.returnPath)
                    break
        if vis == True:
            vis1 = self.visibility(self.currentCell, newCell)
            if vis1 == False:
                self.searchReturnPath(newCell)           
    
    
    def pointOfLine(self, A, B):
        s = self.step(A, B)
        xp = A[0] + s*(B[0] - A[0])
        yp = A[1] + s*(B[1] - A[1]) 
        return [xp, yp]
        
        
    def step(self, A, B):
        distMax = self.euclideanDist(A, B)
        step = self.STEP / distMax 
        return step
        
      
    def numSteps(self, A, B):
        distMax = self.euclideanDist(A, B)
        numSteps =  distMax / self.STEP         
        d = numSteps - int(numSteps)
        if d > 0:
            numSteps = numSteps + 1
        return int(numSteps)

    
    def visibility(self, A, B):
        visibility = True
        nextPoint = []
        A = self.pix2coord(A[0], A[1])
        B = self.pix2coord(B[0], B[1])
        self.paintPoint(A, 128, self.mapEVisCopy)
        self.paintPoint(B, 128, self.mapEVisCopy)
        numSteps = self.numSteps(A,B)
        if nextPoint == []:
            nextPoint = A
        for i in range(0, numSteps):
            P = self.pointOfLine(nextPoint, B)
            pPix = self.coord2pix(P[0],P[1])
            self.paintPoint(pPix, 128, self.mapEVisCopy)
            obst = self.isObstacle(pPix)
            if obst == True:
                visibility = False
                break
            else:
                nextPoint = P
        return visibility
        
        
    def isObstacle(self, P):
        P = [int(P[0]), int(P[1])]
        if self.mapEVis[P[1]][P[0]] == 0:
            obst = True
        else:
            obst = False
        return obst

        
    def execute(self):

        # TODO 
               
        self.sweep()
        self.paintMap()
        #self.showMap()

algorithm = MyAlgorithm(HAL.getPose3d, HAL.motors, HAL.laser, HAL.bumper, cv2, np, math)

while True:
    algorithm.execute()