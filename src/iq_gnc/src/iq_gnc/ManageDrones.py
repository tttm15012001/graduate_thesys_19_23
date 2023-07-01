# Constant
TAKING_OFF = 'takingoff'
TAKEOFF = 'takeoff'
LANDING = 'landing'
LANDED = 'landed'
PRE_LANDING = 'pre_landing'
OUT = 'out'
MIN_DISTANCE = 500
MAX_DISTANCE = 1200
HEIGHT = 6
TIME_TO_LAND = 40
WAIT_TIME = 20

class manageDrones:
    def __init__(self, num):
        self.num = num
        self.dronesSignal = [False] * self.num
        self.droneIsReady = [False] * self.num
        self.requestTakeOff = [False] * self.num
        self.coordinatesAll = []
        self.dataSend = ''
        self.state = LANDED
        self.needPublish = False
        self.numBefore = 0
    
    def unReachAll(self):
        for drone in self.dronesSignal:
            drone = False
            print(drone)

    def isReachAll(self):
        for idx in range(0, self.numBefore, 1):
            if self.dronesSignal[idx] == False:
                return False
        return True
        # for drone in self.dronesSignal:
        #     print('signal: ' + str(drone))
        #     if drone == False:
        #         return False
        # return True
    
    def isReadyAll(self):
        for drone in self.droneIsReady:
            if drone == False:
                return False
        return True
    
    def updateSignal(self, index):
        self.dronesSignal[index] = True
        self.requestTakeOff[index] = False
        
    def updateReady(self, index):
        self.droneIsReady[index] = True
    
    def requestXDroneTakeOff(self, X):
        count = 0
        self.numBefore = X
        while count < X:
            self.requestTakeOff[count] = True
            count += 1
        while count < self.num:
            self.requestTakeOff[count] = False
            count += 1
    
    def createData(self, data):
        self.dataSend = data
        
    def checkEnough(self, num):
        if num != self.numBefore:
            return False
        return True
    
    def updateState(self, key, value):
        print(key + self.state)
        self.state = value
        print(key + self.state)
        
class coordinates:
    def __init__(self, coor_x, coor_y, coor_z) -> None:
        self.coor_x = coor_x
        self.coor_y = coor_y
        self.coor_z = coor_z
        self.coor_psi = 0