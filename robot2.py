import math
import utils
import struct
import geometry
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def readData(self):
        self.heading = self.get_compass_heading()*180/math.pi
        self.robot_pos = self.get_gps_coordinates()
        if(self.name[0] == 'B'):
            self.robot_pos[0] *= -1
            self.robot_pos[1] *= -1
        self.sonar = self.get_sonar_values()
        if self.is_new_ball_data():
            self.isBall = True
            self.ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(self.ball_data['direction'][1], self.ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.0166666/(abs(self.ball_data['direction'][2])/math.sqrt(1 - self.ball_data['direction'][2]**2)))
            self.ball_x =-math.sin((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y = math.cos((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            self.ball_pos = [self.ball_x, self.ball_y]
        else:
            self.isBall = False
        self.robot_x = self.robot_pos[0]
        self.robot_y = self.robot_pos[1]
        self.behind_ball = [self.ball_x, self.ball_y - 0.2]
    def moveToAngle(self, angle):
        if angle > 180: angle -= 360
        if angle <-180: angle += 360
        if -90 < angle < 90:
            if angle > 40:
                self.right_motor.setVelocity(10)
                self.left_motor.setVelocity(-10)
            elif angle <-40:
                self.right_motor.setVelocity(-10)
                self.left_motor.setVelocity(10)
            else:
                self.right_motor.setVelocity(utils.velocity(10 + angle/3))
                self.left_motor.setVelocity(utils.velocity(10 - angle/3))
        else:
            if angle < 0: angle = -180 - angle
            elif angle > 0: angle =  180 - angle
            if angle > 40:
                self.right_motor.setVelocity(-10)
                self.left_motor.setVelocity(10)
            elif angle <-40:
                self.right_motor.setVelocity(10)
                self.left_motor.setVelocity(-10)
            else:
                self.right_motor.setVelocity(utils.velocity(-10 - angle/3))
                self.left_motor.setVelocity(utils.velocity(-10 + angle/3))
    def move(self, dest, wait=False):
        dest_angle = math.atan2(self.robot_pos[0]-dest[0],dest[1]-self.robot_pos[1])*180/math.pi
        angle = self.heading - dest_angle
        if utils.getDistance(self.robot_pos, self.ball_pos) < 0.2 and wait:
            self.stop()
        else:
            self.moveToAngle(angle)
    def lookAtPos(self, dest):
        dest_angle = math.atan2(self.robot_pos[0]-dest[0],dest[1]-self.robot_pos[1])*180/math.pi
        angle = self.heading - dest_angle
        if angle > 180: angle -= 360
        if angle <-180: angle += 360
        if -90 < angle < 90:
            self.right_motor.setVelocity(utils.velocity(angle/5))
            self.left_motor.setVelocity(utils.velocity(-angle/5))
        else:
            if angle < 0: angle = -180 - angle
            elif angle > 0: angle =  180 - angle
            self.right_motor.setVelocity(utils.velocity(-angle/5))
            self.left_motor.setVelocity(utils.velocity(angle/5))
    def stop(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)
    def sendTeamData(self):
        packet = struct.pack(utils.dataFormat, self.robot_id, self.robot_x, self.robot_y, self.isBall, self.ball_x, self.ball_y)
        self.team_emitter.send(packet)
    def getTeamData(self):
        self.robot_positions[self.robot_id - 1][0] = self.robot_x
        self.robot_positions[self.robot_id - 1][1] = self.robot_y
        while self.is_new_team_data():
            packet = self.team_receiver.getData()
            self.team_receiver.nextPacket()
            unpacked = struct.unpack(utils.dataFormat, packet)
            self.robot_positions[unpacked[0] - 1][0] = unpacked[1]
            self.robot_positions[unpacked[0] - 1][1] = unpacked[2]
            if not self.isBall and unpacked[3]:
                self.ball_x = unpacked[4]
                self.ball_y = unpacked[5]
                self.ball_pos = [self.ball_x, self.ball_y]
                self.behind_ball = [self.ball_x, self.ball_y - 0.2]
                self.isBall = True
        distances = [0, 0, 0]
        distances[0] = utils.getDistance([self.robot_positions[0][0], self.robot_positions[0][1]], self.ball_pos)
        distances[1] = utils.getDistance([self.robot_positions[1][0], self.robot_positions[1][1]], self.ball_pos)
        distances[2] = utils.getDistance([self.robot_positions[2][0], self.robot_positions[2][1]], self.ball_pos)
        if distances[self.robot_id - 1] == max(distances):
            self.gaolKeeper = True
        else:
            self.gaolKeeper = False
        ############################# Define Passor
        if(self.name[1] == '1'):
            if((distances[0] > distances[1] and distances[0] < distances[2]) or (distances[0] < distances[1] and distances[0] > distances[2])):
                self.passor = True
        if(self.name[1] == '2'):
            if((distances[1] > distances[2] and distances[1] < distances[0]) or(distances[1] < distances[2] and distances[1] > distances[0])):
                self.passor = True
        if(self.name[1] == '3'):
            if((distances[2] > distances[0] or distances[2] < distances[1]) or(distances[2] < distances[0] or distances[2] > distances[1])):
                self.passor = True
        
        min_dist = min(distances)
        if(min_dist == distances[0] and self.name[1] == '1'):
            self.passor = False
        if(min_dist == distances[1] and self.name[1] == '2'):
            self.passor = False
        if(min_dist == distances[2] and self.name[1] == '3'):
            self.passor = False
        ############################# Lack off progress calculation
        if abs(self.last_ball_pos[0] - self.ball_x) < 0.08 and abs(self.last_ball_pos[1] - self.ball_y) < 0.08:
            if time.time() - self.last_ball_time > 3:
                self.kickoff = True
            else:
                self.kickoff = False
        else:
            self.last_ball_time = time.time()
            self.kickoff = False
        ############################# Corner Position Check for Pass
        if self.ball_y > 0.2 and (self.ball_x > 0.2 or self.ball_x < -0.2):
            self.corner = True
        else:
            self.corner = False        
        ############################# Avoid multi Passor for one robot inside pass position
        if(self.name[1] == '1'):
            if self.robot_positions[1][0] > -0.2 and self.robot_positions[1][0] < 0.2:
                self.extraPassor = True
            elif self.robot_positions[2][0] > -0.2 and self.robot_positions[2][0] < 0.2:
                self.extraPassor = True
            else:
                self.extraPassor = False
        elif(self.name[1] == '2'):
            if(self.robot_positions[0][0] > -0.2 and self.robot_positions[0][0] < 0.2):
                self.extraPassor = True
            elif(self.robot_positions[2][0] > -0.2 and self.robot_positions[2][0] < 0.2):
                self.extraPassor = True
            else:
                self.extraPassor = False
        elif(self.name[1] == '3'):
            if(self.robot_positions[1][0] > -0.2 and self.robot_positions[1][0] < 0.2):
                self.extraPassor = True
            elif(self.robot_positions[0][0] > -0.2 and self.robot_positions[0][0] < 0.2):
                self.extraPassor = True
            else:
                self.extraPassor = False
        ############################# Last position of the ball
        self.ball_speed = math.sqrt((self.ball_x - self.x2_ball)**2 + (self.ball_y - self.y2_ball)**2)/(time.time()-self.time1)
        self.x2_ball = self.ball_x
        self.y2_ball = self.ball_y
        self.time1 = time.time()
    def getMinDist(self, arr):
        minDist = 999999
        index = -1
        for i in range(len(arr)):
            d = math.sqrt((self.ball_x - arr[i]['x'])**2 + (self.ball_y - arr[i]['y'])**2)
            if d < minDist:
                minDist = d
                index = i
        return index
    def guessNeutralPoint(self):
        NEUTRAL_SPOTS = [
            {'y': 0,   'x': 0},
            {'y': -0.3,'x': -0.3},
            {'y': -0.2,'x': 0},
            {'y': -0.3,'x': 0.3},
            {'y': 0.3, 'x':  0.35},
            {'y': 0.2, 'x':  0},
            {'y': 0.3, 'x':  -0.35}
        ]
        nearest = self.getMinDist(NEUTRAL_SPOTS)
        return NEUTRAL_SPOTS[nearest]
    def defend(self):
        ############################# Goal Keeper kick off position in lack off progress
        if(self.kickoff):
            if(self.ball_y > 0):
                if(abs(-0.1 - self.robot_y) < 0.05 and abs(self.robot_x) < 0.03):
                    self.lookAtPos(self.O_Goal)
                    self.turnTimeout = 0
                else:
                    self.move([0, -0.1])
            else:
                if(abs(-0.4 - self.robot_y) < 0.05 and abs(self.robot_x) < 0.03):
                    self.lookAtPos(self.O_Goal)
                    self.turnTimeout = 0
                else:
                    self.move([0, -0.4])
        ############################# Goal Keeper
        else:
            m = 0
            if self.ball_y > self.last_ball_pos[1]:
                self.goalKeeper_x = self.ball_x
            elif self.ball_y - self.last_ball_pos[1] != 0:
                m = (self.ball_x - self.last_ball_pos[0])/(self.ball_y - self.last_ball_pos[1])
                self.goalKeeper_x = m*(self.T_Goal[1] - self.ball_y) + self.ball_x
            else:
                self.goalKeeper_x = self.ball_x
            if self.goalKeeper_x > 0.5: self.goalKeeper_x = 0.5
            if self.goalKeeper_x <-0.5: self.goalKeeper_x =-0.5
            self.move([self.goalKeeper_x , self.T_Goal[1]], True)
    def attack(self):
        ############################# Dangerus Position
        if self.ball_y < -0.55 and self.robot_y < -0.55:
            if self.robot_y > self.ball_y + 0.08:
                if self.ball_x > 0:
                    self.move([self.ball_x - 0.2, self.ball_y - 0.12])    
                else:
                    self.move([self.ball_x + 0.2, self.ball_y - 0.12])
            elif self.ball_x > 0:
                if self.robot_x < self.ball_x:
                    self.move([self.ball_x, self.ball_y - 0.05])
                else:
                    self.move([self.ball_x, self.ball_y + 0.2])
            else: 
                if(self.robot_x > self.ball_x):
                    self.move([self.ball_x, self.ball_y - 0.05])
                else:
                    self.move([self.ball_x, self.ball_y + 0.2])
        ############################# Lack off progress forward robot
        elif self.passor and self.neutralPos['x'] != 0 and self.neutralPos['y'] != 0 and self.kickoff:
            if abs(self.neutralPos['y'] - 0.12 - self.robot_y) < 0.05 and abs(self.neutralPos['x'] - self.robot_x) < 0.05:
                self.lookAtPos(self.O_Goal)
            else:
                self.move([self.neutralPos['x'], self.neutralPos['y'] - 0.12])
        ############################# Go to Pass position
        # elif self.passor and self.corner and not self.extraPassor:
        #     if abs((self.ball_y - 0.1) - self.robot_y) < 0.05 and abs(self.O_Goal[0] - self.robot_x) < 0.05:
        #         self.lookAtPos(self.O_Goal)
        #     else:
        #         self.move([self.O_Goal[0], self.ball_y - 0.1])
        ############################# kick the ball in Pass position
        # elif (not self.extraPassor) and self.x_robot > 0.2 and self.y_robot > -0.1 and self.y_robot < 0.1 and self.x_ball > self.x_robot - 0.1:
        #     if(self.ball_distance < 0.1 and abs(self.y_robot - self.y_ball) > 0.05):
        #         if(self.y_ball > 0):
        #             self.motor(10,-10)
        #         else:
        #             self.motor(-10,10)
        #     else:
        #         self.move([self.x_ball, 0])
        ############################# Run away from goal keeper lack off progress position
        elif(self.robot_y > self.ball_y and self.robot_y > 0.2 and self.robot_x > -0.2 and self.robot_x < 0.2):
            if(self.robot_x > self.ball_x):
                self.move([self.ball_x + 0.5, (self.robot_y + self.ball_y)/2])
            else:
                self.move([self.ball_x - 0.5, (self.robot_y + self.ball_y)/2])
        ############################# go behind the ball
        elif self.robot_y > self.ball_y:
            if(self.ball_x < -0.3):
                if(self.ball_y > 0):
                    self.move([self.ball_x - 0.2, self.ball_y - 0.12])
                else:
                    self.move([self.ball_x + 0.2, self.ball_y - 0.12])
            else:
                if(self.robot_x > self.ball_x):
                    self.move([self.ball_x + 0.2, self.ball_y - 0.12])
                else:
                    self.move([self.ball_x - 0.2, self.ball_y - 0.12])    
        ############################# Move towards the ball
        elif abs(self.robot_x - self.ball_x) > 0.1 and abs(self.robot_y - self.ball_y) > 0.1:
            self.move([self.ball_x, self.ball_y - 0.2])
        ############################# Push the ball
        else:
            self.move([self.ball_x, self.ball_y])
    def run(self):
        self.ball_x = 0
        self.ball_y = 0
        self.isBall = False
        self.T_Goal = [0, -0.68]
        self.O_Goal = [0, 0.68]
        self.ball_pos = [0, 0]
        self.robot_positions = [[0, 0] , [0, 0] , [0, 0]]
        self.robot_id = int(self.name[1])
        self.gaolKeeper = False
        self.goalKeeper_x = 0
        self.last_ball_pos = self.ball_pos
        self.role = 'forward'
        self.neutralPos = {'x': 0, 'y': 0}
        self.corner = False
        self.passor = False
        self.extraPassor = False
        self.cnt = 0
        self.kickoff = False
        self.turnTimeout = 0
        self.last_ball_time = time.time()
        self.x2_ball = 0
        self.y2_ball = 0
        self.time1 = time.time()
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.waitingForKick = self.get_new_data()['waiting_for_kickoff']
                self.readData()
                self.sendTeamData()
                self.getTeamData()
                self.neutralPos = self.guessNeutralPoint()
                if self.waitingForKick:
                    self.stop()
                elif not self.isBall:
                    if self.robot_id == 1:
                        self.move([ 0.3, -0.3], True)
                    elif self.robot_id == 2:
                        self.move([-0.3, -0.3], True)
                    else:
                        self.move([ 0,   -0.65], True)
                elif self.gaolKeeper:
                    self.defend()
                else:
                    self.attack()
                self.last_ball_pos = self.ball_pos