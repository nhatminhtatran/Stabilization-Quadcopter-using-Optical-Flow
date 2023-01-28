from ast import While
from pickle import TRUE
import time
import curses
from collections import deque
from itertools import cycle
import numpy as np
import queue
from yamspy import MSPy
from threading import Thread
from read_altitude_node import read_altitude
#from optical_node import Optical
from threading import Event
import cv2
import imutils
from imutils.video import WebcamVideoStream
from kalmanfilter import SimpleKalmanFilter
from functools import reduce
# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...
NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
lower_height = 1
upper_height = 1.8 
throttle = 1000
tem_throttle = 0
ARMED = False
Kp_Throttle = 0
Ki_Throttle = 0
Kd_Throttle = 0
error_throttle = 0
PID_T = 0
desired_throttle = 0
altitude_value = 0
roll = 1000
pitch = 1000

# On Linux, your serial port will probably be something like
# /dev/ttyACM0 or /dev/ttyS0 or the same names with numbers different from 0
#
# On Windows, I would expect it to be 
# COM1 or COM2 or COM3...
#
# This library uses pyserial, so if you have more questions try to check its docs:
# https://pyserial.readthedocs.io/en/latest/shortintro.html
#
#
SERIAL_PORT = "/dev/ttyUSB0"

def run_curses(external_function):
    result=1

    try: 
        # get the curses screen window
        screen = curses.initscr()

        # turn off input echoing
        curses.noecho()

        # respond to keys immediately (don't wait for enter)
        curses.cbreak()

        # non-blocking
        screen.timeout(0)

        # map arrow keys to special values
        screen.keypad(True)

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'm' to change mode, 'a' to arm, 'd' to disarm and arrow keys to control", curses.A_BOLD)
        
        result = external_function(screen)

    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def keyboard_controller(screen):
    global CMDS, CMDS_ORDER , ARMED , throttle , desired_throttle

    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'yaw':      1500, # yaw
            'throttle': 1000, # throttle
            'CH5':     1000, # arm, disarm
            'CH6':     1000,
            'CH7':     1000,
            'CH8':     1000 
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.q
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes CH5, CH6...
    
    #CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw',  'CH5', 'CH6']
    CMDS_ORDER =  ['roll', 'pitch', 'throttle', 'yaw', 'CH5', 'CH6','CH7','CH8']

    # "print" doesn't work with curses, use addstr instead
    controller = Controller()
    
    try:
        screen.addstr(15, 0, "Connecting to the FC...")
        

        with MSPy(device=SERIAL_PORT, loglevel='DEBUG', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if board.INAV:
                command_list.append('MSPV2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            if board.INAV:
                cellCount = board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0 # MSPV2_INAV_ANALOG is necessary
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            #screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            #screen.clrtoeol()
            #screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            #screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            #screen.addstr(16,50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            #screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))
            #screen.addstr(17, 50, "name: {}".format(board.CONFIG['name']))


            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            take_off_flags = False
            take_off_lock = False
            PID_altitude_flag = False
            hovering_flag = False
            f1 = open("altitude_final_13.txt","a")
            f3 = open("attitude_final_4.txt","a")
            km = SimpleKalmanFilter(1, 1, 0.01)
            #global altitude_value
            while True:
                start_time = time.time()
                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                altitude_value = altitude_thread.distance_kalman
                screen.addstr(8, 50, "Altitude: {}".format(round(altitude_value,2)))
                screen.clrtoeol() 
                board.fast_read_attitude()
                roll_value = board.SENSOR_DATA['kinematics'][0]
                roll_value_kalman = km.update(roll_value)
                pitch_value = board.SENSOR_DATA['kinematics'][1]
                pitch_value_kalman = km.update(pitch_value)
                yaw_value = board.SENSOR_DATA['kinematics'][2]
                yaw_value_kalman = km.update(yaw_value)
                screen.addstr(13, 0, f"Roll : {roll_value_kalman} ; Pitch : {pitch_value_kalman} , Yaw : {yaw_value_kalman}") 
                screen.clrtoeol()
                f3.write("roll:" + str(round(roll_value_kalman,2)) + "pitch" + str(round(pitch_value_kalman,2)) + "yaw" +str(round(yaw_value_kalman,2)))
                f3.write('\n') 
                screen.addstr(17, 0, "Len: {}".format(len(format(board.process_mode(board.CONFIG['mode']))) ) ) 
                screen.clrtoeol()
                if(altitude_value >= 0.3):
                    controller.pid_throttle(altitude_value, board.ANALOG['voltage'] )
                    if hovering_flag == True:
                        controller.pid_roll_pitch(optical_thread.ROLL_DRIFT_SUM, optical_thread.PITCH_DRIFT_SUM, altitude_value)


                    #f.write("altitude: " + str(altitude_thread.distance/100))
                    #f.write('\n') 
                    
                f1.write("altitude:" + str(altitude_value) )
                
                f1.write('\n') 
                #f.write("throttle:" + str(throttle) )
                #f.write('\n') 

                screen.addstr(17,0, "PID-Throttle : {}".format(PID_T))
                screen.clrtoeol()
                screen.addstr(17,50, "Throttle : {}".format(throttle))
                screen.clrtoeol()
                screen.addstr(18,0, "PID-Roll : {}".format(PID_R))
                screen.clrtoeol()
                screen.addstr(18,50, "Roll : {}".format(roll))
                screen.clrtoeol()
                screen.addstr(19,0, "PID-Pitch : {}".format(PID_P))
                screen.clrtoeol()
                screen.addstr(19,50, "Pitch : {}".format(pitch))
                screen.clrtoeol()



                #
                # Key input processing
                #

                #
                # KEYS (NO DELAYS)
                #
                if char == ord('q') or char == ord('Q'):
                    break

                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['CH5'] = 1000  
                
                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    #exit_event_altitude.set()
                    #altitude_thread.terminate()
                    time.sleep(0.5)
                    break

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['CH5'] = 1500
                    take_off_flags = True
                    
                    #if(len(format(board.process_mode(board.CONFIG['mode']))) == 7):
                        #take_off_flags = True
                elif char == ord('s') or char == ord('S'):
                    take_off_flags = False
                    cursor_msg = 'Locking command...'
                    CMDS['CH5'] = 1000
                elif char == ord('h') or char == ord('H'):
                    if take_off_flags:
                        
                        while CMDS['throttle'] <= 1520:
                            throttle = throttle + 10
                            CMDS['throttle'] = throttle
                            cursor_msg = 'Flying throttle:{}'.format(throttle)  
                        PID_altitude_flag = True
                        
                elif char == ord('o') or char == ord('O'):
                    hovering_flag = True

                elif char == ord('w') or char == ord('W'):
                    
                    if CMDS['throttle'] + 10 <= 1600 :
                        throttle = throttle + 10
                        CMDS['throttle'] = throttle
                            
                    else: 
                        CMDS['throttle']
                    cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])
                    


                    #if (altitude_thread.distance/100) > lower_height and (altitude_thread.distance/100)< upper_height:
                        #cursor_msg = 'Turn on ALTHOLD command...'
                        #CMDS['CH6'] = 1900
                #
                # The code below is expecting the drone to have the
                # modes set accordingly since everything is hardcoded.
                #
                elif char == ord('m') or char == ord('M'):
                    if CMDS['CH6'] <= 1300:
                        cursor_msg = 'Horizon Mode...'
                        CMDS['CH6'] = 1500
                    elif 1700 > CMDS['CH6'] > 1300:
                        cursor_msg = 'Flip Mode...'
                        CMDS['CH6'] = 2000
                    elif CMDS['CH6'] >= 1700:
                        cursor_msg = 'Angle Mode...'
                        CMDS['CH6'] = 1000

                
                        
                elif char == ord('e') or char == ord('E'):
                    CMDS['throttle'] = CMDS['throttle'] - 10 if CMDS['throttle'] - 10 >= 1000 else CMDS['throttle']
                    cursor_msg = 'E Key - throttle(-):{}'.format(CMDS['throttle'])
                    

                elif char == curses.KEY_RIGHT:
                    CMDS['roll'] = CMDS['roll'] + 1 if CMDS['roll'] + 1 <= 2000 else CMDS['roll']
                    cursor_msg = 'Right Key - roll(-):{}'.format(CMDS['roll'])

                elif char == curses.KEY_LEFT:
                    CMDS['roll'] = CMDS['roll'] - 1 if CMDS['roll'] - 1 >= 1000 else CMDS['roll']
                    cursor_msg = 'Left Key - roll(+):{}'.format(CMDS['roll'])

                elif char == curses.KEY_UP:
                    CMDS['pitch'] = CMDS['pitch'] + 1 if CMDS['pitch'] + 1 <= 2000 else CMDS['pitch']
                    cursor_msg = 'Up Key - pitch(+):{}'.format(CMDS['pitch'])

                elif char == curses.KEY_DOWN:
                    CMDS['pitch'] = CMDS['pitch'] - 1 if CMDS['pitch'] - 1 >= 1000 else CMDS['pitch']
                    cursor_msg = 'Down Key - pitch(-):{}'.format(CMDS['pitch'])

                elif char == ord('o') or char == ord('O'):
                    CMDS['yaw'] = CMDS['yaw'] + 1 if CMDS['yaw'] + 1 <= 2000 else CMDS['yaw']
                    cursor_msg = 'o Key - yaw(+):{}'.format(CMDS['yaw'])

                elif char == ord('p') or char == ord('P'):
                    CMDS['yaw'] = CMDS['yaw'] - 1 if CMDS['yaw'] - 1 >= 1000 else CMDS['yaw']
                    cursor_msg = 'p Key - yaw(-):{}'.format(CMDS['yaw'])
                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        
                        board.process_recv_data(dataHandler)

                #
                # SLOW MSG processing (user GUI)
                #
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()

                    next_msg = next(slow_msgs) # circular list

                    # Read info from the FC
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                    if next_msg == 'MSP_ANALOG':
                        voltage = board.ANALOG['voltage']
                        voltage_msg = ""
                        if min_voltage < voltage <= warn_voltage:
                            voltage_msg = "LOW BATT WARNING"
                        elif voltage <= min_voltage:
                            voltage_msg = "ULTRA LOW BATT!!!"
                        elif voltage >= max_voltage:
                            voltage_msg = "VOLTAGE TOO HIGH"

                        screen.addstr(8, 0, "Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                        screen.clrtoeol()
                        screen.addstr(8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = board.bit_check(board.CONFIG['mode'],0)
                        screen.addstr(5, 0, "ARMED: {}".format(ARMED), curses.A_BOLD)
                        screen.clrtoeol()

                        screen.addstr(5, 50, "armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                        screen.clrtoeol()

                        screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                        screen.clrtoeol()
                        screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                        screen.clrtoeol()

                        screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                        screen.clrtoeol()

                        screen.addstr(7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                        screen.clrtoeol()


                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(9, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_RC':
                        screen.addstr(10, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.clrtoeol()
                    
                    
 


                    screen.addstr(11, 0, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                                                                                                1/(sum(average_cycle)/len(average_cycle))))
                    screen.clrtoeol()

                    screen.addstr(3, 0, cursor_msg)
                    screen.clrtoeol()
                    

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()
class Controller():
    def __init__(self):
        self.previous_error_throttle = 0
        self.previous_error_roll = 0
        self.previous_error_pitch = 0
        self.ARMED = False
        self.array_throttle = []
        self.array_roll = []
        self.array_pitch = []
        
    def pid_throttle(self,altitude,voltage,setpoints= 1.35):
        global throttle, Kp_Throttle, Ki_Throttle , Kd_Throttle , error_throttle, PID_T, desired_throttle
        Kp_Throttle = 0.3
        Ki_Throttle = 0.0001
        Kd_Throttle = 1.5
        minPID = -1
        maxPID = 1
        self.setpoint = setpoints
        error_throttle = (self.setpoint - round(altitude,2))
        voltageHelper = float(12.3) - float(voltage)
        if voltageHelper < 0 :
            voltageHelper = 0
        elif voltageHelper > 0.5:
            voltageHelper = 1
        #if altitude == float(setpoints):
            #desired_throttle = throttle


        if ARMED and altitude >= 0.3:
            self.array_throttle.append(error_throttle)
            #throttle calculatin
            P_throttle = Kp_Throttle * error_throttle
            if throttle <1550 or throttle  > 1400:
                I_throttle = np.sum(self.array_throttle)*Ki_Throttle
            D_throttle = Kd_Throttle*(error_throttle - self.previous_error_throttle)
            PID_T = P_throttle + I_throttle + D_throttle
            if PID_T > maxPID:
                maxPID = PID_T
            elif PID_T < minPID:
                minPID = PID_T
            slope = 200/(maxPID - minPID)
            new_throttle = 1400 + slope*(PID_T + 1) 
            #new_throttle = 1600 + 100*(PID_T - 1)
            throttle = new_throttle
            if throttle >= 1600:
                throttle = 1600
            elif throttle <= 1400:
                throttle = 1400
            CMDS['throttle'] = throttle
            
            self.previous_error_throttle = error_throttle
        else:
            CMDS['throttle'] = 1000
    def pid_roll_pitch(self,error_roll_img, error_pitch_img, altitude):
        global roll_angle, Kp_roll, Ki_roll , Kd_roll , error_roll, PID_R
        global pitch_angle, Kp_pitch, Ki_pitch , Kd_pitch , error_pitch, PID_P
        Kp_roll = 0.3
        Ki_roll = 0.0001
        Kd_roll = 1.5
        minPID_roll = -2
        maxPID_roll = 2
        Kp_pitch = 0.3
        Ki_pitch = 0.0001
        Kd_pitch = 1.5
        minPID_pitch = -2
        maxPID_pitch = 2    
        error_roll = error_roll_img
        error_pitch = error_pitch_img
        if ARMED and altitude >= 0.3:
            self.array_roll.append(error_roll)
            #roll calculatin
            P_roll = Kp_roll * error_roll
            if roll <1550 or roll  > 1400:
                I_roll = np.sum(self.array_roll)*Ki_roll
            D_roll = Kd_roll*(error_roll - self.previous_error_roll)
            PID_R = P_roll + I_roll + D_roll
            if PID_R > maxPID_roll:
                maxPID_roll = PID_R
            elif PID_R < minPID_roll:
                minPID_roll = PID_R
            slope = 200/(maxPID_roll - minPID_roll)
            new_roll = 1400 + slope*(PID_R + 1) 
                #new_throttle = 1600 + 100*(PID_T - 1)
            roll = new_roll
            if roll >= 1600:
                roll = 1600
            elif roll <= 1400:
                roll = 1400
                CMDS['roll'] = roll
                
            self.previous_error_roll = error_roll
            self.array_pitch.append(error_pitch)
                #pitch calculatin
            P_pitch = Kp_pitch * error_pitch
            if pitch <1550 or pitch  > 1400:
                I_pitch = np.sum(self.array_pitch)*Ki_pitch
            D_pitch = Kd_pitch*(error_pitch - self.previous_error_pitch)
            PID_P = P_pitch + I_pitch + D_pitch
            if PID_P > maxPID_pitch:
                maxPID_pitch = PID_P
            elif PID_R < minPID_pitch:
                minPID_pitch = PID_P
            slope = 200/(maxPID_pitch - minPID_pitch)
            new_pitch = 1400 + slope*(PID_P + 1) 
            #new_throttle = 1600 + 100*(PID_T - 1)
            pitch = new_pitch
            if pitch >= 1600:
                pitch = 1600
            elif pitch <= 1 400:
                pitch = 1400
            CMDS['pitch'] = pitch
                
            self.previous_error_pitch = error_pitch
        else:
            CMDS['roll'] = 1000
            CMDS['pitch'] = 1000
            
class Optical(Thread):
    def __init__(self):
        #init thread
        Thread.__init__(self)
        #the for each point tracked, the location of that point in the previous 10 frames is stored
        self.trajectory_len = 10
        # a new set of points is detected very n frames
        self.detect_interval = 5
        self.trajectories = []
        self.frame_idx = 0
        # inital fps will be recalculated after self.fps_interval frames
        #self.height = height

        # a new fps is calculated every n frames
        self.fps_interval = 5
        # list of velocities relating to movement of points frame to frame
        self.vlist = []
        self.lk_params = dict(winSize  = (25, 25),
                maxLevel = 1,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.feature_params = dict(maxCorners = 15,
                    qualityLevel = 0.3,
                    minDistance = 10,
                    blockSize = 7 )

        self.ROLL_DRIFT_SUM = 0
        self.PITCH_DRIFT_SUM = 0
        self.ROLL_test = 0
    def run(self):
        cap = WebcamVideoStream(src = 0).start()
        #fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        #out = cv2.VideoWriter('output4.mp4', fourcc, 25, (640, 480), isColor=True)
        fps = 50
        t1 = time.time()
        f2 = open("optical_test_18.txt","a")
        pitch_drifts = []
        roll_drifts = []
        FC = [1455.40, 1446.54]  # focal lengths 
        CC = [637.53, 316.53]  # principle points for same
        KC = [0.03, 0.21, 0.00, -0.00, 0.46]  # distortion coeffs for same
        fx, fy = FC
        cx, cy = CC
        cam_matrix = np.array([[fx,  0, cx],
                               [ 0, fy, cy],
                               [ 0,  0,  1]], dtype='float32')
        distortion_profile = np.array(KC, dtype='float32')
        #altitude_value = altitude_thread.distance_kalman
        while True:

    # start time to calculate FPS
    #start = time.time()

            frame = cap.read()
            frame_undisorted = cv2.undistort(frame, cam_matrix, distortion_profile)
            frame_gray = cv2.cvtColor(frame_undisorted, cv2.COLOR_BGR2GRAY)
            img = frame_gray.copy()
            altitude_value = altitude_thread.distance_kalman
            


            # Calculate optical flow for a sparse feature set using the iterative Lucas-Kanade Method
            if len(self.trajectories) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([trajectory[-1] for trajectory in self.trajectories]).reshape(-1, 1, 2)
                p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
                p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
                d = abs(p0-p0r).reshape(-1, 2).max(-1)
                good = d < 1

                new_trajectories = []

                # Get all the trajectories
                for  trajectory, (x, y), good_flag in zip(self.trajectories, p1.reshape(-1, 2), good):
                    if not good_flag:
                        continue
                    trajectory.append((x, y))
                    if len(trajectory) > self.trajectory_len:
                        del trajectory[0]
                    new_trajectories.append(trajectory)
                    # Newest detected point
                    cv2.circle(img, (int(x), int(y)), 2, (50, 125, 255), -1)

                self.trajectories = new_trajectories
                #calculate velocity
                distances = []
                for tr in self.trajectories:
                    
                    prev_x , prev_y = tr[0][0], tr[0][1]
                    # for every position for a given point get the distance between
                    # two consecutive points, average that distance and calculate the
                    # speed in pxls/s
                    for i in range(len(tr)):
                        curr_x, curr_y = tr[i][0], tr[i][1]
                        x,y=curr_x-prev_x,curr_y-prev_y
                        prev_x, prev_y = curr_x, curr_y
                        #self.px = 1.5/(float(fx)*(x-cx))
                        #self.py = 1.5/(float(fy)*(y-cy))
                    # if the distance between two points is less than 1 pixel, ignore it
                    if abs(x) < 1.0 and abs(y) < 1.0:
                        continue
                    distances.append([x,y])
                    roll_drifts.append(x)
                    pitch_drifts.append(y)
                
                #total = [0,0]
                if len(pitch_drifts) < 1 or len(roll_drifts) < 1:
                    print('No drift points')
                elif len(pitch_drifts) < 2 or len(roll_drifts) < 2:
                    self.ROLL_DRIFT_SUM += roll_drifts[0]
                    self.PITCH_DRIFT_SUM += pitch_drifts[0]
                else:
                    self.ROLL_DRIFT_SUM += reduce(lambda a, b: a + b, roll_drifts) / len(roll_drifts)
                    self.PITCH_DRIFT_SUM += reduce(lambda a, b: a + b, pitch_drifts) / len(pitch_drifts)
                
                # sum up all the distances for a given point
                #for distance in distances:
                    #print("number of feature is : " , len(p))
                    #total[0] += distance[0] # displacment x
                    #self.ROLL_DRIFT_SUM = (total[0]* altitude_value)/(fx)
                    #self.ROLL_test = 
                    
                    #total[1] += distance[1] #displacement y
                    #self.PITCH_DRIFT_SUM = (total[1]* altitude_value)/(fy)
                #f.write(str(total[0]))
                #f2.write(" the roll value is :" + str(self.ROLL_DRIFT_SUM))
                #f2.write('\n') 
                #f2.write(" the displacment x value is :" + str(total[0]))
                #f2.write('\n') 
                #f2.write(" the pitch value is :" + str(self.PITCH_DRIFT_SUM))
                #f2.write('\n') 

                #f2.write(" the displacemnt y value is :" + str(total[1]))
                #f2.write('\n') 
                #f.write('\n')
                # get the average of the summed points and calculate velocity of that
                #if len(distances) != 0:
                    #avg_distance = [total[0]/(len(distances)), total[1]/(len(distances))]
                    #v = [avg_distance[0]*fps, avg_distance[1]*fps]
                    #cv2.putText(img,, (20, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
                    #print("velocity is : ", v)

                    #self.vlist.append(v)
                # Draw all the trajectories
                cv2.polylines(img, [np.int32(trajectory) for trajectory in self.trajectories], False, (0, 255, 150))
                #cv2.putText(img, 'track count: %d' % len(self.trajectories), (20, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
            if self.frame_idx % self.fps_interval:
                t2 = time.time()
                fps = self.fps_interval/(t2-t1)
                t1 = t2


            # Update interval - When to update and detect new features
            if self.frame_idx % self.detect_interval == 0:

                mask = np.zeros_like(frame_gray)
                del self.vlist[:]
                mask[:] = 255

                # Lastest point in latest trajectory
                for x, y in [np.int32(trajectory[-1]) for trajectory in self.trajectories]:
                    cv2.circle(mask, (x, y), 5, 0, -1)

                # Detect the good features to track
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **self.feature_params)
                
                if p is not None:
                    # If good features can be tracked - add that to the trajectories
                    for x, y in np.float32(p).reshape(-1, 2):
                        self.trajectories.append([(x, y)])
                        #cv2.putText(img, 'point count: %d' % len(p), (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)


            self.frame_idx += 1
            self.prev_gray = frame_gray
            # calculate the FPS for current frame detection
            #fps = 1 / (end-start)
            
            # Show Results
            #cv2.putText(img, f"{fps:.2f} FPS", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            #cv2.imshow('Optical Flow', img)
            #out.write(img)
            #print("velocity shape is: ")
                        # press escape to exit the program
            ch = 0xFF & cv2.waitKey(1)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                #print("velocity: " + str(self.vlist))
                cv2.destroyAllWindows()
                cap.stop()
                #out.release()
                break

    

if __name__ == '__main__':
    try:
        #event = Event()
        #global altitude_value
        altitude_thread = read_altitude()
        altitude_thread.start()
        #altitude_value = altitude_thread.distance
        optical_thread = Optical()
        optical_thread.start()
        run = Thread(target = run_curses, args =(keyboard_controller,))
        run.start()

        #run_curses(keyboard_controller)
        altitude_thread.join()
        run.join()
        #optical_thread.join()
    except KeyboardInterrupt:
        print("Error")
