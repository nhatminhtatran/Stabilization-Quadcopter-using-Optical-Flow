import numpy as np
import cv2
import time
import imutils
from imutils.video import WebcamVideoStream
from threading import Thread
from functools import reduce 


FC = [1455.40, 1446.54]  # focal lengths for GoPro Hero3+ Black
CC = [637.53, 316.53]  # principle points for same
KC = [0.03, 0.21, 0.00, -0.00, 0.46]  # distortion coeffs for same
fx, fy = FC
cx, cy = CC
cam_matrix = np.array([[fx,  0, cx],
                       [ 0, fy, cy],
                       [ 0,  0,  1]], dtype='float32')
distortion_profile = np.array(KC, dtype='float32')
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
        self.height = 1.5

        # a new fps is calculated every n frames
        self.fps_interval = 5
        # list of velocities relating to movement of points frame to frame
        self.vlist = []
        self.x = 0
        self.y = 0
        self.current_roll = 0
        self.current_pitch = 0
        self.lk_params = dict(winSize  = (25, 25),
                maxLevel = 1,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.feature_params = dict(maxCorners = 15,
                    qualityLevel = 0.3,
                    minDistance = 10,
                    blockSize = 7 )
        self.px = 0
        self.py = 0
        self.ROLL_DRIFT_SUM = 0
        self.PITCH_DRIFT_SUM = 0

    def run(self):
        #cap = WebcamVideoStream(src = 0).start()
        cap = cv2.VideoCapture('imageprocess.mp4')
        #fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        #out = cv2.VideoWriter('output.mp4', fourcc, 25, (640, 480), isColor=True)
        fps = 30
        
        #f = open("optical1.txt","a")
        #t1 = time.time()

        pitch_drifts = []
        roll_drifts = []
        width = 960
        height = 564
        dsize = (width, height)

        while True:

    # start time to calculate FPS
    #start = time.time()

            ret, frame = cap.read()
            frame = cv2.resize(frame, dsize)
            frame_undisorted = cv2.undistort(frame, cam_matrix, distortion_profile)
            
            frame_gray = cv2.cvtColor(frame_undisorted, cv2.COLOR_BGR2GRAY)
            FrameW, FrameH = frame_gray.shape
            img = frame_undisorted.copy()

            # Calculate optical flow for a sparse feature set using the iterative Lucas-Kanade Method
            if len(self.trajectories) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([trajectory[-1] for trajectory in self.trajectories]).reshape(-1, 1, 2)
                p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
                NoKPts = 0
                if p1 is not None:
                    X_avg = 0
                    Y_avg = 0
                p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
                d = abs(p0-p0r).reshape(-1, 2).max(-1)
                good = d < 1

                new_trajectories = []
                for  trajectory, (x, y), good_flag in zip(self.trajectories, p1.reshape(-1, 2), good):
                    if not good_flag:
                        continue
                    trajectory.append((x, y))
                    if len(trajectory) > self.trajectory_len:
                        del trajectory[0]
                    new_trajectories.append(trajectory)
                    # Newest detected point
                    cv2.circle(img, (int(x), int(y)), 4, (50, 125, 255), -1)

                self.trajectories = new_trajectories
                #calculate velocity
                for tr in self.trajectories:
                    distances = []
                    prev_x , prev_y = tr[0][0], tr[0][1]
                    # for every position for a given point get the distance between
                    # two consecutive points, average that distance and calculate the
                    # speed in pxls/s
                    for i in range(len(tr)):
                        curr_x, curr_y = tr[i][0], tr[i][1]
                        self.x =curr_x-prev_x
                        self.y = curr_y-prev_y
                        prev_x, prev_y = curr_x, curr_y
                        #self.px = self.height/(float(fx)*(self.x-cx))
                        #self.py = self.height/(float(fy)*(self.y-cy))




                    # if the distance between two points is less than 1 pixel, ignore it
                    if abs(self.x) < 1.0 and abs(self.y) < 1.0:
                        continue
                    distances.append([self.x,self.y])
                    roll_drifts.append(self.px)
                    pitch_drifts.append(self.py)
                total = [0,0]
                # sum up all the distances for a given point
                for distance in distances:
                    total[0] += distance[0] # displacment x
                    total[1] += distance[1] #displacement y
                    NoKPts += 1
                



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
            cv2.putText(img, f"{fps:.2f} FPS", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Optical Flow', img)
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
        optical_thread = Optical()
        optical_thread.start()
    except:
        print("error")







        

