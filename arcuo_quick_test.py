import cv2
import numpy as np
import cv2.aruco as aruco
import glob
import time
import math

#video="http://admin:admin@192.168.1.101:8081/"
video_path = "C:/Users/jtian/Documents/test_1min.mkv"
#video_path = "C:/Users/jtian/Documents/video_0409_1.avi"
#video_path = "C:/Users/jtian/Documents/video_0409_2.avi"
cap = cv2.VideoCapture(1)
# testimg = "D:\PotPlayer_20190321\PotPlayer\Capture/test_1min.mkv_20200409_192249.355.jpg"

testimg = "test/1.png"
# testimg = "test2/a.png"
# cap = cv2.VideoCapture(video_path)
#标定矩阵-iphone
# mtx=np.array([[ 9.8683003394937907e+02, 0., 6.3024385802829329e+02], [0.,
#        9.8219061527821100e+02, 3.3729209232671968e+02], [0., 0., 1. ]])
# dist=np.array([[ 2.3486557730381857e-01, -1.5256460930699594e+00,
#        1.9751744701860651e-03, 4.8758282702873718e-04,
#        2.8999670903018195e+00 ]])


#标定矩阵-kinectdk
# mtx=np.array([[586.71992539,0., 642.5673099],
#             [0.,577.7185321,358.14158103],
#             [0.,0.,1.]])
# dist=np.array([[ 0.06001582 , 0.10857292 ,-0.0030517  , 0.00037337 ,-0.28575739]])


#c920
mtx=np.array([[609.710537,0.000000, 303.200522],
[0.000000, 606.011374, 258.905227],
[0.000000, 0.000000, 1.000000]])
dist=np.array([[0.084930, -0.153198, 0.011283, -0.000882, 0.000000]])


makersize=0.1
#标定矩阵-laptop ins
# mtx=np.array([[1.00706563e+03 ,0.00000000e+00 ,6.16095593e+02],
#  [0.00000000e+00 ,1.00438375e+03 ,3.15525824e+02],
#  [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])
# dist=np.array([[-0.1274405,0.23326563,-0.00852498,0.00213784,-0.09599759]])

xxx=0
pi = np.pi
cord_makers = {'0': (-2,8.5,pi/2), 
        '1': (-3,8.5,-pi/2),
        '2': (-4.5,13,pi),
        '3': (-7,9,0),
        '4': (-7,13,pi),
        '5': (-10.5,11.5,-pi/2),
        '6': (-10.5,9,0),
        '7': (-12,12,pi),
        '8': (-12.5,9,0),
        '9': (-16.5,13,pi),
        '10': (-16.5,9,0),
        '11': (-19,13,pi),
        '12': (-19,9,0),
        '13': (-23,13,pi),
        '14': (-23.5,11.5,pi),
        '15': (-23,9,0),
        '16': (-25.5,9,0),
        '17': (-25.5,11.5,pi),
        '18': (-32,10,-pi/2),
        '19': (-29.5,12,-pi/2),
        '20': (-28,13.5,-pi/2),
        '21': (-24,17.5,pi/2),
        '22': (-28,20,-pi/2),
        '23': (-27.5,25),
        '24': (-23.5,25,-pi/2),
        '25': (-19,21.5),
        '26': (-21,23.5,pi/2),
        '27': (-24,26.5,-pi/2),
        '28': (-25,29.5,-pi/2),
        '29': (-22.5,38.5),
        '30': (-18.5,30.5)
        }

###------------------ ARUCO TRACKER ---------------------------
rlst = []
tlst = []

# Function to draw the axis
# Draw axis function can also be used.
# def draw(img, corners, imgpts):
#     corner = tuple(corners[0].ravel())
#     img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
#     img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
#     img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
#     return img



def rotationMatrixToEulerAngles(rvecs):
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    #print('dst:', R)
    x = x*180.0/3.141592653589793
    y = y*180.0/3.141592653589793
    z = z*180.0/3.141592653589793
    return x,y,z


def put_test_estimatePose(frame,corners,ids):
    if np.all(ids != None):
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, makersize, mtx, dist)
        tvec = np.around( tvec, decimals=3)
        for i in range(0, ids.size):
           aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

        # axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
        # imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
        # print(imgpts)
        # frame = draw(frame,corners,imgpts)

        aruco.drawDetectedMarkers(frame, corners)
        n=30
        m=0
        t_lst = tvec.tolist()
        r_lst = rvec.tolist()
        for id,i in enumerate(t_lst):      
            maker_id = ids[t_lst.index(i)][0]
            if(maker_id>30):
                print("Invalid marker ID: " + str(maker_id))
                continue
                                                    #ID                       #tvec
            cv2.putText(frame, "tvec " + str(ids[t_lst.index(i)][0]) +"--> "+str(i[0])  , (10,50+n*m), font, 0.7, (0,0,255),2,cv2.LINE_AA)
            r = np.linalg.norm(np.array(i[0]) - np.array([0,0,0]))  #Distance
            r = np.around( r, decimals=3)
            cv2.putText(frame, "id:{},distance: ".format(str(ids[t_lst.index(i)][0])) + str(r), (0,450+n*m), font, 0.7, (0,255,0),2,cv2.LINE_AA)
            
            


            #Initial Cal Fn ( W/O Marker---rot--->Camera  )
            # Mx,My = np.array(cord_makers[str(ids[t_lst.index(i)][0])][0:2])*0.6
            # WMxy = np.array([Mx,My])

            # theta = cord_makers[str(ids[t_lst.index(i)][0])][2]
            # rot_M = np.array([[np.cos(theta),-np.sin(theta)],
            #                 [np.sin(theta),np.cos(theta)]
            #                 ])

            # CMx,CMy,CMz = np.array(i[0])
            # MCxy = np.array([CMx,CMz])

            # dx,dy,dz = np.array(i[0])

            # ROT_XY = rot_M.dot(MCxy)
            # New_XY = ROT_XY + WMxy
            # cam_in_map = (New_XY/0.6)



            #Head West   <---
            # cam_in_map = ((Mx - dz)/0.6, (My - dx)/0.6)

            # #Head North   up
            # cam_in_map = ((Mx +dx)/0.6, (My - dz)/0.6)
            # #rot
            
            ########## New P - Y - R #############
            # rvec_i = np.array(r_lst[id][0])
            # rx,ry,rz = rotationMatrixToEulerAngles(rvec_i)
            # print("rvec_i---->  ",rx,"  -",ry," - ",rz)
            # cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" ---> P  -  Y  -  R : {:.2f},{:.2f},{:.2f}".format(rx,ry,rz), (0,400+n*(m-1)), font, 0.7, (255,255,255),2,cv2.LINE_AA)

            rvec_i = np.array(r_lst[id][0])
            rx,ry,rz = rotationMatrixToEulerAngles(rvec_i)
            cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" ---> P  -  Y  -  R : {:.1f}".format(ry), (0,400+n*(m-4)), font, 0.7, (0,0,255),2,cv2.LINE_AA)

            tve = np.array(i[0])
            rotM = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec_i, rotM, jacobian = 0)
            cameraPosition = -np.matrix(rotM).T * np.matrix(tve).T





            cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" Rvec : " + str(np.around( rvec_i, decimals=2)), (0,400+n*(m-3)), font, 0.7, (0,255,0),2,cv2.LINE_AA)
            
            cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" Tvec : " + str(np.around( tve, decimals=2)), (0,400+n*(m-2)), font, 0.7, (0,255,0),2,cv2.LINE_AA)
            cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" CameraPosition : " + str(np.around( cameraPosition, decimals=2)), (0,400+n*(m-1)), font, 0.7, (0,0,255),2,cv2.LINE_AA)


            # ypr = cv2.RQDecomp3x3(rotM)

            # cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" ---> rvec_i: {:.2f},{:.2f},{:.2f}".format(rx,ry,rz), (0,400+n*(m-1)), font, 0.7, (255,255,255),2,cv2.LINE_AA)

            # cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" ---> cam floor cords: {:.2f},{:.2f}".format(cam_in_map[0],cam_in_map[1]), (0,400+n*m), font, 0.7, (0,0,255),2,cv2.LINE_AA)
            # cv2.putText(frame, str(ids[t_lst.index(i)][0]) +" ---> cam yaw: "+str(ypr[0]), (0,430+n*m), font, 0.7, (0,0,255),2,cv2.LINE_AA)
            m+=2

        d_dic = dict()
        for i in range(len(t_lst)):
            for j in range(len(t_lst)):
                if i!=j:
                    d = np.linalg.norm(np.array(t_lst[i][0]) - np.array(t_lst[j][0]))
                    d = np.around( d, decimals=3)
                    d_dic.update({d:str((ids[i][0],ids[j][0]))})
        m=0
        for i in d_dic:
            cv2.putText(frame, "arcuo CodeId:{} spacing:{}".format(str(d_dic[i]),str(i)), (0,300+n*m), font, 0.7, (0,255,0),2,cv2.LINE_AA)
            m+=1
        strg = ''

        for i in range(0, ids.size):
            strg += str(ids[i][0])+', '
        cv2.putText(frame, "Id: " + strg, (0,200), font, 0.7, (0,255,0),2,cv2.LINE_AA)
    else:

        cv2.putText(frame, "No Ids", (0,200), font, 0.7, (0,255,0),2,cv2.LINE_AA)
    return frame

# if True:
#     frame= cv2.imread(testimg)
while (True):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    frame = put_test_estimatePose(frame,corners,ids)
    cv2.imshow('frame',frame)
    # time.sleep(1)
    # img_save_path='img_save/test_{}.jpg'.format(str(time.time()))
    # print("img_save_path---> ", img_save_path)
    # cv2.imwrite(img_save_path, frame)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()