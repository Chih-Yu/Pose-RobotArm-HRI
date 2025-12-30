import numpy as np
import cv2
import sys

import math
import EIH_calib


def RotationTrans(v):
    # print("----------VVVVVV------")
    # print("v:",v)
    pi = np.pi / 180
    
    # ================Epson===============
    # tmp_v = v[0]
    # v[0] = v[2]
    # v[2] = tmp_v
    # ================Epson===============

    # pi =   1
    r1_mat = np.zeros((3, 3), np.float32)
    r2_mat = np.zeros((3, 3), np.float32)
    r3_mat = np.zeros((3, 3), np.float32)

    r = np.zeros((3, 1), np.float32)
    r[0] = 0
    r[1] = 0
    r[2] = float(v[2]) * pi
    r3_mat, jacobian = cv2.Rodrigues(r)
    # print("r3_mat:",r3_mat)
    r[0] = 0
    r[1] = float(v[1]) * pi
    r[2] = 0
    # print('ys ', math.sin(v[1]))
    # print('yc ', math.cos(v[1]))
    r2_mat, jacobian = cv2.Rodrigues(r)
    # print("r2_mat:",r2_mat)
    r[0] = float(v[0]) * pi
    r[1] = 0
    r[2] = 0
    r1_mat, jacobian = cv2.Rodrigues(r)
    # print("r1_mat:",r1_mat)

    result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    # print(v)
    # v = [element / 180 * np.pi for element in v]
    # print(v)
    # rxs = math.sin(v[0])
    # rxc = math.cos(v[0])
    # rys = math.sin(v[1])
    # ryc = math.cos(v[1])
    # rzs = math.sin(v[2])
    # rzc = math.cos(v[2])

    # r1_mat = np.array([[1,0,0],[0, rxc, -rxs], [0, rxs, rxc]])
    # print(r1_mat)
    # r2_mat = np.array([[ryc,0,rys],[0, 1, 0], [-rys, 0, ryc]])
    # print(r2_mat)
    # r3_mat = np.array([[rzc,-rzs,0],[rzs, rzc, 0], [0, 0, 1]])
    # print(r3_mat)
    
    

    # result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    
    # print('result')
    print(result)
    return result


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    # print([x,y,z])
    return np.array([x, y, z])


def EIHCali():
    w = 11
    h = 8
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, sys.float_info.epsilon)
    objp = np.zeros((w * h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2) * 20  # 在这里乘棋盘大小
    # print("OBJP", objp)
    # 储存棋盘格角点的世界坐标和图像坐标对
    objpoints = []  # 在世界坐标系中的三维点
    imgpoints = []  # 在图像平面的二维点
    for i in range(200, 221):
        # img = cv2.imread("epson/EIH/img/" + 'color_' + str(i) + ".png")

        img = cv2.imread("source/EIH/" + 'color_' + str(i) + ".png")

        # img = cv2.imread("epson/EIH/stereo/" + 'collectedImg_' + str(i) + ".png")
        # img = cv2.imread("TT/data2/"+str(i)+"_Color.png")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 找到棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, (w, h),
                                                 flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS)
        # 如果找到足够点对，将其存储起来
        if ret == True:
            print(i)
            cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners)
            # 将角点在图像上显示
            # cv2.drawChessboardCorners(img, (w, h), corners, ret)
            # cv2.namedWindow("EIH Calibration", cv2.WINDOW_NORMAL)
            # cv2.resizeWindow('EIH Calibration', (1000, 1000))
            # cv2.imshow("EIH Calibration",img)
            # cv2.waitKey(0)
        else:
            print("ERROR!!: " + str(i) + ": no chessboard ")
    cv2.destroyAllWindows()
    # Get Rotation and Translation from Camera To Plane
    # objpoints = [x * 15 for x in objpoints]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print('dist', dist)
    print('rvecs', rvecs)
    print('tvecs', tvecs)


    for i in range(21):
        # print(rvecs[i])
        # tmpr = cv2.Rodrigues(np.array(rvecs[i]))[0]
        # print(tmpr)
        print(tvecs[i][0][0], ',', tvecs[i][1][0], ',', tvecs[i][2][0], ',', rvecs[i][0][0], ',', rvecs[i][1][0], ',',
              rvecs[i][2][0], ',')
    # print("-----------\n",rvecs,"-----------\n",tvecs,"-----------")
    PlaneRCamera = []
    PlaneTCamera = []
    Plane2Camera = []  # west
    for i in range(21):
        # print(rvecs[i])
        Rvecs = cv2.Rodrigues(rvecs[i])[0]
        Tvecs = tvecs[i]
        # print("Tvecs",Tvecs)
        # print("Rvecs",Rvecs)
        RPlanePos = Rvecs
        TPlanePos = Tvecs.reshape(3, 1)
        # print(TPlanePos)

        # RotationMatrix check
        # print(isRotationMatrix(RPlanePos))

        PlaneRCamera.append(RPlanePos)
        PlaneTCamera.append(TPlanePos)
        Plane2Camera.append(np.r_[np.c_[RPlanePos, TPlanePos], [[0, 0, 0, 1]]])
    # print("PlaneRCamera",PlaneRCamera)
    # print("PlaneTCamera",PlaneTCamera)
    # Get Rotation and Translation from Gripper To Base
    # fr = open(sys.path[0] + "/PosSet.txt", 'r+')
    # dic = eval(fr.read())
    # CaliPos = StrToArray.StrToArray(dic['CaliPos'])
    # print("CaliPos:", CaliPos)
    # fr.close()
    # CaliPos = np.loadtxt('epson/EIH/Epson_CalibPos.txt', delimiter=',')

    # CaliPos = np.loadtxt('source/TM_CalibPos.txt', delimiter=',')
    EIH_calib_tmp =  EIH_calib.calib()
    print(EIH_calib_tmp)
    CaliPos = np.array(EIH_calib_tmp.pos)

    # CaliPos = np.loadtxt('epson/EIH/Epson_CalibPos2.txt', delimiter=',')
    # CaliPos = np.loadtxt('TT/data2/TT_CalibPos.txt', delimiter=',')
    print(CaliPos)
    GripperRBase = []
    GripperTBase = []
    Gripper2BasePos = []  # west
    for i in range(21):
        RvecsB = RotationTrans([float(CaliPos[i, 3]), float(CaliPos[i, 4]), float(CaliPos[i, 5])])
        # print("Rvecs", i, RvecsB)
        # print(CaliPos[i, 3], ' ', CaliPos[i, 4], ' ', CaliPos[i, 5])
        TvecsB = [[float(CaliPos[i, 0])], [float(CaliPos[i, 1])], [float(CaliPos[i, 2])]]
        # print(TvecsB)
        Gripper2Base = np.r_[np.c_[RvecsB, TvecsB], [[0, 0, 0, 1]]]
        # Gripper2Base = np.linalg.inv(Gripper2Base)
        RCaliPos = Gripper2Base[:3, :3]
        TCaliPos = Gripper2Base[:3, 3].reshape(3, 1)
        # print("TCaliPos:", TCaliPos)

        # RotationMatrix check
        # print(isRotationMatrix(RCaliPos))

        GripperRBase.append(RCaliPos)
        # print(RCaliPos)
        GripperTBase.append(TCaliPos)
        Gripper2BasePos.append(np.r_[np.c_[RCaliPos, TCaliPos], [[0, 0, 0, 1]]])
    # print("GripperRBase", GripperRBase)
    # print("GripperTBase", GripperTBase)

    for i in range(21):
        # print(rvecs[i])
        # tmpr = cv2.Rodrigues(np.array(rvecs[i]))[0]
        # print(tmpr)
        print(float(CaliPos[i, 0]), ',', float(CaliPos[i, 1]), ',',float(CaliPos[i, 2]), ',', float(CaliPos[i, 3]), ',', float(CaliPos[i, 4]), ',',
              float(CaliPos[i, 5]), ',')

    for i in range(21):
        print(i)
        print('rot: ', GripperRBase[i])
        print('trans: ', GripperTBase[i])
        print('plane')
        print('rot: ', PlaneRCamera[i])
        print('trans: ', PlaneTCamera[i])
    print(np.array(GripperRBase[0]).flatten())
    print(np.array(GripperTBase[0]).squeeze(1))
    tmp_GripperTBase = []
    tmp_PlaneTCamera = []
    for i in range(21):
        tmp_GripperTBase.append(np.array(GripperTBase[i]).squeeze(1))
        tmp_PlaneTCamera.append(np.array(PlaneTCamera[i]).squeeze(1))
    print(np.array(tmp_GripperTBase).shape)
    print(np.array(tmp_PlaneTCamera).shape)
    print(np.array(PlaneRCamera).shape)
    print(np.array(GripperRBase).shape)
    print(np.array(GripperRBase))
    print(np.array(tmp_GripperTBase))
    print('===================')
    CameraRGripper, CameraTGripper = cv2.calibrateHandEye(np.array(GripperRBase), np.array(tmp_GripperTBase),
                                                          np.array(PlaneRCamera), np.array(tmp_PlaneTCamera),
                                                          None, None, cv2.CALIB_HAND_EYE_ANDREFF)
    Camera2Gripper = np.r_[np.c_[CameraRGripper, CameraTGripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_ANDREFF')
    print(Camera2Gripper)
    print(CameraRGripper)
    rot, jacobian = cv2.Rodrigues(CameraRGripper)
    print(rot)
    print(CameraTGripper)
    CameraRGripper, CameraTGripper = cv2.calibrateHandEye(np.array(GripperRBase), np.array(tmp_GripperTBase),
                                                          np.array(PlaneRCamera), np.array(tmp_PlaneTCamera),
                                                          None, None, cv2.CALIB_HAND_EYE_DANIILIDIS)
    Camera2Gripper = np.r_[np.c_[CameraRGripper, CameraTGripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_DANIILIDIS')
    print(Camera2Gripper)
    print(CameraRGripper)
    rot, jacobian = cv2.Rodrigues(CameraRGripper)
    print(rot)
    print(CameraTGripper)
    CameraRGripper, CameraTGripper = cv2.calibrateHandEye(np.array(GripperRBase), np.array(tmp_GripperTBase),
                                                          np.array(PlaneRCamera), np.array(tmp_PlaneTCamera),
                                                          None, None, cv2.CALIB_HAND_EYE_HORAUD)
    Camera2Gripper = np.r_[np.c_[CameraRGripper, CameraTGripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_HORAUD')
    print(Camera2Gripper)
    print(CameraRGripper)
    rot, jacobian = cv2.Rodrigues(CameraRGripper)
    print(rot)
    print(CameraTGripper)
    CameraRGripper, CameraTGripper = cv2.calibrateHandEye(np.array(GripperRBase), np.array(tmp_GripperTBase),
                                                          np.array(PlaneRCamera), np.array(tmp_PlaneTCamera),
                                                          None, None, cv2.CALIB_HAND_EYE_PARK)
    Camera2Gripper = np.r_[np.c_[CameraRGripper, CameraTGripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_PARK')
    print(Camera2Gripper)
    print(CameraRGripper)
    rot, jacobian = cv2.Rodrigues(CameraRGripper)
    print(rot[0] * 180 / np.pi)
    print(CameraTGripper)
    CameraRGripper, CameraTGripper = cv2.calibrateHandEye(np.array(GripperRBase), np.array(tmp_GripperTBase),
                                                          np.array(PlaneRCamera), np.array(tmp_PlaneTCamera),
                                                          None, None, cv2.CALIB_HAND_EYE_TSAI)

    Camera2Gripper = np.r_[np.c_[CameraRGripper, CameraTGripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_TSAI')
    print(Camera2Gripper)
    print(CameraRGripper)
    rot, jacobian = cv2.Rodrigues(CameraRGripper)
    print(rot[0] * 180 / np.pi)
    print(CameraTGripper)

    # RotationMatrix check
    # print(isRotationMatrix(CameraRGripper))

    # =======================================

    # fs = cv2.FileStorage("Extrinsic.txt", cv2.FILE_STORAGE_WRITE)

    # fs.write('intrinsic', mtx)
    # fs.write('Camera2Gripper', Camera2Gripper)
    # fs.write('distortion', dist)

    # ========================================
    g0_gi = []
    g0_b = Gripper2BasePos[0]
    for i in range(1, len(Gripper2BasePos)):
        g0_gi.append(np.dot(np.linalg.inv(Gripper2BasePos[i]), g0_b))
    # print(g0_gi)
    c0_ci = []
    c0_t = np.linalg.inv(Plane2Camera[0])
    for i in range(1, len(Plane2Camera)):
        c0_ci.append(np.dot(Plane2Camera[i], c0_t))
    # print(c0_ci)

    for i in range(len(g0_gi)):
        ax = np.dot(g0_gi[i], Camera2Gripper)
        xb = np.dot(Camera2Gripper, c0_ci[i])
        diff = ax - xb
        print(np.sum(diff[:3, :3]))
        print(np.sum(diff[:3, 3]))


    xError = []
    yError = []
    zError = []
    xrotError = []
    yrotError = []
    zrotError = []
    for i in range(len(g0_gi)):
        ax = np.dot(g0_gi[i], Camera2Gripper)
        xb = np.dot(Camera2Gripper, c0_ci[i])
        ax_rot = ax[:3,:3]
        xb_rot = xb[:3,:3]

        ax_rotV = rotationMatrixToEulerAngles(ax_rot)
        xb_rotV = rotationMatrixToEulerAngles(xb_rot)

        # print(ax_rotV)
        # print(xb_rotV)
        xError.append(ax[0, 3] - xb[0,3])
        yError.append(ax[1, 3] - xb[1,3])
        zError.append(ax[2, 3] - xb[2,3])
        xrotError.append(ax_rot[0] - xb_rot[0])
        yrotError.append(ax_rot[1] - xb_rot[1])
        zrotError.append(ax_rot[2] - xb_rot[2])
        

        # print(ax[0,3], ' x ', xb[0,3])
        # print(ax[1,3], ' y ', xb[1,3])
        # print(ax[2,3], ' z ', xb[2,3])

        # rot_error += math.pow(np.sum(ax_rotV - xb_rotV), 2)
        # trans_error += math.pow(np.sum(ax[:3,3] - xb[:3,3]), 2)
        # x_error += math.pow(np.sum(ax[0,3] - xb[0,3]), 2)
        # y_error += math.pow(np.sum(ax[1,3] - xb[1,3]), 2)
        # z_error += math.pow(np.sum(ax[2,3] - xb[2,3]), 2)

        # rotx_error += math.pow(np.sum(ax_rotV[0] - xb_rotV[0]), 2)
        # roty_error += math.pow(np.sum(ax_rotV[1] - xb_rotV[1]), 2)
        # rotz_error += math.pow(np.sum(ax_rotV[2] - xb_rotV[2]), 2)
    xError = np.array(xError)
    yError = np.array(yError)
    zError = np.array(zError)
    xrotError = np.array(xrotError)
    yrotError = np.array(yrotError)
    zrotError = np.array(zrotError)

    print('==============average======================')
    print('x: ', np.average(xError))
    print('y: ', np.average(yError))
    print('z: ', np.average(zError))
    print('x rot: ', np.average(xrotError))
    print('y rot: ', np.average(yrotError))
    print('z rot: ', np.average(zrotError))


    print('==============average(abs)======================')
    print('x: ', np.average(np.absolute(xError)))
    print('y: ', np.average(np.absolute(yError)))
    print('z: ', np.average(np.absolute(zError)))
    print('x rot: ', np.average(np.absolute(xrotError)))
    print('y rot: ', np.average(np.absolute(yrotError)))
    print('z rot: ', np.average(np.absolute(zrotError)))

    print('==============std======================')
    print('x: ', np.std(xError))
    print('y: ', np.std(yError))
    print('z: ', np.std(zError))
    print('x rot: ', np.std(xrotError))
    print('y rot: ', np.std(yrotError))
    print('z rot: ', np.std(zrotError))

    

    



    return mtx, Camera2Gripper, Plane2Camera, Gripper2BasePos

# original
# def EstimateCoord(pointx, pointy, depth):
#     # TODO: You need to use cv::SVD::solveZ() to get a,b,c,d (4 points is enough MAYBE)
#     # Plz run in C++, python do not have cv::SVD::solveZ()
#     a = -2.5596638e-09
#     b = 3.436568e-10
#     c = -0.030336918
#     d = 0.99953579
#     # d = d + 15 * c
#     # TODO: Get Intrinsic parameters
#     Intrinsic, Camera2Gripper, t2c, g2b = EIHCali()
#     # print('Camera2Gripper')
#     # print(Camera2Gripper)
#     # print('Camera2Gripper')

#     # jang
#     # Camera2Gripper = np.linalg.inv(Camera2Gripper)
#     # jang

#     print('Intrinsic')
#     print(Intrinsic)
#     Intrinsicinv = np.linalg.inv(Intrinsic)
#     # Camera2Gripper = EIHCali()
#     # RvecsB = RotationTrans([853.71,-674.67,985.18])
#     RvecsB = RotationTrans([-179.1046, -0.271, 179.115])
#     # TvecsB = [[178.79 * 1000],[0.68 * 1000],[0.32 * 1000]]
#     TvecsB = [[-325.6122], [18.64053], [219.3423]]
#     Gripper2Base = np.r_[np.c_[RvecsB, TvecsB], [[0, 0, 0, 1]]]
#     # Gripper2Base = np.linalg.inv(Gripper2Base)
#     Camera2Base = np.dot(np.linalg.inv(Camera2Gripper), np.linalg.inv(Gripper2Base))
#     # Camera2Base = np.dot(Camera2Gripper, Gripper2Base)
#     # Camera2BaseInv = np.linalg.inv(Camera2Base)
#     # print("Camera2Base\n",Camera2Base)
#     CameraRBase = Camera2Base[:3, :3]
#     CameraRBaseinv = np.linalg.inv(CameraRBase)
#     # CameraRBaseinv = CameraRBase
#     # print(CameraRBaseinv)
#     CameraTBase = Camera2Base[:3, 3].reshape(3, 1)
#     # print(CameraTBase)
#     pointMat = np.array([[pointx], [pointy], [1]])
#     # print("pointMat:",pointMat)
#     Coor1 = np.dot(np.dot(CameraRBaseinv, Intrinsicinv), pointMat)
#     # print("Coor1:",Coor1)
#     Coor2 = np.dot(CameraRBaseinv, CameraTBase)
#     # print("Coor2:",Coor2)
#     s = (a * Coor2[0] + b * Coor2[1] + c * Coor2[2] - d) / (a * Coor1[0] + b * Coor1[1] + c * Coor1[2])
#     print('before s: ', s)

#     M_dot_uv = np.dot(Intrinsicinv,pointMat)
#     print('M_dot_uv',M_dot_uv,type(M_dot_uv))
#     s = depth / M_dot_uv[2][0]

#     print("s:",s)
#     result = s * Coor1 - Coor2
#     print("result:", result)
#     print(a * result[0] + b * result[1] + c * result[2] + d)

#     return result


def EstimateCoord(pointx, pointy, depth):
    # TODO: You need to use cv::SVD::solveZ() to get a,b,c,d (4 points is enough MAYBE)
    # Plz run in C++, python do not have cv::SVD::solveZ()
    a = -3.469446839078007e-19
    # a = -2.3964581541e-5
    b = 2.1105802902823e-19
    # b = 6.29286172165e-5
    c = -0.01652666818666113
    # c = -0.00235536090
    d = 0.9998634252929987
    # d = d + 15 * c
    # TODO: Get Intrinsic parameters
    # Intrinsic, Camera2Gripper, t2c, g2b = EIHCali()
    fs = cv2.FileStorage( 'Extrinsic.txt', cv2.FILE_STORAGE_READ )
    fn = fs.getNode("intrinsic")
    Intrinsic = fn.mat()
    fn = fs.getNode("Camera2Gripper")
    Camera2Gripper = fn.mat()

    print('Intrinsic')
    print(Intrinsic)
    Intrinsicinv = np.linalg.inv(Intrinsic)
    # Camera2Gripper = EIHCali()
    # RvecsB = RotationTrans([853.71,-674.67,985.18])
    RvecsB = RotationTrans([179.98, -0.04045901, -135.05])
    # TvecsB = [[178.79 * 1000],[0.68 * 1000],[0.32 * 1000]]
    TvecsB = [[376.0587], [-436.6104], [548.17]]
    Gripper2Base = np.r_[np.c_[RvecsB, TvecsB], [[0, 0, 0, 1]]]
    # Gripper2Base = np.linalg.inv(Gripper2Base)
    Camera2Base = np.dot(np.linalg.inv(Camera2Gripper), np.linalg.inv(Gripper2Base))
    # Camera2Base = np.dot(Camera2Gripper, Gripper2Base)
    # Camera2BaseInv = np.linalg.inv(Camera2Base)
    # print("Camera2Base\n",Camera2Base)
    CameraRBase = Camera2Base[:3, :3]
    CameraRBaseinv = np.linalg.inv(CameraRBase)
    # CameraRBaseinv = CameraRBase
    # print(CameraRBaseinv)
    CameraTBase = Camera2Base[:3, 3].reshape(3, 1)
    # print(CameraTBase)
    pointMat = np.array([[pointx], [pointy], [1]])
    # print("pointMat:",pointMat)
    Coor1 = np.dot(np.dot(CameraRBaseinv, Intrinsicinv), pointMat)
    # print("Coor1:",Coor1)
    Coor2 = np.dot(CameraRBaseinv, CameraTBase)
    # print("Coor2:",Coor2)
    s = (a * Coor2[0] + b * Coor2[1] + c * Coor2[2] - d) / (a * Coor1[0] + b * Coor1[1] + c * Coor1[2])
    print('before s: ', s)
    before_result = s * Coor1 - Coor2
    print(a * before_result[0] + b * before_result[1] + c * before_result[2] + d)
    print("before result:", before_result)

    M_dot_uv = np.dot(Intrinsicinv,pointMat)
    print('depth ', depth)
    print('M_dot_uv',M_dot_uv,type(M_dot_uv))
    print(M_dot_uv[2][0])
    # depth add some compensation value
    s = (depth - 10) / M_dot_uv[2][0]

    print("s:",s)
    result = s * Coor1 - Coor2
    print("result:", result)
    print(a * result[0] + b * result[1] + c * result[2] + d)

    return result

    # error Test
    # ==========================================================================================
    # g0_gi = []
    # g0_b = g2b[0]
    # for i in range(1, len(g2b)):
    #     g0_gi.append(np.dot(np.linalg.inv(g2b[i]), g0_b))
    # # print(g0_gi)
    # c0_ci = []
    # c0_t = np.linalg.inv(t2c[0])
    # for i in range(1, len(t2c)):
    #     c0_ci.append(np.dot(t2c[i], c0_t))
    # # print(c0_ci)

    # for i in range(len(g0_gi)):
    #     ax = np.dot(g0_gi[i], Camera2Gripper)
    #     xb = np.dot(Camera2Gripper, c0_ci[i])
    #     diff = ax - xb
    #     print(np.sum(diff[:3, :3]))
    #     print(np.sum(diff[:3, 3]))


    # rot_error = 0
    # trans_error = 0
    # x_error = 0
    # y_error = 0
    # z_error = 0
    # rotx_error = 0
    # roty_error = 0
    # rotz_error = 0
    # for i in range(len(g0_gi)):
    #     ax = np.dot(g0_gi[i], Camera2Gripper)
    #     xb = np.dot(Camera2Gripper, c0_ci[i])
    #     # ax = np.dot(Camera2Gripper, c0_ci[i])
    #     # xb = np.dot(g0_gi[i], Camera2Gripper)
    #     # print('xb ', xb[:3,3])
    #     # print('ax ', ax[:3,3])
    #     # print(np.sum(abs(ax[:3,3]) - abs(xb[:3,3])))
    #     ax_rot = ax[:3,:3]
    #     xb_rot = xb[:3,:3]

    #     ax_rotV = rotationMatrixToEulerAngles(ax_rot)
    #     xb_rotV = rotationMatrixToEulerAngles(xb_rot)

    #     # print(ax_rotV)
    #     # print(xb_rotV)

    #     print(ax[0,3], ' x ', xb[0,3])
    #     print(ax[1,3], ' y ', xb[1,3])
    #     print(ax[2,3], ' z ', xb[2,3])

    #     rot_error += math.pow(np.sum(ax_rotV - xb_rotV), 2)
    #     trans_error += math.pow(np.sum(ax[:3,3] - xb[:3,3]), 2)
    #     x_error += math.pow(np.sum(ax[0,3] - xb[0,3]), 2)
    #     y_error += math.pow(np.sum(ax[1,3] - xb[2,3]), 2)
    #     z_error += math.pow(np.sum(ax[2,3] - xb[3,3]), 2)

    #     rotx_error += math.pow(np.sum(ax_rotV[0] - xb_rotV[0]), 2)
    #     roty_error += math.pow(np.sum(ax_rotV[1] - xb_rotV[1]), 2)
    #     rotz_error += math.pow(np.sum(ax_rotV[2] - xb_rotV[2]), 2)

    # print('rot ', math.sqrt(rot_error)/len(g0_gi))
    # print('trans ', math.sqrt(trans_error)/len(g0_gi))

    # print(math.sqrt(x_error/len(g0_gi)))
    # print(math.sqrt(y_error/len(g0_gi)))
    # print(math.sqrt(z_error/len(g0_gi)))

    # print(math.sqrt(rotx_error/len(g0_gi)))
    # print(math.sqrt(roty_error/len(g0_gi)))
    # print(math.sqrt(rotz_error/len(g0_gi)))
    # ==========================================================================================

    # init = np.dot(np.dot(g2b[0], Camera2Gripper), t2c[0])
    # for idx in range(1, len(t2c)):
    #     print(idx, ' ', np.sum((init - np.dot(np.dot(g2b[idx], Camera2Gripper), t2c[idx]))))
    #     a = rotationMatrixToEulerAngles(init[:3,:3])
    #     print(a)
    #     a = np.dot(np.dot(g2b[idx], Camera2Gripper), t2c[idx])[:3,:3]
    #     print(rotationMatrixToEulerAngles(a))
    #     # print(np.dot(np.dot(t2c[1], Camera2Gripper), g2b[1]))

    return result

if __name__ == '__main__':
    # EstimateCoord(194,122) #707， -503
    # EstimateCoord(496,110) #958， -494
    # EstimateCoord(302,301) #799， -664，
    # EstimateCoord(411,263) #888， -625， 536
    # EstimateCoord(612, 414)  # 958， -494
    EIHCali()

# EIHCali()
