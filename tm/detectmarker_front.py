import numpy as np
import cv2
import math


def RotationTrans(v_org):
    v = np.copy(v_org)

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
    r[0] = 0
    r[1] = float(v[1]) * pi
    r[2] = 0
    r2_mat, jacobian = cv2.Rodrigues(r)
    r[0] = float(v[0]) * pi
    r[1] = 0
    r[2] = 0
    r1_mat, jacobian = cv2.Rodrigues(r)

    result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    # print(result)
    return result


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    # assert(isRotationMatrix(R))
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

    # ================Epson===============
    # x, z = z, x
    # ================Epson===============

    return np.array([x, y, z]) * 180 / np.pi


def eulerrot(x_rot=0, y_rot=0, z_rot=0):
    deg2rad = np.pi / 180
    x_rotmat = cv2.Rodrigues(np.array([x_rot * deg2rad, 0, 0]))[0]
    y_rotmat = cv2.Rodrigues(np.array([0, y_rot * deg2rad, 0]))[0]
    z_rotmat = cv2.Rodrigues(np.array([0, 0, z_rot * deg2rad]))[0]

    return np.dot(np.dot(x_rotmat, y_rotmat), z_rotmat)


def rotateuvw(pos_org, x_rot=0, y_rot=0, z_rot=0):
    if len(pos_org) == 3:
        pos = np.copy(pos_org)
    if len(pos_org) == 6:
        pos = np.copy(pos_org[3:])

    rotmat = eulerrot(x_rot, y_rot, z_rot)

    pos_mat = RotationTrans(pos)
    print('pos_mat', pos_mat)

    posrot_mat = np.dot(pos_mat, rotmat)
    print('posrot_mat', posrot_mat)
    posrot = rotationMatrixToEulerAngles(posrot_mat)

    if len(pos_org) == 6:
        posrot = np.append(pos_org[:3], posrot)
    print('posrot', posrot)

    return posrot


def rt_4x4(rvecs, tvecs):
    Rvecs = RotationTrans(rvecs)
    Tvecs = tvecs

    RPlanePos = Rvecs
    TPlanePos = Tvecs.reshape(3, 1)
    return np.r_[np.c_[RPlanePos, TPlanePos], [[0, 0, 0, 1]]]


def calM2C(img, mtx, id_order):
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters_create()
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
    print('markerCorners\n', markerCorners)
    print('markerIds\n', markerIds)

    # img_draw = cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
    # s = img_draw.shape
    # print(s)
    # img_draw = cv2.resize(img_draw, (int(s[1] / 2), int(s[0] / 2)))
    # cv2.imshow('', img_draw)
    # cv2.waitKey(0)

    # markerCorners_tmp=np.zeros(markerCorners.shape)
    markerCorners_tmp = np.copy(markerCorners)
    for i in range(len(id_order)):
        tmp = np.where(markerIds == id_order[i])[0][0]
        print(tmp)
        print(markerCorners[tmp])
        markerCorners_tmp[i] = markerCorners[tmp]

    markerCorners = markerCorners_tmp

    M2C_R, M2C_T, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 35, mtx, distCoeffs=None)
    print('M2C_T\n', M2C_T)
    for i in range(4):
        img_draw = cv2.aruco.drawAxis(img,mtx,None,M2C_R[i],M2C_T[i],50)
    cv2.imshow('', img_draw)
    cv2.waitKey(0)

    M2C = []
    for i in range(2):
        M2C.append(rt_4x4(M2C_R[i, 0], M2C_T[i, 0]))

    markerCenters = np.average(markerCorners, axis=2)

    return M2C, markerCorners, markerCenters.reshape(2, 2)


def calM2B(G2B_R, G2B_T, C2G, M2C, depth=None):
    G2B = rt_4x4(G2B_R, G2B_T)
    print('G2B_R\n', G2B_R)
    print('G2B_T\n', G2B_T)
    print('G2B\n', G2B)

    # replace marker depth
    if type(depth) == float:
        print('M2C[2, 3]', M2C[2, 3])
        M2C[2, 3] = depth
        print('Marker2Camera', M2C)
        print('depth', depth)
    else:
        print('None')

    print('C2G', C2G)
    # calculate marker2base rt
    C2B = np.dot(G2B, C2G)
    print('Camera2Base', C2B)
    M2B = np.dot(C2B, M2C)
    print('Marker2Base', M2B)

    M2B_R = np.array([M2B[0][:3], M2B[1][:3], M2B[2][:3]])
    M2B_T = np.array([M2B[0][3], M2B[1][3], M2B[2][3]])
    print('Marker2Base_r', M2B_R)
    print('Marker2Base_t', M2B_T)

    return M2B_R, M2B_T


def calM2B_R(M2B_T):
    M2B_t_vect = np.zeros((3, 3))

    M2B_t_vect[0] = M2B_T[1] - M2B_T[0]
    M2B_t_vect[1] = np.array([0,1,0])

    crossvect = np.cross(M2B_t_vect[0], M2B_t_vect[1])
    norvect = crossvect / np.sqrt(np.sum(crossvect ** 2))

    M2B_t_vect[2] = norvect

    M2B_t_calrot = M2B_t_vect.T

    w = ''
    M2F_t = np.array([[w, 0, 0], [0, 1, 0], [0, 0, 1]])
    M2F_t_calrot = M2F_t.T

    M2B_r_refined = np.dot(M2B_t_calrot, np.linalg.pinv(M2F_t_calrot))

    u, s, v = np.linalg.svd(M2B_r_refined)
    M2B_r_refined = np.dot(u, v)

    M2B_r_refined_uvw = rotationMatrixToEulerAngles(M2B_r_refined)
    print('M2B_r_refined_uvw:', M2B_r_refined_uvw)

    F2B = np.zeros((4, 4))
    F2B[:3, :3] = M2B_r_refined
    F2B[:3, 3] = M2B_T[0]
    F2B[3, 3] = 1

    return F2B, np.append(M2B_T[0], M2B_r_refined_uvw)



def goDetectMarker():
    pos = np.array([685.153, 164.948, 672.308, -165.662, -3.228, -44.487])
    id_order = np.array([[1], [2]])

    # move to capture marker
    # self.robot.set_TMPos(pos)

    img = cv2.imread('color_tm4.png')
    fs = cv2.FileStorage('tm/Extrinsic.txt', cv2.FILE_STORAGE_READ)
    fn = fs.getNode("intrinsic")
    mtx = fn.mat()
    fn = fs.getNode('Camera2Gripper')
    c2g = fn.mat()

    M2C, markerCorners, markerCenters = calM2C(img, mtx, id_order)
    print('markerCenters', markerCenters)

    M2B_R = []
    M2B_T = []
    for i in range(2):
        # depth = self.LiDar.depth_frame.as_depth_frame().get_distance(int(markerCenters[i][0]), int(markerCenters[i][1]))*1000
        M2B_R_tmp, M2B_T_tmp = calM2B(pos[3:], pos[:3], c2g, M2C[i], depth=None)
        M2B_R.append(M2B_R_tmp)
        M2B_T.append(M2B_T_tmp)

    print(M2B_R)
    for i in range(2):
        print('Marker', i)
        print(M2B_T[i])

    F2B, pos = calM2B_R(M2B_T)

    pos[2] = pos[2] + 400

    print('F2B', F2B)
    print('pos', pos)

    # self.robot.set_TMPos(pos)


# goDetectMarker()