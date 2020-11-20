#!/usr/bin/env python3
import cv2
import numpy as np
import pickle
import matplotlib.pyplot as plt
import triangle
from shapely.ops import nearest_points
import random
import itertools
from shapely.geometry import LineString, Point, asMultiPoint, Polygon


def make_triang(img, ver):
    print('We are starting triang')
    img = cv2.imread("/home/chris/Desktop/map.jpg")
    # ver = pickle.load(open("/home/chris/Desktop/pts.pkl", 'rb'))
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray = np.float32(imgray)
    dst = cv2.cornerHarris(gray, 5, 5, 0.04)

    # result is dilated for marking the corners, not important
    # dst = cv2.dilate(dst, None)
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
    # print(img.shape)
    img[dst > 0 * dst.max()] = [0, 0, 255]
    corners = corners.astype(int)
    # font = cv2.FONT_HERSHEY_SIMPLEX

    sgms = []
    # x_offset = -61.7999992371
    # y_offset = -26.6000003815
    x_offset = 0
    y_offset = 0
    # for i in range(len(contours)):
    res = 0.05
    for i in range(len(ver)):
        ver[i] = (ver[i][0]/res + x_offset, ver[i][1]/res + y_offset)

        # ver[i] = (ver[i][0], ver[i][1])
    ver = np.array(ver)
    # plt.scatter((ver[:, 0]), (ver[:, 1]))
    # plt.imshow(gray)
    #
    # plt.show()
    # ver = np.array(sorted(ver, key=lambda row: row[0]))
    # vr = asMultiPoint(ver)
    for i in range(len(corners)):
        if i in [11,14,25, 28, 52, 53, 54, 0, 45, 44, 49, 50, 63, 26, 40]:
            corners[i] = [0, 0]

    #  corners = np.array([x for (k,x) in enumerate(corners) if k not in [11,14,25, 28, 52, 53, 54, 0, 45, 44, 49, 50, 63 ]])
    segms2, idc, v3 = [], [], []
    # for i in range(len(corners)-1):
    p = [[31, 27], [27, 8], [27, 30], [8, 7], [7, 29], [29, 13], [13, 12], [12, 3], [3, 4], [4, 24],
         [24, 21], [21, 22], [21, 5], [5, 6], [6, 17], [17, 18], [18, 1], [1, 2], [2, 19], [19, 15],
         [15, 20], [15, 9], [9, 10], [10, 16], [16, 23], [23, 37], [37, 42], [42, 32], [32, 33], [33, 35], [33, 51],
         [51, 56], [56, 36], [36, 38], [38, 57], [38, 39],  [57, 58], [58, 43], [43, 41], [43, 46], [46, 47],
         [46, 59], [59, 60], [60, 48], [48, 55], [55, 61], [61, 62], [62, 31]]
    # poli = []

    # plt.scatter((corners[:, 0]), (corners[:, 1]))
    # plt.imshow(img)
    # for idx, value in enumerate(corners):
    #     plt.text((value[0]), (value[1]), str(idx), fontsize=12, color='white')
    # plt.show()

    # pz = [(x) for xs in p for x in xs]
    # corners = np.array([x for (k,x) in enumerate(corners) if k in pz])
    # for i in range(len(p)):
    #     sgms.append(LineString([corners[p[i][1]], corners[p[i][0]]]))
    #     curr_line = [list(corners[p[i][1]]), list(corners[p[i][0]])]
    #     # poli.extend(curr_line)
    #     plt.plot(np.array(curr_line)[:, 0], np.array(curr_line)[:, 1])
        # # find nearest points of lines and vr
        # s1, v1 = nearest_points(sgms[-1], vr)
        # # find nearest points' indices in vr
        # idx, v2 = [(idx, (pt.x, pt.y)) for (idx, pt) in enumerate(vr) if pt == v1][0]
        # idc.append(idx)  # keeps the indices of nearest points in vr
        # v3.append(v2)  # keeps the coords of nearest points
    # plt.scatter((corners[:, 0]), (corners[:, 1]))
    # # plt.scatter(np.array(ver)[:, 0], np.array(ver)[:, 1])
    # plt.show()

    # Poli = Polygon(p)
    #
    #
    # def generate_random(number, pl):
    #     points = []
    #     minx, miny, maxx, maxy = pl.bounds
    #     while len(points) < number:
    #         pnt = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
    #         if pl.contains(pnt):
    #             points.append([pnt.x, pnt.y])
    #     return points
    # vv = generate_random(5, Poli)
    #
    z = list(corners)
    z.extend(ver)

    # z = ver
    plt.rcParams['agg.path.chunksize'] = 1000
    # ver = np.asarray(z, dtype=int)
    segments = []
    segments = np.asarray(p)
    data = dict(vertices=z, segments=segments)
    z = np.array(z)

    tri = triangle.triangulate(data, 'pAa100000')


    small_edges = set()
    large_edges = set()
    for k, tr in enumerate(tri['triangles']):
        for i in range(3):
            edge_idx0 = tr[i]
            edge_idx1 = tr[(i+1)%3]
            if (edge_idx1, edge_idx0) in small_edges:                # print('g')

                continue  # already visited this edge from other side
            if (edge_idx1, edge_idx0) in large_edges:
                continue
            p0 = z[edge_idx0]
            p1 = z[edge_idx1]
            lenth = np.linalg.norm(p1 - p0)
            if [edge_idx0, edge_idx1] in p or [edge_idx1, edge_idx0] in p:
                small_edges.add((edge_idx0, edge_idx1))
            if np.linalg.norm(p1 - p0) < 40:
                small_edges.add((edge_idx0, edge_idx1))
            else:
                large_edges.add((edge_idx0, edge_idx1))
    plt.figure()
    plt.plot(z[:, 0], z[:, 1], '.')
    for i, j in small_edges:
        plt.plot(z[[i, j], 0], z[[i, j], 1], 'b')
    for i, j in large_edges:
        plt.plot(z[[i, j], 0], z[[i, j], 1], 'black')
    plt.show()
    # triangle.compare(plt, tri, data)
    # plt.show()
    (w, h, z1) = img.shape
    z = z.astype(int)
    new_img = np.zeros((w, h), dtype=np.uint8)
    for i in range(len(z[:, 0])):
        new_img[z[i, 1], z[i, 0]] = 1

    for i, j in small_edges:
        new_img[z[[i, j], 1], z[[i, j], 0]] = 1


    # find contour

    centers = []
    z = np.array(z)
    # for i in tri['triangles']:
    #     centers.append(np.sum(z[i], axis=0, dtype='int') / 3.0)
    # colors = np.array([(x - w / 2.) ** 2 + (y - h / 2.) ** 2 for x, y in centers])
    # plt.tripcolor(z[:, 0], z[:, 1], tri['triangles'].copy(), facecolors=colors, edgecolors='k')
    # plt.gca().set_aspect('equal')
    # plt.show()
    print('end')

