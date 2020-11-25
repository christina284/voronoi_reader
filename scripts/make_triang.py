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


def find_corners(float_img, show):
    dst = cv2.cornerHarris(float_img, 5, 5, 0.09)

    # result is dilated for marking the corners, not important
    # dst = cv2.dilate(dst, None)
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(float_img, np.float32(centroids), (5, 5), (-1, -1), criteria)
    # print(img.shape)
    corners = corners.astype(int)
    if show:
        plt.scatter((corners[:, 0]), (corners[:, 1]))
        plt.imshow(float_img)
        for idx, value in enumerate(corners):
            plt.text((value[0]), (value[1]), str(idx), fontsize=12, color='white')
        plt.show()
    return corners, dst


def make_triang(img, ver, outline, corners):
    print('We are starting triang')

    corners = [(x[1], x[0]) for x in corners]
    sgms = []
    # x_offset = -61.7999992371
    # y_offset = -26.6000003815
    x_offset = 0
    y_offset = 0
    res = 0.05
    for i in range(len(ver)):
        ver[i] = (ver[i][0]/res + x_offset, ver[i][1]/res + y_offset)

        # ver[i] = (ver[i][0], ver[i][1])
    # ver = np.array(ver, dtype =int)
    # plt.scatter((ver[:, 0]), (ver[:, 1]))
    # plt.imshow(gray)
   
    #
    p = outline
    z = list(corners)
    z.extend([(int(x[0]), int(x[1])) for x in ver])
    # z = list(set(z))
    # z.extend(list(set([(int(x[0]), int(x[1])) for x in ver])))
    # z = ver
    plt.rcParams['agg.path.chunksize'] = 1000
    # ver = np.asarray(z, dtype=int)
    segments = []
    segments = np.asarray(p)
    data = dict(vertices=z, segments=segments)
    z = np.array(z)
    # plt.scatter((z[:, 0]), (z[:, 1]))
    # plt.imshow(img)
    # for idx, value in enumerate(z):
    #     plt.text((value[0]), (value[1]), str(idx), fontsize=12, color='white')
    # plt.show()
    # for l in segments:
    #     point1 = z[l[0]]
    #     point2 = z[l[1]]
    #     x_values = [point1[0], point2[0]]

    #     y_values = [point1[1], point2[1]]
    #     plt.scatter((z[:, 0]), (z[:, 1]))
    #     for idx, value in enumerate(z):
    #         plt.text((value[0]), (value[1]), str(idx), fontsize=12, color='red')
    #     plt.plot(x_values, y_values)
    # plt.show()
    # print(segments)
    tri = triangle.triangulate(data, 'epq0')
    # triangle.compare(plt, tri, data)
    # plt.show()
    tri['vertices'] = tri['vertices'].astype(int)
    z1 = tri['vertices']


    # plt.scatter((z1[:, 0]), (z1[:, 1]))
    # plt.imshow(img)
    # for idx, value in enumerate(z1):
    #     plt.text((value[0]), (value[1]), str(idx), fontsize=12, color='white')
    # plt.show()
    # z = z1
    small_edges = set()
    large_edges = set()
    for k, tr in enumerate(tri['triangles']):
        for i in range(3):
            edge_idx0 = tr[i]
            edge_idx1 = tr[(i+1)%3]
            if (edge_idx1, edge_idx0) in small_edges:

                continue  # already visited this edge from other side
            if (edge_idx1, edge_idx0) in large_edges:
                continue
        
            p0 = z[edge_idx0]
            p1 = z[edge_idx1]
            outline_list = [l for l in outline]
            if [edge_idx0, edge_idx1] in outline_list or [edge_idx1, edge_idx0] in outline_list:
                small_edges.add((edge_idx0, edge_idx1))
                print('added ', [edge_idx0, edge_idx1])
            if np.linalg.norm(p1 - p0) < 25:
                small_edges.add((edge_idx0, edge_idx1))
            else:
                large_edges.add((edge_idx0, edge_idx1))

    plt.figure()
    plt.plot(z[:, 0], z[:, 1], '.')
    for i, j in small_edges:
 
        plt.plot(z[[i, j], 0], z[[i, j], 1], 'red')

    # for i, j in large_edges:
    #     plt.plot(z[[i, j], 0], z[[i, j], 1], 'black')

    plt.show()
    # triangle.compare(plt, tri, data)
    # plt.show()
    (w, h, z1) = img.shape
    z = z.astype(int)
    # new_img = np.zeros((w, h), dtype=np.uint8)
    # for i in range(len(z[:, 0])):
    #     new_img[z[i, 1], z[i, 0]] = 1

    # for i, j in small_edges:
    #     new_img[z[[i, j], 1], z[[i, j], 0]] = 1


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
    return tri

