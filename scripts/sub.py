#!/usr/bin/env python3
from __future__ import division 

import rospy
from tuw_multi_robot_msgs.msg import Graph, Vertex
import numpy as np
from geometry_msgs.msg import Point
from shapely.geometry import Point as Pnt2
from voronoi_reader.msg import Vertx, Edges, PointList
import csv
from fn import *
import pickle
from make_triang import *
import triangle
import matplotlib.pyplot as plt
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.srv import GetMap
import cv2
import os, sys
# from skimage.draw import line
import time

def callback(msg):
    # 0.2 , 4, 0.75
    obst_dist = 0.2
    dead_end_max_length = 4
    vertex_merging_thresh = 1
    # print(msg.header.info)
    vd = msg.vertices

    # create list of nodes
     # create list of nodes
    VertxArray = []
    for i in range(len(vd)):
        v = Vertx(node_id=i, pos=vd[i].path[0], degree=0, width=vd[i].width, edges=[])
        VertxArray.append(v)
        del v

    

    VertxArray_unq = []
    positions = []
    # delete multiple vertices with same position
    for i in VertxArray:
        if i.pos not in positions:
            VertxArray_unq.append(i)
            positions.append(i.pos)

    EdgesArray = []
    max_len = max([x.node_id for x in VertxArray_unq])
    it = 0
    for i in range(len(vd)):
        # we iterate at vd because we want all the edges
        e = Edge(edge_id=i, source=0, target=-1, line=vd[i].path, length=0, width=vd[i].width)

        # find start of nodes
        e.source = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[0]][0]

        # find node_id that corresponds to the end of the edge
        e.target = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[-1]]

        # if at the end there is no vertex, create a vertex
        if not e.target:
            it += 1
            v = Vertx(node_id = max_len + it, pos=e.line[-1], degree=0, width=1, edges=[])
            e.target = v.node_id
            VertxArray_unq.append(v)
            del v
        else:
            e.target = e.target[0]

        # find length of edge
        e.length = comp_dist(vd[i].path)

        EdgesArray.append(e)
        del e




    # algorithm 1:-Filter with obstacle distance
    e1 = [x for x in EdgesArray if x.width > obst_dist]
    VD0 = {'Vertices': VertxArray_unq, 'Edges': EdgesArray}
    # find degree - edges that start or end in a vertex
    for i in VertxArray_unq:
        i.edges = [x.edge_id for x in e1 if x.line[0] == i.pos or x.line[-1] == i.pos]
        i.degree = len(i.edges)
        if i.degree == 0:
            i.node_id = 10000
    VertxArray_unq = [x for x in VertxArray_unq if x.node_id != 10000]


    VD0 = {'Vertices': VertxArray_unq, 'Edges': EdgesArray}
    VD1_1 = {'Vertices': VertxArray_unq, 'Edges': e1}

    v2_msg11 = convert_back(VD1_1)
    new2_msg11 = msg
    new2_msg11.vertices = v2_msg11
    pub4 = rospy.Publisher('filter_w_obst_dist', Graph, queue_size=1)
    pub4.publish(new2_msg11)

   

    v0 = skip_vertices(VertxArray_unq)
    # vertices to skip
    v_skipped = [x for x in VertxArray_unq if x not in v0]  # V^VD1\v0

   

    VD1 = VD1_1.copy()
    e2 = skipping_edges(VD1['Vertices'], VD1['Edges'], v0)

    VD2_1 = {'Vertices': v0, 'Edges': e2}   # graph after skip edges and skip vertices
    v2_msg12 = convert_back(VD2_1)
    new2_msg12 = msg
    new2_msg12.vertices = v2_msg12
    pub4 = rospy.Publisher('skipping_edges', Graph, queue_size=1)
    pub4.publish(new2_msg12)

    VD2 = VD2_1.copy()

   
    # algorithm 5
    v3, e3 = vertex_merging(VD2['Vertices'], VD2['Edges'], vertex_merging_thresh)
    VD3_1 = {'Vertices': v3, 'Edges': e3}   # graph after merging vertices  

    v2_msg13 = convert_back(VD3_1)
    new2_msg13 = msg
    new2_msg13.vertices = v2_msg13
    pub4 = rospy.Publisher('vertex_merging', Graph, queue_size=1)
    pub4.publish(new2_msg13)

   

    VD3 = VD3_1.copy()
    e4, v4 = short_deadend_removal(VD3.copy(), dead_end_max_length)


    VD4 = {'Vertices': v4, 'Edges': e4}   # graph after deleting short dead ends  
    v2_msg14 = convert_back(VD4)
    new2_msg14 = msg
    new2_msg14.vertices = v2_msg14
    pub4 = rospy.Publisher('short_Deadend_removal', Graph, queue_size=1)
    pub4.publish(new2_msg14)


    # alphashape
    points, pointz, points2, points3, xs, ys = [], [], [], [], [], []
    for i in v4:
        point = Point(i.pos.x, i.pos.y, 0)
        point3 = Pnt2(i.pos.x, i.pos.y)
        point1 = (i.pos.x, i.pos.y)
        xs.append(i.pos.x)
        ys.append(i.pos.y)
        points.append(point)
        pointz.append(point1)
        points3.append(point3)
    pickle.dump(pointz, open("/home/chris/Desktop/pts.pkl", "wb"))


    # take map from service
    print('Waiting for service....')
    rospy.wait_for_service('static_map')

    try:
        stc_map = rospy.ServiceProxy('static_map', GetMap)
        print('Service called')

    except rospy.ServiceException:
        print('Service call failed')

    p =  stc_map()

    # edit map and find outer shape
    tmp = np.array(p.map.data)
    map_ = np.reshape(tmp, (p.map.info.height, p.map.info.width))
    kernel = np.ones((5, 5), np.uint8) 
    map_ = np.float32(map_)
    img_dilation = cv2.dilate(map_, kernel, iterations=1) 
    img_erosion = cv2.erode(img_dilation, kernel, iterations=1) 
    # img_dilation = cv2.dilate(img_erosion, kernel, iterations=1) 
    # img_erosion = cv2.dilate(img_dilation, kernel, iterations=1) 

    # map_ = map_ > 0
    img = map_.astype(np.uint8)
    cv2.imwrite('/home/chris/Desktop/map.jpg', map_)
    # triangs = make_triang(img, pointz)  # should change the corners 
    corners, _ = find_corners(img_erosion, False)
    current_dict = VD4
    # cv2.imshow('g', img_erosion)
    # cv2.waitKey(0)
    def map_on_map(curr_dict):
        # x_offset = msg.origin.position.x
        # y_offset = msg.origin.position.y
        x_offset = 0
        y_offset = 0
        res = 0.05
        v_on_map = [(x.pos.x/res + x_offset, x.pos.y/res + y_offset) for x in curr_dict['Vertices']]
        edges_on_map = [ [((i.x/res - x_offset), (i.y/res - y_offset)) for i in ej.line ] for ej in curr_dict['Edges'] ]
        return v_on_map, edges_on_map

    v_on_map, edges_on_map = map_on_map(current_dict)
    walls = np.where(img_dilation == 100)
    idx_walls = list(zip(*walls))
    corners = np.flip(corners, axis=1)
    corners = list(map(tuple, corners))
    cn = [(x[0], x[1]) for x in corners if x in idx_walls]
    print('found ', len(cn), 'corners in map')


    x_ = [x[1] for x in cn]
    y_ = [x[0] for x in cn]

    # visualize edges_on_map
    # implot = plt.imshow(map_)
    # # for i in range(len(edges_on_map)):
    # #     plt.plot(*np.array(edges_on_map[i]).T)
    # plt.scatter(x=walls[1], y=walls[0], c='b', s=2)
    # plt.scatter(x_, y_, c='r', s =4)
    # plt.show()



    def find_interm(p1, p2):
        ps = intermediates(Point(p1[0], p1[1]), Point(p2[0], p2[1]), 50)
        ps = [(int(o.x), int(o.y)) for o in ps]
        return ps


    # find outline
    empty_ar = np.zeros(map_.T.shape)
    it = 1
    it_size = len(list(itertools.combinations(cn, 2)))
    lines = []
    for i, j in itertools.combinations(cn, 2):

        t1= time.time()
        ppts = []
        ps = find_interm(i, j)

        ppts = [x for x in ps if x in idx_walls]
        R = 0
        it +=1
        if len(ppts)==1*len(ps):
            # cv2.line(empty_ar, i, j, (255,255,255), 1) 
            lines.append([cn.index(i), cn.index(j)])
        t2 = time.time()
        time_per_it = (t2-t1)/60 # in mins
        time_whole = time_per_it * it_size
        print("iteration ", it, "/", it_size, 'mins left: ', time_whole - it*time_per_it)

    lines_2 = lines
    for l in lines:
        A = l[0]
        B = l[1]
        # print(['A ', A, 'B ', B])
        lines_with_A = []
        for l1 in [x for x in lines if x not in [l]]:
            if l1[0] == A or l1[1] == A:
                lines_with_A.append(l1)
                # print(['lines with A ', lines_with_A])
        for l2 in lines_with_A:
            C = [x for x in l2 if x not in [A]][0]  # find the other corner
            for l3 in lines:
                if l3[0] == C or l3[1] == C:
                    # print(['this is lines with C ', l3, ' <--------'])
                    other_corner_of_c = [x for x in l3 if x not in [C, [A]]]
                    if other_corner_of_c[0] == B:
                        # print('A and B are uselless because line l3 connects them ')
                        # print('l ', l, 'line w/ A ', l2, 'line w/ B', l3)
                        AB = [Point(cn[A][0], cn[A][1]), Point(cn[B][0], cn[B][1])]
                        AC = [Point(cn[A][0], cn[A][1]), Point(cn[C][0], cn[C][1])]
                        BC = [Point(cn[C][0], cn[C][1]), Point(cn[B][0], cn[B][1])]
                        dAB = comp_dist(AB)
                        dAC = comp_dist(AC)
                        dBC = comp_dist(BC)
                        max_len = max([dAB, dAC, dBC])
                        if dAB == max_len:
                            to_del = l
                        elif dAC == max_len:
                            to_del = l2
                        else:
                            to_del = l3
                        lines_2 = [x for x in lines_2 if x != to_del]
                    
            


    # cv2.imshow('g', empty_ar)
    # cv2.waitKey(0)

    with open('lines.pkl', 'wb') as f:
        pickle.dump(lines_2, f)
    # A -----C----------B
    with open('lines.pkl', 'rb') as f:
        lines_2 = pickle.load(f)     



    # delete "duplicate lines", eg there are lines like AB,BC, AC, delete AC
    # for every line (AB), go to the "beggining" node (A), and find all lines getting out of it (A~)  
    # for every line that gets out of it, we find its other end node (C)
    # for this node, find again all lines (C~)
    
  

    print('done with outline')

    for l in lines_2:
        point1 = cn[l[0]]
        point2 = cn[l[1]]
        x_values = [point1[0], point2[0]]

        y_values = [point1[1], point2[1]]
        plt.plot(x_values, y_values)
    # plt.scatter(x=walls[1], y=walls[0], c='b', s=2)
    # plt.scatter(x_, y_, c='r', s =4)
    plt.show()
    tri = make_triang(map_, pointz, lines_2, cn)







    print("MarkerArray creation")
    ma = MarkerArray()
    ma_temp = []
    for i in range(len(current_dict['Edges'])):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = i
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.position.x = msg.origin.position.x
        marker.pose.position.y = msg.origin.position.y
        marker.pose.position.z = msg.origin.position.z
        marker.pose.orientation.x = msg.origin.orientation.x
        marker.pose.orientation.y = msg.origin.orientation.y
        marker.pose.orientation.z = msg.origin.orientation.z
        marker.pose.orientation.w = msg.origin.orientation.w
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = current_dict['Edges'][i].line
        ma_temp.append(marker)
        del marker
    ma = ma_temp
    vis_pub = rospy.Publisher('points_markers', MarkerArray, queue_size=1)
    vis_pub.publish( ma )




    print('done, lets go convert back')
    

    # v2_msg = convert_back(VD1_1)
    # new2_msg = msg
    # new2_msg.vertices = v2_msgimport shapely.geometry as sg


    return vd


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber("segments", Graph, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
