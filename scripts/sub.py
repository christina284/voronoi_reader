#!/usr/bin/env python
import rospy
from tuw_multi_robot_msgs.msg import Graph, Vertex
import numpy as np
from geometry_msgs.msg import Point
from shapely.geometry import Point as Pnt2
from shapely.geometry import MultiPoint
from itertools import combinations
from voronoi_reader.msg import Vertx, Edges, PointList
import csv
import pickle
import triangle
from shapely.ops import triangulate
from rospy_message_converter import message_converter
import pandas as pd
from alphashape import alphashape
import matplotlib.pyplot as plt
from descartes import PolygonPatch
from CGAL.CGAL_Kernel import Point_2
from CGAL import CGAL_Alpha_shape_2
from itertools import combinations 
from geometry_msgs.msg import PolygonStamped, Polygon, PointStamped
import shapely.geometry as sg
import shapely.ops as so
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.srv import GetMap
import cv2
from scipy.spatial import Delaunay
import alphashape
from descartes import PolygonPatch
from alpha_shape import alpha_shape
from matplotlib.tri import Triangulation
from uuid import uuid1
import copy
from shapely.geometry import LineString, Point, asMultiPoint, Polygon



class Vertx:
    def __init__(self, node_id=np.nan, pos=(0, 0), degree=0, width=0, edges=[]):
        self.node_id = node_id
        self.pos = pos
        self.degree = degree
        self.width = width
        self.edges = edges


class Edge:

    def __init__(self, edge_id=0, source=0, target=-1, line=[], length=0, width=1):
        self.edge_id = edge_id
        self.source = source
        self.target = target
        self.line = line
        self.length = length
        self.width = width


def callback(msg):

    obst_dist = 0.
    delaunay_thres = 6  # user defined delaunay_thresold
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

    # convert back to Graph msg
    def convert_back(vor):
        print('Got in conversion')
        v_msg = []
        for i in range(len(vor['Edges'])):
            # msg : id, valid, path, suc, pred, width, weight
            # vtx:  node_id=np.nan, pos=(0, 0), degree=0, width=0, edges=[])
            # edge :edge_id=0, source=0, target=-1, line=[], length=0, ray='False'):
            # if i == 5 :
            #     print('id 5 ', vor['Edges'][i].__dict__)
         
            v = Vertex(i, True, [], [], [], 0, 0)

            try:
                s_node = [x for x in vor['Vertices'] if x.node_id == vor['Edges'][i].source][0]
            except IndexError:
                print("error")
                # print( [x.node_id for x in vor['Vertices']], 'edz', vor['Edges'][i].__dict__ )

            v.successors = s_node.edges

            v.path = vor['Edges'][i].line
            v.width = s_node.width
            v.weight = 1
            v.predecessors = []
            # if i == 7 :
            #     print('id 7 ', vor['Edges'][i].__dict__, v)
            v_msg.append(v)
            del v
        return v_msg


    def comp_dist(path):
        xcoords, ycoords = [], []
        for j in range(len(path)):
            xcoords.append(path[j].x)
            ycoords.append(path[j].y)
        sum_dist = 0
        for k in range(len(xcoords) - 1):
            sum_dist += np.sqrt((xcoords[k + 1] - xcoords[k])
                                ** 2 + (ycoords[k + 1] - ycoords[k]) ** 2)
        return sum_dist

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

    print('Got in 2')



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

    # algorithm 2: Skipping vertices
    v0 = []  # vertices to keep
    for i in VertxArray_unq:
        if i.degree == 1 or i.degree > 2:
            v0.append(i)

    # vertices to skip
    v_skipped = [x for x in VertxArray_unq if x not in v0]  # V^VD1\v0

    def skipping_edges(ver, edges, v0):

        """
        :param ver: all vertices
        :param edges: all edges
        :param v0: nodes with degree != 2
        :return: E: new edges
        """
        print('skipping edges')
        pickle.dump(ver, open("/home/chris/Desktop/ver.pkl", "wb"))
        pickle.dump(v0, open("/home/chris/Desktop/v0.pkl", "wb"))
        pickle.dump(edges, open("/home/chris/Desktop/edges.pkl", "wb"))

        ids_gone = []
        E = []
        for i in range(len(edges)):
            e_curr = edges[i]
            if e_curr.edge_id in ids_gone:
                continue
            pt = []

            p1 = copy.copy(e_curr.source)
            # if e_curr starts from a deleted node, skip
            while True:
                # find if e_curr starts from a node with deg != 2
                src = [x for x in v0 if x.node_id == p1]
                
                # if it doesnt, it means its src has 2 edges, we go to the other one
                if not src:
                    p1_node = [x for x in ver if x.node_id == p1][0]
                    pt = p1_node.node_id
                    # go to the next edge
                    e_curr = [x for x in edges if x.edge_id in p1_node.edges and x != e_curr][0]
                    # [value_false, value_true][<test>]
                    # if the source of the edges is the same, go to the other target node
                    p1 = [e_curr.source, e_curr.target][p1 == e_curr.source]
                else:
                    print('destuck at ', pt, "from ", src[0].node_id)
                    break
            v_curr = [x for x in ver if x.node_id == p1][0]  # p1_node
            l_sum = 0
            e_new = Edge(edge_id=e_curr.edge_id, source=v_curr.node_id, target=-1, line=[], length=0)
            edges_joined = []
            while True:
                l_sum += e_curr.length
                e_new.line.extend(e_curr.line)
                ids_gone.append(e_curr.edge_id)
                # find next vertex that is on the edge and it not the current
                # it searches in ver because some edges might end in vertices that are skipped
                # v_next = [x for x in ver if e_curr.edge_id in x.edges and x != v_curr][0]
                edge_ends = [e_curr.source, e_curr.target]
                v_next = [x for x in ver if x.node_id in edge_ends and x.node_id != v_curr.node_id][0]
                e_new.target = v_next.node_id
                e_new.line.extend([v_next.pos])
                edges_joined.append(e_curr.edge_id)

                if v_next.degree != 2:
                    break
                # find the other edge of v_next
                e_next = [x for x in edges if x.edge_id in v_next.edges and x != e_curr][0]
                e_curr = e_next
                v_curr = v_next
            # e_new.line.extend(e_curr.line)
            # change the edges id on vertices, to curr edge id
            for vtx in v0:
                # [x + 1 if x >= 45 else x + 5 for x in l]
                vtx.edges = list(set([e_new.edge_id if x in edges_joined else x for x in vtx.edges]))
                vtx.degree = len(vtx.edges)
            e_new.length = l_sum    
            E.append(e_new)
            del e_new
        return E

    VD1 = VD1_1.copy()
    e2 = skipping_edges(VD1['Vertices'], VD1['Edges'], v0)

    VD2_1 = {'Vertices': v0, 'Edges': e2}   # graph after skip edges and skip vertices
    v2_msg12 = convert_back(VD2_1)
    new2_msg12 = msg
    new2_msg12.vertices = v2_msg12
    pub4 = rospy.Publisher('skipping_edges', Graph, queue_size=1)
    pub4.publish(new2_msg12)

    VD2 = VD2_1.copy()

    def intermediates(p1, p2, parts):
        p = zip(np.linspace(p1.x, p2.x, parts+1),
                    np.linspace(p1.y, p2.y, parts+1))
        path = [Point(x,y,0) for (x,y) in p]
        return path

    def vertex_merging(ver, edges, sigma):
        print('merging vertices..')
        for e in edges:
            if e.length < sigma:
                # merge source and target
                n1 = [x for x in ver if x.node_id == e.source][0]
                n2 = [x for x in ver if x.node_id == e.target][0]
                pos_of_scs = n1.pos
                pos_of_tgt = n2.pos
                # find their middlepoint
                midpoint = Point((pos_of_scs.x+pos_of_tgt.x)/2, (pos_of_scs.y+pos_of_tgt.y)/2, 0.0)

                # edges of nodes to be merged
                edges_ = list(set(n1.edges + n2.edges))

                # find edges of nodes by their ids
                edges_ = [x for x in edges if x.edge_id in edges_ if x != e]

                # find which edges starts or ends in the deleted vertices
                targs = [x for x in edges_ if x.target == n1.node_id or x.target == n2.node_id]
                scs = [x for x in edges_ if x.source == n1.node_id or x.source == n2.node_id]
                # let the new node have the node_id of n1 (source of edge w/ len<sigma)
                # we change targets and sources of edges of the nodes to be merged
                for k in targs:
                    k.target = n1.node_id
                for o in scs:
                    o.source = n1.node_id
    
                # delete nodes and edge 
                ver = [x for x in ver if x not in [n1, n2]]
                edges = [x for x in edges if x != e]
                # create new node
                print('ver',len(ver), 'edg',len(edges))
                new_node = Vertx(node_id=n1.node_id, edges=[x.edge_id for x in edges_],
                                    degree=len(edges_), pos=midpoint, width=0)
                ver.extend([new_node])
                # we recreate the edges, with new lengths and lines from the new_node
                for k in (targs + scs):
                    t = [x for x in ver if x.node_id == k.target][0]
                    s = [x for x in ver if x.node_id == k.source][0]
                    k.line = intermediates(s.pos, t.pos, 10)
                    k.length = comp_dist(k.line)
        
        return ver, edges

    # algorithm 5
    v3, e3 = vertex_merging(VD2['Vertices'], VD2['Edges'], vertex_merging_thresh)
    VD3_1 = {'Vertices': v3, 'Edges': e3}   # graph after merging vertices  

    v2_msg13 = convert_back(VD3_1)
    new2_msg13 = msg
    new2_msg13.vertices = v2_msg13
    pub4 = rospy.Publisher('vertex_merging', Graph, queue_size=1)
    pub4.publish(new2_msg13)

    def short_deadend_removal(v):
        edges = v['Edges']
        vertices = v['Vertices']
        # short dead end removal
        for i in edges:
            scs = [x for x in vertices if x.node_id == i.source][0]
            tgt = [x for x in vertices if x.node_id == i.target][0]
            dead_end = [x for x in [scs, tgt] if x.degree == 1]
            if dead_end == []:
                continue
            else:
                dead_end = dead_end[0]
            if dead_end and i.length < dead_end_max_length:
                i.edge_id = 10000
                dead_end.node_id = 10000

        edges_ = [x for x in edges if x.edge_id != 10000]
        vertices_ = [x for x in vertices if x.node_id != 10000]
        return edges_, vertices_

    VD3 = VD3_1.copy()
    e4, v4 = short_deadend_removal(VD3.copy())
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
    # map_ = map_ > 0
    img = map_.astype(np.uint8)
    cv2.imwrite('/home/chris/Desktop/map.jpg', map_1)

    # imgray =img
    # # imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(imgray, 0, 255, 0)
    # ver = pointz
    # gray = np.float32(imgray)
    # dst = cv2.cornerHarris(gray, 5, 5, 0.04)

    # # result is dilated for marking the corners, not important
    # # dst = cv2.dilate(dst, None)
    # ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    # dst = np.uint8(dst)

    # # find centroids
    # ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # # define the criteria to stop and refine the corners
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    # corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
    # img[dst > 0 * dst.max()] = [0, 0, 255]
    # # corners = corners.astype(int)
    # # font = cv2.FONT_HERSHEY_SIMPLEX
    # # plt.scatter((corners[:, 0]), (corners[:, 1]))
    # # plt.imshow(img)
    # # for idx, value in enumerate(corners):
    # #     plt.text((value[0]), (value[1]), str(idx), fontsize=12, color='white')
    # # plt.show()


    # sgms = []
    # # x_offset = -61.7999992371
    # # y_offset = -26.6000003815
    # x_offset = 0
    # y_offset = 0
    # # for i in range(len(contours)):
    # res = 0.05
    # for i in range(len(ver)):
    #     ver[i] = (ver[i][0]/res + x_offset, ver[i][1]/res + y_offset)
    #     # ver[i] = (ver[i][0], ver[i][1])

    # vr = asMultiPoint(ver)
    # segms2, idc, v3 = [], [], []
    # # for i in range(len(corners)-1):
    # p = [[31, 27], [27, 8], [27, 30], [8, 7], [7, 29], [29, 13], [13, 12], [12, 3], [3, 4], [4, 11], [11, 14], [14, 24],
    #      [24, 25], [25, 21], [21, 28], [28, 22], [21, 5], [5, 6], [6, 17], [17, 18], [18, 1], [1, 2], [2, 19], [19, 15],
    #      [15, 20], [15, 9], [9, 10], [10, 16], [16, 23], [23, 37], [37, 42], [42, 32], [32, 33], [33, 35], [33, 51],
    #      [51, 56], [56, 36], [36, 38], [38, 57], [38, 39], [38, 57], [57, 58], [58, 43], [43, 41], [43, 46], [46, 47],
    #      [46, 59], [59, 60], [60, 48], [48, 55], [55, 61], [61, 62], [62, 31]]

    # z = list(corners)
    # z.extend(ver)

    # # z = ver
    # plt.rcParams['agg.path.chunksize'] = 1000
    # ver = np.asarray(z, dtype=int)
    # segments = []
    # z = [(x,y) for (x,y) in z if (x,y) != [0,0]]
    # segments = np.asarray(p)
    # data = dict(vertices=z, segments=segments)
    # tri = triangle.triangulate(data, 'pA')
    # triangle.compare(plt, tri, data)
    # plt.show()



    print("MarkerArray creation")
    ma = MarkerArray()
    ma_temp = []
    for i in range(len(e4)):
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
        marker.points = e4[i].line
        ma_temp.append(marker)
        del marker
    ma = ma_temp
    vis_pub = rospy.Publisher('points_markers', MarkerArray, queue_size=1)
    vis_pub.publish( ma )



  
    # pointz = np.array(pointz)
    # tri = Delaunay(pointz)
    # # Separating small and large edges:
    # print("Delaunay edit")
    # small_edges = set()
    # large_edges = set()
    # for tr in tri.vertices:
    #     for i in range(3):
    #         edge_idx0 = tr[i]
    #         edge_idx1 = tr[(i+1)%3]
    #         if (edge_idx1, edge_idx0) in small_edges:
    #             continue  # already visited this edge from other side
    #         if (edge_idx1, edge_idx0) in large_edges:
    #             continue
    #         p0 = pointz[edge_idx0]
    #         p1 = pointz[edge_idx1]
    #         if np.linalg.norm(p1 - p0) <  delaunay_thres:
    #             small_edges.add((edge_idx0, edge_idx1))
    #         else:
    #             large_edges.add((edge_idx0, edge_idx1))

    # # plt.figure()
    # plt.plot(pointz[:, 0], pointz[:, 1], '.')
    # for i, j in small_edges:
    #     plt.plot(pointz[[i, j], 0], pointz[[i, j], 1], 'b')
    # for i, j in large_edges:
    #     plt.plot(pointz[[i, j], 0], pointz[[i, j], 1], 'c')
    # plt.show()






    print('done, lets go convert back')
    

    # v2_msg = convert_back(VD1_1)
    # new2_msg = msg
    # new2_msg.vertices = v2_msg
    # pub4 = rospy.Publisher('graph_after_prunning', Graph, queue_size=1)
    # pub4.publish(new2_msg)

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
