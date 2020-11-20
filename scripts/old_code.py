        
    # pickle.dump(points, open('/home/chris/Desktop/vtx.pickle', 'wb'))
    # Define alpha parameter
    # alpha = 0.

    # # # Generate the alpha shape
    # alpha_shape = alphashape(points, alpha)
    # pickle.dump(points, open('/home/chris/Desktop/alphashape.pickle', 'wb'))
    # np.savetxt('/home/chris/Desktop/vtx.csv',points,delimiter=',')
    # print(type(alpha_shape))
    # if alpha_shape.geom_type == 'Polygon':
    #     for xc,yc in zip(*alpha_shape.exterior.coords.xy):
    #         p = Point(x=xc + msg.origin.position.x, y=yc + msg.origin.position.y, z=0.0)
    #         points2.append(p)
    #     Poli = Polygon(points2)
    # # elif alpha_shape.geom_type == 'MultiPolygon':
    # else:
    #     polygons = list(alpha_shape)
    #     new_shape = so.cascaded_union(polygons)
    #     for geom in new_shape.geoms:    
    #         for xc,yc in zip(*geom.exterior.coords.xy):
    #             p = Point(x=xc + msg.origin.position.x, y=yc + msg.origin.position.y, z=0.0)
    #             points2.append(p)
    #     Poli = Polygon(points2)
    # else:
    #     print(type(alpha_shape))


    # # Initialize plot
    # fig, ax = plt.subplots()
    # # Plot input points
    # ax.scatter(*zip(*points))

    # # Plot alpha shape
    # # ax.add_patch(PolygonPatch(alpha_shape, alpha=.2))

    # # plt.show()
    # polstmp = PolygonStamped(header=msg.header, polygon=Poli)
    # pub2 = rospy.Publisher('alpha_shape', PolygonStamped, queue_size=1)
    # pub2.publish(polstmp)


 ashape = alpha_shape(np.array(pointz), 1)
    
    print(ashape)
    # np.random.seed(0)
    # x = 3.0 * np.random.rand(2000)
    # y = 2.0 * np.random.rand(2000) - 1.0
    # inside = ((x ** 2 + y ** 2 > 1.0) & ((x - 3) ** 2 + y ** 2 > 1.0))
    # pointz = np.vstack([x[inside], y[inside]]).T

    # Computing the alpha shape
    edges = alpha_shape(pointz, alpha=0.25, only_outer=True)

    # Plotting the output
    # plt.figure()
    # plt.axis('equal')
    # plt.plot(pointz[:, 0], pointz[:, 1], '.')
    # for i, j in ashape:
    #     plt.plot(pointz[[i, j], 0], pointz[[i, j], 1])
    # plt.show()
    # print(alpha_shape)
    # fig, ax = plt.subplots()
    # ax.scatter(xs, ys)
    # poly = sg.Polygon([[p[1], p[0]] for p in ptLst_temp])
    # ax.add_patch(PolygonPatch(alpha_shape, alpha=0.2))
    # plt.show()
    # poli = PolygonStamped()
    # poli.header = msg.header
    # poli.polygon.points = ptLst_temp
    # pol = rospy.Publisher('poligon', PolygonStamped, queue_size=1)
    # pol.publish( poli )

    # Spawn new windows that shows us the donut
    # (in grayscale) and the detected contour
    # cv2.imshow('Donut', im) 
    # cv2.imshow('Output Contour', out)

    # # Wait indefinitely until you push a key.  Once you do, close the windows
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # Test of only two points to see visualization
    # pp = []
    # nzptz  = []
    # for i in range(1,3):
    #     pl = Vertex(i, True, [], 0, 0, [], [])
    #     # 'id', 'valid', 'path', 'weight', 'width', 'successors', 'predecessors']
    #     # pl.source = i
    #     pl.path = [Point(x = (i-1)**2+ 1.0 ,y = (i-1)*2 + 0.0, z=0.0), Point(x = i**2 + 1.0 ,y = i*2 + 0.0, z=0.0)]
    #     print('pl path', pl.path)
    #     pp.extend(pl.path)
    #     print('pp', pp)
    #     nzptz.append(pl)
    #     del pl
    # # pp = [p[0] for p in pp]
    # vvv = [Vertex(1, True, pp, 0, 0, [] ,[])]
    # new2_msg = msg
    # new2_msg.vertices = nzptz

    # pub4 = rospy.Publisher('graph_after_prunning', Graph, queue_size=1)
    # pub4.publish(new2_msg)
    # new3_msg = msg
    # new3_msg.vertices = vvv
    # pub3 = rospy.Publisher('graph_after_prunning1', Graph, q(ueue_size=1)
    # pub3.publish(new3_msg))








I THINK THIS IS EXACTLY THE SAME, BUT JUST IN CASE


    # def vertex_merging(ver, edges, sigma):
#     print('merging vertices..')
#     for e in edges:
#         if e.length < sigma:
#             # merge source and target
#             n1 = [x for x in ver if x.node_id == e.source][0]
#             n2 = [x for x in ver if x.node_id == e.target][0]
#             pos_of_scs = n1.pos
#             pos_of_tgt = n2.pos
#             # find their middlepoint
#             midpoint = Point((pos_of_scs.x+pos_of_tgt.x)/2, (pos_of_scs.y+pos_of_tgt.y)/2, 0.0)

#             # edges of nodes to be merged
#             edges_ = list(set(n1.edges + n2.edges))

#             # find edges of nodes by their ids
#             edges_ = [x for x in edges if x.edge_id in edges_ if x != e]

#             # find which edges starts or ends in the deleted vertices
#             targs = [x for x in edges_ if x.target == n1.node_id or x.target == n2.node_id]
#             scs = [x for x in edges_ if x.source == n1.node_id or x.source == n2.node_id]
#             # let the new node have the node_id of n1 (source of edge w/ len<sigma)
#             # we change targets and sources of edges of the nodes to be merged
#             for k in targs:
#                 k.target = n1.node_id
#             for o in scs:
#                 o.source = n1.node_id

#             # delete nodes and edge 
#             ver = [x for x in ver if x not in [n1, n2]]
#             edges = [x for x in edges if x != e]
#             # create new node
#             print('ver',len(ver), 'edg',len(edges))
#             new_node = Vertx(node_id=n1.node_id, edges=[x.edge_id for x in edges_],
#                                 degree=len(edges_), pos=midpoint, width=0)
#             ver.extend([new_node])
#             # we recreate the edges, with new lengths and lines from the new_node
#             for k in (targs + scs):
#                 t = [x for x in ver if x.node_id == k.target][0]
#                 s = [x for x in ver if x.node_id == k.source][0]
#                 k.line = intermediates(s.pos, t.pos, 10)
#                 k.length = comp_dist(k.line)
    
#     return ver, edges