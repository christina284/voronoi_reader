#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <tuw_multi_robot_msgs/Graph.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Point_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>
#include <fstream>
#include <iostream>
#include <list>
#include <vector>
#include "voronoi_reader/PointList.h"
#include "voronoi_reader/Segment_msg.h"
#include "geometry_msgs/Point.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef K::FT                                                FT;
typedef K::Point_2                                           Point2;
typedef K::Segment_2                                         Segment;
typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>          Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds>                Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;
typedef geometry_msgs::Point                                 Point3;

template <class OutputIterator>
void alpha_edges( const Alpha_shape_2& A, OutputIterator out)
{
  Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
                             end = A.alpha_shape_edges_end();
  for( ; it!=end; ++it)
    *out++ = A.segment(*it);
}
// template <class OutputIterator>
// bool file_input(OutputIterator out)
// {
//   std::ifstream is("./data/fin", std::ios::in);
//   if(is.fail())
//   {
//     std::cerr << "unable to open file for input" << std::endl;
//     return false;
//   }
//   int n;
//   is >> n;
//   std::cout << "Reading " << n << " points from file" << std::endl;
//   std::copy_n(std::istream_iterator<Point>(is), n, out);
//   return true;
// }



  void chatterCallback(const voronoi_reader::PointList::ConstPtr& msg)
{
   
    std::list<Point2> points;
    for (int i = 0; i < msg->pts.size(); i++)
    {
      points.push_back(Point2(msg->pts[i].x, msg->pts[i].y));
    }
    // if(! file_input(std::back_inserter(points)))
    // return -1;
    Alpha_shape_2 A(points.begin(), points.end(), FT(10000), Alpha_shape_2::GENERAL);
    std::vector<Segment> segments;
    alpha_edges(A, std::back_inserter(segments));
    std::cout << "Alpha Shape computed" << std::endl;
    std::cout << segments.size() << " alpha shape edges" << std::endl;
    std::cout << "Optimal alpha: " << *A.find_optimal_alpha(1)<<std::endl;
    
    
    std::list<Segment> sgmnt_temp;
    for (int i = 0; i < segments.size(); i++)
    {
      Point2 p1;
      Point2 p2;
      p1 = segments[i].source();
      p2 = segments[i].target();

      geometry_msgs::Point pp;  // double
      pp.x = p1->x;
      pp.y = p2.y;
      // voronoi_reader::PointList p_tmp = {pp(p1.x, p1.y, 0.0), pp(p2.x, p2.y, 0.0)};
      // p_tmp.pts = ();
      voronoi_reader::PointList p_tmp();
      sgmnt_temp.push_back(p_tmp);

    }
    voronoi_reader::Segment_msg sgmnt(sgmnt_temp); 




    ros::Publisher chatter_pub = n.advertise<Segment_msg>("alpha_shape", 1000);
    chatter_pub.publish(segments);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "alpha_shape");    
    public ros::NodeHandle n;   
    ROS_INFO("Initialize sub and pub");
    ros::Subscriber sub = n.subscribe("point_list", 1000, chatterCallback);



    ros::Rate loop_rate(10);   

    ros::spin();

    return 0;
}