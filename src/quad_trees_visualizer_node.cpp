#include "quadtrees/QuadTree.h"
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

using std::vector;

QuadTree *quad_tree = nullptr;
ros::Publisher marker_pub;
std::string global_frame_id = "map";
nav_msgs::OccupancyGrid::ConstPtr current_map;

std_msgs::ColorRGBA getColorFromValue(int value) {
  std_msgs::ColorRGBA color;
  color.a = 0.7;

  if (value == 0) {
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
  } else if (value == -1) {
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
  } else if (value == 100) {
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
  } else {
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
  }

  return color;
}

void addLeafNodesToMarkerArray(visualization_msgs::MarkerArray &marker_array,
                               QuadTreeNode *node, int &marker_id,
                               const std::string &frame_id,
                               const nav_msgs::OccupancyGrid::ConstPtr &map) {
  if (!node)
    return;

  if (node->is_leaf) {
    float world_x =
        node->x * map->info.resolution + map->info.origin.position.x;
    float world_y =
        node->y * map->info.resolution + map->info.origin.position.y;
    float world_width = node->width * map->info.resolution;
    float world_height = node->height * map->info.resolution;

    visualization_msgs::Marker fillMarker;
    fillMarker.header.frame_id = frame_id;
    fillMarker.header.stamp = ros::Time::now();
    fillMarker.ns = "quadtree_fill";
    fillMarker.id = marker_id;
    fillMarker.type = visualization_msgs::Marker::CUBE;
    fillMarker.action = visualization_msgs::Marker::ADD;

    fillMarker.pose.position.x = world_x + world_width / 2.0;
    fillMarker.pose.position.y = world_y + world_height / 2.0;
    fillMarker.pose.position.z = 0;

    fillMarker.pose.orientation.x = 0.0;
    fillMarker.pose.orientation.y = 0.0;
    fillMarker.pose.orientation.z = 0.0;
    fillMarker.pose.orientation.w = 1.0;

    fillMarker.scale.x = world_width;
    fillMarker.scale.y = world_height;
    fillMarker.scale.z = 0.01;

    fillMarker.color = getColorFromValue(node->value);
    fillMarker.color.a = 0.5;
    fillMarker.lifetime = ros::Duration(1.0);
    marker_array.markers.push_back(fillMarker);

    visualization_msgs::Marker borderMarker;
    borderMarker.header.frame_id = frame_id;
    borderMarker.header.stamp = ros::Time::now();
    borderMarker.ns = "quadtree_border";
    borderMarker.id = marker_id++;
    borderMarker.type = visualization_msgs::Marker::LINE_LIST;
    borderMarker.action = visualization_msgs::Marker::ADD;
    borderMarker.scale.x = 0.1;
    borderMarker.color.r = 1.0;
    borderMarker.color.g = 1.0;
    borderMarker.color.b = 1.0;
    borderMarker.color.a = 1.0;

    float x1 = world_x;
    float y1 = world_y;
    float x2 = world_x + world_width;
    float y2 = world_y + world_height;
    float z = 0.0;

    geometry_msgs::Point p;

    p.x = x1;
    p.y = y1;
    p.z = z;
    borderMarker.points.push_back(p);
    p.x = x2;
    p.y = y1;
    p.z = z;
    borderMarker.points.push_back(p);

    p.x = x2;
    p.y = y1;
    p.z = z;
    borderMarker.points.push_back(p);
    p.x = x2;
    p.y = y2;
    p.z = z;
    borderMarker.points.push_back(p);

    p.x = x2;
    p.y = y2;
    p.z = z;
    borderMarker.points.push_back(p);
    p.x = x1;
    p.y = y2;
    p.z = z;
    borderMarker.points.push_back(p);

    p.x = x1;
    p.y = y2;
    p.z = z;
    borderMarker.points.push_back(p);
    p.x = x1;
    p.y = y1;
    p.z = z;
    borderMarker.points.push_back(p);

    borderMarker.lifetime = ros::Duration(1.0);
    marker_array.markers.push_back(borderMarker);
  } else {
    for (int i = 0; i < 4; i++) {
      if (node->children[i] != nullptr) {
        addLeafNodesToMarkerArray(marker_array, node->children[i], marker_id,
                                  frame_id, map);
      }
    }
  }
}

void publishQuadTreeVisualization() {
  if (!quad_tree || !quad_tree->root || !current_map)
    return;

  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;

  addLeafNodesToMarkerArray(marker_array, quad_tree->root, marker_id,
                            global_frame_id, current_map);

  marker_pub.publish(marker_array);
  ROS_DEBUG("Published QuadTree visualization with %lu markers",
            marker_array.markers.size());
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Received map metadata:");
  ROS_INFO("Width: %d, Height: %d", msg->info.width, msg->info.height);

  current_map = msg;

  int width = msg->info.width;
  int height = msg->info.height;
  vector<vector<int>> grid(height, vector<int>(width));

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      grid[y][x] = msg->data[index];
    }
  }

  int x = 0;
  int y = 3;

  grid[y][x] = -1;

  ROS_INFO("Map built successfully.");
  if (!quad_tree) {
    quad_tree = new QuadTree();
  }
  quad_tree->build(grid);

  ROS_INFO("QuadTree built successfully with leaf nodes: %d",
           quad_tree->getNumLeaves());

  /* vector<QuadTreeNode *> adjacent_leaf_nodes = */
  /*     quad_tree->getAdjacentLeafNodes(x, y); */
  /**/
  /* ROS_INFO("Number of adjacent Leaf nodes of (%d, %d) -> %ld", x, y, */
  /*          adjacent_leaf_nodes.size()); */
  /**/
  /* for (int i = 0; i < adjacent_leaf_nodes.size(); i++) { */
  /*   ROS_INFO( */
  /*       "Adjacent to: (%d, %d) -> %d", adjacent_leaf_nodes[i]->x, */
  /*       adjacent_leaf_nodes[i]->y, */
  /*       quad_tree->query(adjacent_leaf_nodes[i]->x,
   * adjacent_leaf_nodes[i]->y)); */
  /* } */

  global_frame_id = msg->header.frame_id;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quad_trees_visualizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double viz_rate = 10.0;
  private_nh.param("visualization_rate", viz_rate, viz_rate);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "quadtree_visualization", 1);

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);

  ros::Timer viz_timer = nh.createTimer(
      ros::Duration(1.0 / viz_rate),
      [](const ros::TimerEvent &) { publishQuadTreeVisualization(); });

  ROS_INFO("QuadTree visualizer started. Publishing at %.1f Hz", viz_rate);

  ros::spin();

  delete quad_tree;
  return 0;
}
