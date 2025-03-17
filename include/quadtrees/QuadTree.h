#ifndef QUAD_TREE_H
#define QUAD_TREE_H

#include <algorithm>
#include <climits>
#include <unordered_set>
#include <vector>

class QuadTreeNode {
public:
  int x, y;
  int width, height;
  int value;
  bool is_leaf;
  QuadTreeNode *children[4];

  QuadTreeNode(int x, int y, int width, int height, int value, bool is_leaf);
  ~QuadTreeNode();
};

class QuadTree {
private:
  std::vector<QuadTreeNode *> leaf_nodes;
  int max_depth;

  bool isHomogeneous(std::vector<std::vector<int>> &grid, int x, int y,
                     int width, int height);
  bool containsObstacle(std::vector<std::vector<int>> &grid, int x, int y,
                        int width, int height);
  QuadTreeNode *buildRecursive(std::vector<std::vector<int>> &grid, int x,
                               int y, int width, int height, int depth);
  void insertRecursive(QuadTreeNode *node, int x, int y, int value, int depth);
  int queryRecursive(QuadTreeNode *node, int x, int y);
  bool areNodesAdjacent(QuadTreeNode *node1, QuadTreeNode *node2);
  void collectLeafNodes(QuadTreeNode *node,
                        std::vector<QuadTreeNode *> &leaves);
  void
  findAdjacentLeafNodesRecursive(QuadTreeNode *node, QuadTreeNode *target_node,
                                 std::vector<QuadTreeNode *> &adjacent_node);

  bool couldContainAdjacentNodes(QuadTreeNode *node, QuadTreeNode *target_node);

public:
  QuadTreeNode *root;

  QuadTree(int max_depth);
  QuadTree();
  ~QuadTree();
  void build(std::vector<std::vector<int>> &grid);
  void insert(int x, int y, int value);
  int query(int x, int y);
  std::vector<QuadTreeNode *> getAdjacentLeafNodes(int x, int y);
  std::vector<QuadTreeNode *> getLeafNodes();
  std::vector<std::vector<int>> getGrid();
  int getNumLeaves();
  QuadTreeNode *findLeafNode(QuadTreeNode *node, int x, int y);
  void updateLeafNodesList();
  void deleteTree();
};

#endif
