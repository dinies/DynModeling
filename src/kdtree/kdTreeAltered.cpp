// Created by Dinies on 15/10/2018.
#include "kdTreeAltered.hpp"
using namespace Eigen;
using namespace std;


namespace dyn_modeling {


  BaseTreeNode::BaseTreeNode(int dimension_){
    this->_dimension=dimension_;
  }


  LeafNode::LeafNode(int dimension_)
    :BaseTreeNode(dimension_)
  {}


  double LeafNode::findNeighbor(trail& answer,
      const trail& query,
      const double max_distance) const{
    float d_max=std::numeric_limits<double>::max();
    trail t;
    for(size_t i=0; i<_points.size(); i++){
      t = _points[i];
      float d=(t.lineCenterCoords -query.lineCenterCoords ).squaredNorm();
      if (d<d_max){
        answer=_points[i];
        d_max=d;
      }
    }
    if (d_max>max_distance*max_distance)
      return -1;
    return d_max;
  }

  MiddleNode::MiddleNode(int dimension_,
      const Eigen::VectorXd& mean_,
      const Eigen::VectorXd& normal_,
      BaseTreeNode* left_child,
      BaseTreeNode* right_child):
    BaseTreeNode(dimension_)
  {
    assert(normal_.rows()==dimension());
    assert(mean_.rows()==dimension());
    _normal=normal_;
    _mean=mean_;
    _left_child=left_child;
    _right_child=right_child;
  }

  MiddleNode::~MiddleNode() {
    if(_left_child)
      delete _left_child;
    if (_right_child)
      delete _right_child;
  }



  double MiddleNode::findNeighbor
    (trail& answer,
     const trail& query,
     const double max_distance) const{

      bool is_left=side(query);
      if(is_left && _left_child) {
        return _left_child->findNeighbor(answer, query, max_distance);
      }
      if(!is_left && _right_child) {
        return _right_child->findNeighbor(answer, query, max_distance);
      }
      return -1;
    }

  double splitPoints(VectorXd& mean, VectorXd& normal,
      VectorXdVector& left, VectorXdVector& right,
      const VectorXdVector& points){

    trail t;
    // if points empty, nothing to do
    if(! points.size()){
      left.clear();
      right.clear();
      return 0;
    }

    // retrieve the dimension deom the point set
    int dimension=points[0].lineCenterCoords.rows();

    // compute the mean;
    mean=Eigen::VectorXd(dimension);
    mean.setZero();
    for (size_t i=0; i<points.size(); i++){
      t = points[i];
      assert(t.lineCenterCoords.rows()==dimension); // sanity check
      mean+=t.lineCenterCoords;
    }
    double inverse_num_points=1.0d/points.size();
    mean*=inverse_num_points;

    // compute the covariance;
    MatrixXd covariance(dimension, dimension);
    covariance.setZero();
    for (size_t i=0; i<points.size(); i++){
      t = points[i];
      VectorXd delta= t.lineCenterCoords -mean;
      covariance+=delta*delta.transpose();
    }
    covariance*=inverse_num_points;

    // eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver;
    solver.compute(covariance, Eigen::ComputeEigenvectors);
    normal=solver.eigenvectors().col(dimension-1).normalized();

    // the following var will contain the rang of points along the normal vector
    float max_distance_from_plane=0;

    // run through the points and split them in the left or the right set
    left.resize(points.size());
    right.resize(points.size());
    int num_left=0;
    int num_right=0;

    for (size_t i=0; i<points.size(); i++){
      t = points[i];
      double distance_from_plane=normal.dot(t.lineCenterCoords -mean);
      if (fabs(distance_from_plane)>max_distance_from_plane)
        max_distance_from_plane=fabs(distance_from_plane);

      bool side=distance_from_plane<0;
      if (side) {
        left[num_left]=points[i];
        num_left++;
      } else {
        right[num_right]=points[i];
        num_right++;
      }
    }
    left.resize(num_left);
    right.resize(num_right);

    return max_distance_from_plane;
  }

  BaseTreeNode* buildTree(const VectorXdVector& points,
      double max_leaf_range) {
    if (points.size()==0)
      return 0;
    int dimension=points[0].lineCenterCoords.rows();

    VectorXd mean;
    VectorXd normal;
    VectorXdVector left_points;
    VectorXdVector right_points;

    double range=splitPoints(mean, normal,
        left_points, right_points,
        points);

    if (range<max_leaf_range){
      LeafNode* node=new LeafNode(dimension);
      node->points()=points;
      return node;
    }

    MiddleNode* node=new MiddleNode
      (dimension,
       mean,
       normal,
       buildTree(left_points,max_leaf_range), // left child
       buildTree(right_points,max_leaf_range) // right child
      );
    return node;
  }
}

