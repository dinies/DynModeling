/************** Simple didactic implementation of a search tree for unorganized point sets ***********/

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Eigenvalues>
#include <iostream>

#include "structs.hpp"


// uncomment the row below if you want to see debug messages
// #define _VERBOSE_BUILD_

using namespace Eigen;
using namespace std;

namespace dyn_modeling {
  // typedef for defining a vector variable sized points
  typedef std::vector<trail>
  VectorXdVector;

  /**
     BaseTreeNode class. Represents a base class for a node in the search tree.
     It hasa dimension (given on construction).
  */

  class BaseTreeNode{
  public:
    //! ctor
    BaseTreeNode(int dimension_){
      this->_dimension=dimension_;
    }

    //! dtor
    virtual ~BaseTreeNode(){}

    //! function to search for the neighbot
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    virtual double findNeighbor(trail& answer,
                                const trail& query,
                                const double max_distance) const =0;

    //dimension accessor;
    inline int dimension() const {return _dimension;}

  protected:
    int _dimension; //< the dimension of a node in the tree
  };

  /**
     Leaf node: it contains a vector of points on which.
     The search function performs a linear search in the list
  */

  class LeafNode: public BaseTreeNode{
  public:
    //! ctor
    LeafNode(int dimension_):BaseTreeNode(dimension_){}

    //! const accessor to the point vector
    const VectorXdVector& points() const {return _points;}

    //! accessor to the point vector
    VectorXdVector& points()  {return _points;}

    //! function to search for the neighbors. Performs a linear search in the vector
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    virtual double findNeighbor(trail& answer,
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

  protected:
    VectorXdVector _points; //< points stored in the leaf
  };


  /**
     Middle node: it represents a splitting plane, and has 2 child nodes,
     that refer to the set of points to the two sides of the splitting plane.
     A splitting plane is parameterized as a point on a plane and as a normal to the plane.
  */
  class MiddleNode: public BaseTreeNode{
  public:
    //! ctor
    //! @param mean_: point on the split plane
    //! @param normal_: normal vector to the plane
    //! param left_child: left subtree
    //! param right_child: right subtree
    MiddleNode(int dimension_,
               const Eigen::VectorXd& mean_,
               const Eigen::VectorXd& normal_,
               BaseTreeNode* left_child=0,
               BaseTreeNode* right_child=0) :BaseTreeNode(dimension_){
      assert(normal_.rows()==dimension());
      assert(mean_.rows()==dimension());
      _normal=normal_;
      _mean=mean_;
      _left_child=left_child;
      _right_child=right_child;
    }

    //! dtor
    virtual ~MiddleNode() {
      if(_left_child)
        delete _left_child;
      if (_right_child)
        delete _right_child;
    }

    //! mean const accessor
    inline const Eigen::VectorXd& mean() const {return _mean;}

    //! normal const accessor
    inline const Eigen::VectorXd& normal() const {return _normal;}

    //! checks if a point lies to the left or right of a plane
    //! @param query_point: the point to be checked
    //! @returns true if a point lies to the left, false otherwise
    inline bool side(const trail& query_point) const {
      return _normal.dot(query_point.lineCenterCoords -_mean)<0;
    }


    //! binary search for the neighbors. Performs a linear search in the vector
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    virtual double findNeighbor(trail& answer,
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

  protected:
    Eigen::VectorXd _normal;
    Eigen::VectorXd _mean;
    BaseTreeNode* _left_child;
    BaseTreeNode* _right_child;
  };


  /**
     Partitions a point vector in two vectors, computing the splitting plane
     as the largest eigenvalue of the point covariance
     @param mean: the returned mean of the splitting plane
     @param normal: the normal of the splitting plane
     @param left: the returned left vector of points
     @param right: the returned right vector of points
     @param points: the array of points
     @returns the distance of the farthest point from the plane
  */
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

#if _VERBOSE_BUILD_
    cerr << "splitting, num points: " << points.size() << endl;
    cerr << "normal: " << normal.transpose() << endl;
    cerr << "mean:" << mean.transpose() << endl;
#endif

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
#if _VERBOSE_BUILD_
    cerr << "distance from plane: " << max_distance_from_plane;
    cerr << "num left: " << num_left << endl;
    cerr << "num_right: " << num_right << endl;
#endif
    return max_distance_from_plane;
  }

  //! helper function to buil;d a tree
  //! @param points: the points
  //! @param max_leaf_range: specify the size of the "box" below which a leaf node is generated
  //! returns the root of the search tree
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

    MiddleNode* node=new MiddleNode(dimension,
                                    mean,
                                    normal,
                                    buildTree(left_points,max_leaf_range), // left child
                                    buildTree(right_points,max_leaf_range) // right child
                                    );
    return node;
  }

}
