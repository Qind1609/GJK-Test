#include <iostream>
#include "mex.hpp"
#include "mexAdapter.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>

namespace PS = CGAL::Polyline_simplification_2;

typedef CGAL::Exact_predicates_inexact_constructions_kernel                             K;
typedef PS::Vertex_base_2<K>                                                            Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>                                  Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                                    TDS;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, CGAL::Exact_predicates_tag>  CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT>                                     CT;

typedef CT::Point                             Point;
typedef CT::Constraint_id                     Constraint_id;
typedef CT::Constraint_iterator               Constraint_iterator;
typedef CT::Vertices_in_constraint_iterator   Vertices_in_constraint_iterator;
typedef CT::Points_in_constraint_iterator     Points_in_constraint_iterator;
typedef PS::Stop_below_count_ratio_threshold  Stop;
typedef PS::Squared_distance_cost             Cost;

using namespace matlab::data;
using matlab::mex::ArgumentList;

class MexFunction : public matlab::mex::Function {
public:
  explicit MexFunction();
  ~MexFunction();
  void operator()(ArgumentList outputs, ArgumentList inputs)
  {
  // Validate arguments
  checkArguments(outputs, inputs);
  
  CT ct;
  std::vector<Point> points;
  const bool remove_points = false;
  
  ArrayFactory factory;

  // convert lat/lon inputs to vector of points
  TypedArray<double> lat = std::move(inputs[0]);
  TypedArray<double> lon = std::move(inputs[1]);
  const double stop_threshold = inputs[2][0];

  for (int i = 0; i < lat.getNumberOfElements(); i++)
  {
    points.push_back(Point(lat[i], lon[i]));
  }
  
  // Insert constraint
  Constraint_id cid = ct.insert_constraint(points);
  
  // Simplify
  PS::simplify(ct, cid, Cost(), Stop(stop_threshold), remove_points);
  
  // Create output
  std::vector<double> outputLat;
  std::vector<double> outputLon;

  for(Vertices_in_constraint_iterator vit = ct.vertices_in_constraint_begin(cid);
      vit != ct.vertices_in_constraint_end(cid); vit++)
  {
    outputLat.push_back((*vit)->point().x());
    outputLon.push_back((*vit)->point().y());
  }
  

  std::vector<std::vector<double>> origin_pts;
  std::vector<double> origin_pts_Lat;
  std::vector<double> origin_pts_Lon;
  
  for(Points_in_constraint_iterator pit = ct.points_in_constraint_begin(cid);
      pit != ct.points_in_constraint_end(cid); pit++)
  {
    origin_pts.push_back({pit->x(), pit->y()});
    origin_pts_Lat.push_back(pit->x());
    origin_pts_Lon.push_back(pit->y());
  }
  
  std::cout << "The number of points in original object: " << origin_pts.size() << std::endl;
  std::cout << "The number of points in simplified object: " << ct.vertices_in_constraint(cid).size() << std::endl;

  TypedArray<double> outputLatArray = factory.createArray<double>({1, outputLat.size()}, outputLat.data(), outputLat.data() + outputLat.size());
  TypedArray<double> outputLonArray = factory.createArray<double>({1, outputLon.size()}, outputLon.data(), outputLon.data() + outputLon.size());
  TypedArray<double> originPtsLatArray = factory.createArray<double>({1, origin_pts_Lat.size()}, origin_pts_Lat.data(), origin_pts_Lat.data() + origin_pts_Lat.size());
  TypedArray<double> originPtsLonArray = factory.createArray<double>({1, origin_pts_Lon.size()}, origin_pts_Lon.data(), origin_pts_Lon.data() + origin_pts_Lon.size());
  
  CellArray output_1 = factory.createCellArray({1,2}, outputLatArray, outputLonArray);
  CellArray output_2 = factory.createCellArray({1,2}, originPtsLatArray, originPtsLonArray);
  // Assign outputs
  outputs[0] = output_1;
  outputs[1] = output_2;
};

  void checkArguments(ArgumentList outputs, ArgumentList inputs);

};

// Ctor & Dtor
MexFunction::MexFunction()
{

}

MexFunction::~MexFunction()
{

}

void MexFunction::checkArguments(ArgumentList outputs, ArgumentList inputs)
{
  std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
  ArrayFactory factory;
  // check number of inputs
  if (inputs.size() != 3)
  {
      matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Require Latitude and Longitude array and stop threshold") }));
  }
  // check type of inputs
  if (inputs[0].getType() != ArrayType::DOUBLE ||
      inputs[0].getType() == ArrayType::COMPLEX_DOUBLE ||
      inputs[1].getType() != ArrayType::DOUBLE ||
      inputs[1].getType() == ArrayType::COMPLEX_DOUBLE)
  {
    matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("1st and 2nd arguments must be double array") }));
  }

  if (inputs[2].getType() != ArrayType::DOUBLE ||
      inputs[2].getType() == ArrayType::COMPLEX_DOUBLE)
  {
    matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("3rd argument must be double") }));
  }

  // check size of inputs
  if (inputs[0].getNumberOfElements() != inputs[1].getNumberOfElements())
  {
    matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Latitude and Longitude array must have the same size") }));
  }

  //check number of outputs
  if (outputs.size() != 2)
  {
     matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Require 2 outputs array") }));
  }
}

