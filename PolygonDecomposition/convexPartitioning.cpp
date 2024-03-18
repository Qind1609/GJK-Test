#include <iostream>
#include "mex.hpp"
#include "mexAdapter.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <vector>
#include <cassert>
#include <list>

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef CGAL::Partition_traits_2<K>                                     Traits;
typedef Traits::Point_2                                                 Point_2;
typedef Traits::Polygon_2                                               Polygon_2;
typedef std::list<Polygon_2>                                            Polygon_list;
typedef Polygon_2::Vertex_iterator 			                                Vertex_iterator;

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

  TypedArray<double> lat = std::move(inputs[0]);
  TypedArray<double> lon = std::move(inputs[1]);
  const int mode = inputs[2][0];  
  ArrayFactory factory;
  Polygon_2 polygon;
  Polygon_list partition_polys;

  makePolygon(lat, lon, polygon);

  // check whether polygon is convex or not
  if (polygon.is_convex())
  {
    std::cout << "This polygon object is convex already, no need to perform convex partition." << std::endl;
    outputs[0] = factory.createCharArray("Polygon is already convex.");
    return;
  }

  //  check whether polygon is simple or not
  if (!polygon.is_simple())
  {
    std::cout << "This polygon object is not simple, cannot perform convex partition." << std::endl;
    //outputs[0] = factory.createCharArray("Polygon is not simple.");
    //return;
  }

  switch (mode)
  {
    case 0:
      std::cout << "Mode 0: Use Greene's dynamic programming algorithm for optimal convex partitioning" << std::endl;
      CGAL::optimal_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys));
      break;
    case 1:
      std::cout << "Mode 1: Use Hertel and Mehlhorn algorithm for approx convex partitioning" << std::endl;
      CGAL::approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys));
      break;
    case 2:
      std::cout << "Mode 2: Use sweep-line approximation algorithm of Greene for approx convex partitioning" << std::endl;
      CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys));
      break;
    case 3:
      std::cout << "Mode 3: Use monotone parition" << std::endl;
      CGAL::y_monotone_partition_2(polygon.vertices_begin(), polygon.vertices_end(), std::back_inserter(partition_polys));
    default:
      std::cout << "Invalid mode" << std::endl;
      ouputs[0] = factory.createCharArray("Invalid mode");
      return;
  }
    
  if (partition_polys.empty())
  {
    std::cout << "Convex partition list is empty" << std::endl;
    outputs[0] = factory.createCharArray("Convex partition list is empty");
    return;
  }

  if(CGAL::partition_is_valid_2(polygon.vertices_begin(), polygon.vertices_end(),
                                partition_polys.begin(), partition_polys.end()))
  {
      std::cout << "Partitionning succeed !!" << std::endl;
  }
  
  std::cout << "Number of partition polygons: " << partition_polys.size() << std::endl;
  CellArray output_1 = factory.createCellArray({1, partition_polys.size()});

  Polygon_list::iterator it;
  int index = 0;
  for (it = partition_polys.begin(); it != partition_polys.end(); it++)
  {
    Polygon_2 p = *it;
    size_t n = p.size();
          
    TypedArray<double> arr = factory.createArray<double>({n, 2});
    int i = 0;
    for (Vertex_iterator vi = p.vertices_begin(); vi != p.vertices_end(); vi++)
    {
      arr[i][0] = vi->x();
      arr[i][1] = vi->y();
      i++;
    }
    output_1[index] = arr;
    index++;
  }
  
  outputs[0] = output_1;
  
  return;
};

  void checkArguments(ArgumentList outputs, ArgumentList inputs);

  
  void makePolygon(TypedArray<double>& lat, TypedArray<double>& lon, Polygon_2& polygon);
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
      matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Template {array lat, array lon, int partition_mode}") }));
  }
  // check type of inputs
  if (inputs[0].getType() != ArrayType::DOUBLE ||
      inputs[0].getType() == ArrayType::COMPLEX_DOUBLE ||
      inputs[1].getType() != ArrayType::DOUBLE ||
      inputs[1].getType() == ArrayType::COMPLEX_DOUBLE)
  {
    matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Arguments 1 and 2 must be double array") }));
  }

  std::set<int> validMode = {0, 1, 2, 3};
  bool is_in = validMode.find(inputs[2][0]) != validMode.end();
  if (inputs[2].getType() != ArrayType::DOUBLE ||
      !is_in)
  {
    matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Arguments 3 must be in list {0,1,2,3}") }));
  }

  // check size of inputs
  if (inputs[0].getNumberOfElements() != inputs[1].getNumberOfElements())
  {
    matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Latitude and Longitude array must have the same size") }));
  }

  //check number of outputs
  if (outputs.size() != 1)
  {
     matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Only 1 output is generated") }));
  }
}

void MexFunction::makePolygon(TypedArray<double>& lat, TypedArray<double>& lon, Polygon_2& polygon)
  {
    for (int i = 0; i < lat.getNumberOfElements(); i++)
    {
      polygon.push_back(Point_2(lat[i], lon[i]));
    }
    // check whether polygon is clockwise oriented or not, if yes, reverse it
    if (polygon.is_clockwise_oriented())
    {
      std::cout << "Polygon is clockwise oriented, reverse it" << std::endl;
      polygon.reverse_orientation();
    }
  }