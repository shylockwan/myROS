#include <iostream>
#include <vector>
#include "dbscan.h"

int main(int argc, char** argv) {

  // For this example we use a simple testset. The datatype of the testset does not matter at all
  // We can use std::vector, arrays, pointers, anything we can think of, as long as we can use
  // an std::InputIterator to forward iterate over the dataset

  std::vector<double> a = { 0.0, 0.3, 0.5, 0.7, 1.0, 2.0, 2.4, 2.6, 3.0, 3.5, 4.0, 9.0 };       // Testset as a std::vector
  // double a[] = { 0.0, 0.3, 0.5, 0.7, 1.0, 2.0, 2.4, 2.6, 3.0, 3.5, 4.0, 9.0 };               // or try an array instead

  // Creating the clusterer initializes the internal data structures
  // only with const references to the dataset member objects. Values are NOT copied.
  // Because the DBSCAN class depends on the iterator type of the begin and end iterators,
  // weinstanciate the template class with the calculate type of the input iterator

  Clusterer_DBSCAN<decltype(std::begin(a))> dbscan(std::begin(a), std::end(a) );

  // This is where the magic happens. To use the DBSCAN clusterer we have to provide
  // three parameters: epsilon, minPts and a distance metric. In this example, we use
  // the square of the euclidean metric as a lambda funtion. Also we provide eps as a
  // squared value, so this turns out to be identical to the normal euclidean metric
  // because if C >= sqrt( A*A + B*B ) then C*C >= A*A + B*B holds as well.

  // NOTE that because we specify a lambda funtion to calculate the distance between
  // two data set entries X and Y, the DBSCAN algorithm used does not need any
  // knowledge about the internal structure of the data set type. The only time, the 
  // data set is touched, is when the algorithm applies a pair of const references
  // to the distance (lambda) function.

  dbscan.cluster( 0.9*0.9, 3, [](const double& x, const double& y) -> double {  return (x-y)*(x-y); } );

  // The return type of the cluster function is the number of clusters found.
  // In case no clusters were found, -1 is returned.

  // After the clustering, the vector points of the Clusterer_DBSCAN class contains
  // a list of the referenced points of the dataset annotated with the calculated
  // cluster number. The cluster index is zero-based. Points not assigned to any
  // cluster have an index of -1. In this example, we just print a list of the points
  // in the data set as well as the cluster index they are assigned to.

  for( auto&& i : dbscan.points ) {
    std::cout << i.P << "  Cluster " << i.cluster << std::endl;
  }
  std::cout << std::endl;

  return 0;
}

