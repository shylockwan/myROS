#ifndef __DBSCAN_H__
#define __DBSCAN_H__

#include <vector>
#include <iterator>
#include <functional>

template <typename InputIterator> struct Clusterer_DBSCAN {

	typedef typename std::iterator_traits<InputIterator>::value_type point_type;
	typedef point_type& point_ref;
	typedef typename std::function<double(point_ref, point_ref)> metric;

	struct Point {
		point_ref P;
		std::vector<size_t> n_list;
		int cluster;
		Point(point_ref p) : P(p), n_list(), cluster(-1) {};
	};

	std::vector<Point> points;

	Clusterer_DBSCAN( InputIterator begin, InputIterator end ) : points() {
		for ( auto i=begin; i!=end; ++i ) {
			points.push_back(Point(*i));
		}
	};

	virtual ~Clusterer_DBSCAN() {};

	virtual int cluster( double eps, size_t minPts, metric distance );

protected:
	virtual void init_data(void);
	virtual void calc_graph( double eps, metric distance );
	virtual void grow_cluster( Point& p, size_t minPts, int Cluster );

};

// Clear all previous clustering information and initialize data
template<typename T> inline void Clusterer_DBSCAN<T>::init_data(void) {
	//for ( auto&& i : points ) {
	for(int index=0;index<points.size();index++){	
		struct Point i=points[index];
		i.cluster = -1;
		i.n_list.clear();
	}
	return;
}

// Construct a graph connecting all points within a hypersphere of radius eps in the given metric
template<typename T> void Clusterer_DBSCAN<T>::calc_graph( double eps, typename Clusterer_DBSCAN<T>::metric distance ) {
	for ( auto i = 0; i < points.size()-1; ++i ) {
		for ( auto j = i+1; j < points.size(); ++j ) {                 // we iterate over all unique index pairs ( i, j>i )
			if ( distance(points[i].P, points[j].P) <= eps ) {           // the distance is calculated via the user specified metric
				points[i].n_list.push_back(j);                             // if the points are within the distance eps
				points[j].n_list.push_back(i);                             // we add the other point's index to its list of neighbours
			}
		}
	}
}

// Grow the cluster with index Cluster beginning from a selected point
template<typename T> void Clusterer_DBSCAN<T>::grow_cluster( Point& p, size_t minPts, int Cluster ) {
	if ( p.cluster != -1 ) return;                                   // In case, a point is already assigned to a cluster, we are already finished
	p.cluster = Cluster;                                             // Otherwise we add the point to the given cluster
	if ( p.n_list.size() >= minPts ) {                               // If the inspected point is a core point, i.e. has at least minPts neighbours
		//  for ( auto&& i : p.n_list ) {  
		for(int index=0;index< p.n_list.size();index++){// those points are also added to the given cluster
			int i=p.n_list[index];
			grow_cluster( points[i], minPts, Cluster );                  // this is continued recursively until all core and border points are assigned
		}
	}
	return;
}

// Start a new clustering attempt. Choose a hypersphere of radius 'eps' in the metric 'distance'. Core points have at least 'minPts' neighbours
template<typename T> int Clusterer_DBSCAN<T>::cluster( double eps, size_t minPts,typename Clusterer_DBSCAN<T>::metric distance ) {

	init_data();                                                     // remove all previoues clustering information and initialize data
	calc_graph( eps, distance );                                     // calculate the connectivity graph of points sharing metric spheres of radius eps

	int maxCluster = -1;                                             // so far, we haven't found any clusters
	// for ( auto&& p:points ) {     
	for(int i=0;i<points.size();i++){								// We iterate over all points which haven't been assigned to a cluster, yet,
		struct Point p=points[i];
		if ( p.cluster == -1 && p.n_list.size() >= minPts ) {          // and which have at least minPts neighbours
			grow_cluster( p, minPts, ++maxCluster );                     // From these points we grow a new cluster
		}
	}                                                                // Now, all cluster should be found and all unassigned points are considered noise
	return maxCluster ;                                              // So we return the number of clusters found
}

#endif

