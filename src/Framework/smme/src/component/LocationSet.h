/*
 * LocationSet.h
 *
 *  Created on: Sep 3, 2014
 *      Author: dan
 */

#ifndef LOCATIONSET_H_
#define LOCATIONSET_H_

#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <list>
#include <Geometry.h>

template<class POINT>
class LocationSet{

public:
	double radius;
	typedef std::list<POINT> POINTS;
	POINTS points;

	LocationSet(double r = 3):radius(r){}
	void insert(const POINT& p){
		points.push_back(p);
	}
	typedef typename POINTS::const_iterator const_iterator;
	typedef typename POINTS::iterator iterator;
	void erase(const_iterator i){
		points.erase(i);
	}
	bool nearby(const POINT& p, const POINT& n)const{
		btVector3 vp = btVector3(p.x, p.y, p.z); vp.setZ(0);
		btVector3 vn = btVector3(n.x, n.y, n.z); vn.setZ(0);
		return vp.distance(vn) < radius;
	}
	bool nearby_verb(const POINT& p, const POINT& n, std::ostream& out)const{
		btVector3 vp = btVector3(p.x, p.y, p.z); vp.setZ(0);
		btVector3 vn = btVector3(n.x, n.y, n.z); vn.setZ(0);
		out << "dist( ("<<p.x<<","<<p.y<<"), ("<<n.x<<","<<n.y<<") ) = "<<vp.distance(vn) << std::endl;
		return vp.distance(vn) < radius;
	}
	const_iterator end()const{ return points.end(); }
	iterator end(){ return points.end(); }
	const_iterator begin()const{ return points.begin(); }
	iterator begin(){ return points.begin(); }

	const_iterator find(const POINT& p)const{
		for(const_iterator i=points.begin();i!=points.end();i++){
			if( nearby(p, *i) ) return i;
		}
		return points.end();
	}
	iterator find(const POINT& p){
		for(iterator i=points.begin();i!=points.end();i++){
			if( nearby(p, *i) ) return i;
		}
		return points.end();
	}
	
	const_iterator find_verb(const POINT& p, std::ostream& out)const{
		for(const_iterator i=points.begin();i!=points.end();i++){
			if( nearby_verb(p, *i,out) ){ out<<"found"<<std::endl; return i; }
		}
		out<<"not found"<<std::endl;
		return points.end();
	}
	iterator find_verb(const POINT& p, std::ostream& out){
		for(iterator i=points.begin();i!=points.end();i++){
			if( nearby_verb(p, *i,out) ){ out<<"found"<<std::endl; return i; }
		}
		out<<"not found"<<std::endl;
		return points.end();
	}
};
template<class T>
bool operator<(const T& e, const LocationSet<T>& s){
	return s.find(e)!=s.end();
}
template<class T>
bool operator!=(const T& e, const LocationSet<T>& s){
	return s.find(e)==s.end();
}

template<class T>
bool check_if_contains(const T& e, const LocationSet<T>& s, std::ostream& out){
	bool res = s.find_verb(e,out)!=s.end();
	if(res) out<<"point ("<<e.x<<","<<e.y<<") in point set"<<std::endl;
	else	out<<"point ("<<e.x<<","<<e.y<<") not in point set"<<std::endl;
	return res;
}
template<class T>
bool check_if_not_contains(const T& e, const LocationSet<T>& s, std::ostream& out){
	bool res = s.find_verb(e,out)==s.end();
	if(res) out<<"point ("<<e.x<<","<<e.y<<") not in point set"<<std::endl;
	else	out<<"point ("<<e.x<<","<<e.y<<") in point set"<<std::endl;
	return res;
}

#endif /* LOCATIONSET_H_ */
