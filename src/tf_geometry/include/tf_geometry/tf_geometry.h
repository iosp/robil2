/*
 * tf_geometry.hpp
 *
 *  Created on: May 28, 2014
 *      Author: dan
 */

#ifndef TF_GEOMETRY_HPP_
#define TF_GEOMETRY_HPP_

#define USE_AFFINE 1

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>
#include <angles/angles.h>

#if USE_AFFINE==1
#include <cryph/Matrix4x4.h>
#endif

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/QuaternionStamped.h>

static const double d2r = 0.0174532925;
static const double r2d = 57.2957795;

namespace tf_geometry{

	typedef tf::Quaternion Quaternion;
	typedef tf::Vector3 Vector;
	typedef tf::Matrix3x3 Matrix2D;
	typedef tf::Matrix3x3 RotationMatrix;
#if USE_AFFINE==1
	typedef cryph::Matrix4x4 Matrix3D;
	typedef cryph::Matrix4x4 Affine;
	typedef cryph::AffPoint AffVector;
#endif

	inline const geometry_msgs::Pose& getPose(const geometry_msgs::PoseWithCovarianceStamped& pos){
		return pos.pose.pose;
	}
	inline geometry_msgs::Pose& getPose(geometry_msgs::PoseWithCovarianceStamped& pos){
		return pos.pose.pose;
	}
	inline const geometry_msgs::Pose& getPose(const geometry_msgs::PoseStamped& pos){
		return pos.pose;
	}
	inline geometry_msgs::Pose& getPose(geometry_msgs::PoseStamped& pos){
		return pos.pose;
	}
	inline const geometry_msgs::Pose& getPose(const geometry_msgs::Pose& pos){
		return pos;
	}
	inline geometry_msgs::Pose& getPose(geometry_msgs::Pose& pos){
		return pos;
	}
	inline const geometry_msgs::Quaternion& getQuaternion(const geometry_msgs::Quaternion& q){
		return q;
	}
	inline geometry_msgs::Quaternion& getQuaternion(geometry_msgs::Quaternion& q){
		return q;
	}
	inline const geometry_msgs::Quaternion& getQuaternion(const geometry_msgs::QuaternionStamped& q){
		return q.quaternion;
	}
	inline geometry_msgs::Quaternion& getQuaternion(geometry_msgs::QuaternionStamped& q){
		return q.quaternion;
	}

	class Pose;
	class Position{
	public:
		double x, y, z;
		Position():x(0),y(0),z(0){}
		Position(double x, double y, double z=0):x(x),y(y),z(z){}
		Position& operator=(const Position& point){
			x = point.x; y = point.y; z = point.z;
			return *this;
		}

		Position(const geometry_msgs::Point& point){
			x = point.x; y = point.y; z = point.z;
		}
		Position(const geometry_msgs::Point32& point){
			x = point.x; y = point.y; z = point.z;
		}
		Position(const geometry_msgs::Vector3& point){
			x = point.x; y = point.y; z = point.z;
		}
		Position(const btVector3& point){
			x = point.x(); y = point.y(); z = point.z();
		}
		Position(const tf::Vector3& point){
			x = point.x(); y = point.y(); z = point.z();
		}
		Position(const tf::Matrix3x3& m){
			tf::Vector3 ziro(0,0,0);
			tf::Vector3 point = m*ziro;
			x = point.x(); y = point.y(); z = point.z();
		}
#if USE_AFFINE==1
		Position(const Affine& m){
			AffVector ziro(0,0,0);
			AffVector point = m*ziro;
			x = point.x; y = point.y; z = point.z;
		}
		Position(const AffVector& point){
			x = point.x; y = point.y; z = point.z;
		}
#endif
		operator geometry_msgs::Point ()const{
			geometry_msgs::Point point;
			point.x=x; point.y=y; point.z=z;
			return point;
		}
		operator geometry_msgs::Point32 ()const{
			geometry_msgs::Point32 point;
			point.x=(float)x; point.y=(float)y; point.z=(float)z;
			return point;
		}
		operator geometry_msgs::Vector3 ()const{
			geometry_msgs::Vector3 point;
			point.x=x; point.y=y; point.z=z;
			return point;
		}
		operator btVector3 ()const{
			return btVector3(x,y,z);
		}
		operator tf::Vector3 ()const{
			return tf::Vector3(x,y,z);
		}
#if USE_AFFINE==1
		operator AffVector ()const{
			return AffVector(x,y,z);
		}
#endif
		geometry_msgs::Point to_msg_Point()const{ return *this; }
		geometry_msgs::Point32 to_msg_Point32()const{ return *this; }
		geometry_msgs::Vector3 to_msg_Vector3()const{ return *this; }
		btVector3 to_btVector3()const{ return *this; }
		tf::Vector3 to_tfVector3()const{ return *this; }
#if USE_AFFINE==1
		AffVector to_AffVector()const{ return *this; }
#endif
		tf::Matrix3x3 to_TranslationMatrix2D()const{
			return tf::Matrix3x3(
					1,0,x,
					0,1,y,
					0,0,1
					);
		}
		tf::Matrix3x3 to_RotationMatrix()const{
			double y = angleYaw();
			double p = anglePitch();
			double r = angleRoll();
			tf::Matrix3x3 m; m.setRPY(r,p,y);
			return m;
		}
#if USE_AFFINE==1
		Affine to_AffineTranslation()const{
			return Affine::IdentityMatrix.translation(to_AffVector());
		}
		Affine to_RotationAffine(){
			double y = angleYaw();
			double p = anglePitch();
			double r = angleRoll();
			Affine m;
			m *= m.zRotationRadians(y);
			m *= m.yRotationRadians(p);
			m *= m.xRotationRadians(r);
			return m;
		}
#endif
		Position operator+(const Position& p)const{
			return to_tfVector3()+p.to_tfVector3();
		}
		Position operator-(const Position& p)const{
			return to_tfVector3()+p.to_tfVector3();
		}
		void operator+=(const Position& p){
			(*this)=(*this)+p;
		}
		void operator-=(const Position& p){
			(*this)=(*this)-p;
		}
		double len()const{ return to_tfVector3().length(); }
		double len2()const{ return to_tfVector3().length2(); }
		double dot(const Position& p)const{ return to_tfVector3().dot(p.to_tfVector3()); }
		Position operator*(const double& p)const{
			return Position(x*p,y*p,z*p);
		}
		Position scale(double px,double py,double pz)const{
			return Position(x*px,y*py,z*pz);
		}
		void operator*=(double p){ (*this)=(*this)*p; }

		Position operator*(const Position& p)const{
			return to_tfVector3()*p.to_tfVector3();
		}
		void operator*=(const Position& p){ (*this)=(*this)*p; }

		Position operator/(const double& p)const{
			return Position(x/p,y/p,z/p);
		}
		void operator/=(const double& p){ (*this)=(*this)/p; }
		Position operator/(const Position& p)const{
			return to_tfVector3()/p.to_tfVector3();
		}
		void operator/=(const Position& p){ (*this)=(*this)/p; }

		void normalize(){ *this /= len(); }
		Position normalized()const{ return *this / len(); }

		Position rotate( const Position& wAxis, const double angle ) const
		{
			return to_tfVector3().rotate(wAxis, angle);
		}
		Position rotate( const double angle ) const	{
			return to_tfVector3().rotate(Position(0,0,1), angle);
		}
		Position rotateYaw( const double angle ) const	{
			return to_tfVector3().rotate(Position(0,0,1), angle);
		}
		Position rotatePitch( const double angle ) const	{
			return to_tfVector3().rotate(Position(0,1,0), angle);
		}
		Position rotateRoll( const double angle ) const	{
			return to_tfVector3().rotate(Position(1,0,0), angle);
		}
		Position rotate(const tf::Quaternion& q)const{
			tf::Matrix3x3 m(q); tf::Vector3 v = this->to_tfVector3();
			return Position(m[0].dot(v),m[1].dot(v),m[2].dot(v));
		}

		double angleYaw()const{ return atan2(y,x); }
		double angle()const{ return angleYaw(); }
		double angleRoll()const{ return atan2(y,z); }
		double anglePitch()const{ return atan2(z,x); }
	};

	class Orientation{
	public:
		double x, y, z, w;
		Orientation():x(0),y(0),z(0),w(1){}
		Orientation(double x, double y, double z, double w):x(x),y(y),z(z),w(w){}
		Orientation(const Orientation& q){
			x = q.x; y = q.y; z = q.z; w = q.w;
		}

		Orientation(double yaw_z, double pitch_y, double roll_x)
		{
			tf::Quaternion q; q.setRPY(roll_x,pitch_y,yaw_z);
			x = q.x(); y = q.y(); z = q.z(); w = q.w();
		}

		Orientation(const Position& axis, double& radians){
			tf::Quaternion q;q.setRotation(axis,radians);
			x = q.x(); y = q.y(); z = q.z(); w = q.w();
		}

		Orientation(const geometry_msgs::Quaternion& q){
			x = q.x; y = q.y; z = q.z; w = q.w;
		}
		Orientation(const btQuaternion& q){
			x = q.x(); y = q.y(); z = q.z(); w = q.w();
		}
		Orientation(const tf::Quaternion& q){
			x = q.x(); y = q.y(); z = q.z(); w = q.w();
		}
		Orientation(const tf::Matrix3x3& m){
			tf::Quaternion q; m.getRotation(q);
			x = q.x(); y = q.y(); z = q.z(); w = q.w();
		}

		operator geometry_msgs::Quaternion()const{
			geometry_msgs::Quaternion q;
			q.x = x; q.y = y; q.z = z; q.w = w;
			return q;
		}
		operator btQuaternion()const{
			return btQuaternion(x,y,z,w);
		}
		operator tf::Quaternion()const{
			return tf::Quaternion(x,y,z,w);
		}
		operator tf::Matrix3x3()const{
			return tf::Matrix3x3(to_tfQuaternion());
		}

		geometry_msgs::Quaternion to_msg_Quaternion()const{ return *this; }
		btQuaternion to_btQuaternion()const{ return *this; }
		tf::Quaternion to_tfQuaternion()const{ return *this; }
		tf::Matrix3x3 to_tfMatrix()const{ return *this; }
		tf::Matrix3x3 to_RotationMatrix()const{ return *this; }
#if USE_AFFINE==1
		Affine to_AffineRotation()const{
			double y,p,r; getRPY(r,p,y);
			Affine m;
			m *= m.zRotationRadians(y);
			m *= m.yRotationRadians(p);
			m *= m.xRotationRadians(r);
			return m;
		}
#endif

		void setRPY(double roll_x, double pitch_y, double yaw_z){
			tf::Quaternion q; q.setRPY(roll_x, pitch_y, yaw_z);
			(*this)=q;
		}
		void setEuler(double yaw_z, double pitch_y, double roll_x){
			setRPY(roll_x, pitch_y, yaw_z);
		}
		/**@brief Get the matrix represented as euler angles around ZYX
		* @param yaw Yaw around Z axis
		* @param pitch Pitch around Y axis
		* @param roll around X axis */
		void getEuler(double& yaw_z,double& pitch_y, double& roll_x)const{
			tf::Quaternion q = *this;
			tf::Matrix3x3(q).getRPY(roll_x, pitch_y, yaw_z);
		}
		/**@brief Get the matrix represented as roll pitch and yaw about fixed axes XYZ
		* @param roll around X axis
		* @param pitch Pitch around Y axis
		* @param yaw Yaw around Z axis */
		void getRPY(double& roll_x, double& pitch_y, double& yaw_z)const{
			tf::Quaternion q = *this;
			tf::Matrix3x3(q).getRPY(roll_x, pitch_y, yaw_z);
		}

		double yaw()const{
			double y,p,r; getEuler(y,p,r); return y;
		}
		double pitch()const{
			double y,p,r; getEuler(y,p,r); return y;
		}
		double roll()const{
			double y,p,r; getEuler(y,p,r); return y;
		}
		void set_yaw(double y){
			double _y,p,r; getEuler(_y,p,r); setEuler(y,p,r);
		}
		void set_pitch(double p){
			double y,_p,r; getEuler(y,_p,r); setEuler(y,p,r);
		}
		void set_roll(double r){
			double y,p,_r; getEuler(y,p,_r); setEuler(y,p,r);
		}

		double getX()const{ return x; }
		double getY()const{ return y; }
		double getZ()const{ return z; }
		double getW()const{ return y; }
		double getYaw()const{ return yaw(); }
		double getPitch()const{ return pitch(); }
		double getRoll()const{ return roll(); }
		void setX(double x){this->x=x;}
		void setY(double y){this->y=y;}
		void setZ(double z){this->z=z;}
		void setW(double w){this->w=w;}
		void setYaw(double y){set_yaw(y);}
		void setPitch(double p){set_pitch(p);}
		void setRoll(double r){set_roll(r);}

		Orientation operator*(const Orientation& o){
			return to_tfQuaternion()*o.to_tfQuaternion();
		}
		Orientation rotate(const Orientation& o){
			return o.to_tfQuaternion()*to_tfQuaternion();
		}
	};

	class Scale;
	Pose apply(const Pose& p, const Scale& s);

	class Scale{
	public:
		double x, y, z;
		Scale():x(1),y(1),z(1){}
		Scale(double x):x(x),y(x),z(x){}
		Scale(double x, double y, double z=1):x(x),y(y),z(z){}
		Position operator()(const Position& p)const{ return Position(p.x*x,p.y*y,p.z*z); }
		Position operator*(const Position& p)const{ return (*this)(p); }
		Scale operator*(const Scale& s)const{ return Scale(x*s.x,y*s.y,z*s.z); }
		Scale operator+(const Scale& s)const{ return Scale(x+s.x,y+s.y,z+s.z); }
		Scale operator/(const Scale& s)const{ return Scale(x/s.x,y/s.y,z/s.z); }
		Scale operator-(const Scale& s)const{ return Scale(x-s.x,y-s.y,z-s.z); }

#if USE_AFFINE == 1
		Affine to_AffineScale()const{
			return Affine::IdentityMatrix.scale(x, y, z);
		}
#endif
	};
	struct ScalePosePrimitive{
		Position p;
		Orientation o;
		Scale s;
		ScalePosePrimitive(Position p, Orientation o, Scale s):p(p),o(o),s(s){}
	};
	inline Position operator*(const Position& p, const Scale& s){ return s*p; }
	inline ScalePosePrimitive operator*(const Scale& s, const ScalePosePrimitive& p){
		ScalePosePrimitive ps( s*p.p,  p.o,  s*p.s );
		return ps;
	}
	inline Scale operator*(double x, const Scale& s){ return Scale(x)*s; }
	inline Scale operator+(double x, const Scale& s){ return Scale(x)+s; }
	inline Scale operator/(double x, const Scale& s){ return Scale(x)/s; }
	inline Scale operator-(double x, const Scale& s){ return Scale(x)-s; }


	class Pose{
	public:
		Position position;
		Orientation orientation;
		Scale scale;
		Pose(){}
		Pose(const Pose& p):position(p.position),orientation(p.orientation),scale(p.scale){}
		Pose(const Position& p, const Orientation& o):position(p),orientation(o){}
		Pose(const Position& p):position(p),orientation(Orientation()){}
		Pose(const Orientation& o):position(Position()),orientation(o){}
		Pose(const geometry_msgs::Pose& pose)
			:position(pose.position),orientation(pose.orientation)
		{ }
		Pose& operator=(const Pose& pose){
			position = pose.position;
			orientation = pose.orientation;
			scale = pose.scale;
			return *this;
		}

		Pose(const geometry_msgs::Pose2D& pose)
			:position(pose.x,pose.y),orientation(pose.theta,0,0)
		{ }

		Pose(const tf::Transform& tf){
			position = tf.getOrigin();
			orientation = tf.getBasis();
		}
		Pose(const geometry_msgs::Transform& tf){
			position = tf.translation;
			orientation = tf.rotation;
		}
		Pose(const ScalePosePrimitive& p)
		:position(p.p),orientation(p.o),scale(p.s){}

		operator geometry_msgs::Pose()const{
			geometry_msgs::Pose pose;
			pose.position = position.to_msg_Point();
			pose.orientation = orientation.to_msg_Quaternion();
			return pose;
		}
		operator geometry_msgs::Pose2D()const{
			geometry_msgs::Pose2D pose;
			pose.x = position.x;
			pose.y = position.y;
			pose.theta = orientation.yaw();
			return pose;
		}
		operator tf::Transform()const{
			return tf::Transform(orientation.to_tfMatrix(), position.to_tfVector3());
		}
		operator geometry_msgs::Transform()const{
			geometry_msgs::Transform pose;
			pose.translation = position.to_msg_Vector3();
			pose.rotation = orientation.to_msg_Quaternion();
			return pose;
		}
#if USE_AFFINE==1
		operator Affine()const{
			return position.to_AffineTranslation() * orientation.to_AffineRotation() * scale.to_AffineScale();
		}
#endif
		operator ScalePosePrimitive()const{
			ScalePosePrimitive p(position, orientation, scale);
			return p;
		}

		geometry_msgs::Pose to_msg_Pose()const{ return (*this); }
		geometry_msgs::Pose2D to_msg_Pose2D()const{ return (*this); }
		geometry_msgs::Transform to_msg_Transform()const{ return (*this); }
		geometry_msgs::TransformStamped to_msg_TransformStamped(std::string parent, std::string child)const{
			geometry_msgs::TransformStamped m;
			m.child_frame_id = child;
			m.header.frame_id = parent;
			m.header.stamp = ros::Time::now();
			m.transform = to_msg_Transform();
			return m;
		}
#if USE_AFFINE==1
		Affine to_Affine()const{ return *this; }
		static Affine AffineIdentity(){ return Affine::IdentityMatrix; }
		static Affine AffineZero(){ return Affine::ZeroMatrix; }
		static Affine AffineTranslate(double x, double y, double z){
			return Affine::IdentityMatrix.translation(AffVector(x, y, z));
		}
		static Affine AffineRotate(double roll_x, double pitch_y, double yaw_z){
			return Affine::IdentityMatrix.
					xRotationRadians(roll_x).
					yRotationRadians(pitch_y).
					zRotationRadians(yaw_z);
		}
		static Affine AffineScale(double x, double y, double z){
			return Affine::IdentityMatrix.scale(x,y,z);
		}
#endif
		tf::Transform to_tfTransform()const{ return *this; }

		Pose operator()(const Pose& p)const{
			tf::Transform t;
			Pose pose = p*scale;
			t.mult(to_tfTransform(), pose.to_tfTransform());
			return t;
		}
		Pose operator*(const Pose& p)const{
			return (*this)(p);
		}
		Pose operator%(const Pose& p)const{
			return p(*this);
		}

		Position operator()(const Position& p)const{
			return to_tfTransform()*(scale*p).to_tfVector3();
		}
		Position operator*(const Position& p)const{
			return (*this)(p);
		}
		Pose operator%(const Position& p)const{
			Pose pose(p,Orientation());
			return pose(*this);
		}

		Orientation operator()(const Orientation& o)const{
			return to_tfTransform()*o.to_tfQuaternion();
		}
		Orientation operator*(const Orientation& o)const{
			return (*this)(o);
		}
		Pose operator%(const Orientation& o)const{
			Pose pose(Position(),o);
			return pose(*this);
		}

		Pose operator*(const Scale& s)const{
			Pose pose(*this);
			pose.position = s*position;
			pose.scale = s*pose.scale;
			return pose;
		}

	};
	Pose apply(const Pose& p, const Scale& s){
		return p*s;
	}

#if USE_AFFINE == 1
	template<class AFF>
	AFF operator%(const Affine& a1, const AFF& a2){ return a2*a1; }
	Position operator%(const Position& a1, const Pose& a2){ return a2*a1; }
	Orientation operator%(const Orientation& a1, const Pose& a2){ return a2*a1; }
#endif

}



#endif /* TF_GEOMETRY_HPP_ */
