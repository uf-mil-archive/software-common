#ifndef C3TRAJECTORY_C3FILTER_H
#define C3TRAJECTORY_C3FILTER_H

#include <Eigen/Dense>

namespace subjugator {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    
	class C3Trajectory {
		public:
		struct Limits {
			Vector6d vmin_b;
			Vector6d vmax_b;
			Vector6d amin_b;
			Vector6d amax_b;
			Eigen::Vector3d arevoffset_b;
			Vector6d umax_b;
		};

		struct Point {
			Vector6d q;
			Vector6d qdot;

			Point() { }

			Point(const Vector6d &q, const Vector6d &qdot) :
				q(q), qdot(qdot) { }
		};

		struct Waypoint {
			Point r;
			double speed;
			bool coordinate_unaligned;

			Waypoint() { }
		Waypoint(const Point &r) : r(r), speed(0), coordinate_unaligned(true) { }
		};

		C3Trajectory(const Point &start, const Limits &limits);
		void update(double dt, const Waypoint &waypoint, double waypoint_t);

		Point getCurrentPoint() const;

	private:
		Vector6d q;
		Vector6d qdot;
		Vector6d qdotdot_b;
		Vector6d u_b;

		Limits limits;

		static double c3filter(double q, double qdot, double qdotdot,
		                       double r, double rdot, double rdotdot,
		                       double vmin, double vmax,
		                       double amin, double amax,
		                       double umax);

		static std::pair<Eigen::Matrix4d, Eigen::Matrix4d> transformation_pair(const Vector6d &q);
		static std::pair<Eigen::Vector3d, Eigen::Vector3d> limit(const Eigen::Vector3d &vmin, const Eigen::Vector3d &vmax, const Eigen::Vector3d &delta);
	};
};

#endif
