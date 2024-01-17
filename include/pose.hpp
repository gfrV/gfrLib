#include <array>
#include <cmath>


class Pose {
        public:
            float x;
            float y;
            float theta;
            float linearVel;
            float angularVel;
            float pathDistance;
            /**
             * @brief Create a new pose
             * 
             * @param x component
             * @param y component
             * @param theta heading. Defaults to 0
             */
            Pose(float x, float y, float theta = 0);
            /**
             * @brief Add a pose to this pose
             * 
             * @param other other pose
             * @return Pose 
             */
            Pose operator + (const Pose &other);
            /**
             * @brief Subtract a pose from this pose
             * 
             * @param other other pose
             * @return Pose 
             */
            Pose operator - (const Pose &other);
            /**
             * @brief Multiply a pose by this pose
             * 
             * @param other other pose
             * @return Pose 
             */
            float operator * (const Pose &other);
            /**
             * @brief Multiply a pose by a float
             * 
             * @param other float
             * @return Pose 
             */
            Pose operator * (const float &other);
            /**
             * @brief Divide a pose by a float
             * 
             * @param other float
             * @return Pose 
             */
            Pose operator / (const float &other);
            /**
             * @brief Linearly interpolate between two poses
             * 
             * @param other the other pose
             * @param t t value
             * @return Pose 
             */
            Pose lerp(Pose other, float t);
            /**
             * @brief Get the distance between two poses
             * 
             * @param other the other pose
             * @return float 
             */
            float distance(Pose other);
            /**
             * @brief Get the angle between two poses
             * 
             * @param other the other pose
             * @return float in radians
             */
            float angle(Pose other);
            /**
             * @brief Rotate a pose by an angle
             * 
             * @param angle angle in radians
             * @return Pose 
             */
            Pose rotate(float angle);
    };

// union Point {
// 	Point operator-() {
// 		return {-x, -y};
// 	}

// 	Point operator+(const Point& o) {
// 		return {x + o.x, y + o.y};
// 	}

// 	Point operator-(const Point& o) {
// 		return {x - o.x, y - o.y};
// 	}

// 	Point operator*(const Point& o) {
// 		return {x * o.x, y * o.y};
// 	}

// 	Point operator/(const Point& o) {
// 		return {x / o.x, y / o.y};
// 	}

// 	Point& operator+=(Point& o) {
// 		x += o.x, y += o.y;
// 		return *this;
// 	}

// 	Point& operator-=(Point& o) {
// 		x -= o.x, y -= o.y;
// 		return *this;
// 	}

// 	Point& operator*=(Point& o) {
// 		x *= o.x, y *= o.y;
// 		return *this;
// 	}

// 	Point& operator/=(Point& o) {
// 		x /= o.x, y /= o.y;
// 		return *this;
// 	}

// 	double& operator[](unsigned int index) {
// 		return data[index];
// 	}

// 	// This is a stop gap until the codebase is made to use Point for bot
// 	// coordinates
// 	//
// 	//  TODO: Replace PID code to use this class
// 	std::array<double, 2> std() {
// 		return {x, y};
// 	}

// 	struct {
// 		double x, y;
// 	};
// 	double data[2];
// };

// inline Point operator*(double s, const Point& v) {
// 	return {s * v.x, s * v.y};
// }

// inline Point operator*(const Point& v, double s) {
// 	return {s * v.x, s * v.y};
// }

// inline Point operator/(const Point& v, double s) {
// 	return {v.x / s, v.y / s};
// }

// inline Point& operator*=(Point& v, double s) {
// 	v.x *= s, v.y *= s;
// 	return v;
// }

// inline Point& operator/=(Point& v, double s) {
// 	v.x /= s, v.y /= s;
// 	return v;
// }

// inline double dot(Point& a, Point& b) {
// 	return a.x * b.x + a.y * b.y;
// }

// inline double length2(Point& p) {
// 	return p.x * p.x + p.y * p.y;
// }

// // Why does c++ have weird things like r-value references... ;_;
// inline double length2(Point&& p) {
// 	return p.x * p.x + p.y * p.y;
// }

// inline double length(Point& p) {
// 	if (p.x == 0.0 && p.y == 0.0)
// 		return 0.0;
// 	else
// 		return std::sqrt(p.x * p.x + p.y * p.y);
// }

// inline double length(Point&& p) {
// 	if (p.x == 0.0 && p.y == 0.0)
// 		return 0.0;
// 	else
// 		return std::sqrt(p.x * p.x + p.y * p.y);
// }

// inline Point normalize(Point& a) {
// 	return a / length(a);
// }

// inline Point normalize(Point&& a) {
// 	return a / length(a);
// }