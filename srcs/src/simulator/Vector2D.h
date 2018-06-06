#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <math.h>

class Vector2D {
	public:
		float x;
		float y;

		/*! Constructs a null vector. */
		inline Vector2D() { x = 0; y = 0; }
		/*!

		@brief Copy constructor.
		@param v The vector to be copied. */
		inline Vector2D(const Vector2D& v) { x = v.x; y = v.y; }

		/*!
		@brief Constructor.
		@param _x The x-component of the new vector.
		@param _y The y-component of the new vector. */
		inline Vector2D(float _x, float _y) { x = _x; y = _y; }

		/*!
		@brief Vector addition.
		@param v The vector to be added.
		@return The sum of the two vectors. */
		inline Vector2D operator+(const Vector2D& v) const { return Vector2D(x + v.x, y + v.y); }

		/*!
		@brief Vector subtraction.
		@param v The vector to be added.
		@return The vector difference. */
		inline Vector2D operator-(const Vector2D& v) const { return Vector2D(x - v.x, y - v.y); }

		/*!
		@brief Dot product.
		@param v The right hand side vector
		@return The dot product of the two vectors.  */
		inline float operator*(const Vector2D& v) const { return x * v.x + y * v.y; }

		/*!
		@brief Scalar multiplication
		@param a The scalar.  */
		inline Vector2D operator*(const float a) const { return Vector2D(x * a, y * a); }

		/*!
		@brief Scalar division
		@param a The scalar.  */
		inline Vector2D operator/(const float a) const { return Vector2D(x / a, y / a); }

		/*!
		@brief Cross product.
		@param v The right hand side vector
		@return The cross product of the two vectors.  */
		inline float operator^(const Vector2D& v) const { return x*v.y - y*v.x;}

		/*!
		@brief Adds a 2d vector to the current vector.
		@param v The vector to be added. */
		inline void operator+=(const Vector2D& v) { x += v.x; y += v.y;}

		/*!
		@brief Subtracts a 2d vector from the current vector.
		@param v The vector to be substracted. */
		inline void operator-=(const Vector2D& v) { x -= v.x; y -= v.y; }

		/*!
		@brief Multiplies the vector by a scalar.
		@param a The scalar. */
		inline void operator*=(const float& a) { x *= a; y *= a;}

		/*!
		@brief Divides the vector by a scalar.
		@param a The scalar.  */
		inline void operator/=(const float& a) { x /= a; y /= a;}

		/*!
		@brief Vector equality.
		@param v The right hand side vector
		@return True if the vectors are equal. False otherwise.  */
		inline bool operator==(const Vector2D& v) const { return (x == v.x && y == v.y); }

		/*!
		@brief Vector inequality.
		@param v The right hand side vector
		@return True if the two vector are not equal. False otherwise.  */
		inline bool operator!=(const Vector2D& v) const { return (x != v.x || y != v.y); }

		/*! @return This function normalizes the x and y coordinate. */
		inline void normalize() { float d=sqrtf(x*x+y*y); if(d>0) { x/=d; y/=d; }}

		/*! @return The magnitude of the vector.  */
		inline float length() const { return sqrtf(x*x+y*y);  }

		/*! @return The squared magnitude of the vector.  */
		inline float lengthSqr() const { return x*x+y*y; }

		/*! @return A vector perpendicular to the current vector.  */
		inline Vector2D perpendicular() const { return Vector2D(-y, x); }

		/*!
		@brief Normalization of a vector
		@param v A vector
		@return The normalized vector.  */
		inline Vector2D normalize(const Vector2D& v) { float d=sqrtf(v.x*v.x+v.y*v.y); if(d>0) return v/d; return v;}

		/*!
		@brief The dot product between two vectors.
		@param v1 A vector
		@param v2 A vector
		@return Returns the dot product between the two vectors */
		inline float dot(const Vector2D& v1, const Vector2D& v2) { return v1.x*v2.x + v1.y*v2.y ; }

		/*!
		@brief The cross product between two vectors
		@param v1 A vector
		@param v2 A vector
		@return Returns the cross product between the two vectors i.e determinant of the 2x2 matrix formed by using v1 as the first row and v2 as the second row. */
		inline float det(const Vector2D& v1, const Vector2D& v2) { return v1.x*v2.y - v1.y*v2.x; }

};

#endif
