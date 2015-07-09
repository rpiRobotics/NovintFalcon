#include "Vector3.h"
#include<math.h>


Vector3::Vector3(double x, double y, double z) 
	: X(x), Y(y), Z(z)
{

}
Vector3::Vector3(double x)
	: X(x), Y(x), Z(x)
{

}
Vector3::~Vector3()
{
	
}

double Vector3::Length()
{
	double x_squared = pow(X, 2);
	double y_squared = pow(Y, 2);
	double z_squared = pow(Z, 2);
	double length = sqrt(x_squared + y_squared + z_squared);
	return length;
}
Vector3 Vector3::Normalize()
{
	double magnitude = (*this).Length();
	Vector3 direction = (*this) / (magnitude == 0 ? 1 : magnitude);
	return direction;
}
Vector3  Vector3::Normalize(double threshold, double maxMagnitude)
{
	double magnitude = (*this).Length();
	Vector3 direction = (*this) / (magnitude == 0 ? 1 : magnitude);

	double normalizedMagnitude = 0;
	if (magnitude - threshold > 0){
		normalizedMagnitude = fmin((magnitude - threshold) / (maxMagnitude - threshold), 1);
	}
	return direction * normalizedMagnitude;

}

const Vector3 Vector3::operator/(const double scalar) const
{
	Vector3 result(*this);
	result.X /= scalar;
	result.Y /= scalar;
	result.Z /= scalar;
	return result;
}
const Vector3 Vector3::operator*(const double scalar) const
{
	Vector3 result(*this);
	result.X *= scalar;
	result.Y *= scalar;
	result.Z *= scalar;
	return result;
}