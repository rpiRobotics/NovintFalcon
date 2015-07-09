#pragma once
class Vector3
{
public:
	double X;
	double Y;
	double Z;

	Vector3(double x, double y, double z);
	Vector3(double x);
	~Vector3();
	

	Vector3 Normalize(void);
	Vector3 Normalize(double threshold, double maxMagnitude);
	double Length(void);
	const Vector3 operator/(const double scalar) const;
	const Vector3 operator*(const double scalar) const;
};

