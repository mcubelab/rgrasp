#if !defined(QUATERNION_INCLUDED)
#define QUATERNION_INCLUDED


//class Vec;
class RotMat;

class Quaternion: public Vec
{
public:
	//Constructors
	Quaternion();
	Quaternion(const double constant);	//Initialize to constant value
	Quaternion(double const *values);// Initialize to values in C-style array a	
	Quaternion(char const *string);  //Initialize to values in string
  Quaternion(double const q0, Vec const v);  //Initialize with scalar and vector
	Quaternion(const Vec &origin_vector);	// Conversion constructor
	Quaternion(const Quaternion &origin_quaternion);  //Copy constructor

	//Operators
	//(Explicitely Inherited for preserving the output class label)
	Quaternion & operator =(const double constant);	//assign a to every element
	Quaternion & operator +=(const Quaternion &original);
	Quaternion & operator -=(const Quaternion &original);
	Quaternion operator +(const Quaternion &original) const;
	Quaternion operator -(const Quaternion &original) const;
	Quaternion operator -() const;
	Quaternion operator *(const double constant) const;
	Quaternion operator /(const double constant) const;
	//(New)
	Quaternion operator ^(const Quaternion &original) const; // Quaternion product.
	Quaternion operator /(const Quaternion &original) const; // Quat divide (mult by inv)
	double operator *(const Quaternion &original) const;

	//Quaternion Algebra
	Quaternion conjugate() const;
	Quaternion inverse() const;
	double getScalar() const;
	Vec getVector() const;
	void setScalar(const double s);
	void setVector(const Vec &vector);
	Mat leftMat() const;
	Mat rightMat() const;

	// Distance between 2 quaternions
	double dist(const Quaternion &original) const;

	//Transformation
	double getAngle() const;
	Vec getAxis() const;
	RotMat getRotMat() const;
	Quaternion angle2quat(double r1, double r2, double r3);
	void quat2angle(double &r1, double &r2, double &r3);
	void threeAxisRot(double r11, double r12, double r21, 
			  double r31, double r32, double &r1, 
			  double &r2, double &r3);
	
};

#endif	// !defined(QUATERNION_INCLUDED)
