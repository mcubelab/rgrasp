#if !defined(HOMOGTRANSF_INCLUDED)
#define HOMOGTRANSF_INCLUDED

class HomogTransf : public Mat
{
public:
	//Constructors
	HomogTransf();	// Null translation and rotation.
  HomogTransf(const double values[4][4]);
  HomogTransf(const double pose[7]);
    HomogTransf(char const *string);
	HomogTransf(const HomogTransf &original);		// Copy constructor.
	HomogTransf(const Quaternion &quat, const Vec &trans);
	HomogTransf(const RotMat &rot, const Vec &trans);
	HomogTransf(const Mat &original); // Conversion constructor.
	
	//Operators
	//(Explicitely Inherited for preserving the output class label)
	HomogTransf& operator=(const double constant);
	HomogTransf& operator+=(const HomogTransf &original);
	HomogTransf& operator-=(const HomogTransf &original);
	HomogTransf& operator*=(const double constant);
	HomogTransf& operator/=(const double constant);
	HomogTransf operator +(const HomogTransf &original) const;
	HomogTransf operator -(const HomogTransf &original) const;
	HomogTransf operator *(const HomogTransf &original) const;
	HomogTransf operator *(const double constant) const;
	HomogTransf operator /(const double constant) const;
	//New Operator
	Vec operator *(const Vec &original) const;  //If original is a 4-vector: Standard product.
									//If original is a 3-vector: Transformation of a point in R3.
	//Interface
	void setPose(const double pose[7]); // x y z q0 qx qy qz
	void setTranslation(const Vec &trans);
	void setRotation(const RotMat &rot);
	void setQuaternion(const Quaternion &quat);
	Quaternion getQuaternion() const;
	RotMat getRotation() const;
	Vec getTranslation() const;


	//Transformations
	void setScrew(const Vec &point,const Vec &director_vector,const double displacement,const double angle);

	//Algebra
	HomogTransf inv() const;
	
};

#endif	// !defined(HOMOGTRANSF_INCLUDED)
