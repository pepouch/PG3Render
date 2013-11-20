#pragma once 

#include "math.hxx"

class Material
{
public:
    Material()
    {
        Reset();
    }

    void Reset()
    {
        mDiffuseReflectance = Vec3f(0);
        mPhongReflectance   = Vec3f(0);
        mPhongExponent      = 1.f;
    }

	Vec3f evalBrdf( const Vec3f& wil, const Vec3f& wol ) const
	{
		if( wil.z <= 0 && wol.z <= 0)
			return Vec3f(0);

		Vec3f diffuseComponent = mDiffuseReflectance / PI_F;

    Vec3f r = ((2 * Dot(Vec3f(0, 0, 1), wil)) * Vec3f(0, 0, 1)) - wil;
    float cosTheta = Dot(wol, r);
    Vec3f glossyComponent = (mPhongExponent+2) * pow(cosTheta, mPhongExponent) * mPhongReflectance / (2*PI_F);
		return diffuseComponent  +  glossyComponent;
	}

    Vec3f mDiffuseReflectance;
    Vec3f mPhongReflectance;
    float mPhongExponent;
};
