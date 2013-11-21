#pragma once 

#include "math.hxx"
#include "rng.hxx"

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

  Vec3f sampleBrdfHemisphere(const Vec2f &sample, float* oPdf, Rng& rng) const
  {
    Vec3f dir;
    float pdf;
    
    float maxDiffuse = this->mDiffuseReflectance.Max();
    float maxSpecular = this->mPhongReflectance.Max();
    float probDiffuse = maxDiffuse / (maxDiffuse + maxSpecular);
    float probSpecular = maxSpecular / (maxDiffuse + maxSpecular);

    if (rng.GetFloat() < probDiffuse) 
    {
      dir = this->sampleDiffuse(sample, &pdf);
      *oPdf = pdf * probDiffuse;
    }
    else 
    {
      dir = this->sampleSpecular(sample, &pdf);
      *oPdf = pdf * probSpecular;
    }
    return dir;
  }

private:

  Vec3f sampleDiffuse(const Vec2f &sample, float* pdf) const
  {
    Vec3f result (
      cos(2*PI_F*sample.x) * sqrt(1-sample.y),
      sin(2*PI_F*sample.x) * sqrt(1-sample.y),
      sqrt(sample.y)
      );
    float theta = acos(sqrt(sample.y));
    *pdf = cos(theta) / PI_F;
    return result;
  }

  Vec3f sampleSpecular(const Vec2f &sample, float* pdf) const
  {
    float poweredTerm = pow(sample.y, (1 / this->mPhongExponent + 1));
    float sqrtTerm = sqrt(1 - pow(sample.y, (2 / this->mPhongExponent + 1)));

    Vec3f result (
      cos(2 * PI_F * sample.x) * sqrtTerm,
      sin(2 * PI_F * sample.x) * sqrtTerm,
      poweredTerm
      );

    float theta = acos(poweredTerm);

    *pdf = (this->mPhongExponent + 1) / ((2 * PI_F) * pow(cos(theta), this->mPhongExponent));
    return result;
  }

public:

    Vec3f mDiffuseReflectance;
    Vec3f mPhongReflectance;
    float mPhongExponent;
};