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

  virtual bool isMirror() const { return false; }

  virtual Vec3f evalBrdf( const Vec3f& wil, const Vec3f& wol ) const
  {
    return this->evalBrdfDiffuse(wil, wol) + this->evalBrdfSpecular(wil, wol);
  }

private:
  Vec3f evalBrdfDiffuse( const Vec3f& wil, const Vec3f& wol ) const
  {
    if( wil.z <= 0 || wol.z <= 0)
      return Vec3f(0);

    return mDiffuseReflectance / PI_F;
  }

  Vec3f evalBrdfSpecular( const Vec3f& wil, const Vec3f& wol ) const
  {
    if( wil.z <= 0 || wol.z <= 0)
      return Vec3f(0);

    Vec3f r = ((2 * Dot(Vec3f(0, 0, 1), wil)) * Vec3f(0, 0, 1)) - wil;
    float cosTheta = Dot(wol, r);
    return (mPhongExponent+2) * pow(cosTheta, mPhongExponent) * mPhongReflectance / (2*PI_F);
  }

public:
  virtual Vec3f sampleBrdfHemisphere(const Vec2f &sample, float* oPdf, Vec3f* oBrdf, const Vec3f& wol, Rng& rng) const
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
      *oBrdf = this->evalBrdfDiffuse(dir, wol);
    }
    else 
    {
      dir = this->sampleSpecular(sample, &pdf, wol);
      *oPdf = pdf * probSpecular;
      *oBrdf = this->evalBrdfSpecular(dir, wol);
      if (dir.z < 0)
        *oBrdf = Vec3f(0);
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
    float cosTheta = sqrt(sample.y);
    *pdf = cosTheta / PI_F;
    return result;
  }

  Vec3f sampleSpecular(const Vec2f &sample, float* pdf, const Vec3f& wol) const
  {
    float cosTheta = pow(sample.y, 1.f / (this->mPhongExponent + 1.f));
    float sinTheta = sqrt(1.f - pow(sample.y, 2.f / (this->mPhongExponent + 1.f)));

    Vec3f result (
      cos(2.f * PI_F * sample.x) * sinTheta,
      sin(2.f * PI_F * sample.x) * sinTheta,
      cosTheta
      );

    Frame reflectedRayFrame;
    Vec3f r = Vec3f(0, 0, 2.f * wol.z) - wol;
    reflectedRayFrame.SetFromZ(r);
    result = reflectedRayFrame.ToWorld(result);

    *pdf = (this->mPhongExponent + 1.f) * pow(cosTheta, this->mPhongExponent) / (2.f * PI_F) ;
    return result;
  }
public:
  virtual float getPdf(const Vec3f& wil, const Vec3f& wol) const
  {
    float pdfDiffuse, pdfSpecular;

    float maxDiffuse = this->mDiffuseReflectance.Max();
    float maxSpecular = this->mPhongReflectance.Max();
    float probDiffuse = maxDiffuse / (maxDiffuse + maxSpecular);
    float probSpecular = maxSpecular / (maxDiffuse + maxSpecular);

    pdfDiffuse = wil.z / PI_F;
    Vec3f r = Vec3f(0, 0, 2.f * wol.z) - wol;
    float cosTheta = Dot(r, wil);
    pdfSpecular = (this->mPhongExponent + 1.f) * pow(cosTheta, this->mPhongExponent) / (2.f * PI_F) ;

    return probDiffuse * pdfDiffuse + probSpecular * pdfSpecular;
  }

public:

  Vec3f mDiffuseReflectance;
  Vec3f mPhongReflectance;
  float mPhongExponent;
};


class MaterialMirror : public Material
{
  public:
    virtual bool isMirror() const override
    {
      return true;
    }

  Vec3f sampleBrdfHemisphere(const Vec2f &sample, float* oPdf, Vec3f* oBrdf, const Vec3f& wol, Rng& rng) const override
  {
    // normal = (0,0,1)

    *oPdf = 1;
    *oBrdf = this->mPhongReflectance;
    return Vec3f (-wol.x, -wol.y, wol.z);
  }

  Vec3f evalBrdf( const Vec3f& wil, const Vec3f& wol ) const override
  {
    return Vec3f(0, 0, 0);
  }

  float getPdf(const Vec3f& wil, const Vec3f& wol) const override
  {
    return 1;
  }
};