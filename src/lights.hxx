#pragma once

#include <vector>
#include <cmath>
#include "math.hxx"
#include "rng.hxx"

class AbstractLight
{
public:

  virtual Vec3f sampleIllumination(Rng& rng, const Vec3f& aSurfPt, const Frame& aFrame, Vec3f& oWig, float& oLightDist, float& oPdf) const
  {
    return Vec3f(0);
  }

  virtual Vec3f getRadiance() const = 0;
  virtual bool isBackground() const { return false; }
  virtual float getCosGamma(const Vec3f& dir) const
  {
    return 1;
  }
  virtual float transformPdfToLight(float pdfBrdf, const Vec3f& wig, float distance) const
  {
    return pdfBrdf;
  }

  virtual float getPdf(const Ray& ray, const Isect& iSect) const = 0;
  virtual Ray generateRay(Rng& rng, float* oPdfA, float* oPdfW) const
  {
    if (oPdfA)
      *oPdfA = 1.f;
    if (oPdfW)
      *oPdfW = 1.f;
    return Ray(); 
  }
};

//////////////////////////////////////////////////////////////////////////
class AreaLight : public AbstractLight
{
public:

  AreaLight(
    const Vec3f &aP0,
    const Vec3f &aP1,
    const Vec3f &aP2)
  {
    p0 = aP0;
    e1 = aP1 - aP0;
    e2 = aP2 - aP0;

    Vec3f normal = Cross(e1, e2);
    float len    = normal.Length();
    mInvArea     = 2.f / len;
    mFrame.SetFromZ(normal);
  }
  virtual Vec3f sampleIllumination(
    Rng& rng, 
    const Vec3f& aSurfPt, 
    const Frame& aFrame, 
    Vec3f& oWig, 
    float& oLightDist,
    float& oPdf) const
  {
    float a1 = 2, a2 = 2;
    do
    {
      a1 = rng.GetFloat();
      a2 = rng.GetFloat();
    } while (a1 + a2 > 1.f) ;

    Vec3f p = p0 + a1 * e1 + a2 * e2;
    oWig           = p - aSurfPt;
    float distSqr  = oWig.LenSqr();
    oLightDist     = sqrt(distSqr);

    oWig /= oLightDist;

    float cosTheta = Dot(aFrame.mZ, oWig);
    float cosGamma = Dot(-mFrame.mZ, oWig);
    oPdf =  mInvArea;

    if(cosGamma < EPS_COSINE)
      return Vec3f(0);

    return mRadiance * cosTheta * cosGamma / distSqr;
  }

  virtual Vec3f getRadiance() const override
  {
    return this->mRadiance;
  }

  virtual float getCosGamma(const Vec3f& dir) const override
  {
    return Dot(this->mFrame.mZ, dir);
  }

  virtual float getPdf(const Ray& ray, const Isect& iSect) const override
  {
    float cosGamma = Dot(-this->mFrame.mZ, ray.dir);
    return this->mInvArea * iSect.dist * iSect.dist / cosGamma;
  }

  virtual float transformPdfToLight(float pdfBrdf, const Vec3f& wig, float distance) const
  {
    pdfBrdf *= this->getCosGamma(-wig);
    pdfBrdf /= distance * distance;
    return pdfBrdf;
  }

  virtual Ray generateRay(Rng& rng, float* oPdfA, float* oPdfW) const override
  {
    float a1 = 2, a2 = 2;
    do
    {
      a1 = rng.GetFloat();
      a2 = rng.GetFloat();
    } while (a1 + a2 > 1.f) ;

    Vec3f p = p0 + a1 * e1 + a2 * e2;
    Vec2f sample = rng.GetVec2f();
    
    Vec3f sampleHemisphere = SamplePowerCosHemisphereW(sample, 0, oPdfW);
    if (oPdfA)
      *oPdfA = this->mInvArea;
    return Ray(p, this->mFrame.ToWorld(sampleHemisphere), EPS_RAY);
  }

public:
  Vec3f p0, e1, e2;
  Frame mFrame;
  Vec3f mRadiance;
  float mInvArea;
};

//////////////////////////////////////////////////////////////////////////
class PointLight : public AbstractLight
{
public:

  PointLight(const Vec3f& aPosition)
  {
    mPosition = aPosition;
  }

  virtual Vec3f sampleIllumination(
    Rng& rng,
    const Vec3f& aSurfPt, 
    const Frame& aFrame, 
    Vec3f& oWig, 
    float& oLightDist,
    float& oPdf) const
  {
    oWig           = mPosition - aSurfPt;
    float distSqr  = oWig.LenSqr();
    oLightDist     = sqrt(distSqr);

    oWig /= oLightDist;

    float cosTheta = Dot(aFrame.mZ, oWig);
    oPdf = 1;

    if(cosTheta <= 0)
      return Vec3f(0);

    return mIntensity * cosTheta / distSqr;
  }
  virtual Ray generateRay(Rng& rng, float* oPdfA, float* oPdfW) const override
  {
    Vec2f sample = rng.GetVec2f();
    Vec3f dir = SampleUniformSphereW(sample, oPdfW);
    if (oPdfA)
      *oPdfA = 1.f;
    return Ray(mPosition, dir, EPS_RAY);
  }

  virtual Vec3f getRadiance() const override
  {
    return this->mIntensity;
  }

  virtual float getPdf(const Ray& ray, const Isect& iSect) const override
  {
    return 1;
  }

  virtual float transformPdfToLight(float pdfBrdf, const Vec3f& wig, float distance) const
  {
    return 0;
  }

public:

  Vec3f mPosition;
  Vec3f mIntensity;
};


//////////////////////////////////////////////////////////////////////////
class BackgroundLight : public AbstractLight
{
public:
  BackgroundLight()
  {
    mBackgroundColor = Vec3f(135, 206, 250) / Vec3f(255.f);
  }

public:

  bool isBackground() const override { return true; }

  virtual Vec3f sampleIllumination(
    Rng& rng,
    const Vec3f& aSurfPt, 
    const Frame& aFrame, 
    Vec3f& oWig, 
    float& oLightDist,
    float& oPdf) const
  {
    Vec3f p;
    do
    {
      p = Vec3f(2.0f*rng.GetFloat()-1.0f, 2.0f*rng.GetFloat()-1.0f, 2.0f*rng.GetFloat()-1.0f);
    }
    while (p.LenSqr() > 1) ;
    p =  (1/p.Length()) * p;
    p = 10000.0 * p;
    oWig           = p - aSurfPt;
    float distSqr  = oWig.LenSqr();
    oLightDist     = sqrt(distSqr);

    oWig /= oLightDist;

    float cosTheta = Dot(aFrame.mZ, oWig);
    oPdf = 1.f / (4.f * PI_F);

    if(cosTheta <= 0)
      return Vec3f(0);
    return mBackgroundColor * cosTheta;
  }

  virtual Vec3f getRadiance() const override
  {
    return this->mBackgroundColor;
  }

  virtual float getPdf(const Ray& ray, const Isect& iSect) const override
  {
    return 1.f / (4.f * PI_F);
  }

  virtual Ray generateRay(Rng& rng, float* oPdfA, float* oPdfW) const override
  {
    Vec3f p;
    do
    {
      p = Vec3f(2.0f*rng.GetFloat()-1.0f, 2.0f*rng.GetFloat()-1.0f, 2.0f*rng.GetFloat()-1.0f);
    }
    while (p.LenSqr() > 1) ;
    Vec3f norm_p = (1/p.Length()) * p;
    p = 10 * norm_p;
    Vec2f sample = rng.GetVec2f();
    Vec3f sampleHemisphere = SamplePowerCosHemisphereW(sample, 0, oPdfW);
    Frame frame;
    frame.SetFromZ(-norm_p);
    if (oPdfA)
      *oPdfA = 1.0 / (4.f * PI_F);
    return Ray(p, frame.ToWorld(sampleHemisphere), EPS_RAY);
  }

  Vec3f mBackgroundColor;
};
