#pragma once

#include <vector>
#include <cmath>
#include "math.hxx"
#include "ray.hxx"

//////////////////////////////////////////////////////////////////////////
// Geometry

class AbstractGeometry
{
public:

  virtual ~AbstractGeometry(){};

  // Finds the closest intersection
  virtual bool Intersect (const Ray& aRay, Isect& oResult) const = 0;

  // Finds any intersection, default calls Intersect
  virtual bool IntersectP(const Ray& aRay, Isect& oResult) const
  {
    return Intersect(aRay, oResult);
  }

  // Grows given bounding box by this object
  virtual void GrowBBox(Vec3f &aoBBoxMin, Vec3f &aoBBoxMax) = 0;
};

class GeometryList : public AbstractGeometry
{
public:

  virtual ~GeometryList()
  {
    for(int i=0; i<(int)mGeometry.size(); i++)
      delete mGeometry[i];
  };

  virtual bool Intersect(const Ray& aRay, Isect& oResult) const
  {
    bool anyIntersection = false;

    for(int i=0; i<(int)mGeometry.size(); i++)
    {
      bool hit = mGeometry[i]->Intersect(aRay, oResult);

      if(hit)
        anyIntersection = hit;
    }

    return anyIntersection;
  }

  virtual bool IntersectP(
    const Ray &aRay,
    Isect     &oResult) const
  {
    for(int i=0; i<(int)mGeometry.size(); i++)
    {
      if(mGeometry[i]->IntersectP(aRay, oResult))
        return true;
    }

    return false;
  }

  virtual void GrowBBox(
    Vec3f &aoBBoxMin,
    Vec3f &aoBBoxMax)
  {
    for(int i=0; i<(int)mGeometry.size(); i++)
      mGeometry[i]->GrowBBox(aoBBoxMin, aoBBoxMax);
  }

public:

  std::vector<AbstractGeometry*> mGeometry;
};

class Triangle : public AbstractGeometry
{
public:

  Triangle(){}

  Triangle(
    const Vec3f &p0,
    const Vec3f &p1,
    const Vec3f &p2,
    int         aMatID)
  {
    p[0] = p0;
    p[1] = p1;
    p[2] = p2;
    matID = aMatID;
    mNormal = Normalize(Cross(p[1] - p[0], p[2] - p[0]));
  }

  virtual bool Intersect(
    const Ray &aRay,
    Isect     &oResult) const
  {
    if (Dot(aRay.dir, mNormal) > 0)
      return false;
    const Vec3f ao = p[0] - aRay.org;
    const Vec3f bo = p[1] - aRay.org;
    const Vec3f co = p[2] - aRay.org;

    const Vec3f v0 = Cross(co, bo);
    const Vec3f v1 = Cross(bo, ao);
    const Vec3f v2 = Cross(ao, co);

    const float v0d = Dot(v0, aRay.dir);
    const float v1d = Dot(v1, aRay.dir);
    const float v2d = Dot(v2, aRay.dir);

    if(((v0d < 0.f)  && (v1d < 0.f)  && (v2d < 0.f)) ||
      ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f)))
    {
      const float distance = Dot(mNormal, ao) / Dot(mNormal, aRay.dir);

      if((distance > aRay.tmin) & (distance < oResult.dist))
      {
        oResult.normal = mNormal;
        oResult.matID  = matID;
        oResult.dist   = distance;
        return true;
      }
    }

    return false;
  }

  virtual void GrowBBox(
    Vec3f &aoBBoxMin,
    Vec3f &aoBBoxMax)
  {
    for(int i=0; i<3; i++)
    {
      for(int j=0; j<3; j++)
      {
        aoBBoxMin.Get(j) = std::min(aoBBoxMin.Get(j), p[i].Get(j));
        aoBBoxMax.Get(j) = std::max(aoBBoxMax.Get(j), p[i].Get(j));
      }
    }
  }

public:

  Vec3f p[3];
  int   matID;
  Vec3f mNormal;
};

class Sphere : public AbstractGeometry
{
public:

  Sphere(){}

  Sphere(
    const Vec3f &aCenter,
    float       aRadius,
    int         aMatID)
  {
    center = aCenter;
    radius = aRadius;
    matID  = aMatID;
  }

  // Taken from:
  // http://wiki.cgsociety.org/index.php/Ray_Sphere_Intersection

  virtual bool Intersect(
    const Ray &aRay,
    Isect     &oResult) const
  {
    // we transform ray origin into object space (center == origin)
    const Vec3f transformedOrigin = aRay.org - center;

    const float A = Dot(aRay.dir, aRay.dir);
    const float B = 2 * Dot(aRay.dir, transformedOrigin);
    const float C = Dot(transformedOrigin, transformedOrigin) - (radius * radius);

    // Must use doubles, because when B ~ sqrt(B*B - 4*A*C)
    // the resulting t is imprecise enough to get around ray epsilons
    const double disc = B*B - 4*A*C;

    if(disc < 0)
      return false;

    const double discSqrt = std::sqrt(disc);
    const double q = (B < 0) ? ((-B - discSqrt) / 2.f) : ((-B + discSqrt) / 2.f);

    double t0 = q / A;
    double t1 = C / q;

    if(t0 > t1) std::swap(t0, t1);

    float resT;

    if(t0 > aRay.tmin && t0 < oResult.dist)
      resT = float(t0);
    else if(t1 > aRay.tmin && t1 < oResult.dist)
      resT = float(t1);
    else
      return false;

    oResult.dist   = resT;
    oResult.matID  = matID;
    oResult.normal = Normalize(transformedOrigin + Vec3f(resT) * aRay.dir);
    return true;
  }

  virtual void GrowBBox(
    Vec3f &aoBBoxMin,
    Vec3f &aoBBoxMax)
  {
    for(int i=0; i<8; i++)
    {
      Vec3f p = center;
      Vec3f half(radius);

      for(int j=0; j<3; j++)
        if(i & (1 << j)) half.Get(j) = -half.Get(j);

      p += half;

      for(int j=0; j<3; j++)
      {
        aoBBoxMin.Get(j) = std::min(aoBBoxMin.Get(j), p.Get(j));
        aoBBoxMax.Get(j) = std::max(aoBBoxMax.Get(j), p.Get(j));
      }
    }
  }

public:

  Vec3f center;
  float radius;
  int   matID;
};

class Cylinder : public AbstractGeometry
{
public:

  Cylinder(){}

  Cylinder(
    const Vec3f &aCenterBottom,
    const Vec3f &aCenterTop,
    float       aOuterRadius,
    float       aInnerRadius,
    int         aMatID) :
  centerBottom(aCenterBottom), centerTop(aCenterTop), outerRadius(aOuterRadius), innerRadius(aInnerRadius), matID(aMatID)
  {
  }

  virtual bool Intersect(
    const Ray &aRay,
    Isect     &oResult) const
  {
    // we transform ray origin into object space (center == origin)
    const Vec3f transformedOrigin = aRay.org - centerBottom;

    float intersectsOuterUpperCap = 0;
    float intersectsOuterLowerCap = 0;
    float intersectsInnerUpperCap = 0;
    float intersectsInnerLowerCap = 0;

    bool intersectsOuterCylinder = this->computeOuterCylinder(aRay, transformedOrigin, oResult, intersectsOuterUpperCap, intersectsOuterLowerCap);
    bool intersectsInnerCylinder = this->computeInnerCylinder(aRay, transformedOrigin, oResult, intersectsInnerUpperCap, intersectsInnerLowerCap);

    /*bool intersectsUpperCap = false;
    bool intersectsLowerCap = false;

    if (intersectsOuterUpperCap != 0 && intersectsInnerUpperCap == 0)
    {
      intersectsUpperCap = true;
      float resT = intersectsOuterUpperCap;
      oResult.dist   = resT;
      oResult.matID  = matID;
      oResult.normal = Vec3f(0, 0, 1);
    }

    if (intersectsOuterLowerCap != 0 && intersectsInnerLowerCap == 0)
    {
      intersectsInnerLowerCap = true;
      float resT = intersectsOuterLowerCap;
      oResult.dist   = resT;
      oResult.matID  = matID;
      oResult.normal = Vec3f(0, 0, -1);
    }*/

    return intersectsOuterCylinder || intersectsInnerCylinder;// || intersectsInnerLowerCap || intersectsUpperCap;
  }

  virtual void GrowBBox(
    Vec3f &aoBBoxMin,
    Vec3f &aoBBoxMax)
  {
  }

public:
  Vec3f centerTop;
  Vec3f centerBottom;
  float outerRadius;
  float innerRadius;
  int   matID;

private:
  bool computeOuterCylinder(const Ray &aRay, const Vec3f aTransformedOrigin, Isect &oResult, float& oIntersectsUpperCap, float& oIntersectsLowerCap) const
  {
    return this->computeCylinder(this->outerRadius, aRay, aTransformedOrigin, oResult, oIntersectsUpperCap, oIntersectsLowerCap);
  }

  bool computeInnerCylinder(const Ray &aRay, const Vec3f aTransformedOrigin, Isect &oResult, float& oIntersectsUpperCap, float& oIntersectsLowerCap) const
  {
    bool retval = this->computeCylinder(this->innerRadius, aRay, aTransformedOrigin, oResult, oIntersectsUpperCap, oIntersectsLowerCap);
    if (retval)
    {
      oResult.normal *= -1;
    }

    return retval;
  }

  bool computeCylinder(const float radius, const Ray &aRay, const Vec3f aTransformedOrigin, Isect &oResult, float& oIntersectsUpperCap, float& oIntersectsLowerCap) const
  {
	const float A = aRay.dir.x*aRay.dir.x + aRay.dir.y*aRay.dir.y;
    const float B = 2 * (aRay.dir.x * aTransformedOrigin.x + aRay.dir.y*aTransformedOrigin.y);
    const float C = (aTransformedOrigin.x*aTransformedOrigin.x + aTransformedOrigin.y*aTransformedOrigin.y) - (radius * radius);

    const double disc = B*B - 4*A*C;

    if(disc >= 0)
    {
      const double discSqrt = std::sqrt(disc);
      const double q = (B < 0) ? ((-B - discSqrt) / 2.f) : ((-B + discSqrt) / 2.f);

      double t0 = q / A;
      double t1 = C / q;

      float resT;
      bool retval = this->chooseIntersection(aRay, t0, t1, oResult.dist, resT, oIntersectsUpperCap, oIntersectsLowerCap);

      if (retval)
      {
        oResult.dist   = resT;
        oResult.matID  = matID;
        oResult.normal = Normalize(Vec3f(1, 1, 0) * aTransformedOrigin + resT * Vec3f(1, 1, 0) * aRay.dir);
      }

      return retval;
    }
    else
    {
      return false;
    }
  }

  /// <returns>Cylinder plane was hitted</returns>
  bool chooseIntersection(const Ray aRay, double t0, double t1, float intersectionDistance, float& oResT, float& oIntersectsUpperCap, float& oIntersectsLowerCap) const
  {
    if (t0 > t1)
    {
      std::swap(t0, t1);
    }

    double z0 = aRay.org.z + t0*aRay.dir.z;
    double z1 = aRay.org.z + t1*aRay.dir.z;

    bool z0Passed = z0 >= this->centerBottom.z && z0 <= this->centerTop.z;
    bool z1Passed = z1 >= this->centerBottom.z && z1 <= this->centerTop.z;
  
    bool t0deltaPassed = t0 > aRay.tmin && t0 < intersectionDistance;
    bool t1deltaPassed = t1 > aRay.tmin && t1 < intersectionDistance;

    if (z0Passed && t0deltaPassed)
    {
      oResT = t0;
      
      if (!z1Passed)
      {
        float capIntersection = (z0 - aRay.org.z) / aRay.dir.z;
        if (aRay.dir.z > 0)
        {
          oIntersectsLowerCap = capIntersection;
        }
        else
        {
          oIntersectsUpperCap = capIntersection;
        }
      }

      return true;
    }
    else if (z1Passed && t1deltaPassed)
    {
      oResT = t1;
      float capIntersection = (z1 - aRay.org.z) / aRay.dir.z;
      if (aRay.dir.z > 0)
      {
        oIntersectsLowerCap = capIntersection;
      }
      else
      {
        oIntersectsUpperCap = capIntersection;
      }

      return true;
    }
    else
    {
      return false;
    }
  }
};