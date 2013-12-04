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
	
	virtual float getPdf(const Ray& ray) const = 0;
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

		if(cosTheta <= 0 || aSurfPt.z > this->p0.z)
			return Vec3f(0);
    oPdf =  distSqr * mInvArea / cosGamma;
		return mRadiance * cosTheta;
	}

	virtual Vec3f getRadiance() const override
	{
		return this->mRadiance;
	}

	virtual float getCosGamma(const Vec3f& dir) const override
	{
		return Dot(this->mFrame.mZ, dir);
	}

	virtual float getPdf(const Ray& ray) const override
	{
		Triangle triangle1(this->p0, this->p0 + this->e1, this->p0 + this->e2, 0);
		Triangle triangle2(this->p0 + this->e1 + this->e2, this->p0 + this->e1, this->p0 + this->e2, 0);

		Isect i;
		if (triangle1.Intersect(ray, i) || triangle2.Intersect(ray, i))
		{
			float cosTheta = Dot(-this->mFrame.mZ, ray.dir);

			return this->mInvArea * i.dist * i.dist / cosTheta;
		}
		else
		{
			return 0;
		}
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

		if(cosTheta <= 0)
			return Vec3f(0);

		return mIntensity * cosTheta / distSqr;
	}

	virtual Vec3f getRadiance() const override
	{
		return this->mIntensity;
	}

	virtual float getPdf(const Ray& ray) const override
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

		if(cosTheta <= 0)
			return Vec3f(0);
		return mBackgroundColor * cosTheta * 4 * PI_F;
	}


	virtual Vec3f getRadiance() const override
	{
		return this->mBackgroundColor;
	}

	virtual float getPdf(const Ray& ray) const override
	{
		return 1.f / (4.f * PI_F);
	}

	Vec3f mBackgroundColor;
};
