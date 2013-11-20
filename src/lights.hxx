#pragma once

#include <vector>
#include <cmath>
#include "math.hxx"
#include "rng.hxx"

class AbstractLight
{
public:

	virtual Vec3f sampleIllumination(Rng& rng, const Vec3f& aSurfPt, const Frame& aFrame, Vec3f& oWig, float& oLightDist) const
	{
		return Vec3f(0);
	}

	virtual Vec3f getRadiance() const = 0;
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
		float& oLightDist) const
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

		if(cosTheta <= 0)
			return Vec3f(0);

		return mRadiance * cosTheta * cosGamma / (distSqr * mInvArea);
	}

	virtual Vec3f getRadiance() const override
	{
		return this->mRadiance;
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
		float& oLightDist) const
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

	virtual Vec3f sampleIllumination(
		Rng& rng,
		const Vec3f& aSurfPt, 
		const Frame& aFrame, 
		Vec3f& oWig, 
		float& oLightDist) const
	{
		Vec3f p;
		do
		{
			p = Vec3f(2.0*rng.GetFloat()-1.0, 2.0*rng.GetFloat()-1.0, 2.0*rng.GetFloat()-1.0);
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
		//   printf("%f %f %f %f\n", mBackgroundColor.x, mBackgroundColor.y, mBackgroundColor.z, cosTheta);
		return mBackgroundColor * cosTheta * 4 * PI_F;
	}


	virtual Vec3f getRadiance() const override
	{
		return this->mBackgroundColor;
	}

	Vec3f mBackgroundColor;
};
