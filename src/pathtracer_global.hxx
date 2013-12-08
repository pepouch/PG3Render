#pragma once

#include <cmath>
#include <omp.h>
#include "pathtracer.hxx"

// Path tracer for direct illumination
class PathTracerGlobal : public PathTracer
{
public:
  PathTracerGlobal(
		const Scene& aScene,
		int aSeed = 1234
		) :
	PathTracer(aScene, aSeed)
	{}

  virtual void RunIteration(int aIteration)
	{
		const int resX = int(mScene.mCamera.mResolution.x);
		const int resY = int(mScene.mCamera.mResolution.y);

		for(int pixID = 0; pixID < resX * resY; pixID++)
		{
			//////////////////////////////////////////////////////////////////////////
			// Generate ray
			const int x = pixID % resX;
			const int y = pixID / resX;

			const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();

			Ray   ray = mScene.mCamera.GenerateRay(sample);
			Isect isect;

			if(mScene.Intersect(ray, isect))
			{
				Vec3f LoDirect = Vec3f(0);

				// if the ray intersected a light source, simply add radiance and continue
        if (isect.lightID >= 0 && mScene.GetLightPtr(isect.lightID)->getCosGamma(-ray.dir) > EPS_COSINE) {
					LoDirect = mScene.GetLightPtr(isect.lightID)->getRadiance();
        }
				else
				{			
          SceneHitState sceneHitState(mScene.GetMaterial( isect.matID ));
          sceneHitState.surfPt = ray.org + ray.dir * isect.dist;
          sceneHitState.frame.SetFromZ(isect.normal);
          sceneHitState.wol = sceneHitState.frame.ToLocal(-ray.dir);


					Vec2f randomVec = this->mRng.GetVec2f();
					float pdf = 0;
					Vec3f brdf(0);
					Vec3f sampleHemisphere;

					sampleHemisphere = sceneHitState.mat.sampleBrdfHemisphere(randomVec, &pdf, &brdf, sceneHitState.wol, this->mRng);
					//brdf = mat.evalBrdf(wol, sampleHemisphere);
          sceneHitState.setRayFromSample(sampleHemisphere);
          LoDirect += this->sampleDirection(sceneHitState) * brdf / pdf;

				}

				mFramebuffer.AddColor(sample, LoDirect);
			}
		}

		mIterations++;
	}
};