#pragma once

#include <cmath>
#include <omp.h>
#include "pathtracer.hxx"

// Path tracer for direct illumination
class PathTracerDirect : public PathTracer
{
public:
  PathTracerDirect(
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

#if (TASK_NUMBER == 1)

          for(int i=0; i<mScene.GetLightCount(); i++)
          {
            Vec3f illum = this->sampleLight(sceneHitState, i);
            LoDirect += illum * sceneHitState.mat.evalBrdf(sceneHitState.frame.ToLocal(sceneHitState.sampledRay.dir), sceneHitState.wol) / sceneHitState.pdfLight;
          }
          
#elif (TASK_NUMBER == 2)

					Vec2f randomVec = this->mRng.GetVec2f();
					float pdf = 0;
					Vec3f brdf(0);
					Vec3f sampleHemisphere;
#if (SUBTASK_NUMBER == 1)
					sampleHemisphere = SamplePowerCosHemisphereW(randomVec, 0, &pdf);
					//sampleHemisphere = sampleUniformHemisphere(randomVec, &pdf);
					brdf = sceneHitState.mat.evalBrdf(sceneHitState.wol, sampleHemisphere);
#elif (SUBTASK_NUMBER == 2)
					sampleHemisphere = sceneHitState.mat.sampleBrdfHemisphere(randomVec, &pdf, &brdf, sceneHitState.wol, this->mRng);
					//brdf = mat.evalBrdf(wol, sampleHemisphere);
#endif
          sceneHitState.setRayFromSample(sampleHemisphere);
          LoDirect += this->sampleDirection(sceneHitState) * brdf / pdf;

#elif (TASK_NUMBER == 3)

#define SAMPLE_BRDF
#define SAMPLE_LIGHT
#define SAMPLE_WEIGHT weight

#ifdef SAMPLE_LIGHT
					for (int i=0; i<mScene.GetLightCount(); i++)
					{
            Vec3f illum = this->sampleLight(sceneHitState, i);
  					float weight = sceneHitState.pdfLight /(sceneHitState.pdfLight + sceneHitState.pdfBrdf);
            LoDirect += illum * sceneHitState.mat.evalBrdf(sceneHitState.frame.ToLocal(sceneHitState.sampledRay.dir), sceneHitState.wol) * (1.f / sceneHitState.pdfLight) * SAMPLE_WEIGHT;
          }
#endif

#ifdef SAMPLE_BRDF
						Vec2f randomVec = this->mRng.GetVec2f();
						Vec3f brdf(0);
            float pdfLight = 0, pdfBrdf = 0;
						Vec3f sampleHemisphere = sceneHitState.mat.sampleBrdfHemisphere(randomVec, &pdfBrdf, &brdf, sceneHitState.wol, this->mRng);
            sceneHitState.setRayFromSample(sampleHemisphere);
            
            Vec3f illum = this->sampleDirection(sceneHitState);
            float weight = pdfBrdf / (sceneHitState.pdfLight + pdfBrdf);
            LoDirect += illum * (brdf / pdfBrdf) * SAMPLE_WEIGHT;
#endif
#endif
				}

				mFramebuffer.AddColor(sample, LoDirect);
			}
		}

		mIterations++;
	}
};