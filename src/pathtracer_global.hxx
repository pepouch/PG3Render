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

          LoDirect += this->pathForward(sceneHitState);
        }

        mFramebuffer.AddColor(sample, LoDirect);
      }
    }

    mIterations++;
  }

  Vec3f pathForward(SceneHitState state)
  {
    float roulette = this->mRng.GetFloat();
    float reflectance = state.mat.mDiffuseReflectance.Max() + state.mat.mPhongReflectance.Max();
    if (roulette > reflectance)
      return Vec3f(0);

    Vec2f randomVec = this->mRng.GetVec2f();
    float pdf = 0;
    Vec3f brdf(0);
    Vec3f sampleHemisphere;

    sampleHemisphere = state.mat.sampleBrdfHemisphere(randomVec, &pdf, &brdf, state.wol, this->mRng);
    state.setRayFromSample(sampleHemisphere);
    Vec3f illum = this->sampleDirection(state);

    if (state.light != NULL)
      return illum  * brdf / (pdf * reflectance);

    SceneHitState newState(mScene.GetMaterial(state.isect.matID));
    newState.surfPt = state.surfPt + state.sampledRay.dir * state.isect.dist;
    newState.frame.SetFromZ(state.isect.normal);
    newState.wol = newState.frame.ToLocal(-state.sampledRay.dir);
    return this->pathForward(newState) * brdf / (pdf * reflectance);
  }
};