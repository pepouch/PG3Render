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

          LoDirect += this->pathForwardMIS(sceneHitState, 0);
        }

        mFramebuffer.AddColor(sample, LoDirect);
      }
    }

    mIterations++;
  }

  float maxSum(Vec3f v, Vec3f w)
  {
    float max = v.x + w.x;
    max = std::max(max, v.y + w.y);
    max = std::max(max, v.z + w.z);
    return max;
  }

  // combines light sampling and brdf random walk
  Vec3f pathForwardMIS(SceneHitState state, int depth)
  {
    float roulette = this->mRng.GetFloat();
    float reflectance = state.mat.mDiffuseReflectance.Max() + state.mat.mPhongReflectance.Max();
    
    if (roulette > reflectance)
    {
      return Vec3f(0);
    }

    Vec2f randomVec = this->mRng.GetVec2f();
    float pdf = 0;
    Vec3f brdf(0);
    Vec3f sampleHemisphere;
    Vec3f LoDirect(0);

    // sample light
    for (int i=0; i<mScene.GetLightCount(); i++)
    {
      Vec3f illum = this->sampleLight(state, i);
      float weight = state.pdfLight /(state.pdfLight + state.pdfBrdf);
      LoDirect += illum * state.mat.evalBrdf(state.frame.ToLocal(state.sampledRay.dir), state.wol) * (1.f / (state.pdfLight * reflectance)) * weight;
    }

    // sample brdf
    sampleHemisphere = state.mat.sampleBrdfHemisphere(randomVec, &pdf, &brdf, state.wol, this->mRng);
    state.setRayFromSample(sampleHemisphere);
    Vec3f illum = this->sampleDirection(state);
    float weight = pdf / (state.pdfLight + pdf);

    if (state.light != nullptr)
    {
      return illum  * brdf / (pdf * reflectance) * weight + LoDirect;
    }

    float cosThetaOut = Dot(state.frame.mZ, state.sampledRay.dir);

    SceneHitState newState(mScene.GetMaterial(state.isect.matID));
    newState.surfPt = state.surfPt + state.sampledRay.dir * state.isect.dist;
    newState.frame.SetFromZ(state.isect.normal);
    newState.wol = newState.frame.ToLocal(-state.sampledRay.dir);

    // Only cotribution from direct illumination is multiplied by weight,
    // the contribution from indirect (recursive) illumination is not
    // - according to slide no. 57 :-)
    return this->pathForwardMIS(newState, depth+1) * brdf / (pdf * reflectance) * cosThetaOut
       + LoDirect;
  }
};