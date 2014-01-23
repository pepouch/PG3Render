#pragma once

#include <cmath>
#include <omp.h>
#include "pathtracer.hxx"

// Path tracer for direct illumination
class LightTracer : public PathTracer
{
public:
  LightTracer(
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
      for (int lightID = 0; lightID < mScene.GetLightCount(); lightID++)
      {
        float pdfA, pdfW;
        Ray   ray = mScene.GetLightPtr(lightID)->generateRay(this->mRng, &pdfA, &pdfW);

        // direct connection light-camera
        const Vec3f cameraRay = mScene.mCamera.mPosition - ray.org;
        const Vec2f rasterHit = mScene.mCamera.WorldToRaster(ray.org);
        float len = cameraRay.Length();
        float cosLightNormalToRay = Dot(Normalize(cameraRay),
                                        dynamic_cast<const AreaLight*>(mScene.GetLightPtr(lightID))->mFrame.mZ);
        mFramebuffer.AddColor(rasterHit, mScene.GetLightPtr(lightID)->getRadiance() 
                                         * (1.0f/cameraRay.LenSqr()) 
                                         * (cosLightNormalToRay));

        Isect isect;
        if(mScene.Intersect(ray, isect))
        {
          Vec3f LoDirect = Vec3f(0);
          SceneHitState sceneHitState(mScene.GetMaterial( isect.matID ));
          sceneHitState.surfPt = ray.org + ray.dir * isect.dist;
          sceneHitState.frame.SetFromZ(isect.normal);
          sceneHitState.wol = sceneHitState.frame.ToLocal(-ray.dir);

          // for now, only one-bounce connection to the camera
          if (true) {
            const Vec3f cameraDir = mScene.mCamera.mPosition - sceneHitState.surfPt;
            const Vec2f rasterHit = mScene.mCamera.WorldToRaster(sceneHitState.surfPt);
            float cosThetaOut = Dot(sceneHitState.frame.mZ, -ray.dir);
            float cosThetaIn = Dot(sceneHitState.frame.mZ, Normalize(cameraDir));
            Vec3f brdf = sceneHitState.mat.evalBrdf(
                         sceneHitState.frame.ToLocal(Normalize(cameraDir)),
                         sceneHitState.frame.ToLocal(-ray.dir));
            Ray cameraRay(sceneHitState.surfPt, Normalize(cameraDir), EPS_RAY);
            Isect cameraIsect;
            if (!mScene.Intersect(cameraRay, cameraIsect) || cameraIsect.dist > (mScene.mCamera.mPosition - sceneHitState.surfPt).Length())
            {
              LoDirect = mScene.GetLightPtr(lightID)->getRadiance() * (1.0f / (pdfA * PdfWtoA(pdfW, isect.dist, cosThetaOut))) * brdf
                                            * cosThetaOut * cosThetaIn
                                            * (1.0f / cameraDir.LenSqr())
                                            * cosLightNormalToRay
                                            * (1.0f / Sqr(isect.dist));
              mFramebuffer.AddColor(rasterHit, LoDirect);
            }
          }
          else
          {			
            LoDirect += this->lightForward(sceneHitState, 0);
          }
        }
      }
    }

    mIterations++;
  }

  // combines light sampling and brdf random walk
  Vec3f lightForward(SceneHitState state, int depth)
  {
    float roulette = this->mRng.GetFloat();
    float reflectance = state.mat.mDiffuseReflectance.Max() + state.mat.mPhongReflectance.Max();
    if (reflectance > 1.0f)
      reflectance = 1.0f;
    
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

    return this->lightForward(newState, depth+1) * brdf / (pdf * reflectance) * cosThetaOut
       + LoDirect;
  }
};