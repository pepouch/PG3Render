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

  Vec3f CameraDir(Vec3f worldPt)
  {
    return Normalize(mScene.mCamera.mPosition - worldPt);
  }

  void HitTheCamera(Vec3f worldPt, Vec3f radiance)
  {
    const Vec3f cameraRay = mScene.mCamera.mPosition - worldPt;
    const Vec2f rasterHit = mScene.mCamera.WorldToRaster(worldPt);
    float cosToCamera = Dot(Normalize(-cameraRay), this->mScene.mCamera.mForward);
    mFramebuffer.AddColor(rasterHit, radiance
                                      * (1.0f/cameraRay.LenSqr())
                                      * 1.0f/Sqr(cosToCamera)
                                      * 1.0f/cosToCamera
                                      * 1.0f / Sqr(2.0f * tan(22.5f/360.0f * 2.0f * PI_F)));
  }

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
        Vec3f lightNormal = dynamic_cast<const AreaLight*>(mScene.GetLightPtr(lightID))->mFrame.mZ;
        float cosCamToNormal = Dot(CameraDir(ray.org), lightNormal);
        this->HitTheCamera(ray.org, this->mScene.GetLightPtr(lightID)->getRadiance()
                                    * (1.0f/pdfA)
                                    * cosCamToNormal);

        float cosRayToNormal = Dot(ray.dir, lightNormal);

        Isect isect;
        if(mScene.Intersect(ray, isect))
        {
          Vec3f LoDirect = Vec3f(0);
          SceneHitState sceneHitState(mScene.GetMaterial( isect.matID ));
          sceneHitState.surfPt = ray.org + ray.dir * isect.dist;
          sceneHitState.frame.SetFromZ(isect.normal);
          sceneHitState.wol = sceneHitState.frame.ToLocal(-ray.dir);
          sceneHitState.isect = isect;

          this->lightForward(mScene.GetLightPtr(lightID)->getRadiance() * (1.0f/pdfA) * (1.0f/PdfWtoA(pdfW, isect.dist, sceneHitState.wol.z)) * cosRayToNormal, sceneHitState, 0);

        }
      }
    }

    mIterations++;
  }


  void lightForward(Vec3f radiance, SceneHitState state, int depth)
  {
    if (depth >0) return;
    float roulette = this->mRng.GetFloat();
    float reflectance = state.mat.mDiffuseReflectance.Max() + state.mat.mPhongReflectance.Max();
    if (reflectance > 1.0f)
      reflectance = 1.0f;
    
    if (roulette > reflectance)
    {
      return;
    }

    Vec2f randomVec = this->mRng.GetVec2f();
    float pdf = 0;
    Vec3f brdf(0);
    Vec3f sampleHemisphere;
    Vec3f LoDirect(0);

    // connect to the camera
    const Vec3f cameraDir = this->CameraDir(state.surfPt);
    const Vec2f rasterHit = mScene.mCamera.WorldToRaster(state.surfPt);
    Ray cameraRay(state.surfPt, cameraDir, EPS_RAY);
    Isect cameraIsect;
    if (!mScene.Intersect(cameraRay, cameraIsect) || cameraIsect.dist > (mScene.mCamera.mPosition - state.surfPt).Length())
    {
      Vec3f L(0);
      float cosToCamera = Dot(Normalize(-cameraDir), this->mScene.mCamera.mForward);
      L = radiance 
                                         * state.mat.evalBrdf(state.wol, state.frame.ToLocal(cameraDir))
                                         * Dot(state.frame.ToWorld(state.wol), state.frame.mZ)  // cos of incoming light to normal
                                         * Dot(cameraDir, state.frame.mZ) // cos of outgoing light to normal
                                         * (1.0f/Sqr(state.isect.dist));   // distance from previous point on scene
      this->HitTheCamera(state.surfPt, L);
    }

    // sample brdf
    sampleHemisphere = state.mat.sampleBrdfHemisphere(randomVec, &pdf, &brdf, state.wol, this->mRng);
    state.setRayFromSample(sampleHemisphere);
    Vec3f illum = this->sampleDirection(state);
    if (state.isect.matID == -1)
      return;

    float cosThetaOut = Dot(state.frame.mZ, state.sampledRay.dir);
    float cosThetaIn = Dot(state.isect.normal, -state.sampledRay.dir);

    SceneHitState newState(mScene.GetMaterial(state.isect.matID));
    newState.surfPt = state.surfPt + state.sampledRay.dir * state.isect.dist;
    newState.frame.SetFromZ(state.isect.normal);
    newState.wol = newState.frame.ToLocal(-state.sampledRay.dir);

    this->lightForward(radiance * brdf / (pdf /* reflectance*/) * cosThetaOut * cosThetaIn / Sqr(state.isect.dist),
                       newState, depth+1);
  }
};