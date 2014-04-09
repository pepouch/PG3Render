#pragma once

#include <cmath>
#include <omp.h>
#include "pathtracer.hxx"
#include "paths.hxx"

// Bidirectional path tracer for direct illumination
class BPTracer : public PathTracer
{
public:
  BPTracer(
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
        const int x = pixID % resX;
        const int y = pixID / resX;
        const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();
      for (int lightID = 0; lightID < mScene.GetLightCount(); lightID++)
      {
        BiPath biPath(mScene, mRng);
        biPath.createLight(lightID);
        biPath.createLightPath();
        float totalPdf = biPath.lightNode.pdfA;
        Vec3f capacity = biPath.lightNode.light->getRadiance();
        capacity *= biPath.lightNode.cosW1;
        for (int i=0; i<biPath.lightPath.size(); i++)
        {
          totalPdf *= biPath.lightPath[i].pdfA;
          capacity *= Inv(Sqr(biPath.lightPath[i].dist1));
          capacity *= biPath.lightPath[i].wl1.z;
          HitTheCamera(biPath.lightPath[i], capacity, totalPdf);
          capacity *= biPath.lightPath[i].wl2.z;
          capacity *= biPath.lightPath[i].brdf;
        }
      }

    }

    mIterations++;
  }

  void HitTheCamera(PathNode pn, Vec3f capacity, float totalPdf, bool isStartingLight = false)
  {
    const Vec3f cameraRay = mScene.mCamera.mPosition - pn.surfPt;
    const Vec2f rasterHit = mScene.mCamera.WorldToRaster(pn.surfPt);
    float cosToCamera = Dot(Normalize(-cameraRay), this->mScene.mCamera.mForward);
    Ray camRay(pn.surfPt, CameraDir(pn.surfPt), EPS_RAY);
    float cosCamToNormal;
    Vec3f brdf; 
    
    if (isStartingLight)
    {
      cosCamToNormal = pn.light->getCosGamma(camRay.dir);
      brdf = 1.f;
    }
    else
    {
      cosCamToNormal = pn.frame.ToLocal(camRay.dir).z;
      brdf = pn.mat->evalBrdf(pn.wl1, pn.frame.ToLocal(camRay.dir));
    }

    Isect isect; if (!this->mScene.Intersect(camRay, isect))
      mFramebuffer.AddColor(rasterHit, capacity
                                        * Inv(totalPdf)
                                        * brdf
                                        * cosCamToNormal
                                        * (1.0f/cameraRay.LenSqr())
                                        * 1.0f/Sqr(cosToCamera)
                                        * 1.0f/cosToCamera
                                        * 1.0f / Sqr(2.0f * tan(22.5f/360.0f * 2.0f * PI_F))
                                        );
  }

  Vec3f CameraDir(Vec3f worldPt)
  {
    return Normalize(mScene.mCamera.mPosition - worldPt);
  }
};