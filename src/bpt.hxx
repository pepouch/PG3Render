#pragma once

#include <cmath>
#include <omp.h>
#include "pathtracer.hxx"

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
        mFramebuffer.AddColor(sample, Vec3f(1, 0.5, 0.5));
    }

    mIterations++;
  }

};