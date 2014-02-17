#pragma once

#include <cmath>
#include <omp.h>
#include "pathtracer.hxx"

struct PathNode
{
  Vec3f surfPt;
  Vec3f normal;
  Vec3f wo1, wo2;
  float cosWo1, cosWo2;

  Vec3f brdf;
  float pdf;
  float dist1, dist2;

  const Material* mat;
  const AbstractLight* light;
};

class Path
{
public:
  Path (const Scene& scene)
    : scene(scene) {}
  bool createLightNodeAndNext(Rng& rng, int lightID)
  {
    PathNode lightNode;
    lightNode.light = this->scene.GetLightPtr(lightID);
    float pdfA, pdfW;
    Ray ray = lightNode.light->generateRay(rng, &pdfA, &pdfW);
    lightNode.surfPt = ray.org;
    lightNode.cosWo2 = lightNode.light->getCosGamma(ray.dir);
    lightNode.pdf = pdfA;
    lightNode.brdf = 1.f;
    lightNode.mat = &this->scene.GetMaterial(lightID);
    Isect isect;
    if (scene.Intersect(ray, isect))
    {
      PathNode newNode;
      newNode.surfPt = lightNode.surfPt + isect.dist * ray.dir;
      newNode.normal = isect.normal;
      Frame newFrame;
      newFrame.SetFromZ(newNode.normal);
      newNode.wo1 = newFrame.ToLocal(Normalize(-ray.dir));
      newNode.cosWo1 = newNode.wo1.z;
      lightNode.dist2 = isect.dist;
      newNode.dist1 = lightNode.dist2;
      newNode.pdf = PdfWtoA(pdfW, newNode.dist1, newNode.cosWo1);
      newNode.mat = &this->scene.GetMaterial(isect.matID);
      newNode.light = this->scene.GetLightPtr(isect.matID);
      this->path.push_back(lightNode);
      this->path.push_back(newNode);
      return true;
    }
    else
    {
      this->path.push_back(lightNode);
      return false;
    }
  }

  bool createNextNode(Rng& rng)
  {
    Vec2f randomVec = rng.GetVec2f();
    PathNode* lastNode = &this->path.back();
    float pdf;
    Frame frame;
    frame.SetFromZ(lastNode->normal);
    lastNode->wo2 = lastNode->mat->sampleBrdfHemisphere(randomVec, &pdf, &lastNode->brdf, lastNode->wo1, rng);
    lastNode->cosWo2 = lastNode->wo2.z;
    Ray ray(lastNode->surfPt, frame.ToWorld(lastNode->wo2), EPS_RAY);
    Isect isect;

    if (scene.Intersect(ray, isect))
    {
      PathNode newNode;
      newNode.surfPt = lastNode->surfPt + isect.dist * ray.dir;
      newNode.normal = isect.normal;
      Frame newFrame;
      newFrame.SetFromZ(newNode.normal);
      newNode.wo1 = newFrame.ToLocal(Normalize(-ray.dir));
      newNode.cosWo1 = newNode.wo1.z;
      lastNode->dist2 = isect.dist;
      newNode.dist1 = lastNode->dist2;
      newNode.pdf = PdfWtoA(pdf, newNode.dist1, newNode.cosWo1);
      newNode.mat = &this->scene.GetMaterial(isect.matID);
      newNode.light = this->scene.GetLightPtr(isect.matID);
      this->path.push_back(newNode);
      return true;
    }
    return false;
  }
//private:
  std::vector<PathNode> path;
  const Scene& scene;
};

class BiPath
{
  Path cameraPath;
  Path lightPath;
};

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
        Path path(this->mScene);
        bool isNext = path.createLightNodeAndNext(this->mRng, lightID);

        float cosCamToNormal = mScene.GetLightPtr(lightID)->getCosGamma(CameraDir(path.path[0].surfPt));
        this->HitTheCamera(path.path[0].surfPt, this->mScene.GetLightPtr(lightID)->getRadiance()
                                    * (1.0f/path.path[0].pdf)
                                    * cosCamToNormal);
        if (isNext) {
          cosCamToNormal = Dot(CameraDir(path.path[1].surfPt), path.path[1].normal);;
          this->HitTheCamera(path.path[1].surfPt, this->mScene.GetLightPtr(lightID)->getRadiance()
                                                  * (1.0f / path.path[0].pdf)
                                                  * (1.0f / Sqr(path.path[0].dist2))
                                                  * (1.0f / path.path[1].pdf)
                                                  * path.path[1].cosWo1
                                                  * path.path[0].cosWo2
                                                  * cosCamToNormal
                                                  * 0.5);
        }
        
      }
    }

    mIterations++;
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

  Vec3f CameraDir(Vec3f worldPt)
  {
    return Normalize(mScene.mCamera.mPosition - worldPt);
  }
};