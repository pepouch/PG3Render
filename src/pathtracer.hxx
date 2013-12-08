#ifndef PATHTRACER_HXX_
#define PATHTRACER_HXX_

#define TASK_NUMBER 3
#define SUBTASK_NUMBER 2

#include <vector>
#include <cmath>
#include <omp.h>
#include <cassert>
#include "renderer.hxx"
#include "rng.hxx"


// Right now this is a copy of EyeLight renderer. The task is to change this 
// to a full-fledged path tracer.
class PathTracer : public AbstractRenderer
{
public:

	PathTracer(
		const Scene& aScene,
		int aSeed = 1234
		) :
	AbstractRenderer(aScene), mRng(aSeed)
	{}

  struct SceneHitState
  {
    SceneHitState(const Material& mat)
      : mat(mat),
        light(NULL)
    {
    }
    void setRayFromSample(const Vec3f& sample)
    {
      this->sampledRay = Ray(this->surfPt, this->frame.ToWorld(sample), EPS_RAY);
    }
    Vec3f surfPt;
    Frame frame;
    Vec3f wol;
    const Material& mat;
    Ray sampledRay;
    // light hit by sampledRay
    const AbstractLight* light;
    // pdf of given direction, as if we were sampling light
    float pdfLight;
    // pdf of given light sample, as if we were sampling brdf
    float pdfBrdf;
  };

protected:
  Vec3f sampleLight(SceneHitState& state, int lightID)
  {
    Vec3f LoDirect(0);

		const AbstractLight* light = mScene.GetLightPtr(lightID);
    state.light = light;
		if (light == NULL)
    {
      state.pdfLight = 1;
      return LoDirect;
    }

		float lightDist;
		Vec3f illum(0);
    Vec3f wig;
    float pdf = 0;

		illum = light->sampleIllumination(mRng, state.surfPt, state.frame, wig, lightDist, pdf);

		if(illum.Max() > 0)
		{
			if( ! mScene.Occluded(state.surfPt, wig, lightDist) )
				LoDirect += illum;
		}

    state.pdfBrdf = state.mat.getPdf(state.frame.ToLocal(wig), state.wol);
    state.pdfBrdf = light->transformPdfToLight(state.pdfBrdf, wig, lightDist);

    state.pdfLight = pdf;
    state.sampledRay.dir = wig;
    return LoDirect;
  }

  Vec3f sampleDirection(SceneHitState& state)
  {
    
    Vec3f LoDirect(0);
	  Isect lightIsect;
    const AbstractLight* light = NULL;

    if(mScene.Intersect(state.sampledRay, lightIsect))
	  {
		  if (lightIsect.lightID >= 0)
		  {
        light = mScene.GetLightPtr(lightIsect.lightID);
        if (light->getCosGamma(-state.sampledRay.dir) > EPS_COSINE)
			    LoDirect = light->getRadiance();
		  }
	  }
	  else
	  {
		  for(int i=0; i<mScene.GetLightCount(); i++)
		  {
			  light = mScene.GetLightPtr(i);
			  if (light->isBackground())
			  {
				  LoDirect += light->getRadiance();
          break;
			  }
		  }
	  }

    state.light = light;
    if (light)
      state.pdfLight = light->getPdf(state.sampledRay,lightIsect);
    else
      state.pdfLight = 1;
    
    float cosThetaOut = Dot(state.frame.mZ, state.sampledRay.dir);
    return LoDirect * cosThetaOut;
  }

public:
	Rng              mRng;
};

#endif // PATHTRACER_HXX_