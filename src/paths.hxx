struct PathNode
{
  Vec3f surfPt;
  Vec3f wl1, wl2;
  Frame frame;

  Vec3f brdf;
  float pdfA;
  float dist1, dist2;

  const Material* mat;
  const AbstractLight* light;

  PathNode()
    : mat(NULL)
    , light(NULL)
  {}
};

struct CameraNode
{
};

struct LightNode
{
  const AbstractLight* light;
  Ray ray;
  float cosW1;
  float pdfA;
  float pdfW;
  
  LightNode()
    : light(NULL)
  {}

  LightNode(Rng& rng, const Scene &scene, int lightID)
  {
    this->light = scene.GetLightPtr(lightID);
    this->ray = this->light->generateRay(rng, &this->pdfA, &this->pdfW);
    this->cosW1 = this->light->getCosGamma(ray.dir);
  }
};

class Path
{
private:
  std::vector<PathNode> path;
  const Scene& scene;

public:
  Path (const Scene& scene)
    : scene(scene) {}

  bool createNode(Ray ray, float pdfW)
  {
    Isect isect;
    if (scene.Intersect(ray, isect))
    {
      PathNode pn = this->create(ray, isect, pdfW);
      this->path.push_back(pn);
      return true;
    }
    else
    {
      return false;
    }
  }

  bool createCameraNodeAndNext(Rng& rng, int x, int y)
  {
  }

  bool createNextNode(Rng& rng)
  {
    int pnCount = this->path.size();
    if (pnCount <= 0 || pnCount > 10)
      return false;
    
    float roulette = rng.GetFloat();
    float reflectance = this->path.back().mat->mDiffuseReflectance.Max() + this->path.back().mat->mPhongReflectance.Max();
    if (reflectance > 1.0f)
      reflectance = 1.0f;
    if (roulette > reflectance)
      return false;

    float pdfW;
    Ray ray = this->generate(&path.back(), &pdfW, rng);
    pdfW *= reflectance;
    if (!this->createNode(ray, pdfW))
      return false;

    this->path[pnCount-1].dist2 = this->path.back().dist1;
    this->path[pnCount-1].wl2 = this->path[pnCount-1].frame.ToLocal(ray.dir);
    return true;
  }

  PathNode& operator[](int index)
  {
    return this->path[index];
  }

  int size()
  {
    return this->path.size();
  }

private:
  PathNode create(Ray ray, Isect isect, float pdfW)
  {
    PathNode pn;
    pn.surfPt = ray.org + isect.dist * ray.dir;
    pn.dist1 = isect.dist;
    pn.frame.SetFromZ(isect.normal);
    pn.wl1 = pn.frame.ToLocal(-ray.dir);
    pn.pdfA = PdfWtoA(pdfW, pn.dist1, pn.wl1.z);
    pn.mat = &this->scene.GetMaterial(isect.matID);
    if (isect.lightID >= 0)
      pn.light = this->scene.GetLightPtr(isect.lightID);
    return pn;
  }

  Ray generate(PathNode* pn, float* pdf, Rng rng)
  {
    Vec2f randomVec = rng.GetVec2f();
    pn->wl2 = pn->mat->sampleBrdfHemisphere(randomVec, pdf, &pn->brdf, pn->wl1, rng);
    Ray ray(pn->surfPt, pn->frame.ToWorld(pn->wl2), EPS_RAY);
    return ray;
  }
};

class BiPath
{
public:
  Path cameraPath;
  Path lightPath;
  CameraNode cameraNode;
  LightNode lightNode;

  BiPath(const Scene& scene, Rng& rng)
    : scene(scene)
    , rng(rng)
    , cameraPath(scene)
    , lightPath(scene)
  {}

  void createLight(int lightID)
  {
    this->lightNode = LightNode(this->rng, this->scene, lightID);
  }

  void createLightPath()
  {
    this->lightPath.createNode(this->lightNode.ray, this->lightNode.pdfW);
    while(this->lightPath.createNextNode(this->rng))
      ;
  }

private:
    const Scene& scene;
    Rng& rng;
};