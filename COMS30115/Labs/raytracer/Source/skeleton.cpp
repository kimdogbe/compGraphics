#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "limits.h"
#include <fstream>
#include <string>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

float m = std::numeric_limits<float>::max();

#define SCREEN_WIDTH 260
#define SCREEN_HEIGHT 240
#define FULLSCREEN_MODE false

struct Intersection
{
vec4 position;
float distance;
int triangleIndex;
float bumpX;
float bumpY;
};

vector<Triangle> triangles;
float focalLength = SCREEN_WIDTH/2;
vec4 cameraPos(0, 0, -2, focalLength);

/*camera rotation*/
mat4 R;
float yaw = 0;
float pitch = 0;

/*illumination*/
vec4 lightPos( 0, -0.5, -0.7, 1.0 );
vec3 lightColor = 14.f * vec3( 1, 1, 1 );

vec3 indirectLightColor = 0.52f * vec3(1,1,1);

const float glossiness = 0.15f;
const int numTestRays = 4;
const int numBounces = 1;
const float bounceColourRetainment = 0.2;

vector<int> bumpMap;
const int mapSizeX = 500;
const int mapSizeY = 300;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
vec4 RotateCameraY(vec4 vectorRotate, float dYaw);
vec4 RotateCameraX(vec4 vectorRotate, float dPitch);
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles,
   Intersection& closestIntersection );
vec3 DirectLight(const Intersection& i);
vec4 crossVec4(vec4 _v1, vec4 _v2);
vec3 BounceRay(const vector<Triangle>& triangles, vec4 incomingRay, Intersection currentIntersection, float glossiness, int numTestRays, int remainingBounces);
float FetchMapValue(Intersection in);
vec4 AdjustNormal(Intersection in, vec4 originalNormal, vec4 cameraPos);

int main( int argc, char* argv[] )
{

  ifstream mapFile ("Build/metal.pgm", ios::in);
  for (int i = 0; i < mapSizeX; i++) {
    for (int j = 0; j< mapSizeY; j++) {
      int mapVal;
      mapFile >> mapVal;
      bumpMap.insert(bumpMap.begin(), mapVal);
    }
  }
  mapFile.close();

  /*fill vector with triangles from testModel*/
  LoadTestModel(triangles);

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( Update() )
    {
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  vec3 colour(0.0, 0.0, 0.0);

  Intersection closestIntersection;
  Intersection shadowIntersection;

  for(int y = 0; y < screen->height; y++){
    for(int x = 0; x < screen->width; x++){
      vec4 start(x, y, 0, 1.0);
      vec4 rayDir(x - screen->width/2, y - screen->height/2, focalLength, 1.0);

      if(ClosestIntersection(cameraPos, rayDir, triangles, closestIntersection)){
        vec4 shadowRay = normalize(lightPos - closestIntersection.position);
        ClosestIntersection(closestIntersection.position + 0.01f*triangles[closestIntersection.triangleIndex].normal, shadowRay, triangles, shadowIntersection);
        float surfaceToSurface = shadowIntersection.distance;
        float surfaceToLight = length(closestIntersection.position - lightPos);
        colour = BounceRay(triangles, rayDir, closestIntersection, glossiness, numTestRays, numBounces) * indirectLightColor ;
        if (surfaceToSurface >= surfaceToLight) {
          colour += (DirectLight(closestIntersection)) *
           triangles[closestIntersection.triangleIndex].color;
        }
      }else{
        colour = vec3(0.0, 0.0, 0.0);
      }
      PutPixelSDL(screen, x, y, colour);
    }
  }

  /*vec3 colour(1.0,0.0,0.0);
  for(int i=0; i<1000; i++)
  {
    uint32_t x = rand() % screen->width;
    uint32_t y = rand() % screen->height;
    PutPixelSDL(screen, x, y, colour);
  }*/
}

vec4 LookAt(vec4 target, vec4 cameraPos, vec4 dir){
  vec4 diff = target - cameraPos;
  vec3 diff3(diff.x, diff.y, diff.z);
  vec3 normalDiff = normalize(diff3);

  vec3 unitVec(0,1,0);
  vec3 right = cross(unitVec, normalDiff);
  vec3 up = cross(normalDiff, right);

  mat4 rot(right.x,      right.y,      right.z,      0,
           up.x,         up.y,         up.z,         0,
           normalDiff.x, normalDiff.y, normalDiff.z, 0,
           cameraPos.x , cameraPos.y,  cameraPos.z,  1);

  return rot*dir;

  /*
  xzDiff.y = 0;
  xzDiff =  normalize(xzDiff);
  float yRot = asin(xzDiff.x);
  //float yRot = atan2(sqrt(normalDiff.x * normalDiff.x + normalDiff.z * normalDiff.z), normalDiff.y) - M_PI_2;
  std::cout<<yRot<<endl;

  //dir = RotateCameraX(dir, xRot);
  dir = RotateCameraY(dir, yRot);*/
  //return dir;
}

/*Place updates of parameters here*/
bool Update()
{
  /* Compute frame time */
  static int t = SDL_GetTicks();
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/

  SDL_Event e;
  while(SDL_PollEvent(&e))
  {
    if (e.type == SDL_QUIT)
    {
      return false;
    }
    else if (e.type == SDL_KEYDOWN)
    {
      int key_code = e.key.keysym.sym;
      switch(key_code)
        {
        case SDLK_w:
          /* Move light forward */
          lightPos.z += 0.2f;
          break;
        case SDLK_s:
          /* Move light backwards */
          lightPos.z -= 0.2f;
          break;
        case SDLK_a:
          /* Move light left */
          lightPos.x -= 0.2f;
          break;
        case SDLK_d:
          /* Move light right */
          lightPos.x += 0.2f;
          break;
        case SDLK_q:
          /* Move light right */
          lightPos.y += 0.2f;
          break;
        case SDLK_e:
          /* Move light right */
          lightPos.y -= 0.2f;
          break;

        case SDLK_UP:
          /* Move camera forward */
          cameraPos.z += 0.2f;
          break;
        case SDLK_DOWN:
          /* Move camera backwards */
          cameraPos.z -= 0.2f;
          break;
        case SDLK_LEFT:
          /* Move camera left */
          cameraPos.x -= 0.2f;
          yaw += 0.2f;
          break;
        case SDLK_RIGHT:
          /* Move camera right */
          cameraPos.x += 0.2f;
          yaw -= 0.2f;
          break;
        case SDLK_ESCAPE:
          /* Move camera quit */
          return false;
      }
    }
  }
  return true;
}

vec4 RotateCameraY(vec4 vectorRotate, float dYaw){
  mat4 rot(cos(dYaw),0,sin(dYaw),0,
           0,1,0,0,
           -sin(dYaw),0,cos(dYaw),0,
           0,0,0,1);

  return rot*vectorRotate;
}

vec4 RotateCameraX(vec4 vectorRotate, float dPitch){
  mat4 rot(1,0,0,0,
           0,cos(dPitch),-sin(dPitch),0,
           0,sin(dPitch),cos(dPitch), 0,
           0,0,0,1);

  return rot*vectorRotate;
}

vec4 crossVec4(vec4 _v1, vec4 _v2){
    vec3 vec1 = vec3(_v1[0], _v1[1], _v1[2]);
    vec3 vec2 = vec3(_v2[0], _v2[1], _v2[2]);
    vec3 res = cross(vec1, vec2);
    return vec4(res[0], res[1], res[2], 1);
}

vec3 BounceRay(const vector<Triangle>& triangles, vec4 incomingRay, Intersection currentIntersection, float glossiness, int numTestRays, int remainingBounces) {
  // Take incoming ray's mirror direction on the plane defined by triangle's normal
  vec3 selfColour = triangles[currentIntersection.triangleIndex].color;
  if (remainingBounces == 0) return selfColour;
  vec4 incomingNormal = normalize(triangles[currentIntersection.triangleIndex].normal);
  vec4 incomingRayEndpoint = currentIntersection.position;
  vec4 mirroredRay = incomingRay - 2 * dot(incomingRay, incomingNormal) * incomingNormal;

  vec3 colourSum = vec3(0.0,0.0,0.0);
  // Calculate numTestRays random directions in the 'hemisphere" defined by the mirrored direction and the full hemisphere scaled by glossiness down to that size
  for (int i = 0; i < numTestRays; i++) {
    float radDev = (rand() * (float)M_PI * 2 - (float)M_PI) * glossiness;
    float angleToPlane = acos(dot(normalize(mirroredRay), incomingNormal));
    float scalingDev = ((rand()  * M_PI - (M_PI / 2)) - angleToPlane) * glossiness ;

    vec4 rotV = mirroredRay * cos(radDev) + crossVec4(incomingNormal,mirroredRay) * sin(radDev) +
                incomingNormal * dot(incomingNormal, mirroredRay) * (1.0f - cos(radDev));
    vec4 mirroredRotV = rotV - 2 * dot(rotV, incomingNormal) * incomingNormal;
    vec4 secondRotationAxis = normalize(crossVec4(rotV,mirroredRotV));
    vec4 finalV = rotV * cos(scalingDev) + (secondRotationAxis * rotV) * sin(scalingDev) +
                secondRotationAxis * dot(secondRotationAxis, rotV) * (1.0f - cos(scalingDev));

    // test ray against triangles
    vec4 nextNormal;
    Intersection bounceIntersection;
    if (ClosestIntersection(incomingRayEndpoint, finalV, triangles, bounceIntersection)) {
      // If it hits a triangle, continue recursion
      colourSum += BounceRay(triangles, finalV, bounceIntersection, glossiness, numTestRays, remainingBounces-1);
    } else {
      colourSum += selfColour;
    }

  }
  // at 0 glossiness it's a straight mirror and all test rays go in direction of the mirrored ray
  // at 1 glossiness they go in any direction on the full hemisphere around the normal

  colourSum = ((colourSum / (float)numTestRays) * (1-bounceColourRetainment)) + (selfColour * bounceColourRetainment);
  return colourSum;
}

// Start position of a ray, direction of the ray, triangles to check, return value if true
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles,
   Intersection& closestIntersection ){

  //dir = RotateCameraY(dir, yaw);
  vec4 targetView(0.5f,0.5f,0.5f,1.0f);
  dir = LookAt(targetView, cameraPos, dir);

  bool inTriangle = false;
  float m = std::numeric_limits<float>::max();
  closestIntersection.distance = m;

  for(size_t i = 0; i < triangles.size(); i++){
    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;
    vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
    vec3 d;
    d.x = dir.x;
    d.y = dir.y;
    d.z = dir.z;
    glm::mat3 A( -d, e1, e2 );
    vec3 x = glm::inverse( A ) * b;

    vec4 currentPoint;
    currentPoint.x = start.x + x.x*d.x;
    currentPoint.y = start.y + x.x*d.y;
    currentPoint.z = start.z + x.x*d.z;
    float currentDistance = x.x;

    if (x.y >= 0 && x.z >= 0 && x.y + x.z <= 1 && x.x >= 0) {
      if(closestIntersection.distance > currentDistance){
        closestIntersection.position = currentPoint;
        closestIntersection.distance = x.x;
        closestIntersection.bumpX = x.y;
        closestIntersection.bumpY = x.z;
        /*might need refining by substracting start from it*/
        //closestIntersection.distance = currentDistance;
        closestIntersection.triangleIndex = i;
        inTriangle = true;
      }
    }
  }
  return inTriangle;
}

float FetchMapValue(Intersection in) {
    int mapX = (int)round(in.bumpX * (float)mapSizeX);
    int mapY = (int)round(in.bumpY * (float)mapSizeY);
    float retVal = (float)(bumpMap[mapY*mapSizeX + mapX]) * (M_PI / (255.0f * 2.0f));
    //printf("%f \n", (float)(bumpMap[mapY*mapSizeX + mapX]));
    return retVal;
    //return sin(mapX)+cos(mapY);
}
vec4 AdjustNormal(Intersection in, vec4 originalNormal, vec4 cameraPos) {
    vec4 rotAxis = normalize(crossVec4(originalNormal,cameraPos));
    float rotValue = FetchMapValue(in) * (float)M_PI / 2.0f;
    vec4 newNormal = originalNormal * cos(rotValue) + (rotAxis * originalNormal) * sin(rotValue) +
                rotAxis * dot(rotAxis, originalNormal) * (1.0f - cos(rotValue));
    return newNormal;
}

vec3 DirectLight(const Intersection& i){
  vec4 n = triangles[i.triangleIndex].normal;
  vec4 r = lightPos - i.position;
  n = AdjustNormal(i, n, r);
  float rLength = length(r);
  float maxval = max(dot(normalize(r), n),0.0f);
  vec3 direct = (lightColor * maxval) / (4.0f * (float)M_PI * rLength * rLength);
  return direct;
}
