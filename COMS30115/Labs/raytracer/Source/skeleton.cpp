#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "limits.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

float m = std::numeric_limits<float>::max();

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

struct Intersection
{
vec4 position;
float distance;
int triangleIndex;
};

vector<Triangle> triangles;
float focalLength = SCREEN_WIDTH/2;
vec4 cameraPos(0, 0, -2, focalLength);

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles,
   Intersection& closestIntersection );

int main( int argc, char* argv[] )
{

  /*fill vector with triangles from testModel*/
  LoadTestModel(triangles);

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
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

  for(int y = 0; y < screen->height; y++){
    for(int x = 0; x < screen->width; x++){
      vec4 start(x, y, 0, 1.0);
      vec4 rayDir(x - screen->width/2, y - screen->height/2, focalLength, 1.0);

      if(ClosestIntersection(cameraPos, rayDir, triangles, closestIntersection)){
        colour = triangles[closestIntersection.triangleIndex].color;
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

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

// Start position of a ray, direction of the ray, triangles to check, return value if true
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles,
   Intersection& closestIntersection ){

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
    float currentDistance = sqrt( currentPoint.x*currentPoint.x +
                                  currentPoint.y*currentPoint.y +
                                  currentPoint.z*currentPoint.z);

    if (x.y > 0 && x.z > 0 && x.y + x.z < 1 && x.x >= 0) {
      if(closestIntersection.distance > currentDistance){
        closestIntersection.position = currentPoint;
        //closestIntersection.distance = x.x * dir.length();
        /*might need refining by substracting start from it*/
        closestIntersection.distance = currentDistance;
        closestIntersection.triangleIndex = i;
        inTriangle = true;
      }
    }
  }
  return inTriangle;
}
