#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

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

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);

int main( int argc, char* argv[] )
{

  /*fill vector of triangles*/
  LoadTestModel(triangles );

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

  vec3 colour(1.0,0.0,0.0);
  for(int i=0; i<1000; i++)
    {
      uint32_t x = rand() % screen->width;
      uint32_t y = rand() % screen->height;
      PutPixelSDL(screen, x, y, colour);
    }
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

  for(size_t i = 0; i < triangles.size(); i++){
    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;
    vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
    glm::mat3 A( -b, e1, e2 );
    vec3 x = glm::inverse( A ) * b;

    if (x.y > 0 && x.z > 0 && x.x >= 0) {
      closestIntersection.position = x.x * dir;
      closestIntersection.distance = x.x * dir.length();
      closestIntersection.triangleIndex = i;
      return true;
    }
  }
  return false;
}
