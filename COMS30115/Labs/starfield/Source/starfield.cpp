#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false


/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
int t;
vector<vec3> stars(1000);

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void Interpolate( float a, float b, vector<float>& result);
void Interpolate( vec3 a, vec3 b, vector<vec3>& result);


int main( int argc, char* argv[] )
{

  /*randomise star positions*/
  for(size_t i = 0; i < stars.size(); i++){
    float posRandX = float(rand()) / float(RAND_MAX);
    float negRandX = (float(rand()) / float(RAND_MAX));
    float randX = posRandX - negRandX;
    float posRandY = float(rand()) / float(RAND_MAX);
    float negRandY = (float(rand()) / float(RAND_MAX));
    float randY = posRandY - negRandY;
    float randZ = float(rand()) / float(RAND_MAX);

    stars[i].x = randX;
    stars[i].y = randY;
    stars[i].z = randZ;
  }

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/

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

  float u, v;
  float f = screen->height / 2;
  vec3 colour(1.0,1.0,1.0);

  for(size_t s = 0; s<stars.size(); ++s){
    vec3 color = 0.2f * vec3(1,1,1) / (stars[s].z*stars[s].z);
    u = f * (stars[s].x / stars[s].z) + (screen->width / 2);
    v = f * (stars[s].y / stars[s].z) + (screen->height / 2);
    PutPixelSDL(screen, u, v, color);
  }

      // uint32_t x = rand() % screen->width;
      // uint32_t y = rand() % screen->height;
      // PutPixelSDL(screen, x, y, colour);
}

/*Place updates of parameters here*/
void Update()
{
  //static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  float velocity = 0.0001;

  for(size_t s = 0; s < stars.size(); ++s){
    /*update z position of stars*/
    stars[s].z -= velocity*dt;

    if(stars[s].z <= 0){
      stars[s].z += 1;
    }
    if(stars[s].z > 1){
      stars[s].z -= 1;
    }
  }

}

void Interpolate( float a, float b, vector<float>& result){
  int counter = 0;
  float step = (b - a)/(result.size()-1);

   for (float i = a; i < b+1; i += step){
     result[counter] = i;
     counter++;
   }
}

void Interpolate( vec3 a, vec3 b, vector<vec3>& result){
  //int count = 0;
  float step0 = (b.x - a.x) / (result.size()-1);
  float step1 = (b.y - a.y) / (result.size()-1);
  float step2 = (b.z - a.z) / (result.size()-1);

  for(size_t i = 0; i < result.size(); i++){
    result[i].x = a.x + i*step0;
    result[i].y = a.y + i*step1;
    result[i].z = a.z + i*step2;
  }
}
