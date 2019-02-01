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

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void Interpolate( float a, float b, vector<float>& result);
void Interpolate( vec3 a, vec3 b, vector<vec3>& result);


int main( int argc, char* argv[] )
{

  /*testing liner interpolation
  vector<float> result(10);
  Interpolate(5, 14, result);
  for(int i = 0; i < result.size(); ++i){
    std::cout << result[i] << " ";
  }
  end of first test*/

  /*testing vector interpolation
  vector<vec3> result( 4 );
  vec3 a(1,4,9.2);
  vec3 b(4,1,9.8);
  Interpolate( a, b, result );
  for( int i=0; i<result.size(); ++i )
  {
    cout << "( "
    << result[i].x << ", "
    << result[i].y << ", "
    << result[i].z << " ) ";
  }
  end of second test*/

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/

  while( NoQuitMessageSDL() )
    {
      Draw(screen);
      Update();
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
  vec3 topLeft(1,0,0); // red
  vec3 topRight(0,0,1); // blue
  vec3 bottomRight(0,1,0); // green
  vec3 bottomLeft(1,1,0); // yellow

  vector<vec3> leftSide( SCREEN_HEIGHT );
  vector<vec3> rightSide( SCREEN_HEIGHT );
  vector<vec3> rowColour( SCREEN_WIDTH );
  Interpolate( topLeft, bottomLeft, leftSide );
  Interpolate( topRight, bottomRight, rightSide );

  for(int y = 0; y < screen->height; y++){
    Interpolate( leftSide[y], rightSide[y], rowColour);
    for(int x = 0; x < screen->width; x++){
        PutPixelSDL(screen, x, y, rowColour[x]);
    }
  }

  /*for(int i=0; i<1000; i++)
    {
      uint32_t x = rand() % screen->width;
      uint32_t y = rand() % screen->height;
      PutPixelSDL(screen, x, y, colour);
    }*/
}

/*Place updates of parameters here*/
void Update()
{
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  //float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  //std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
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

  for(int i = 0; i < (int)result.size(); i++){
    result[i].x = a.x + i*step0;
    result[i].y = a.y + i*step1;
    result[i].z = a.z + i*step2;
  }
}
