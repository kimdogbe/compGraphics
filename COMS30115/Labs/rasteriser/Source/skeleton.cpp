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
using glm::ivec2;
using glm::mat4x4;

SDL_Event event;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
vector<Triangle> triangles;
float focalLength = SCREEN_HEIGHT/2;
vec4 cameraPos(0, 0, -3.001, 1);
mat4 R;
float yaw = 0;
/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
bool Update();
void Draw(screen* screen);
void TransformationMatrix(mat4x4 posCamera, mat4x4 rotation,
                          mat4x4 negCamera, vec4 worldPoint);
void VertexShader( const vec4& v, ivec2& p );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices , screen* screen);

int main( int argc, char* argv[] )
{
  LoadTestModel(triangles);

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while ( Update())
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

  for( uint32_t i=0; i<triangles.size(); ++i )
  {
    vector<vec4> vertices(3);
    vertices[0] = triangles[i].v0 - cameraPos;
    vertices[1] = triangles[i].v1 - cameraPos;
    vertices[2] = triangles[i].v2 - cameraPos;

    DrawPolygonEdges(vertices, screen);

    // for(int v=0; v<3; ++v)
    // {
    //   ivec2 projPos;
    //   VertexShader( vertices[v], projPos );
    //
    //   vec3 color(1,1,1);
    //   PutPixelSDL( screen, projPos.x, projPos.y, color );
    // }
  }
}

/*Place updates of parameters here*/
bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  SDL_Event e;
  while(SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT)
	{
	  return false;
	}
      else
	if (e.type == SDL_KEYDOWN)
	  {
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
	      {
	      case SDLK_UP:
        cameraPos.z += 0.2f;
		/* Move camera forward */
		break;
	      case SDLK_DOWN:
        cameraPos.z -= 0.2f;
		/* Move camera backwards */
		break;
	      case SDLK_LEFT:
        cameraPos.x -= 0.2f;
		/* Move camera left */
		break;
	      case SDLK_RIGHT:
        cameraPos.x += 0.2f;
		/* Move camera right */
		break;
	      case SDLK_ESCAPE:
		/* Move camera quit */
		return false;
	      }
	  }
    }
  return true;
}

void TransformationMatrix(mat4x4 posCamera, mat4x4 rotation, mat4x4 negCamera, vec4 worldPoint)
{
  cameraPos = posCamera * rotation * negCamera * worldPoint;
}

void VertexShader( const vec4& v, ivec2& p )
{
  p.x = focalLength * (v.x / v.z) + (SCREEN_WIDTH/2);
  p.y = focalLength * (v.y / v.z) + (SCREEN_HEIGHT/2);
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result )
{
  int N = result.size();
  glm::vec2 step = glm::vec2(b-a) / float(max(N-1,1));
  glm::vec2 current( a );
  for( int i=0; i<N; ++i )
  {
    result[i] = current;
    current += step;
  }
}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color )
{
  ivec2 delta = glm::abs(a - b);
  int pixels = glm::max( delta.x, delta.y) + 1;
  std::vector<ivec2> line(pixels);
  Interpolate( a , b , line);

  for(uint32_t i = 0; i < line.size(); ++i){
    PutPixelSDL( screen, line[i].x, line[i].y, color);
  }
}

void DrawPolygonEdges( const vector<vec4>& vertices , screen* screen)
{
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i )
  {
    VertexShader( vertices[i], projectedVertices[i] );
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i )
  {
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}
