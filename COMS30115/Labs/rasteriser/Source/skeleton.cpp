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
mat4 TM;
float yaw = 0;
float pitch = 0;
float roll = 0;
float tX = 0;
float tY = 0;
float tZ = 0;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
bool Update();
void Draw(screen* screen);
void TransformationMatrix();
void VertexShader( const vec4& v, ivec2& p );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices , screen* screen);
void ComputePolygonRows(const vector<ivec2>& vertexPixels,
                              vector <ivec2>& leftPixels,
                              vector <ivec2>& rightPixels);

int main( int argc, char* argv[] )
{
  LoadTestModel(triangles);

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<ivec2> vertexPixels(3);
  vertexPixels[0] = ivec2(10, 5);
  vertexPixels[1] = ivec2( 5,10);
  vertexPixels[2] = ivec2(15,15);
  vector<ivec2> leftPixels;
  vector<ivec2> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  for( uint row=0; row<leftPixels.size(); ++row )
  {
  cout << "Start: ("
  << leftPixels[row].x << ","
  << leftPixels[row].y << "). "
  << "End: ("
  << rightPixels[row].x << ","
  << rightPixels[row].y << "). " << endl;
  }

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
    vertices[0] = TM*triangles[i].v0 - cameraPos;
    vertices[1] = TM*triangles[i].v1 - cameraPos;
    vertices[2] = TM*triangles[i].v2 - cameraPos;

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
        case SDLK_a:
          yaw += 0.2f;
          TransformationMatrix();
          /* Move camera right */
          break;
        case SDLK_d:
          yaw -= 0.2f;
          TransformationMatrix();
          /* Move camera right */
          break;
        case SDLK_w:
          pitch += 0.2f;
          TransformationMatrix();
          /* Move camera right */
          break;
        case SDLK_s:
          pitch -= 0.2f;
          TransformationMatrix();
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
void ComputePolygonRows(const vector<ivec2>& vertexPixels,
                              vector <ivec2>& leftPixels,
                              vector <ivec2>& rightPixels) {
  // Compute number of rows required
  int minY =  numeric_limits<int>::max();
  int maxY = -numeric_limits<int>::max();
  for (uint i = 0; i < vertexPixels.size(); ++i) {
    if (vertexPixels[i].y > maxY) maxY = vertexPixels[i].y;
    if (vertexPixels[i].y < minY) minY = vertexPixels[i].y;
  }
  // Resize vectors to approporiate number of rows
  uint numRows = maxY - minY + 1;
  leftPixels = vector<ivec2>(numRows);
  rightPixels = vector<ivec2>(numRows);
  // Init left and rightPixels vectors
  for( uint i = 0; i<numRows; ++i )
  {
    leftPixels[i].x = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
  }
  for (uint i = 0; i < vertexPixels.size(); ++i) {
    // Take all edges of the polygon
    ivec2 firstVertex = vertexPixels[i];
    ivec2 secondVertex = vertexPixels[(i+1) % vertexPixels.size()];
    // Interpolate 1 result for each row
    int edgeHeight = abs(firstVertex.y - secondVertex.y) + 1;
    int startingY = firstVertex.y;
    int direction;
    if (firstVertex.y < secondVertex.y) direction = 1;
    else  direction = -1;
    vector<ivec2> interpolationResults = vector<ivec2>(edgeHeight);
    Interpolate(firstVertex, secondVertex, interpolationResults);

    for (int j = 0; j < edgeHeight; j++) {
      int currentY = startingY + (j * direction) - minY;
      if (interpolationResults[j].x < leftPixels[currentY].x) {
        leftPixels[currentY].x = interpolationResults[j].x;
        leftPixels[currentY].y = interpolationResults[j].y;
      }
      if (interpolationResults[j].x > rightPixels[currentY].x) {
        rightPixels[currentY].x = interpolationResults[j].x;
        rightPixels[currentY].y = interpolationResults[j].y;
      }
    }
  }
}

void TransformationMatrix()
{
  //rotX = pitch, rotY = yaw, rotZ = roll
  TM = mat4(cos(yaw)*cos(roll),                                      cos(yaw)*sin(roll),                                    -sin(yaw),           0,
            sin(pitch)*sin(yaw)*cos(roll) - cos(pitch)*sin(roll),    sin(pitch)*sin(yaw)*sin(roll) + cos(pitch)*cos(roll),  sin(pitch)*cos(yaw), 0,
            cos(pitch)*sin(yaw)*cos(roll) + sin(pitch)*sin(roll),    cos(pitch)*sin(yaw)*sin(roll) - sin(pitch)*cos(roll),  cos(pitch)*cos(yaw), 0,
            0,                                                       0,                                                     0,                   1);
  //cameraPos = posCamera * rotation * negCamera * worldPoint;
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
