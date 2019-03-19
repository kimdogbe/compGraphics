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

struct Pixel
{
  int x;
  int y;
  float zinv;
};

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
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
bool Update();
void Draw(screen* screen);
void TransformationMatrix();
void VertexShader( const vec4& v, Pixel& p );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void Interpolate(Pixel a, Pixel b, vector<Pixel> &result);
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices , screen* screen);
void ComputePolygonRows(const vector<Pixel>& vertexPixels,
                              vector <Pixel>& leftPixels,
                              vector <Pixel>& rightPixels);
void DrawRows(screen* screen,
              const vector<Pixel>& leftPixels,
              const vector<Pixel>& rightPixels,
              vec3 colour );
void DrawPolygon(screen* screen,  const vector<vec4>& vertices, vec3 colour );

/* ----------------------------------------------------------------------------*/
/* FUNCTION IMPLEMENTED                                                                   */
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
    vertices[0] = TM*triangles[i].v0 - cameraPos;
    vertices[1] = TM*triangles[i].v1 - cameraPos;
    vertices[2] = TM*triangles[i].v2 - cameraPos;

    //DrawPolygonEdges(vertices, screen);
    DrawPolygon(screen, vertices, triangles[i].color);

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
  for( int y=0; y<SCREEN_HEIGHT; ++y ){
    for( int x=0; x<SCREEN_WIDTH; ++x ){
      depthBuffer[y][x] = 0;
    }
  }

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
void ComputePolygonRows(const vector<Pixel>& vertexPixels,
                              vector <Pixel>& leftPixels,
                              vector <Pixel>& rightPixels) {
  // Compute number of rows required
  int minY =  numeric_limits<int>::max();
  int maxY = -numeric_limits<int>::max();
  for (uint i = 0; i < vertexPixels.size(); ++i) {
    if (vertexPixels[i].y > maxY) maxY = vertexPixels[i].y;
    if (vertexPixels[i].y < minY) minY = vertexPixels[i].y;
  }
  // Resize vectors to approporiate number of rows
  uint numRows = maxY - minY + 1;
  leftPixels = vector<Pixel>(numRows);
  rightPixels = vector<Pixel>(numRows);
  // Init left and rightPixels vectors
  for( uint i = 0; i<numRows; ++i )
  {
    leftPixels[i].x = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
  }
  for (uint i = 0; i < vertexPixels.size(); ++i) {
    // Take all edges of the polygon
    Pixel firstVertex = vertexPixels[i];
    Pixel secondVertex = vertexPixels[(i+1) % vertexPixels.size()];
    // Interpolate 1 result for each row
    int edgeHeight = abs(firstVertex.y - secondVertex.y) + 1;
    int startingY = firstVertex.y;
    int direction;
    if (firstVertex.y < secondVertex.y) direction = 1;
    else  direction = -1;
    vector<Pixel> interpolationResults = vector<Pixel>(edgeHeight);
    Interpolate(firstVertex, secondVertex, interpolationResults);

    for (int j = 0; j < edgeHeight; j++) {
      int currentY = startingY + (j * direction) - minY;
      if (interpolationResults[j].x < leftPixels[currentY].x) {
        leftPixels[currentY].x = interpolationResults[j].x;
        leftPixels[currentY].y = interpolationResults[j].y;
        leftPixels[currentY].zinv = interpolationResults[j].zinv;
      }
      if (interpolationResults[j].x > rightPixels[currentY].x) {
        rightPixels[currentY].x = interpolationResults[j].x;
        rightPixels[currentY].y = interpolationResults[j].y;
        rightPixels[currentY].zinv = interpolationResults[j].zinv;
      }
    }
  }
}

void DrawPolygon(screen* screen,  const vector<vec4>& vertices, vec3 colour )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i )
    VertexShader( vertices[i], vertexPixels[i] );
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawRows(screen, leftPixels, rightPixels, colour );
}

void DrawRows(screen* screen,
              const vector<Pixel>& leftPixels,
              const vector<Pixel>& rightPixels,
              vec3 colour ) {

  for (uint i = 0; i < leftPixels.size(); ++i) {
    int resultsLength = (rightPixels[i].x - leftPixels[i].x) + 1;
    std::vector<Pixel> currentRow(resultsLength);
    Interpolate(leftPixels[i], rightPixels[i], currentRow);
    for (int j = 0; j < resultsLength; ++j) {
      if(currentRow[j].zinv  >= depthBuffer[currentRow[j].x][currentRow[j].y] - 0.0002){
          PutPixelSDL(screen, currentRow[j].x, currentRow[j].y, colour);
          depthBuffer[currentRow[j].x][currentRow[j].y] = currentRow[j].zinv;
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

void VertexShader( const vec4& v, Pixel& p )
{
  p.zinv = 1 / v.z;
  p.x = focalLength * (v.x / v.z) + (SCREEN_WIDTH/2);
  p.y = focalLength * (v.y / v.z) + (SCREEN_HEIGHT/2);

  if(p.zinv > depthBuffer[p.y][p.x]){
    depthBuffer[p.y][p.x] = p.zinv;
  }
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

void Interpolate(Pixel a, Pixel b, vector<Pixel> &result){
  int N = result.size();
  float stepx = ( b.x - a.x ) / float(max(N-1, 1));
  float stepy = ( b.y - a.y ) / float(max(N-1, 1));
  float stepz = ( (1 / b.zinv) - (1 / a.zinv) ) / float(max(N-1, 1));
  float currentx = a.x;
  float currenty = a.y;
  float currentz = 1 / a.zinv;

  for(int i=0; i<N; ++i){
    result[i].x = currentx;
    currentx += stepx;
    result[i].y = currenty;
    currenty+= stepy;
    result[i].zinv = 1 / currentz;
    currentz += stepz;
  }
}

void DrawLineSDL( screen* screen, Pixel a, Pixel b, vec3 color )
{
  // Pixel delta = glm::abs(a - b);
  int deltax = a.x - b.x;
  int deltay = a.y = b.y;
  int pixels = glm::max( deltax, deltay) + 1;
  std::vector<Pixel> line(pixels);
  Interpolate( a , b , line);

  for(uint32_t i = 0; i < line.size(); ++i){
    PutPixelSDL( screen, line[i].x, line[i].y, color);
  }
}

void DrawPolygonEdges( const vector<vec4>& vertices , screen* screen)
{
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<Pixel> projectedVertices( V );
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
