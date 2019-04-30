#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <stdio.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;
using glm::mat4x4;

SDL_Event event;

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 512
#define FULLSCREEN_MODE false

struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
};
struct Vertex {
  vec4 position;
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

vec4 lightPos(0, -0.5f, 2.5f, 1);
vec3 lightPower = 10.1f * vec3(1, 1, 1);
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );

vec4 currentNormal;
float currentReflectance;

vector< unsigned int > vertexIndices, uvIndices, normalIndices;
vector<vec4> temp_vertices;
vector<glm::vec2> temp_uvs;
vector<vec4> temp_normals;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
bool Update();
void Draw(screen* screen);
void TransformationMatrix();
void VertexShader( const Vertex& v, Pixel& p );
void PixelShader( screen* screen, const Pixel& p, vec3 pixelOriginalColour );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void Interpolate(Pixel a, Pixel b, vector<Pixel> &result);
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<Vertex>& vertices , screen* screen);
void ComputePolygonRows(const vector<Pixel>& vertexPixels,
                              vector <Pixel>& leftPixels,
                              vector <Pixel>& rightPixels);
void DrawRows(screen* screen,
              const vector<Pixel>& leftPixels,
              const vector<Pixel>& rightPixels,
              vec3 colour );
void DrawPolygon(screen* screen,  const vector<Vertex>& vertices, vec3 colour );
bool loadOBJ(
    const char * path,
    vector <vec3> & out_vertices,
    vector <glm::vec2> & out_uvs,
    vector <vec3> & out_normals
);

/* ----------------------------------------------------------------------------*/
/* FUNCTION IMPLEMENTED                                                                   */
int main( int argc, char* argv[] )
{
  LoadTestModel(triangles);
  vector<vec3> vertices;
  vector<glm::vec2> uvs;
  vector<vec3> normals;
  bool res = loadOBJ("sphere.obj", vertices, uvs, normals);
  if(!res) printf("Problem loading obj file\n");

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
    vector<Vertex> vertices(3);

    currentNormal = triangles[i].normal;
    currentReflectance = 1.f;

    vertices[0].position = TM*triangles[i].v0 - cameraPos;
    vertices[1].position = TM*triangles[i].v1 - cameraPos;
    vertices[2].position = TM*triangles[i].v2 - cameraPos;

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
          /* Rotate clockwise around y-axis */
          yaw += 0.2f;
          TransformationMatrix();
          /*move light left*/
          // lightPos.x -= 0.2f;
          break;
        case SDLK_d:
          /* Rotate anticlockwise around y-axis */
          yaw -= 0.2f;
          TransformationMatrix();
          /*move light right*/
          // lightPos.x += 0.2f;
          break;
        case SDLK_w:
          /* Move camera right */
          pitch += 0.2f;
          TransformationMatrix();
          /*move light down*/
          // lightPos.y -= 0.2f;
          break;
        case SDLK_s:
          /* Move camera right */
          pitch -= 0.2f;
          TransformationMatrix();
          /*move light up*/
          // lightPos.y += 0.2f;
          break;
        case SDLK_e:
          /*move light away*/
          lightPos.z += 0.2f;
          break;
        case SDLK_q:
          /*move light closer*/
          lightPos.z -= 0.2f;
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
  // 1. Find max and min y-value of the polygon
  //and compute the number of rows it occupies.

  // Compute number of rows required
  int minY =  numeric_limits<int>::max();
  int maxY = -numeric_limits<int>::max();

  for (uint i = 0; i < vertexPixels.size(); ++i) {
    if (vertexPixels[i].y > maxY) maxY = vertexPixels[i].y;
    if (vertexPixels[i].y < minY) minY = vertexPixels[i].y;
  }

  // 2. Resize leftPixels and rightPixels
  //so that they have an element for each row.

  // Resize vectors to approporiate number of rows
  uint numRows = maxY - minY + 1;
  leftPixels = vector<Pixel>(numRows);
  rightPixels = vector<Pixel>(numRows);

  // 3. Initialize the x-coordinates in leftPixels
  //to some really large value and the x-coordinates
  //in rightPixels to some really small value.

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
        leftPixels[currentY].pos3d = interpolationResults[j].pos3d;
        //printf("%f\n", leftPixels[currentY].pos3d.y);
      }
      if (interpolationResults[j].x > rightPixels[currentY].x) {
        rightPixels[currentY].x = interpolationResults[j].x;
        rightPixels[currentY].y = interpolationResults[j].y;
        rightPixels[currentY].zinv = interpolationResults[j].zinv;
        rightPixels[currentY].pos3d = interpolationResults[j].pos3d;
      }
    }
  }
}

void DrawPolygon(screen* screen,  const vector<Vertex>& vertices, vec3 colour )
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
        PixelShader(screen, currentRow[j], colour);
    }
  }
}

void PixelShader(screen* screen, const Pixel& p, vec3 pixelOriginalColour )
{
  int x = p.x;
  int y = p.y;

  vec4 n = currentNormal;
  vec4 r = lightPos - p.pos3d;
  float rLength = length(r);
  float maxval = max(dot(r, n), 0.0f);
  vec3 direct = (lightPower * maxval) / (4.0f * (float)M_PI * rLength * rLength);
  vec3 total = currentReflectance * (direct + indirectLightPowerPerArea);

  if( p.zinv > depthBuffer[y][x] ) {
      depthBuffer[y][x] = p.zinv;
      PutPixelSDL( screen, x, y, (total * pixelOriginalColour) );
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

void VertexShader( const Vertex &v, Pixel& p )
{
  p.zinv = 1 / v.position.z;
  p.x = (focalLength * v.position.x * p.zinv) + (SCREEN_WIDTH/2);
  p.y = (focalLength * v.position.y * p.zinv) + (SCREEN_HEIGHT/2);
  p.pos3d = v.position;

  // vec4 n = v.normal;
  // vec4 r = lightPos - v.position;
  // float rLength = length(r);
  // float maxval = max(dot(r, n),0.0f);
  // vec3 direct = (lightPower * maxval) / (4.0f * (float)M_PI * rLength * rLength);
  // vec3 total = v.reflectance * (direct + indirectLightPowerPerArea);
  // p.illumination = total;

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
  vec4 stepIlllumination = (b.pos3d - a.pos3d) / float(max(N-1, 1));
  float currentx = a.x;
  float currenty = a.y;
  float currentz = 1 / a.zinv;
  vec4 currentPosition = a.pos3d;

  for(int i=0; i<N; ++i){
    result[i].x = currentx;
    currentx += stepx;
    result[i].y = currenty;
    currenty+= stepy;
    result[i].zinv = 1 / currentz;
    currentz += stepz;
    result[i].pos3d = currentPosition;
    currentPosition += stepIlllumination;
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

void DrawPolygonEdges( const vector<Vertex>& vertices , screen* screen)
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

bool loadOBJ(const char * path, vector <vec3> & out_vertices, vector <glm::vec2> & out_uvs,
              vector <vec3> & out_normals)
{
  printf("%s\n", path);
  FILE * file = fopen(path, "r");
  if( file == NULL ){
      printf("Impossible to open the file !\n");
      return false;
  }

  while( 1 ){

    char lineHeader[128];
    // read the first word of the line
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF)
        break; // EOF = End Of File. Quit the loop.

    // else : parse lineHeader

    if ( strcmp( lineHeader, "v" ) == 0 ){
      glm::vec4 vertex;
      fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z );
      temp_vertices.push_back(vertex);
    // }else if ( strcmp( lineHeader, "vt" ) == 0 ){
    //   glm::vec2 uv;
    //   fscanf(file, "%f %f\n", &uv.x, &uv.y );
    //   temp_uvs.push_back(uv);
    }else if ( strcmp( lineHeader, "vn" ) == 0 ){
      glm::vec4 normal;
      fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z );
      temp_normals.push_back(normal);
    }else if ( strcmp( lineHeader, "f" ) == 0 ){
      std::string vertex1, vertex2, vertex3;
      unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
      int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
      if (matches != 9){
          printf("File can't be read by our simple parser : ( Try exporting with other options\n");
          return false;
      }
      Triangle currentTriangle = Triangle(temp_vertices[vertexIndex[0]-1]/-6.0f, (temp_vertices[vertexIndex[1]-1]/-6.0f), temp_vertices[vertexIndex[2]-1]/-6.0f, vec3(1.0f, 0.0f, 1.0f ));
      currentTriangle.v0.z -= 1;
      currentTriangle.v1.z -= 1;
      currentTriangle.v2.z -= 1;
      vec3 e1 = vec3(temp_vertices[vertexIndex[1]-1].x-temp_vertices[vertexIndex[0]-1].x,
                     temp_vertices[vertexIndex[1]-1].y-temp_vertices[vertexIndex[0]-1].y,
                     temp_vertices[vertexIndex[1]-1].z-temp_vertices[vertexIndex[0]-1].z);
      vec3 e2 = vec3(temp_vertices[vertexIndex[2]-1].x-temp_vertices[vertexIndex[0]-1].x,
                     temp_vertices[vertexIndex[2]-1].y-temp_vertices[vertexIndex[0]-1].y,
                     temp_vertices[vertexIndex[2]-1].z-temp_vertices[vertexIndex[0]-1].z);
      vec3 normal3 = glm::normalize( glm::cross(e2, e1));
      currentTriangle.normal.x = normal3.x;
      currentTriangle.normal.y = normal3.y;
      currentTriangle.normal.z = normal3.z;
      currentTriangle.normal.w = 1.0f;

      // currentTriangle.normal = temp_normals[normalIndex[0]]/ 3.0f + temp_normals[normalIndex[1]]/ 3.0f + temp_normals[normalIndex[2]] / 3.0f;
      triangles.push_back(currentTriangle);
      // vertexIndices.push_back(vertexIndex[0]);
      // vertexIndices.push_back(vertexIndex[1]);
      // vertexIndices.push_back(vertexIndex[2]);
      // uvIndices    .push_back(uvIndex[0]);
      // uvIndices    .push_back(uvIndex[1]);
      // uvIndices    .push_back(uvIndex[2]);
      // normalIndices.push_back(normalIndex[0]);
      // normalIndices.push_back(normalIndex[1]);
      // normalIndices.push_back(normalIndex[2]);
    }
  }

  // For each vertex of each triangle
  // for( unsigned int i=0; i<vertexIndices.size(); i++ ){
  //   unsigned int vertexIndex = vertexIndices[i];
  //   glm::vec3 vertex = temp_vertices[ vertexIndex-1 ];
  //   out_vertices.push_back(vertex);
  // }

  return true;
}
