
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "Renderer.h"

//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//OpenGL context
SDL_GLContext gContext;

//Render flag
bool gRenderQuad = true;

// Screen dimension constants
const int SCREEN_WIDTH = 2560;
const int SCREEN_HEIGHT = 1440;

float cpx = 0.0f;
float cpy = 0.0f;
float cpz = -1.0f;

float lookat_x = 0.0f;
float lookat_y = 0.0f;
float lookat_z = 0.0f;

bool initGL()
{
    bool success = true;
    GLenum error = GL_NO_ERROR;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, SCREEN_WIDTH * 1.0f / SCREEN_HEIGHT, 1, 200);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(cpx, cpy, cpz, lookat_x, lookat_y, lookat_z, 0, -1, 0);

    return success;
}


Renderer::Renderer()
{
    // https://stackoverflow.com/questions/62035106/how-to-change-the-view-perspective-in-opengl

    SDL_Init(SDL_INIT_VIDEO);

    // Use OpenGL 2.1
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    int success;

    // Create window
    gWindow = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (gWindow == NULL)
    {
        printf("Window could not be created! SDL Error: %s\n", SDL_GetError());
        success = false;
    }
    else
    {
        // Create context
        gContext = SDL_GL_CreateContext(gWindow);
        if (gContext == NULL)
        {
            printf("OpenGL context could not be created! SDL Error: %s\n", SDL_GetError());
            success = false;
        }
        else
        {
            // Use Vsync
            if (SDL_GL_SetSwapInterval(1) < 0)
            {
                printf("Warning: Unable to set VSync! SDL Error: %s\n", SDL_GetError());
            }

            // Initialize OpenGL
            if (!initGL())
            {
                printf("Unable to initialize OpenGL!\n");
                success = false;
            }
        }
    }
}

static void drawAxis()
{
    glBegin(GL_LINES);

    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);

    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);

    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);

    glEnd();
}

static bool isBlack(const char *colorMap, int k)
{
    return !colorMap[k * 4 + 0] && !colorMap[k * 4 + 1] && !colorMap[k * 4 + 2];
}

void Renderer::update(std::vector<Eigen::Vector3f> &vertices, const char *colorMap)
{
    // Clear color buffer
    glClear(GL_COLOR_BUFFER_BIT);

    drawAxis();

    int depthHeight = 480;
    int depthWidth = 640;
    float edgeThreshold = 0.02f;

    for (auto i = 0; i < depthHeight - 1; i++)
    {
        for (auto j = 0; j < depthWidth - 1; j++)
        {
            unsigned int i0 = i * depthWidth + j;
            unsigned int i1 = (i + 1) * depthWidth + j;
            unsigned int i2 = i * depthWidth + j + 1;
            unsigned int i3 = (i + 1) * depthWidth + j + 1;

            bool valid0 = vertices.at(i0).allFinite();
            bool valid1 = vertices.at(i1).allFinite();
            bool valid2 = vertices.at(i2).allFinite();
            bool valid3 = vertices.at(i3).allFinite();

            if (valid0 && valid1 && valid2)
            {
                float d0 = (vertices.at(i0) - vertices.at(i1)).norm();
                // std::cout << d0 << std::endl;
                float d1 = (vertices.at(i0) - vertices.at(i2)).norm();
                // std::cout << d1 << std::endl;
                float d2 = (vertices.at(i1) - vertices.at(i2)).norm();
                // std::cout << d2 << std::endl;
                if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
                    // if (!isBlack(colorMap, i0) && !isBlack(colorMap, i0) && !isBlack(colorMap, i0))
                    {
                        glBegin(GL_TRIANGLES); // Drawing Using Triangles

                        glColor3ub(colorMap[i0 * 4 + 0], colorMap[i0 * 4 + 1], colorMap[i0 * 4 + 2]);
                        glVertex3f(vertices[i0][0], vertices[i0][1], vertices[i0][2]);

                        glColor3ub(colorMap[i1 * 4 + 0], colorMap[i1 * 4 + 1], colorMap[i1 * 4 + 2]);
                        glVertex3f(vertices[i1][0], vertices[i1][1], vertices[i1][2]);

                        glColor3ub(colorMap[i2 * 4 + 0], colorMap[i2 * 4 + 1], colorMap[i2 * 4 + 2]);
                        glVertex3f(vertices[i2][0], vertices[i2][1], vertices[i2][2]);

                        glEnd();
                    }
            }
            if (valid1 && valid2 && valid3)
            {
                float d0 = (vertices.at(i3) - vertices.at(i1)).norm();
                float d1 = (vertices.at(i3) - vertices.at(i2)).norm();
                float d2 = (vertices.at(i1) - vertices.at(i2)).norm();
                if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
                    // if (!isBlack(colorMap, i1) && !isBlack(colorMap, i3) && !isBlack(colorMap, i2))
                    {
                        glBegin(GL_TRIANGLES); // Drawing Using Triangles

                        glColor3b(colorMap[i1 * 4 + 0], colorMap[i1 * 4 + 1], colorMap[i1 * 4 + 2]);
                        glVertex3f(vertices[i1][0], vertices[i1][1], vertices[i1][2]);

                        glColor3b(colorMap[i3 * 4 + 0], colorMap[i3 * 4 + 1], colorMap[i3 * 4 + 2]);
                        glVertex3f(vertices[i3][0], vertices[i3][1], vertices[i3][2]);

                        glColor3b(colorMap[i2 * 4 + 0], colorMap[i2 * 4 + 1], colorMap[i2 * 4 + 2]);
                        glVertex3f(vertices[i2][0], vertices[i2][1], vertices[i2][2]);

                        glEnd();
                }
            }
        }
    }

    // Update screen
    SDL_GL_SwapWindow(gWindow);
}

void drawMesh(std::vector<Triangle> &triangles, std::vector<Vertex> &vertices)
{
    glBegin(GL_TRIANGLES); // Drawing Using Triangles
    for (auto &triangle : triangles)
    {
        auto i0 = triangle.idx0;
        auto i1 = triangle.idx1;
        auto i2 = triangle.idx2;

        if (vertices[i0].position.allFinite() && vertices[i1].position.allFinite() && vertices[i2].position.allFinite())
        {
            // glColor3ub(250, 250, 250);
            glColor3ub(vertices[i0].color.x(), vertices[i0].color.y(), vertices[i0].color.z());
            glVertex3f(vertices[i0].position.x(), vertices[i0].position.y(), vertices[i0].position.z());

            // glColor3ub(250, 250, 250);
            glColor3ub(vertices[i1].color.x(), vertices[i1].color.y(), vertices[i1].color.z());
            glVertex3f(vertices[i1].position.x(), vertices[i1].position.y(), vertices[i1].position.z());

            // glColor3ub(250, 250, 250);
            glColor3ub(vertices[i2].color.x(), vertices[i2].color.y(), vertices[i2].color.z());
            glVertex3f(vertices[i2].position.x(), vertices[i2].position.y(), vertices[i2].position.z());

            // printf("color = %d\n", vertices[i2].color.x());
        }
    }
    glEnd();
}

// __attribute__((optimize("O0")))
void drawBoundingBox(Vector3f &minpt, Vector3f &maxpt, bool drawGrid)
{
    glBegin(GL_LINES);

    float nx = minpt[0];
    float ny = minpt[1];
    float nz = minpt[2];
    float px = maxpt[0];
    float py = maxpt[1];
    float pz = maxpt[2];

    glColor3f(0, 1, 1);

    // top

    glVertex3f(px, py, pz);
    glVertex3f(px, ny, pz);

    glVertex3f(px, ny, pz);
    glVertex3f(nx, ny, pz);

    glVertex3f(nx, ny, pz);
    glVertex3f(nx, py, pz);

    glVertex3f(nx, py, pz);
    glVertex3f(px, py, pz);

    // bottom

    glVertex3f(px, py, nz);
    glVertex3f(px, ny, nz);

    glVertex3f(px, ny, nz);
    glVertex3f(nx, ny, nz);

    glVertex3f(nx, ny, nz);
    glVertex3f(nx, py, nz);

    glVertex3f(nx, py, nz);
    glVertex3f(px, py, nz);

    // connectors

    glVertex3f(px, py, pz);
    glVertex3f(px, py, nz);

    glVertex3f(px, ny, pz);
    glVertex3f(px, ny, nz);

    glVertex3f(nx, ny, pz);
    glVertex3f(nx, ny, nz);

    glVertex3f(nx, py, pz);
    glVertex3f(nx, py, nz);

    if (drawGrid)
    {
        // assume 128
        float ddx = (px - nx) / 128;
        float ddy = (py - ny) / 128;
        float ddz = (pz - nz) / 128;

        for (auto dx = nx; dx <= px; dx += ddx)
            for (auto dy = ny; dy <= py; dy += ddy)
                glVertex3f(dx, dy, pz),
                    glVertex3f(dx, dy, nz);
    }

    glEnd();
}

// __attribute__((optimize("O0")))
static void drawVoxel(Volume &volume)
{
    glPointSize(5.0f); 
    glBegin(GL_POINTS);

    for (auto x = 0; x < 256; ++x)
        for (auto y = 0; y < 256; ++y)
            for (auto z = 0; z < 256; ++z)
            {
                auto world = volume.voxelToWorld({x, y, z, 0});
                auto v = volume.get({x, y, z, 0});
                if (volume.getTSDF({x, y, z, 0}) != std::numeric_limits<float>::max())
                    if (-1.0f < volume.getTSDF({x, y, z, 0}) && volume.getTSDF({x, y, z, 0}) < 1.0f)
                    {
                        if (volume.getTSDF({x, y, z, 0}) > 0)
                            glColor3f(0, volume.getTSDF({x, y, z, 0}), 0);
                        else
                            glColor3f(volume.getTSDF({x, y, z, 0}) * -1.0f, 0, 0);

                        glVertex3f(world[0], world[1], world[2]);
                    }
            }
    glEnd();
}

int run = true;
int halt = false;
int renderMode = 0;
int viewMode = 0;
int writeMesh = 0;

static void inputHandling()
{
    SDL_Event event = {};
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_KEYDOWN:
            // render control
            if (event.key.keysym.sym == SDLK_p)
                run = false;
            else if (event.key.keysym.sym == SDLK_c)
                run = true, halt = false;
            else if (event.key.keysym.sym == SDLK_n)
                run = halt = true;
            // render mode control
            else if (event.key.keysym.sym == SDLK_r)
                renderMode = (renderMode + 1) % 3;
            // movement control
            else if (event.key.keysym.sym == SDLK_a)
                cpx -= 0.1f, lookat_x -= 0.1f;
            else if (event.key.keysym.sym == SDLK_d)
                cpx += 0.1f, lookat_x += 0.1f;
            else if (event.key.keysym.sym == SDLK_w)
                cpz += 0.1f, lookat_z += 0.1f;
            else if (event.key.keysym.sym == SDLK_s)
                cpz -= 0.1f, lookat_z -= 0.1f;
            // reset viewport
            else if (event.key.keysym.sym == SDLK_v)
            {
                viewMode = (viewMode + 1) % 3;
                lookat_x = lookat_y = lookat_z = 0;
                cpx = cpy = cpz = 0;
                if (viewMode == 0)
                    cpz = -1;
                else if (viewMode == 1)
                    cpy = -2;
                else if (viewMode == 2)
                    cpx = -1;
            }
            else if (event.key.keysym.sym == SDLK_m)
                writeMesh = !writeMesh;
            // else if (event.key.keysym.sym == SDLK_LEFT)
            //     cpx -= 0.1f;
            // else if (event.key.keysym.sym == SDLK_RIGHT)
            //     cpx += 0.1f;

            else if (event.key.keysym.sym == SDLK_ESCAPE)
                exit(0);
            break;
        case SDL_KEYUP:
        default:
            break;
        }
    }
}

void drawCameraTrajectory(std::vector<vector4f> &camPos)
{
    glPointSize(10.0f);
    glBegin(GL_POINTS);

    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    for (auto &p : camPos)
        glVertex3f(p[0], p[1], p[2]);

    glEnd();
}

void Renderer::update(std::vector<Triangle> &triangles, std::vector<Vertex> &vertices, Vector3f &minpt, Vector3f &maxpt,
                      Volume &volume,
                      std::vector<vector4f> &camPos)
{
    inputHandling();

    // Clear color buffer
    glClearColor(0.5f, 0.5f, 0.5f, 0.5f);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(cpx, cpy, cpz, lookat_x, lookat_y, lookat_z, 0, -1, 0);

    drawAxis();

    drawBoundingBox(minpt, maxpt, 0);

    if (renderMode == 0)
        drawMesh(triangles, vertices);
    else if (renderMode == 1)
        drawVoxel(volume);
    else
        ;

    drawCameraTrajectory(camPos);

    SDL_GL_SwapWindow(gWindow);
}
