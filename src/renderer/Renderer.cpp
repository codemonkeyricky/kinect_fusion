
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

bool initGL()
{
    bool success = true;
    GLenum error = GL_NO_ERROR;

    // // Initialize Projection Matrix
    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // gluPerspective(60.0, 4.0 / 3.0, 1, 40);
    // assert(GL_NO_ERROR == glGetError());

    // // Initialize Modelview Matrix
    // // glFrustum(-5, 5, -5, 5, 15, 150);
    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
    // gluLookAt(0, 0, 0.5, 0, 0, 0, 0, 1, 0);
    // assert(GL_NO_ERROR == glGetError());

    // // Initialize clear color
    // glClearColor(0.f, 0.f, 0.f, 1.f);
    // assert(GL_NO_ERROR == glGetError());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1920.0 / 1080.0, 1, 200);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, -3.0, 2.0, 0, 0, 0, 0, 0, 1);

    return success;
}

//Screen dimension constants
const int SCREEN_WIDTH = 1920;
const int SCREEN_HEIGHT = 1080;

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

void Renderer::update(std::vector<Eigen::Vector3f> &vertices)
{
    // Clear color buffer
    glClear(GL_COLOR_BUFFER_BIT);

    if (0)
    {

        glColor3f(0, 1, 1);
        glBegin(GL_LINES);
        for (int i = 0; i <= 10; i++)
        {
            // horizontal
            glVertex3f(-50.0f + i * 10.0f, -50.0f, 0.0f);
            glVertex3f(-50.0f + i * 10.0f, 50.0f, 0.0f);

            // vertical
            glVertex3f(-50.0f, -50.0f + i * 10.0f, 0.0f);
            glVertex3f(50.0f, -50.0f + i * 10.0f, 0.0f);
        }
        glEnd();

        glBegin(GL_TRIANGLES);          // Drawing Using Triangles
        glVertex3f(0.0f, 1.0f, 0.0f);   // Top
        glVertex3f(-1.0f, -1.0f, 0.0f); // Bottom Left
        glVertex3f(1.0f, -1.0f, 0.0f);  // Bottom Right
        glEnd();
    }

    // Render quad
    if (0)
    {
        // glRotatef(0.4f, 0.0f, 1.0f, 0.0f); // Rotate The cube around the Y axis
        // glRotatef(0.2f, 1.0f, 1.0f, 1.0f);
        glColor3f(0.0f, 1.0f, 0.0f);

        glBegin(GL_QUADS);
        glVertex2f(-0.5f, -0.5f);
        glVertex2f(0.5f, -0.5f);
        glVertex2f(0.5f, 0.5f);
        glVertex2f(-0.5f, 0.5f);
        glEnd();
    }

    if (1)
    {
        glColor3f(0, 1, 1);
        glBegin(GL_LINES);
        for (int i = 0; i <= 10; i++)
        {
            // horizontal
            glVertex3f(-50.0f + i * 10.0f, -50.0f, 0.0f);
            glVertex3f(-50.0f + i * 10.0f, 50.0f, 0.0f);

            // vertical
            glVertex3f(-50.0f, -50.0f + i * 10.0f, 0.0f);
            glVertex3f(50.0f, -50.0f + i * 10.0f, 0.0f);
        }
        glEnd();


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
                    {
                        glColor3f(0.0f, 1.0f, 0.0f);
                        glBegin(GL_TRIANGLES); // Drawing Using Triangles
                        glVertex3f(vertices[i0][0], vertices[i0][1], vertices[i0][2]);
                        glVertex3f(vertices[i1][0], vertices[i1][1], vertices[i1][2]);
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
                    {
                        glColor3f(1.0f, 0.0f, 0.0f);
                        glBegin(GL_TRIANGLES); // Drawing Using Triangles
                        glVertex3f(vertices[i1][0], vertices[i1][1], vertices[i1][2]);
                        glVertex3f(vertices[i3][0], vertices[i3][1], vertices[i3][2]);
                        glVertex3f(vertices[i2][0], vertices[i2][1], vertices[i2][2]);
                        glEnd();
                    }
                }
            }
        }

    }

    // {
    //     // Create triangles
    //     std::vector<Vector3i> mTriangles;
    //     mTriangles.reserve((depthHeight - 1) * (depthWidth - 1) * 2);
    //     for (unsigned int i = 0; i < depthHeight - 1; i++)
    //     {
    //         for (unsigned int j = 0; j < depthWidth - 1; j++)
    //         {
    //             unsigned int i0 = i * depthWidth + j;
    //             unsigned int i1 = (i + 1) * depthWidth + j;
    //             unsigned int i2 = i * depthWidth + j + 1;
    //             unsigned int i3 = (i + 1) * depthWidth + j + 1;

    //             bool valid0 = mVerticesGlobal->at(i0).allFinite();
    //             bool valid1 = mVerticesGlobal->at(i1).allFinite();
    //             bool valid2 = mVerticesGlobal->at(i2).allFinite();
    //             bool valid3 = mVerticesGlobal->at(i3).allFinite();

    //             if (valid0 && valid1 && valid2)
    //             {
    //                 float d0 = (mVerticesGlobal->at(i0) - mVerticesGlobal->at(i1)).norm();
    //                 // std::cout << d0 << std::endl;
    //                 float d1 = (mVerticesGlobal->at(i0) - mVerticesGlobal->at(i2)).norm();
    //                 // std::cout << d1 << std::endl;
    //                 float d2 = (mVerticesGlobal->at(i1) - mVerticesGlobal->at(i2)).norm();
    //                 // std::cout << d2 << std::endl;
    //                 if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
    //                     mTriangles.emplace_back(Vector3i(i0, i1, i2));
    //             }
    //             if (valid1 && valid2 && valid3)
    //             {
    //                 float d0 = (mVerticesGlobal->at(i3) - mVerticesGlobal->at(i1)).norm();
    //                 float d1 = (mVerticesGlobal->at(i3) - mVerticesGlobal->at(i2)).norm();
    //                 float d2 = (mVerticesGlobal->at(i1) - mVerticesGlobal->at(i2)).norm();
    //                 if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
    //                     mTriangles.emplace_back(Vector3i(i1, i3, i2));
    //             }
    //         }
    //     }
    // }

    // Update screen
    SDL_GL_SwapWindow(gWindow);
}
