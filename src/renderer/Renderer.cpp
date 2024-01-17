
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

    // Initialize Projection Matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Check for error
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        success = false;
    }

    // Initialize Modelview Matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Check for error
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        success = false;
    }

    // Initialize clear color
    glClearColor(0.f, 0.f, 0.f, 1.f);

    // Check for error
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        success = false;
    }

    return success;
}

//Screen dimension constants
const int SCREEN_WIDTH = 1920;
const int SCREEN_HEIGHT = 1080;

Renderer::Renderer()
{
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

void Renderer::update()
{
    // Clear color buffer
    glClear(GL_COLOR_BUFFER_BIT);

    // Render quad
    if (1)
    {
        glRotatef(0.4f, 0.0f, 1.0f, 0.0f); // Rotate The cube around the Y axis
        glRotatef(0.2f, 1.0f, 1.0f, 1.0f);
        glColor3f(0.0f, 1.0f, 0.0f);

        glBegin(GL_QUADS);
        glVertex2f(-0.5f, -0.5f);
        glVertex2f(0.5f, -0.5f);
        glVertex2f(0.5f, 0.5f);
        glVertex2f(-0.5f, 0.5f);
        glEnd();
    }

    // Update screen
    SDL_GL_SwapWindow(gWindow);
}
