
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "matrix.h"

SDL_Window *gWindow = nullptr;

//OpenGL context
SDL_GLContext gContext;

//Render flag
bool gRenderQuad = true;


const int SCREEN_WIDTH = 1920;
const int SCREEN_HEIGHT = 1080;

bool initGL()
{
    bool success = true;
    GLenum error = GL_NO_ERROR;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1920.0 / 1080.0, 1, 200);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(-3, 0.0, -3, 0, 0, 0, 0, -1, 0);

    return success;
}


void init()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glColor3f(1.0, 1.0, 1.0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 4.0 / 3.0, 1, 40);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, -2, 1.25, 0, 0, 0, 0, 1, 0);
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

void updateCamera()
{
    static float x = -3.0f;
    x += 0.1f;
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(x, 0.0, -3, 0, 0, 0, 0, -1, 0);
}

void captureInput()
{
    SDL_Event event{};
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_QUIT:
            exit(0);
            break;
        case SDL_TEXTEDITING:
            printf("text edit: %s %d %d\n", event.edit.text, event.edit.start, event.edit.length);
            break;
        case SDL_TEXTINPUT:
            printf("text input: %s\n", event.text.text);
            break;
        default:
            printf("default %d\n", event.type);
        }
    }
}

int main()
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

    while (true)
    {
        // process events
        bool should_exit = false;

        captureInput(); 

        // Clear color buffer
        glClearColor(0.5f, 0.5f, 0.5f, 0.5f);
        glClear(GL_COLOR_BUFFER_BIT);

        updateCamera();

        drawAxis();

        // drawMesh(triangles, vertices);

        SDL_GL_SwapWindow(gWindow);

        SDL_Delay(33);
    }
}