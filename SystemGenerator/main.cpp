//
//  main.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

extern "C" {
#include <cstdio>
#include "chipmunk.h"
#include "GL/glew.h"
#include "GL/glfw.h"
#include "ChipmunkDebugDraw.h"

#include "ChipmunkDemoTextSupport.h"
}

#include "MachinePart.h"
#include "MachineSystem.h"
#include "AdeolaBadAlgorithm.h"

static cpBool paused = cpFalse;
static cpBool step = cpFalse;

// for time step
static double LastTime = 0.0;


void createGroundBox(cpSpace *space)
{
    
    // place the world edges - i put a hill in the middle
    
    int ww, wh;
	glfwGetWindowSize(&ww, &wh);
    
    //bottom1
    cpBody *groundBody = cpBodyNewStatic();
    cpShape *groundShape = cpSegmentShapeNew(groundBody, cpv(-ww/2, -wh/2), cpv(0, -wh/3), 0.01f);
    //   cpShapeSetFriction(groundShape, 0.3);
    cpSpaceAddStaticShape(space, groundShape);
    
    //bottom2
    groundShape = cpSegmentShapeNew(groundBody, cpv(ww/2, -wh/2), cpv(0, -wh/3), 0.01f);
    // cpShapeSetFriction(groundShape, 0.3);
    cpSpaceAddStaticShape(space, groundShape);
    
    //left
    groundShape = cpSegmentShapeNew(groundBody, cpv(-ww/2, -wh/2), cpv(-ww/2, wh/2), 0.01f);
    cpShapeSetElasticity(groundShape, 1.0);
    cpShapeSetFriction(groundShape, 0.0);
    
    cpSpaceAddStaticShape(space, groundShape);
    
    //right
    groundShape = cpSegmentShapeNew(groundBody, cpv(ww/2, -wh/2), cpv(ww/2, wh/2), 0.01f);
    cpShapeSetElasticity(groundShape, 1.0);
    cpSpaceAddStaticShape(space, groundShape);
    
    
}

cpSpace *setupSpace()
{
    cpSpace *space = cpSpaceNew();
	cpSpaceSetIterations(space, 5);
    // allow bodies to 'sleep' so that we have less CPU use when nothing is moving
   // cpSpaceSetSleepTimeThreshold(space, 0.5);
	cpSpaceSetGravity(space, cpv(0, 0));
    
    
    // createGroundBox(space);
    // insertTestMachines(space);
    
    return space;
}

static void
DrawInstructions()
{
	ChipmunkDemoTextDrawString(cpv(-300, 220),
                               "Controls:\n"
                               "` - pause/resume simulation\n"
                               "1 - step once (while paused)\n"
                               "x - generate new system\n"
                               "m - mutate current system\n"
                               "< - rotate input body counterclockwise\n"
                               "> - rotate input body clockwise\n"
                               );
}

static void DrawStats(float inputAngle, float outputAngle)
{
    static char output[127];
    sprintf(output, "Input angle: %.3f, output angle: %.3f", inputAngle, outputAngle);
    ChipmunkDemoTextDrawString(cpv(-300, -220), output);
}

static void
Reshape(int width, int height)
{
	glViewport(0, 0, width, height);
	
	float scale = (float)cpfmin(width/640.0, height/480.0);
	float hw = width*(0.5f/scale);
	float hh = height*(0.5f/scale);
	
	ChipmunkDebugDrawPointLineScale = scale;
	glLineWidth((GLfloat)scale);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-hw, hw, -hh, hh);
}

static int
WindowClose()
{
	glfwTerminate();
	exit(EXIT_SUCCESS);
	
	return GL_TRUE;
}


//void refreshWall(cpSpace *space, void *key, void *data)
//{
//    if (sys)
//        delete sys;
//    
//    sys = new MachineSystem(200, 200, 8, 8, cpvzero, space);
//    
//    randomGenerator1(sys);
//
//}
//
//void mutateWall(cpSpace *space, void *key, void *data)
//{
//    if (sys) {
//        MachineSystem *newSys = attachmentMutator1(sys);
//        delete sys;
//        sys = newSys;
//    }
//}

static void
Keyboard(int key, int state)
{
	if(state == GLFW_RELEASE) return;
	
//	int index = key - 'a'
	
    if(key == '`'){
		paused = !paused;
    } else if(key == '1'){
		step = cpTrue;
	} else if(key == '\\'){
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_POINT_SMOOTH);
//    } else if (key == 'x') {
//        if (!cpSpaceIsLocked(worldSpace))
//            refreshWall(worldSpace, sys, NULL);
//        else
//            cpSpaceAddPostStepCallback(worldSpace, refreshWall, sys, NULL);
//    } else if (key == 'm') {
//        if (!cpSpaceIsLocked(worldSpace))
//            mutateWall(worldSpace, sys, NULL);
//        else
//            cpSpaceAddPostStepCallback(worldSpace, mutateWall, sys, NULL);
//    } else if (key == '<' || key == '>') {
//        MachinePart *inputPart = sys->partAtPosition(sys->inputMachinePosition);
//        cpBody *inputBody = inputPart->body;
//        cpFloat angVel = cpBodyGetAngVel(inputBody);
//        if (key == '<')
//            angVel += M_PI/3; // 60 degree increment
//        else
//            angVel -= M_PI/3;
//
//        cpBodySetAngVel(inputBody, angVel);
    }
}

void
SetupGL(void)
{
	glewExperimental = GL_TRUE;
	cpAssertHard(glewInit() == GLEW_NO_ERROR, "There was an error initializing GLEW.");
	cpAssertHard(GLEW_ARB_vertex_array_object, "Requires VAO support.");
	
	ChipmunkDebugDrawInit();
    ChipmunkDemoTextInit();
	
	glClearColor(52.0f/255.0f, 62.0f/255.0f, 72.0f/255.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
    
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glHint(GL_POINT_SMOOTH_HINT, GL_DONT_CARE);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
}

void setupGLFW()
{
    
    cpAssertHard(glfwInit(), "Error initializing GLFW.");
    
    cpAssertHard(glfwOpenWindow(640, 480, 8, 8, 8, 8, 0, 0, GLFW_WINDOW), "Error opening GLFW window.");
    glfwSwapInterval(1);
    
    SetupGL();
    
    glfwSetWindowSizeCallback(Reshape);
    glfwSetWindowCloseCallback(WindowClose);
    
    
    glfwSetCharCallback(Keyboard);
}


void updateWorld(cpSpace *space, MachineSystem *sys, cpVect translation)
{
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    //	glScalef((GLfloat)scale, (GLfloat)scale, 1.0f);
    
    glTranslatef((GLfloat)translation.x, (GLfloat)translation.y, 0.0f);
    
    ChipmunkDebugDrawClearRenderer();
    
    ChipmunkDebugDrawPushRenderer();
    
    ChipmunkDebugDrawShapes(space);
    ChipmunkDebugDrawConstraints(space);

    
    // outline input and output bodies
    cpBody *body = sys->partAtPosition(sys->inputMachinePosition)->body;
    cpFloat currentInputAngle = cpBodyGetAngle(body);
    __block cpShape *shape = NULL;
    cpBodyEachShape_b(body, ^(cpShape *s) {
        shape = s;
    });
    if (shape) {
        //input in blue
        ChipmunkDebugDrawShape(shape, RGBAColor(0.0f, 0.0f, 1.0f, 1.0f), LAColor(0.0f, 0.0f));
    }
    
    body = sys->partAtPosition(sys->outputMachinePosition)->body;
    cpFloat currentOutputAngle = cpBodyGetAngle(body);

    shape = NULL;
    cpBodyEachShape_b(body, ^(cpShape *s) {
        shape = s;
    });
    if (shape) {
        //output in red
        ChipmunkDebugDrawShape(shape, RGBAColor(1.0f, 0.0f, 0.0f, 1.0f), LAColor(0.0f, 0.0f));
    }
    
	// Draw the renderer contents and reset it back to the last tick's state.
	ChipmunkDebugDrawFlushRenderer();
	ChipmunkDebugDrawPopRenderer();
		
    ChipmunkDemoTextPushRenderer();
	DrawInstructions();
     DrawStats(currentInputAngle, currentOutputAngle);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix(); {
		// Draw the text at fixed positions,
		// but save the drawing matrix for the mouse picking
		glLoadIdentity();
		
		ChipmunkDemoTextFlushRenderer();
		ChipmunkDemoTextPopRenderer();
	} glPopMatrix();
    
	glfwSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
    
    
}

int main(int argc, char **argv)
{
    setupGLFW();
    
    AdeolaAlgorithm *a = new AdeolaAlgorithm(5);

    while(1) {
        a->tick();
        double now = glfwGetTime();
        if (now - LastTime > 0.5) {
            LastTime = now;
            MachineSystem *best = a->bestSystem();
            
            updateWorld(best->getSpace(), best, cpvzero);
        }
    }
    
    return 0;
}