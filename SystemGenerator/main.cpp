
//
//  main.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#define CP_ALLOW_PRIVATE_ACCESS 1

extern "C" {
    
#include "chipmunk.h"
#include "GL/glew.h"
#include "GL/glfw.h"
#include "ChipmunkDebugDraw.h"

#include "ChipmunkDemoTextSupport.h"
}

#include <cstdio>
#include <wordexp.h>
#include <sstream>
#include "MachinePart.h"
#include "MachineSystem.h"
#include "AlgorithmList.h"

static cpBool paused = cpFalse;
static cpBool step = cpFalse;

static cpBool restartAlgorithm = cpFalse;
static cpBool terminateAlgorithm = cpFalse;
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
                               "r - restart EA\n"
                               "` - pause\n"
                               "s - step once\n"
                               "x - stop at current generation\n"
                               );
}

static void DrawStats(char *inputStr, char *outputStr)
{
    static char output[500];
    sprintf(output, "%s\t%s", inputStr, outputStr);
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


static void
Keyboard(int key, int state)
{
	if(state == GLFW_RELEASE) return;
	
//	int index = key - 'a'
	
    if(key == '`'){
		paused = !paused;
    } else if(key == 's'){
		step = cpTrue;
	} else if(key == '\\'){
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_POINT_SMOOTH);
    } else if(key == 'r') {
        restartAlgorithm = true;
    } else if (key == 'x') {
        terminateAlgorithm = true;
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

void run_space(cpSpace *space, cpFloat timePassed) {
    const float dt = 0.1;
    cpFloat timeLeft;
    for (timeLeft = timePassed; timeLeft > dt; timeLeft -= dt) {
        cpSpaceStep(space, dt);
    }
    
    if (timeLeft > 0) {
        cpSpaceStep(space, timeLeft);
    }
}

void updateWorld(cpSpace *space, MachineSystem *sys, cpVect translation, cpFloat timeStep, char *inputDescription = NULL, char *outputDescription = NULL)
{
    glEnable (GL_DEPTH_TEST);   // Enables Depth Testing
    glDepthFunc(GL_GREATER);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    //	glScalef((GLfloat)scale, (GLfloat)scale, 1.0f);
    
    glTranslatef((GLfloat)translation.x, (GLfloat)translation.y, 0.0f);
    
    ChipmunkDebugDrawClearRenderer();
    
    ChipmunkDebugDrawPushRenderer();
    
    if (timeStep > 0)
        run_space(space, timeStep);
    
    ChipmunkDebugDrawShapes(space);
    ChipmunkDebugDrawConstraints(space);

    
    // outline input and output bodies
    cpBody *body = sys->partAtPosition(sys->inputMachinePosition)->body;
    __block cpShape *shape = NULL;
    cpBodyEachShape_b(body, ^(cpShape *s) {
        shape = s;
    });
    if (shape) {
        //input in blue
        ChipmunkDebugDrawShape(shape, RGBAColor(0.0f, 0.0f, 1.0f, 1.0f), LAColor(0.0f, 0.0f));
    }
    
    body = sys->partAtPosition(sys->outputMachinePosition)->body;

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
    
    if (inputDescription && outputDescription)
        DrawStats(inputDescription, outputDescription);
	
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
    time_t now = time(NULL);
    int run_number = 1;
    wordexp_t directory;
    wordexp("~/temp/machines/", &directory, 0);
    
    setupGLFW();
    while (1) {
        restartAlgorithm = false;
      // AdeolaRotationAlgorithm *a = new AdeolaRotationAlgorithm(50, 100, 150);
     // MarkAlgorithm *a = new MarkAlgorithm(5, 1000, 15);

       // AdeolaDisplacementAlgorithm *a = new AdeolaDisplacementAlgorithm(5, 1000, 15);
     //   AdeolaConstantToSinusoidalAlgorithm *a = new AdeolaConstantToSinusoidalAlgorithm(5, 1000, 150);
        
        // sorry about the name, this is actually the rotation algorithm
        //NEATDisplacementToX *a = new NEATDisplacementToX(50, 1000, 150);
        NEATSpatialSinRotation *a = new NEATSpatialSinRotation(100, 300, 50);
        MachineSystem *best = NULL;//s;
        
        paused = false;
        terminateAlgorithm = false;
        
        while(!restartAlgorithm && !terminateAlgorithm) {
            double now = glfwGetTime();
            if ((!paused && now - LastTime > 0.1) || (paused && step)) { // slow your roll...
                step = false;
                if (a->tick())
                    break;
                LastTime = now;
                best = a->bestSystem();
            }
            if (best)
                updateWorld(best->getSpace(), best, cpvzero, 0.0);
        }
        LastTime = glfwGetTime();
        cpConstraint *drivingMotor = NULL;
        if (!restartAlgorithm) {
            best = a->bestSystem();
            fprintf(stderr, "Found best system after %ld generations!\n", a->getNumberOfIterations());
            
            std::stringstream s;
          
            s << directory.we_wordv[0];
            s << "best" << now << "-" << run_number++ << ".machine";
            std::string filename = s.str();
            
            best->saveToDisk(filename.c_str()); // expands the tilde
            
            cpBody *inputBody = best->partAtPosition(best->inputMachinePosition)->body;
            cpBody *staticBody = cpSpaceGetStaticBody(best->getSpace());
            
            // a motor will drive the angVel of a body even when there are other forces acting on it
            drivingMotor = cpSimpleMotorNew(staticBody, inputBody, M_PI_2);
            cpSpaceAddConstraint(best->getSpace(), drivingMotor);
            cpConstraintSetMaxForce(drivingMotor, 80000);
        }
        if (!restartAlgorithm)
            a->debug();
        while(!restartAlgorithm) {
            double now = glfwGetTime();
            //sinusoidal motor!
            updateWorld(best->getSpace(), best, cpvzero, now-LastTime, a->inputDescription(), a->outputDescription());
            LastTime = now;
        }
        
        if (drivingMotor) {
            cpSpaceRemoveConstraint(best->getSpace(), drivingMotor);
            drivingMotor = NULL;
        }
        delete a;
        a = NULL;
    }
    return 0;
}
