//
//  main.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#include <stdio.h>
#include "chipmunk.h"
#include "GL/glew.h"
#include "GL/glfw.h"
#include "ChipmunkDebugDraw.h"

#include "ChipmunkDemoTextSupport.h"

#import "Machine.h"
#import "MachineWall.h"
#import "Algorithm.h"

cpSpace *worldSpace;
static cpBool paused = cpFalse;
static cpBool step = cpFalse;

// mouse input
cpVect ChipmunkDemoMouse;
static cpBody *mouse_body = NULL;
static cpConstraint *mouse_joint = NULL;

// for time step
static double Accumulator = 0.0;
static double LastTime = 0.0;


MachineWall *wall = NULL;


static void refreshWall()
{
    if (wall) {
        mgMachineWallFree(wall);
    }
    wall = mgMachineWallNew(200, 200, 8, 8, cpv((-200-10)/2, (-200-10)/2), worldSpace);
    randomGenerator1(wall);
}

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
    cpSpaceSetSleepTimeThreshold(space, 0.5);
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
                               "Use the mouse to grab objects.\n"
                               );
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
    cpSpaceFree(worldSpace);
	exit(EXIT_SUCCESS);
	
	return GL_TRUE;
}



static void
Keyboard(int key, int state)
{
	if(state == GLFW_RELEASE) return;
	
	int index = key - 'a';
	
	
     if(key == '`'){
		paused = !paused;
    } else if(key == '1'){
		step = cpTrue;
	} else if(key == '\\'){
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_POINT_SMOOTH);
    } else if (key == 'x') {
        if (cpSpaceIsLocked(worldSpace)) {
            cpSpaceAddPostStepCallback(worldSpace, refreshWall, "refreshWall", NULL);
        } else {
            refreshWall();
        }
    }
	
//	GLfloat translate_increment = 50.0f/(GLfloat)scale;
//	GLfloat scale_increment = 1.2f;
//	if(key == '5'){
//		translate.x = 0.0f;
//		translate.y = 0.0f;
//		scale = 1.0f;
//	}else if(key == '4'){
//		translate.x += translate_increment;
//	}else if(key == '6'){
//		translate.x -= translate_increment;
//	}else if(key == '2'){
//		translate.y += translate_increment;
//	}else if(key == '8'){
//		translate.y -= translate_increment;
//	}else if(key == '7'){
//		scale /= scale_increment;
//	}else if(key == '9'){
//		scale *= scale_increment;
//	}
}

static cpVect
MouseToSpace(int x, int y)
{
	GLdouble model[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model);
	
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX, proj);
 	
	GLint view[4];
	glGetIntegerv(GL_VIEWPORT, view);
	
	int ww, wh;
	glfwGetWindowSize(&ww, &wh);
	
	GLdouble mx, my, mz;
	gluUnProject(x, wh - y, 0.0f, model, proj, view, &mx, &my, &mz);
	
	return cpv(mx, my);
}

static void
Mouse(int x, int y)
{
	ChipmunkDemoMouse = MouseToSpace(x, y);
}

static void
Click(int button, int state)
{
	if(button == GLFW_MOUSE_BUTTON_1){
		if(state == GLFW_PRESS){
			cpShape *shape = cpSpacePointQueryFirst(worldSpace, ChipmunkDemoMouse, CP_ALL_LAYERS, CP_NO_GROUP);
			if(shape){
				cpBody *body = shape->body;
				mouse_joint = cpPivotJointNew2(mouse_body, body, cpvzero, cpBodyWorld2Local(body, ChipmunkDemoMouse));
				mouse_joint->maxForce = 50000.0f;
				mouse_joint->errorBias = cpfpow(1.0f - 0.15f, 60.0f);
				cpSpaceAddConstraint(worldSpace, mouse_joint);
			}
		} else if(mouse_joint){
			cpSpaceRemoveConstraint(worldSpace, mouse_joint);
			cpConstraintFree(mouse_joint);
			mouse_joint = NULL;
		}
	} else if(button == GLFW_MOUSE_BUTTON_2){
	//	ChipmunkDemoRightDown = ChipmunkDemoRightClick = (state == GLFW_PRESS);
	}
}

static void
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
   //     glfwSetKeyCallback(SpecialKeyboard);
        
        glfwSetMousePosCallback(Mouse);
        glfwSetMouseButtonCallback(Click);
}

static void
eachShape(cpShape *shape, void *unused)
{
    cpSpace *s = cpShapeGetSpace(shape);
    if (s) {
        
    } else {
        
    }
}

static void
Tick(double dt)
{
	if(!paused || step){
				
		// Completely reset the renderer only at the beginning of a tick.
		// That way it can always display at least the last ticks' debug drawing.
		ChipmunkDebugDrawClearRenderer();
		
		cpVect new_point = cpvlerp(mouse_body->p, ChipmunkDemoMouse, 0.25f);
		mouse_body->v = cpvmult(cpvsub(new_point, mouse_body->p), 60.0f);
		mouse_body->p = new_point;
		
		// update bodies
		cpSpaceStep(worldSpace, dt);
      //  cpSpaceEachShape(worldSpace, eachShape, NULL);
  //      cpSpaceEachBody(worldSpace, &eachBody, NULL);
        
//		ChipmunkDemoTicks++;
//		ChipmunkDemoTime += dt;
		
		step = cpFalse;
		
	}
}

static void
Update(void)
{
	double time = glfwGetTime();
	double dt = time - LastTime;
	if(dt > 0.2) dt = 0.2;
	
	double fixed_dt = 1.0/60;
	
	for(Accumulator += dt; Accumulator > fixed_dt; Accumulator -= fixed_dt){
		Tick(fixed_dt);
	}
	
	LastTime = time;
}

static void Display(void)
{
    ChipmunkDebugDrawShapes(worldSpace);
    ChipmunkDebugDrawConstraints(worldSpace);
}

void updateWorld()
{
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
//	glTranslatef((GLfloat)translate.x, (GLfloat)translate.y, 0.0f);
//	glScalef((GLfloat)scale, (GLfloat)scale, 1.0f);

    
	Update();    
	
	ChipmunkDebugDrawPushRenderer();
    Display();
    
	// Highlight the shape under the mouse because it looks neat.
	cpShape *nearest = cpSpaceNearestPointQueryNearest(worldSpace, ChipmunkDemoMouse, 0.0f, CP_ALL_LAYERS, CP_NO_GROUP, NULL);
	if(nearest) ChipmunkDebugDrawShape(nearest, RGBAColor(1.0f, 0.0f, 0.0f, 1.0f), LAColor(0.0f, 0.0f));
	
	// Draw the renderer contents and reset it back to the last tick's state.
	ChipmunkDebugDrawFlushRenderer();
	ChipmunkDebugDrawPopRenderer();
	
    ChipmunkDemoTextPushRenderer();
	// Now render all the UI text.
	DrawInstructions();
	
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
    worldSpace = setupSpace();

    mouse_body = cpBodyNew(INFINITY, INFINITY);
    
    refreshWall();
    
    while(1) {
        updateWorld();
    }
    
    // should technically free al

    
    return 0;
}