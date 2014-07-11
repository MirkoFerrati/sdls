#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <ctype.h>
#include <iostream>
#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glui.h>

#include "Main.h"

// Make slowdown factor larger to make the simulation take larger, less frequent steps
// Make the constant factor in Tstep larger to make time pass more quickly
//const int SlowdownFactor = 40;
const int SlowdownFactor = 10;		// Make higher to take larger steps less frequently
const int SleepsPerStep=SlowdownFactor;
int SleepCounter=0;
const double Tstep = 0.0005*(double)SlowdownFactor;		// Time step
double T = -Tstep;				// Current time
int RestPositionOn;

/*   FOLLOWING BLOCK OF CODE USED FOR MAKING MOVIES
#include "RgbImage.h"
RgbImage theScreenImage;
int DumpCounter = 0;
int DumpCounterStart = 1000;
int DumpCounterEnd = 1600;
*/


FILE *fp;

int main( int argc, char *argv[] )
{
	BuildTreeYShape(nodeY, treeY);
	jacobY = new Jacobian(&treeY);

	BuildTreeDoubleYShape(nodeDoubleY, treeDoubleY);
	jacobDoubleY = new Jacobian(&treeDoubleY);

	BuildTreeDoubleYShape(nodeDoubleYDLS, treeDoubleYDLS);
	jacobDoubleYDLS = new Jacobian(&treeDoubleYDLS);

	BuildTreeDoubleYShape(nodeDoubleYSDLS, treeDoubleYSDLS);
	jacobDoubleYSDLS = new Jacobian(&treeDoubleYSDLS);

	/*
	fp = fopen("./temp.txt", "w");
	fprintf(fp, "X = [\n");
	*/

	glutInit( &argc, argv );
	InitGraphics();
	InitLists();
	Reset();
	InitGlui();
	glutMainLoop();

	/*
	fprintf(fp, "]\n");
	fclose(fp);
	*/

	return 0;
}


// Update target positions

void UpdateTargets( double T ) {
	switch (WhichShape) {
	case YSHAPE:
		target[0]=KDL::Vector(2.0f+1.5*sin(6*T), -0.5+1.7f+0.2*sin(7*T), 0.3f+0.2*sin(8*T));
		target[1]=KDL::Vector(-0.7f+0.4*sin(4*T), -0.5+1.3f+0.3*sin(4*T), -0.2f+0.2*sin(3*T));
		assert( treeY.GetNumEffector() == 2 );
		break;
	case DBLYSHAPE:
                target[0]=KDL::Vector(2.0f+1.5*sin(3*T)*2, -0.5+1.0f+0.2*sin(7*T)*2, 0.3f+0.7*sin(5*T)*2);
                target[1]=KDL::Vector(0.5f+0.4*sin(4*T)*2, -0.5+0.9f+0.3*sin(4*T)*2, -0.2f+1.0*sin(3*T)*2);
                target[2]=KDL::Vector(-0.5f+0.8*sin(6*T)*2, -0.5+1.1f+0.2*sin(7*T)*2, 0.3f+0.5*sin(8*T)*2);
                target[3]=KDL::Vector(-1.6f+0.8*sin(4*T)*2, -0.5+0.8f+0.3*sin(4*T)*2, -0.2f+0.3*sin(3*T)*2);
		assert( treeDoubleY.GetNumEffector() == 4);
		break;
	}
}


// Does a single update (on one kind of tree)
void DoUpdateStep() {
	if ( WhichMethod!=COMPARE ) {
	
		if ( SleepCounter==0 ) {
			T += Tstep;
			UpdateTargets( T );
		} 

		Jacobian *jacob;
		switch ( WhichShape ) {
			case YSHAPE:
				jacob = jacobY;
				break;
			case DBLYSHAPE:
				jacob = jacobDoubleY;
				break;
			default:
				assert ( 0 );
		}

		if ( UseJacobianTargets ) {
			jacob->SetJtargetActive();
		}
		else {
			jacob->SetJendActive();
		}
		jacob->ComputeJacobian();						// Set up Jacobian and deltaS vectors

		// Calculate the change in theta values 
		switch (WhichMethod) {
			case JACOB_TRANS:
				jacob->CalcDeltaThetasTranspose();		// Jacobian transpose method
				break;
			case DLS:
				jacob->CalcDeltaThetasDLS();			// Damped least squares method
				break;
			case PURE_PSEUDO:
				jacob->CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
				break;
			case SDLS:
				jacob->CalcDeltaThetasSDLS();			// Selectively damped least squares method
				break;
			default:
				jacob->ZeroDeltaThetas();
				break;
		}

		if ( SleepCounter==0 ) {
			jacob->UpdateThetas();							// Apply the change in the theta values
			jacob->UpdatedSClampValue();
			SleepCounter = SleepsPerStep;
		}
		else { 
			SleepCounter--;
		}
	}
	else { // COMPARE MODE		(only supports double-Y)
		WhichShape = DBLYSHAPE;
		if ( SleepCounter==0 ) {
			T += Tstep;
			UpdateTargets( T );
		} 
		Jacobian *jacob1 = jacobDoubleYDLS;
		Jacobian *jacob2 = jacobDoubleYSDLS;
		if ( UseJacobianTargets ) {
			jacob1->SetJtargetActive();
			jacob2->SetJtargetActive();
		}
		else {
			jacob1->SetJendActive();
			jacob2->SetJendActive();
		}
		jacob1->ComputeJacobian();						// Set up Jacobian and deltaS vectors
		jacob2->ComputeJacobian();						// Set up Jacobian and deltaS vectors
		jacob1->CalcDeltaThetasDLS();
		jacob2->CalcDeltaThetasSDLS();
		if ( SleepCounter==0 ) {
			jacob1->UpdateThetas();							// Apply the change in the theta values
			jacob1->UpdatedSClampValue();					// Not needed for usual DLS method
			jacob2->UpdateThetas();							// Apply the change in the theta values
			jacob2->UpdatedSClampValue();
			SleepCounter = SleepsPerStep;
		}
		else { 
			SleepCounter--;
		}
	}


}

#include "createY.c"
#include "tests.c"
#include "draw.c"