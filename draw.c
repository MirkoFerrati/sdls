
void Animate( void )
{
    glutSetWindow( GrWindow );
    glutPostRedisplay();
}

void Buttons( int id )
{
    switch( id ) {
        case RESET:
            Reset();
            break;
        case QUIT:
            Glui->close();
            glFinish();
            glutDestroyWindow( GrWindow );
            exit( 0 );
        case RUNTEST:
            RunTest();
            break;
    }
    Glui->sync_live();
}

void DrawTarget(double T)
{
    GLfloat target_ambient_and_diffuse[] = { 1.0f, 0.0f, 0.0f, 1.0f };
    GLfloat mat_ambient_and_diffuse[] = { 0.2f, 0.2f, 0.8f, 1.0f };
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, target_ambient_and_diffuse);
    
    int numTarget = (WhichShape==YSHAPE) ? 2 : 4;
    
    for (int i=0; i<numTarget; i++) {
        glPushMatrix();
        glTranslatef(target[i].x(), target[i].y(), target[i].z());
        glutSolidSphere(0.1, 10, 10);
        glPopMatrix();
    }
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_ambient_and_diffuse);
}

int numIteration = 1;
double error = 0.0;
double errorDLS = 0.0;
double errorSDLS = 0.0;
double sumError = 0.0;
double sumErrorDLS = 0.0;
double sumErrorSDLS = 0.0;
int numWinOverall = 0;
int numWinAt0 = 0;
int numWinAt1 = 0;
int numWinAt2 = 0;
int numWinAt3 = 0;
int numWinAt4 = 0;

#ifdef _DYNAMIC
bool initMaxDist = true;
extern double Excess[];
extern double dsnorm[];
#endif

void Display( void )
{
    
    DoUpdateStep();
    
    float scale2;           /* real glui scale factor               */
    
    glutSetWindow( GrWindow );
    
    glDrawBuffer( GL_BACK );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    
    glShadeModel( GL_FLAT );
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    
    if (WhichProjection == ORTHO)
        glOrtho(-3., 3.,     -1.7, 1.7,     0.1, 1000.);
    //glOrtho(-3., 3.,     -3., 3.,     0.1, 1000.);
    else
        gluPerspective(75., 1., 0.1, 1000.);
    // gluPerspective(90., 1.,      0.1, 1000.);
    
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    gluLookAt( -3., 0., 3.,     0., 0., 0.,     0., 1., 0. );
    
    glTranslatef( TransXYZ[0], TransXYZ[1], -TransXYZ[2] );
    
    glRotatef( Yrot, 0., 1., 0. );
    glRotatef( Xrot, 1., 0., 0. );
    glMultMatrixf( (const GLfloat *) RotMatrix );
    
    glScalef( Scale, Scale, Scale );
    scale2 = 1. + Scale2;           /* because glui translation starts at 0. */
    if( scale2 < MINSCALE )
        scale2 = MINSCALE;
    glScalef( scale2, scale2, scale2 );
    
    if( AxesOn ) {
        glDisable(GL_LIGHTING);
        glCallList( AxesList );
        glEnable(GL_LIGHTING);
    }
    
    GLUI_Master.set_glutIdleFunc( Animate );
    
    DrawTarget(T);
    
    if (WhichMethod != COMPARE) {
        Tree* tree = ( WhichShape==YSHAPE ) ? &treeY : &treeDoubleY;
        tree->Draw();
    } else {
        GLfloat blue[] = { 0.2f, 0.2f, 0.8f, 1.0f };
        GLfloat green[] = { 0.3f, 0.6f, 0.3f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, green);
        treeDoubleYDLS.Draw();
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, blue);
        treeDoubleYSDLS.Draw();
    }
    
    if (EigenVectorsOn) {
        if ( WhichMethod == SDLS || WhichMethod == DLS || WhichMethod == PURE_PSEUDO ) {
            Jacobian* jacob;
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
            jacob->DrawEigenVectors();
        }
    }
    
    glFlush();
    
    /*   FOLLOWING BLOCK OF CODE USED FOR MAKING MOVIES
     *        if ( WhichMethod == JACOB_TRANS && WhichShape==DBLYSHAPE && (SleepCounter%SleepsPerStep)==0 ) { 
     *                if (DumpCounter==0) {
     *                        T = 0.0;                // Set time back to zero
     } 
     if ( DumpCounter >= DumpCounterStart && DumpCounter<DumpCounterEnd ) {
         theScreenImage.LoadFromOpenglBuffer();
         int fileNum = DumpCounter - DumpCounterStart;
         char filename[23];
         sprintf( filename, "JTRANSPOSE/temp%03d.bmp", fileNum );
         theScreenImage.WriteBmpFile( filename );
     }
     DumpCounter++;
     }
     */
    
    glutSwapBuffers();
     }
     
     void InitGlui(void)
     {
         GLUI_Panel *panel;
         GLUI_RadioGroup *group;
         GLUI_Rotation *rot;
         GLUI_Translation *trans, *scale;
         
         Glui = GLUI_Master.create_glui( (char *) GLUITITLE, 0, 0, 0);
         
         Glui->add_statictext( (char *) GLUITITLE );
         Glui->add_separator();
         
         panel = Glui->add_panel("Shape & Motion");
         group = Glui->add_radiogroup_to_panel( panel, &WhichShape, 0 );
         Glui->add_radiobutton_to_group( group, "Y" );
         Glui->add_radiobutton_to_group( group, "Double Y" );
         
         panel = Glui->add_panel("Method");
         group = Glui->add_radiogroup_to_panel( panel, &WhichMethod, 0 );
         Glui->add_radiobutton_to_group( group, "Jacobian Transpose" );
         Glui->add_radiobutton_to_group( group, "Pure Pseudoinverse" );
         Glui->add_radiobutton_to_group( group, "Damped Least Squares" );
         Glui->add_radiobutton_to_group( group, "Selectively Damped Least Squares" );
         Glui->add_radiobutton_to_group( group, "Compare" );
         
         panel = Glui->add_panel("Options");
         //Glui->add_checkbox_to_panel( panel, "Joint Limits", &JointLimitsOn, 0, NULL);
         //Glui->add_checkbox_to_panel( panel, "Rest Position", &RestPositionOn, 0, NULL);
         Glui->add_checkbox_to_panel( panel, "Jacobian Targets", &UseJacobianTargets, 0);
         
         panel = Glui->add_panel("View");
         Glui->add_checkbox_to_panel( panel, "Eigen Vectors", &EigenVectorsOn, 0);
         Glui->add_checkbox_to_panel( panel, "Axes", &AxesOn, 0);
         Glui->add_checkbox_to_panel( panel, "Perspective", &WhichProjection, 0);
         Glui->add_checkbox_to_panel( panel, "Rotation Axes", &RotAxesOn, 0);
         
         panel = Glui->add_panel( "Object Transformation" );
         rot = Glui->add_rotation_to_panel( panel, "Rotation", (float *) RotMatrix, 0);
         rot->set_spin( 1.0 );
         Glui->add_column_to_panel( panel, FALSE );
         scale = Glui->add_translation_to_panel( panel, "Scale",  GLUI_TRANSLATION_Y , &Scale2, 0);
         scale->set_speed( 0.01f );
         Glui->add_column_to_panel( panel, FALSE );
         trans = Glui->add_translation_to_panel( panel, "Trans XY", GLUI_TRANSLATION_XY, &TransXYZ[0], 0);
         trans->set_speed( 0.1f );
         Glui->add_column_to_panel( panel, FALSE );
         trans = Glui->add_translation_to_panel( panel, "Trans Z",  GLUI_TRANSLATION_Z , &TransXYZ[2], 0);
         trans->set_speed( 0.1f );
         
         panel = Glui->add_panel( "", FALSE );
         Glui->add_button_to_panel( panel, "Run Test", RUNTEST, (GLUI_Update_CB) Buttons );
         Glui->add_column_to_panel( panel, FALSE );
         Glui->add_button_to_panel( panel, "Reset", RESET, (GLUI_Update_CB) Buttons );
         Glui->add_button_to_panel( panel, "Quit", QUIT, (GLUI_Update_CB) Buttons );
         Glui->set_main_gfx_window( GrWindow );
         GLUI_Master.set_glutIdleFunc( NULL );
     }
     
     void InitGraphics( void )
     {
         glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
         
         glutInitWindowSize(500, 300);
         glutInitWindowPosition(300, 200);
         
         GrWindow = glutCreateWindow( WINDOWTITLE );
         glutSetWindowTitle( WINDOWTITLE );
         
         glClearColor( BACKCOLOR[0], BACKCOLOR[1], BACKCOLOR[2], BACKCOLOR[3] );
         
         glutSetWindow( GrWindow );
         glutDisplayFunc( Display );
         glutMouseFunc( MouseButton );
         glutMotionFunc( MouseMotion );
         glutKeyboardFunc( Keyboard );
         glutReshapeFunc( resizeWindow );
         
         GLfloat global_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
         
         GLfloat light0_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
         GLfloat light0_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
         GLfloat light0_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
         GLfloat light0_position[] = { 3.0f, 3.0f, 3.0f, 0.0 };
         
         GLfloat light1_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
         GLfloat light1_diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
         GLfloat light1_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
         GLfloat light1_position[] = { -6.0f, 3.0f, 3.0f, 0.0 };
         
         GLfloat mat_ambient_and_diffuse[] = { 0.2f, 0.2f, 0.8f, 1.0f };
         GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
         GLfloat mat_shininess[] = { 15.0f };
         
         // light model
         glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
         glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
         
         // light0
         glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
         glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
         glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
         glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
         
         // light1
         glLightfv(GL_LIGHT1, GL_AMBIENT, light0_ambient);
         glLightfv(GL_LIGHT1, GL_DIFFUSE, light0_diffuse);
         glLightfv(GL_LIGHT1, GL_SPECULAR, light0_specular);
         glLightfv(GL_LIGHT1, GL_POSITION, light0_position);
         
         // material properties
         glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_ambient_and_diffuse);
         glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
         glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
         
         glEnable(GL_LIGHTING);
         glEnable(GL_LIGHT0);
         glEnable(GL_LIGHT1);
     }
     
     void InitLists( void )
     {
         AxesList = glGenLists( 1 );
         glNewList( AxesList, GL_COMPILE );
         glColor3fv( AXES_COLOR );
         glLineWidth( AXES_WIDTH );
         Axes( 1.5 );
         glLineWidth( 1. );
         glEndList();
     }
     
     void resizeWindow ( int w, int h )
     {
         // Define the portion of the window used for OpenGL rendering.
         glViewport( 0, 0, w, h );       // View port uses whole window
         
         
     }
     
     void MouseButton( int button, int state, int x, int y )
     {
         int b;                  /* LEFT, MIDDLE, or RIGHT               */
         
         switch( button ) {
             case GLUT_LEFT_BUTTON:
                 b = LEFT;               break;
             case GLUT_MIDDLE_BUTTON:
                 b = MIDDLE;             break;
             case GLUT_RIGHT_BUTTON:
                 b = RIGHT;              break;
             default:
                 b = 0;
                 std::cerr << "Unknown mouse button: " << button << "\n";
         }
         
         if( state == GLUT_DOWN ) {
             Xmouse = x;
             Ymouse = y;
             ActiveButton |= b;              /* set the proper bit   */
         } else {
             ActiveButton &= ~b;             /* clear the proper bit */
         }
     }
     
     void MouseMotion( int x, int y )
     {
         int dx, dy;             /* change in mouse coordinates          */
         
         dx = x - Xmouse;                /* change in mouse coords       */
         dy = y - Ymouse;
         
         if( ActiveButton & LEFT ) {
             switch( LeftButton ) {
                 case ROTATE:
                     Xrot += ( ANGFACT*dy );
                     Yrot += ( ANGFACT*dx );
                     break;
                 case SCALE:
                     Scale += SCLFACT * (float) ( dx - dy );
                     if( Scale < MINSCALE )
                         Scale = MINSCALE;
                     break;
             }
         }
         
         if( ActiveButton & MIDDLE ) {
             Scale += SCLFACT * (float) ( dx - dy );
             if( Scale < MINSCALE )
                 Scale = MINSCALE;
         }
         
         Xmouse = x;                     /* new current position         */
         Ymouse = y;
         
         glutSetWindow( GrWindow );
         glutPostRedisplay();
     }
     
     void Keyboard(unsigned char c, int x, int y)
     {
         Glui->sync_live();
         glutSetWindow(GrWindow);
         glutPostRedisplay();
     }
     
     void Reset( void )
     {
         ActiveButton = 0;
         AxesOn = false;
         LeftButton = ROTATE;
         Scale  = 1.0;
         Scale2 = 0.0;           /* because add 1. to it in Display()    */
         WhichProjection = ORTHO;
         Xrot = Yrot = 0.;
         TransXYZ[0] = TransXYZ[1] = TransXYZ[2] = 0.;
         WhichShape = YSHAPE;
         WhichMethod = SDLS;
         RotAxesOn = false;
         JointLimitsOn = false;
         RestPositionOn = false;
         UseJacobianTargets = false;
         EigenVectorsOn = false;
         
         RotMatrix[0][1] = RotMatrix[0][2] = RotMatrix[0][3] = 0.;
         RotMatrix[1][0]                   = RotMatrix[1][2] = RotMatrix[1][3] = 0.;
         RotMatrix[2][0] = RotMatrix[2][1]                   = RotMatrix[2][3] = 0.;
         RotMatrix[3][0] = RotMatrix[3][1] = RotMatrix[3][3]                   = 0.;
         RotMatrix[0][0] = RotMatrix[1][1] = RotMatrix[2][2] = RotMatrix[3][3] = 1.;
         
         treeY.Init();
         treeY.Compute();
         jacobY->Reset();
         
         treeDoubleY.Init();
         treeDoubleY.Compute();  
         jacobDoubleY->Reset();
         
         treeDoubleYDLS.Init();
         treeDoubleYDLS.Compute();
         jacobDoubleYDLS->Reset();
         
         treeDoubleYSDLS.Init();
         treeDoubleYSDLS.Compute();
         jacobDoubleYSDLS->Reset();
         
         glutSetWindow( GrWindow );
         glutPostRedisplay();
     }
     