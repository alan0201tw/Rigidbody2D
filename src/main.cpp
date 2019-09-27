#include <iostream>

#include <GL/freeglut.h>

void Keyboard(unsigned char key, int x, int y)
{

}

void Mouse( int button, int state, int x, int y )
{

}

// void PhysicsLoop( void )
// {
//     glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
// }

class TMP
{
public:
    static void PhysicsLoop( void )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        //glDisable(GL_CULL_FACE);

        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();
        gluLookAt(
        0, 0, 0,
		0, 0, -1,
		0, 1, 0);

        glPushMatrix();
        glTranslated(0, 0, -70);
        glutSolidTeapot(120.0);
        // glutWireTeapot(100.0);
        glPopMatrix();

        glutSwapBuffers( );
    }
};

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
    glutInitWindowSize( 600, 600 );
    glutCreateWindow( "PhyEngine" );
    glutDisplayFunc( TMP::PhysicsLoop );
    glutKeyboardFunc( Keyboard );
    glutMouseFunc( Mouse );
    //glutIdleFunc( TMP::PhysicsLoop );

    // glViewport(0, 0, 600, 600);

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    // gluOrtho2D sets up a two-dimensional orthographic viewing region.
    // This is equivalent to calling glOrtho with near = -1 and far = 1 .
    // gluOrtho2D( -300, 300, -300, 300 );
    glOrtho(-300, 300, -300, 300, 0.01, 1000.0);


    glutMainLoop();

    return 0;
}