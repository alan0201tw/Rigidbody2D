#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>

#include <GL/freeglut.h>

#include "linalg.h"

#include "clock.hpp"
#include "scene.hpp"
#include "circle.hpp"
#include "aabb.hpp"
#include "integrator.hpp"
#include "joint.hpp"

namespace
{
    const float deltaTime = 1.0f / 60.0f;
    const uint32_t positional_correction_iterations = 10;
    const float accumulate_upper_bound = 
        std::max(deltaTime, 0.1f);

    Scene scene(deltaTime, positional_correction_iterations,
        std::make_shared<ExplicitEulerIntegrator>()
    );

    int screen_width = 600;
	int screen_height = 600;

    typedef linalg::aliases::float2 float2;
}

class GLUTCallback
{
private:
    static float accumulator;

    static void RenderScene()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(
        0, 0, 0,
		0, 0, -1,
		0, 1, 0);

        // glPushMatrix();
        // {
        //     glTranslated(0, 0, 0);

        //     glutWireTeapot(10.0);
        // }
        // glPopMatrix();

        scene.Render();

        glutSwapBuffers();
        glutPostRedisplay();
    }

public:
    static void MainLoop()
    {
        accumulator += (float)Clock::Elapsed();
        Clock::Reset();

        // Notice that in order to prevent a burst of calling to step(), 
        // we clamp the accumulated time steps. However, if the delta time
        // is larger than the upper bound of clamp, step() will never be
        // called.
        accumulator = std::clamp(accumulator, 0.0f, accumulate_upper_bound);
        while(accumulator >= deltaTime)
        {
            scene.Step();

            accumulator -= deltaTime;
        }

        RenderScene();
    }

    static void Keyboard(unsigned char key, int x, int y)
    {

    }

    static void Reshape(int width, int height)
    {
        screen_width = width;
        screen_height = height;

        glViewport( 0, 0, width, height );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        // when init, window size is 600x600 mapping to [-30 ~ 30]
        float2 ortho_size((float)width / 20.0f, (float)height / 20.0f);
        
        // gluOrtho2D sets up a two-dimensional orthographic viewing region.
        // This is equivalent to calling glOrtho with near = -1 and far = 1 .
        gluOrtho2D(-ortho_size.x, ortho_size.x, -ortho_size.y, ortho_size.y);
    }

    static void Mouse(int button, int state, int x, int y)
    {
        if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
        {
            float2 ortho_size((float)screen_width / 20.0f, (float)screen_height / 20.0f);
            float2 position = float2( (float)x / 10.0f - ortho_size.x, (float)y / -10.0f + ortho_size.y );

            std::shared_ptr<Circle> shape = std::make_shared<Circle>(
                3.0f
            );
            auto body = scene.AddRigidBody(shape, position);
        }
        if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
        {
            float2 ortho_size((float)screen_width / 20.0f, (float)screen_height / 20.0f);
            float2 position = float2( (float)x / 10.0f - ortho_size.x, (float)y / -10.0f + ortho_size.y );

            std::shared_ptr<AABB> shape = std::make_shared<AABB>(
                //float2 (3, 3)
#ifdef _MSC_VER
				float2(((float)rand() / (RAND_MAX)) * 5 + 3, ((float)rand() / (RAND_MAX)) * 5 + 3)
#else
                float2( drand48() * 5 + 3, drand48() * 5 + 3 )
#endif
            );
            auto body = scene.AddRigidBody(shape, position);
        }
    }
};

float GLUTCallback::accumulator = 0.0f;

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(screen_width, screen_height);
    glutCreateWindow("PhyEngine");
    glutDisplayFunc(GLUTCallback::MainLoop);
    glutKeyboardFunc(GLUTCallback::Keyboard);
    glutMouseFunc(GLUTCallback::Mouse);
    glutReshapeFunc(GLUTCallback::Reshape);

    // NOTE : please do not use glutTimerFunc.
    // We need you to practice on designing the loop itself,
    // resolving the different update rate of physics and rendering.

    // fill in the scene
    // floor
    {
        std::shared_ptr<AABB> shape3 = std::make_shared<AABB>(
            float2 (35, 1)
        );
        auto body1 = scene.AddRigidBody(shape3, float2(0, -10));
        // setting an infinite mass
        body1->SetMass(0.0f);
    }
    {
        std::shared_ptr<AABB> shape3 = std::make_shared<AABB>(
            float2 (35, 1)
        );
        auto body1 = scene.AddRigidBody(shape3, float2(10, -20));
        // setting an infinite mass
        body1->SetMass(0.0f);
        // std::shared_ptr<Circle> shape = std::make_shared<Circle>(100.0f);
        // auto body = scene.AddRigidBody(shape, float2(0.0f, -100.0f));
        // body->m_mass = 0.0f;
    }
    // dynamic objects
    {
        std::shared_ptr<Circle> shape = std::make_shared<Circle>(2.0f);
        auto body = scene.AddRigidBody(shape, float2(-12.5f, 5.0f));
        body->SetVelocity(float2(15, 0));
        // body->m_torque = 1.0f;
    }
    {
        std::shared_ptr<Circle> shape1 = std::make_shared<Circle>(2.0f);
        scene.AddRigidBody(shape1, float2(-5, 6));
    }
    {
        std::shared_ptr<AABB> shape2 = std::make_shared<AABB>(
            float2 (5, 5)
        );
        auto body1 = scene.AddRigidBody(shape2, float2(-5, 20));
        body1->SetVelocity(float2(8, 5));
        // body1->m_torque = 5.0f;
    }
    {
        std::shared_ptr<AABB> shape3 = std::make_shared<AABB>(
            float2 (15, 15)
        );
        auto body1 = scene.AddRigidBody(shape3, float2(5, 20));
        // body1->SetVelocity(float2(-8, 5));
        body1->SetMass(0.0f);
        
        std::shared_ptr<Circle> shape = std::make_shared<Circle>(2.0f);
        auto body = scene.AddRigidBody(shape, float2(-5.0f, -5.0f));
		//body->SetMass(5.0f);
        // body->SetVelocity(float2(40, 0));

        std::shared_ptr<DistanceJoint> disJoint = 
            std::make_shared<DistanceJoint>(body, body1, 15.0f);
        scene.AddJoint(disJoint);
    }
    
    glutMainLoop();

    return 0;
}