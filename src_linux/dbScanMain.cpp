#include <GL/glut.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "OctreeGenerator.h"
#include "mouseUtils.h"
#include "dbScan.h"
#include "HTRBasicDataStructures.h"

using namespace std;

vector<htr::Point3D> groupA;
pcl::PointXYZ centroid;

dbScanSpace::dbscan *dbscan, dbscan2;

bool togglePoints = false;

float cameraDistance = 100;

int frameCount = 0;
float fps = 0;
int currentTime = 0, previousTime = 0;

void calculateFPS()
{
    //  Increase frame count
    frameCount++;

    //  Get the number of milliseconds since glutInit called
    //  (or first call to glutGet(GLUT ELAPSED TIME)).
    currentTime = glutGet(GLUT_ELAPSED_TIME);

    //  Calculate time passed
    int timeInterval = currentTime - previousTime;

    if(timeInterval > 1000)
    {
        //  calculate the number of frames per second
        fps = frameCount / (timeInterval / 1000.0f);

        //  Set time
        previousTime = currentTime;

        //  Reset frame count
        frameCount = 0;
    }
}

// Colors to display the generated clusters
float colors[] = {  1,0,0,
                    0,1,0,
                    0,0,1,
                    0.5,0,0,
                    0,0.5,0,
                    0,0,0.5,
                    1,1,0,
                    0,1,1,
                    1,0,1,
                    0,0,1,
                    0,1,1,
                    1,1,1
                    };

void calculateCentroid(vector<htr::Point3D>& points)
{
    for(htr::Point3D point:points)
    {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    centroid.z /= points.size();
}

void readCloudFromFile(const char* filename, vector<htr::Point3D>* points)
{
    FILE *ifp;
    float x, y, z;
    int aux = 0;

    if ((ifp = fopen(filename, "r")) == NULL)
    {
      fprintf(stderr, "Can't open input file!\n");
      return;
    }

    while ((aux = fscanf(ifp, "%f,%f,%f\n", &x, &y, &z)) != EOF)
    {
        if(aux == 3)
        {
            htr::Point3D aux;
            aux.x = x;
            aux.y = y;
            aux.z = z;
            points->push_back(aux);
        }
    }

    calculateCentroid(*points);
}

void init(void)
{
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glShadeModel (GL_FLAT);
    glEnable(GL_DEPTH_TEST);

    mouseUtils::init(cameraDistance);

//    dbscan = new dbScanSpace::dbscan("../Resources/worldCloud15.csv", 20, 25, 30, 50);
    readCloudFromFile("../Resources/worldCloud15.csv", &groupA);

//    dbscan2 = new dbScanSpace::dbscan(points, 20, 25, 30, 50);
    printf("Group A size %f\n", groupA.size()*0.001);
    dbscan2.init(groupA, groupA.size()*0.001, groupA.size()*0.001, 10, 100);
//    dbscan2.init(groupA, 25, 25, 10, 100);

    gluLookAt(0,0,0,dbscan2.getCentroid().x, dbscan2.getCentroid().y, dbscan2.getCentroid().z, 0,1,0);

//    dbscan2.init(groupA, 20, 25, 30, 50);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    dbscan2.generateClusters_fast();
//    dbscan2.generateClusters();
//    dbscan->generateClusters();

    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
//
//    std::cout << "#clusters: "<< dbscan->getClusters().size() << endl;
}

void displayPoints()
{
    // Displays de points in the found clusters
    int j = 0;
    for(dbScanSpace::cluster cluster:dbscan2.getClusters())
    {
        for(auto& point:cluster.clusterPoints)
        {
            glColor3f(colors[j], colors[j+1], colors[j+2]);

            glPointSize(3);
            glBegin(GL_POINTS);
                glVertex3f(point.x, point.y, point.z);
            glEnd();
        }

        j+=3;
        if(j > 36) j = 0;
    }

    // Displays the original point cloud points
    if(togglePoints)
        for(pcl::PointXYZ point:dbscan2.getCloudPoints()->points)
        {
            glColor3f(0,1,1);

            glPointSize(3);
            glBegin(GL_POINTS);
                glVertex3f(point.x, point.y, point.z);
            glEnd();
        }

    // Displays the cloud and clusters centroids
    glColor3f(1,1,0);

    glPointSize(5);
    glBegin(GL_POINTS);
        glVertex3f(dbscan2.getCentroid().x, dbscan2.getCentroid().y, dbscan2.getCentroid().z);
    glEnd();

    for(dbScanSpace::cluster cluster:dbscan2.getClusters())
    {
        glBegin(GL_POINTS);
            glVertex3f(cluster.centroid3D.x, cluster.centroid3D.y, cluster.centroid3D.z);
        glEnd();
    }
}

void display(void)
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glPushMatrix();
       mouseUtils::applyMouseTransform(dbscan2.getCentroid().x, dbscan2.getCentroid().y, dbscan2.getCentroid().z);
       displayPoints();
   glPopMatrix();

   glutSwapBuffers();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(90.0, (GLfloat) w/(GLfloat) h, 1.0, 1000.0);
   glMatrixMode (GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y)
{
    switch(key)
    {
    case 27: // ESCAPE
        exit(0);
        break;
    case 'a':
        togglePoints = !togglePoints;
        break;
    default:
        break;
    }
}

void idle()
{
//    dbscan2.init(groupA, 20, 25, 30, 50);
//    dbscan2.generateClusters();
    dbscan2.init(groupA, groupA.size()*0.001, groupA.size()*0.001, 10, 100);
//    dbscan2.init(groupA, 25, 25, 10, 100);
    dbscan2.generateClusters_fast();
//    calculateFPS();
//    printf("%f\n", fps);
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE| GLUT_RGB | GLUT_DEPTH);
   glutInitWindowSize (500, 500);
   glutInitWindowPosition (100, 100);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutMouseFunc(mouseUtils::mouse);
   glutMotionFunc(mouseUtils::mouseMotion);
   glutKeyboardFunc(keyboard);
   glutIdleFunc(idle);
   glutMainLoop();
   return 0;
}
