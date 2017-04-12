/**
*@file dbScan.cpp
*Cluster 3D point clouds using the DBscan algorithm.
*/


#define _CRT_SECURE_NO_WARNINGS

#include <GL/glut.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "mouseUtils.h"
#include "dbScan.h"
#include "HTRBasicDataStructures.h"

using namespace std;

vector<htr::Point3D> groupA;
vector<vector<htr::Point3D>> clusters;
vector<htr::Point3D> clustersCentroids;
htr::Point3D centroid;

bool togglePoints = false;

float cameraDistance = 100;

// Reads a cloud point file and stores the values
void readPointsFromFile(vector<htr::Point3D> *group, const char* filename)
{
	FILE *ifp;
	float x, y, z;
	int aux = 0;

	if ((ifp = fopen(filename, "r")) == NULL)
	{
		fprintf(stderr, "Can't open input file!\n");
		return;
	}

	int rows = 0; 
	while ((aux = fscanf(ifp, "%f,%f,%f\n", &x, &y, &z)) != EOF)
	{
		if (aux == 3)
		{
			htr::Point3D auxPt;
			auxPt.x = x;
			auxPt.y = y;
			auxPt.z = z;
			group->push_back(auxPt);
		}
		//if (rows > 40000)
		//	break;
		rows++;
	}

	fclose(ifp);
	fclose(ifp);
}

// Colors to display the generated clusters
float colors[] = { 1, 0, 0,
0, 1, 0,
0, 0, 1,
0, 1, 1,
1, 1, 0,
1, 0, 1,
0, 0, 1,
0, 1, 1,
1, 1, 1,
0.5, 0, 0,
0, 0.5, 0,
0, 0, 0.5
};

// Calculates a centroid from a point group
htr::Point3D calculateCentroid(vector<htr::Point3D> group)
{
	htr::Point3D exit;
	for (htr::Point3D point : group)
	{
		exit.x += point.x;
		exit.y += point.y;
		exit.z += point.z;
	}
	exit.x /= group.size();
	exit.y /= group.size();
	exit.z /= group.size();

	return exit;
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
		if (aux == 3)
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
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);

	mouseUtils::init(cameraDistance);
	readPointsFromFile(&groupA, "../../Resources/worldCloud15.csv");
	centroid = calculateCentroid(groupA);

	gluLookAt(0, 0, 0, centroid.x, centroid.y, centroid.z, 0, 1, 0);

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();

	dbScanSpace::DBSCAN_keypoints(&groupA, 46, 100, &clusters);

	end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

	for (vector<htr::Point3D> cluster : clusters)
	{
		if (cluster.size() > 0)
			clustersCentroids.push_back(calculateCentroid(cluster));
	}
}

void displayPoints()
{
	// Displays de points in the found clusters
	int j = 0;
	for (vector<htr::Point3D> cluster : clusters)
	{
		for (int i = 0; i< cluster.size(); i++)
		{
			glColor3f(colors[j], colors[j + 1], colors[j + 2]);

			glPointSize(3);
			glBegin(GL_POINTS);
			glVertex3f(cluster.at(i).x, cluster.at(i).y, cluster.at(i).z);
			glEnd();
		}

		j += 3;
	}

	// Displays the original point cloud points
	if (togglePoints)
		for_each(groupA.begin(), groupA.end(), [](htr::Point3D val)
	{
		glColor3f(0, 0.5, 0);

		glPointSize(1);
		glBegin(GL_POINTS);
		glVertex3f(val.x, val.y, val.z);
		glEnd();
	});

	// Displays the cloud and clusters centroids
	glColor3f(1, 1, 0);

	glPointSize(5);
	glBegin(GL_POINTS);
	glVertex3f(centroid.x, centroid.y, centroid.z);
	glEnd();

	for (htr::Point3D point : clustersCentroids)
	{
		glPointSize(5);
		glBegin(GL_POINTS);
		glVertex3f(point.x, point.y, point.z);
		glEnd();
	}
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
		mouseUtils::applyMouseTransform(centroid.x, centroid.y, centroid.z);
		displayPoints();
	glPopMatrix();

	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, (GLfloat)w / (GLfloat)h, 1.0, 1000.0);
	glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
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
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouseUtils::mouse);
	glutMotionFunc(mouseUtils::mouseMotion);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();
	return 0;
}