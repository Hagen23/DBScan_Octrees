/**
*@file structs.h
* Useful data structures.
*/

#ifndef STRUCTS
#define STRUCTS

struct point
{
    long x, y, z;

    point()
    {
        x = y = z = 0.0;
    }

    void initRandom()
    {
        x = (rand() % 40);
        y = (rand() % 40);
        z = (rand() % 40);
    }

    void initRandom(float _x, float _y, float _z)
    {
        x = (rand() % 30)+_x;
        y = (rand() % 30)+_y;
        z = (rand() % 30)+_z;
    }
};

#endif
