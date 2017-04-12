/**
*@file HTRBasicDataStructures.h
*Data structures that do not depend on external classes.
*/

#ifndef HTR_BASIC_DATA_STRUCTURES_H
#define HTR_BASIC_DATA_STRUCTURES_H

namespace htr{
	struct Index2D{
		int x;
		int y;
		//Index2D():x(0),y(0){}
	};

	struct Point3D{
		float x;
		float y;
		float z;

		void initRandom()
        {
            x = (rand() % 40);
            y = (rand() % 40);
            z = (rand() % 40);
        }
	};

	struct FlaggedPoint3D{
		Point3D point;
		int flag;
	};

	struct DepthPixel{
		int x;
		int y;
		float z;
	};

	struct LabeledPoint{
		Point3D point;
		int label;
	};

	struct CubeBoundary{
		Point3D start;
		Point3D end;
	};

	struct LinearBoundary{
		float start;
		float end;
	};

}

#endif
