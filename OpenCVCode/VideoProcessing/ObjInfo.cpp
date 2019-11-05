#include "ObjInfo.h"

ObjInfo::ObjInfo(TimeObjInf t)
{
	infs.push_back(t);
}

ObjInfo::ObjInfo(double x, double y, double a)
{
	TimeObjInf toi(Point2d(x, y), a);
	infs.push_back(toi);
}


