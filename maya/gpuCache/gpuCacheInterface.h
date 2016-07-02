#ifndef __GPUCACHE_INTERFACE_H__
#define __GPUCACHE_INTERFACE_H__

#include <maya/MTime.h>
#include <maya/MObject.h>
#include <maya/MAnimControl.h>
#include <maya/MPlug.h>

namespace GPUCache {

inline MTime GetShapeCurrentTime(const MObject& shape)
{
    MStatus stat;
    MFnDependencyNode depShape(shape);
    MPlug pTime = depShape.findPlug("time", &stat);


    if (stat)
    {
        MTime pMtime = pTime.asMTime();
        MPlug pOffset = depShape.findPlug("offset", &stat);
        if (stat)
            pMtime += pOffset.asMTime();
        return pMtime;
    }

    return MAnimControl::currentTime();
}

} // namespace GPUCache

#endif