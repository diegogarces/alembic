#ifndef __TRANSFORM_UTILITY_H__
#define __TRANSFORM_UTILITY_H__

#include "Foundation.h"

#include <Alembic/AbcGeom/OXform.h>
#include <Alembic/AbcGeom/XformOp.h>

// AnimChan contains what animated plugs to get as a double, and the helper
// info about what operation and which channel to set in mSample
struct AnimChan
{
    MPlug plug;

    // extra value to multiply the data off of the plug by, used to invert
    // certain operations, and convert radians to degrees
    double scale;

    std::size_t opNum;
    Alembic::Util::uint32_t channelNum;
};


void addTranslate(const MFnDependencyNode & iTrans,
    MString parentName, MString xName, MString yName, MString zName,
    Alembic::Util::uint8_t iHint, bool inverse, bool forceStatic,
    bool forceAnimated, Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList);

// names need to be passed in x,y,z order, iOrder is the order to
// use these indices
void addRotate(const MFnDependencyNode & iTrans,
    MString parentName, const MString* iNames, const unsigned int* iOrder,
    Alembic::Util::uint8_t iHint, bool forceStatic, bool forceAnimated,
    Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList,
    size_t oOpIndex[3]);

// the test on whether or not to add it is very similar to addTranslate
// but the operation it creates is very different
void addShear(const MFnDependencyNode & iTrans, bool forceStatic,
    Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList);

// this test is very similar to addTranslate, except that it doesn't add it
// to the stack if x,y, and z are 1.0
void addScale(const MFnDependencyNode & iTrans,
    MString parentName, MString xName, MString yName, MString zName, bool inverse,
    bool forceStatic, bool forceAnimated, Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList);

bool getSampledRotation(const Alembic::AbcGeom::XformSample& sample,
    const size_t opIndex[3], double& xx, double& yy, double& zz);

bool setSampledRotation(Alembic::AbcGeom::XformSample& sample,
    const size_t opIndex[3], double xx, double yy, double zz);
#endif