#include "TransformUtility.h"
#include "MayaUtility.h"

void addTranslate(const MFnDependencyNode & iTrans,
    MString parentName, MString xName, MString yName, MString zName,
    Alembic::Util::uint8_t iHint, bool inverse, bool forceStatic,
    bool forceAnimated, Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList)
{
    Alembic::AbcGeom::XformOp op(Alembic::AbcGeom::kTranslateOperation, iHint);

    MPlug xPlug = iTrans.findPlug(xName);
    int xSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            xSamp = util::getSampledType(xPlug);
        else
            xSamp = 1;
    }
    double xVal = xPlug.asDouble();

    MPlug yPlug = iTrans.findPlug(yName);
    int ySamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            ySamp = util::getSampledType(yPlug);
        else
            ySamp = 1;
    }
    double yVal = yPlug.asDouble();

    MPlug zPlug = iTrans.findPlug(zName);
    int zSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            zSamp = util::getSampledType(zPlug);
        else
            zSamp = 1;
    }
    double zVal = zPlug.asDouble();

    // this is to handle the case where there is a connection to the parent
    // plug but not to the child plugs, if the connection is there then all
    // of the children are considered animated
    MPlug parentPlug = iTrans.findPlug(parentName);
    int parentSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            parentSamp = util::getSampledType(parentPlug);
        else
            parentSamp = 1;
    }

    if (parentSamp != 0)
    {
        xSamp = 1;
        ySamp = 1;
        zSamp = 1;
    }

    // something is sampled or not identity, add it to the stack
    if (xSamp != 0 || ySamp != 0 || zSamp != 0 || xVal != 0.0 || yVal != 0.0 ||
        zVal != 0.0)
    {
        if (inverse)
        {
            xVal = -xVal;
            yVal = -yVal;
            zVal = -zVal;
        }

        op.setChannelValue(0, xVal);
        op.setChannelValue(1, yVal);
        op.setChannelValue(2, zVal);

        if (xSamp != 0)
        {
            AnimChan chan;
            chan.plug = xPlug;
            chan.scale = 1.0;
            if (inverse)
                chan.scale = -1.0;
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 0;
            oAnimChanList.push_back(chan);
        }

        if (ySamp != 0)
        {
            AnimChan chan;
            chan.plug = yPlug;
            chan.scale = 1.0;
            if (inverse)
                chan.scale = -1.0;
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 1;
            oAnimChanList.push_back(chan);
        }

        if (zSamp != 0)
        {
            AnimChan chan;
            chan.plug = zPlug;
            chan.scale = 1.0;
            if (inverse)
                chan.scale = -1.0;
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 2;
            oAnimChanList.push_back(chan);
        }

        oSample.addOp(op);
    }
}

// names need to be passed in x,y,z order, iOrder is the order to
// use these indices
void addRotate(const MFnDependencyNode & iTrans,
    MString parentName, const MString* iNames, const unsigned int* iOrder,
    Alembic::Util::uint8_t iHint, bool forceStatic, bool forceAnimated,
    Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList,
    size_t oOpIndex[3])
{
    // for each possible rotation axis
    static const Alembic::AbcGeom::XformOperationType rots[3] = {
        Alembic::AbcGeom::kRotateXOperation,
        Alembic::AbcGeom::kRotateYOperation,
        Alembic::AbcGeom::kRotateZOperation
    };

    // this is to handle the case where there is a connection to the parent
    // plug but not to the child plugs, if the connection is there then all
    // of the children are considered animated
    MPlug parentPlug = iTrans.findPlug(parentName);
    int parentSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            parentSamp = util::getSampledType(parentPlug);
        else
            parentSamp = 1;
    }

    // whether we are using the XYZ rotation order
    bool isXYZ = ((iOrder[0] == 0) && (iOrder[1] == 1) && (iOrder[2] == 2));

    // add them in backwards since we are dealing with a stack
    int i = 2;
    for (; i > -1; i--)
    {
        unsigned int index = iOrder[i];
        MPlug plug = iTrans.findPlug(iNames[index]);
        int samp = 0;
        if (!forceStatic)
        {
            if (!forceAnimated)
                samp = util::getSampledType(plug);
            else
                samp = 1;

            if (samp == 0)
                samp = parentSamp;
        }

        double plugVal = plug.asDouble();


        Alembic::AbcGeom::XformOp op(rots[index], iHint);
        op.setChannelValue(0, Alembic::AbcGeom::RadiansToDegrees(plugVal));

        // the sampled case
        if (samp != 0)
        {
            AnimChan chan;
            chan.plug = plug;
            chan.scale = Alembic::AbcGeom::RadiansToDegrees(1.0);
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 0;
            oAnimChanList.push_back(chan);
        }
        // non sampled, XYZ axis and the angle is 0, do not add to the stack
        else if (isXYZ && plugVal == 0.0)
            continue;

        oOpIndex[index] = oSample.addOp(op);
    }
}

// the test on whether or not to add it is very similiar to addTranslate
// but the operation it creates is very different
void addShear(const MFnDependencyNode & iTrans, bool forceStatic,
    Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList)
{
    Alembic::AbcGeom::XformOp op(Alembic::AbcGeom::kMatrixOperation,
        Alembic::AbcGeom::kMayaShearHint);

    MString str = "shearXY";
    MPlug xyPlug = iTrans.findPlug(str);
    int xySamp = 0;
    if (!forceStatic)
    {
        xySamp = util::getSampledType(xyPlug);
    }
    double xyVal = xyPlug.asDouble();

    str = "shearXZ";
    MPlug xzPlug = iTrans.findPlug(str);
    int xzSamp = 0;
    if (!forceStatic)
    {
        xzSamp = util::getSampledType(xzPlug);
    }
    double xzVal = xzPlug.asDouble();

    str = "shearYZ";
    MPlug yzPlug = iTrans.findPlug(str);
    int yzSamp = 0;
    if (!forceStatic)
    {
        yzSamp = util::getSampledType(yzPlug);
    }
    double yzVal = yzPlug.asDouble();

    // this is to handle the case where there is a connection to the parent
    // plug but not to the child plugs, if the connection is there then all
    // of the children are considered animated
    str = "shear";
    MPlug parentPlug = iTrans.findPlug(str);
    if (!forceStatic && util::getSampledType(parentPlug) != 0)
    {
        xySamp = 1;
        xzSamp = 1;
        yzSamp = 1;
    }

    // something is sampled or not identity, add it to the stack
    if (xySamp != 0 || xzSamp != 0 || yzSamp != 0 ||
        xyVal != 0.0 || xzVal != 0.0 || yzVal != 0.0)
    {
        Alembic::Abc::M44d m;
        m.makeIdentity();
        op.setMatrix(m);
        op.setChannelValue(4, xyVal);
        op.setChannelValue(8, xzVal);
        op.setChannelValue(9, yzVal);

        if (xySamp != 0)
        {
            AnimChan chan;
            chan.plug = xyPlug;
            chan.scale = 1.0;
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 4;
            oAnimChanList.push_back(chan);
        }

        if (xzSamp != 0)
        {
            AnimChan chan;
            chan.plug = xzPlug;
            chan.scale = 1.0;
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 8;
            oAnimChanList.push_back(chan);
        }

        if (yzSamp != 0)
        {
            AnimChan chan;
            chan.plug = yzPlug;
            chan.scale = 1.0;
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 9;
            oAnimChanList.push_back(chan);
        }

        oSample.addOp(op);
    }
}

// this test is very similiar to addTranslate, except that it doesn't add it
// to the stack if x,y, and z are 1.0
void addScale(const MFnDependencyNode & iTrans,
    MString parentName, MString xName, MString yName, MString zName, bool inverse,
    bool forceStatic, bool forceAnimated, Alembic::AbcGeom::XformSample & oSample,
    std::vector < AnimChan > & oAnimChanList)
{

    Alembic::AbcGeom::XformOp op(Alembic::AbcGeom::kScaleOperation,
        Alembic::AbcGeom::kScaleHint);

    MPlug xPlug = iTrans.findPlug(xName);
    int xSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            xSamp = util::getSampledType(xPlug);
        else
            xSamp = 1;
    }
    double xVal = xPlug.asDouble();

    MPlug yPlug = iTrans.findPlug(yName);
    int ySamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            ySamp = util::getSampledType(yPlug);
        else
            ySamp = 1;
    }
    double yVal = yPlug.asDouble();

    MPlug zPlug = iTrans.findPlug(zName);
    int zSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            zSamp = util::getSampledType(zPlug);
        else
            zSamp = 1;
    }
    double zVal = zPlug.asDouble();

    // this is to handle the case where there is a connection to the parent
    // plug but not to the child plugs, if the connection is there then all
    // of the children are considered animated
    MPlug parentPlug = iTrans.findPlug(parentName);
    int parentSamp = 0;
    if (!forceStatic)
    {
        if (!forceAnimated)
            parentSamp = util::getSampledType(parentPlug);
        else
            parentSamp = 1;
    }

    if (parentSamp != 0)
    {
        xSamp = 1;
        ySamp = 1;
        zSamp = 1;
    }

    // something is sampled or not identity, add it to the stack
    if (xSamp != 0 || ySamp != 0 || zSamp != 0 || xVal != 1.0 || yVal != 1.0 ||
        zVal != 1.0)
    {
        if (inverse)
        {
            xVal = util::inverseScale(xVal);
            yVal = util::inverseScale(yVal);
            zVal = util::inverseScale(zVal);
        }

        op.setChannelValue(0, xVal);
        op.setChannelValue(1, yVal);
        op.setChannelValue(2, zVal);

        if (xSamp != 0)
        {
            AnimChan chan;
            chan.plug = xPlug;
            chan.scale = 1.0;
            if (inverse)
                chan.scale = -std::numeric_limits<double>::infinity();
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 0;
            oAnimChanList.push_back(chan);
        }

        if (ySamp != 0)
        {
            AnimChan chan;
            chan.plug = yPlug;
            chan.scale = 1.0;
            if (inverse)
                chan.scale = -std::numeric_limits<double>::infinity();
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 1;
            oAnimChanList.push_back(chan);
        }

        if (zSamp != 0)
        {
            AnimChan chan;
            chan.plug = zPlug;
            chan.scale = 1.0;
            if (inverse)
                chan.scale = -std::numeric_limits<double>::infinity();
            chan.opNum = oSample.getNumOps();
            chan.channelNum = 2;
            oAnimChanList.push_back(chan);
        }

        oSample.addOp(op);
    }
}

bool getSampledRotation(const Alembic::AbcGeom::XformSample& sample,
    const size_t opIndex[3], double& xx, double& yy, double& zz)
{
    bool success = false;

    xx = 0.0;
    if (opIndex[0] < sample.getNumOps())
    {
        double angleX = sample[opIndex[0]].getChannelValue(0);
        xx = Alembic::AbcGeom::DegreesToRadians(angleX);
        success = true;
    }

    yy = 0.0;
    if (opIndex[1] < sample.getNumOps())
    {
        double angleY = sample[opIndex[1]].getChannelValue(0);
        yy = Alembic::AbcGeom::DegreesToRadians(angleY);
        success = true;
    }

    zz = 0.0;
    if (opIndex[2] < sample.getNumOps())
    {
        double angleZ = sample[opIndex[2]].getChannelValue(0);
        zz = Alembic::AbcGeom::DegreesToRadians(angleZ);
        success = true;
    }

    return success;
}

bool setSampledRotation(Alembic::AbcGeom::XformSample& sample,
    const size_t opIndex[3], double xx, double yy, double zz)
{
    bool success = false;

    if (opIndex[0] < sample.getNumOps())
    {
        sample[opIndex[0]].setChannelValue(0, Alembic::AbcGeom::RadiansToDegrees(xx));
        success = true;
    }

    if (opIndex[1] < sample.getNumOps())
    {
        sample[opIndex[1]].setChannelValue(0, Alembic::AbcGeom::RadiansToDegrees(yy));
        success = true;
    }

    if (opIndex[2] < sample.getNumOps())
    {
        sample[opIndex[2]].setChannelValue(0, Alembic::AbcGeom::RadiansToDegrees(zz));
        success = true;
    }

    return success;
}
