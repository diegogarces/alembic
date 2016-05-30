#include "MayaInstancerWriter.h"

MayaInstancerWriter::MayaInstancerWriter(MDagPath & iDag,
    Alembic::Abc::OObject & iParent, Alembic::Util::uint32_t iTimeIndex,
    const JobArgs & iArgs, GetMembersMap& gmMap)
    : mIsGeometryAnimated(false),
    mDagPath(iDag)
{
    MStatus status = MS::kSuccess;
    MFnInstancer fnInstancer(mDagPath, &status);
    if (!status)
    {
        MGlobal::displayError("MFnInstancer() failed for MayaInstancerWriter");
    }

    // intermediate objects aren't translated
    MObject oInstancer = iDag.node();

    if (iTimeIndex != 0 && util::isAnimated(oInstancer))
    {
        mIsGeometryAnimated = true;
    }
    else
    {
        iTimeIndex = 0;
    }

    MString instancerName = fnInstancer.name();
    Alembic::AbcGeom::OXform obj(iParent, instancerName.asChar(),
        iTimeIndex);
    mSchema = obj.getSchema();

    Alembic::Abc::OCompoundProperty cp;
    Alembic::Abc::OCompoundProperty up;
    if (AttributesWriter::hasAnyAttr(fnInstancer, iArgs))
    {
        cp = mSchema.getArbGeomParams();
        up = mSchema.getUserProperties();
    }

    mAttrs = AttributesWriterPtr(new AttributesWriter(cp, up, obj, fnInstancer,
        iTimeIndex, iArgs));
}


bool MayaInstancerWriter::isAnimated() const
{
    return mIsGeometryAnimated;
}

unsigned int MayaInstancerWriter::getNumCVs()
{
    MStatus status = MS::kSuccess;
    MFnInstancer fnInstancer(mDagPath, &status);
    if (!status)
    {
        MGlobal::displayError("MFnInstancer() failed for MayaInstancerWriter");
    }

    return fnInstancer.particleCount();
}