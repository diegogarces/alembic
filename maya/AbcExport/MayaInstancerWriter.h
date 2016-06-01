#ifndef __MAYA_INSTANCER_WRITER_H__
#define __MAYA_INSTANCER_WRITER_H__

#include "MayaMeshWriter.h"
#include "TransformUtility.h"

class MayaInstancerWriter
{
public:

    MayaInstancerWriter(MDagPath & iDag, Alembic::Abc::OObject & iParent,
        Alembic::Util::uint32_t iTimeIndex, const JobArgs & iArgs,
        GetMembersMap& gmMap);
    void write();
    bool isAnimated() const;
    unsigned int getNumCVs();
    AttributesWriterPtr getAttrs() { return mAttrs; };
    Alembic::Abc::OObject GetAlembicObject() {return mSchema.getObject();}
private:

    void AddInstances(Alembic::Abc::OObject & iParent, Alembic::Util::uint32_t iTimeIndex,
        const JobArgs & iArgs, GetMembersMap& gmMap);
    void pushTransformStack(const MFnTransform & iTrans, bool iForceStatic);

    bool mIsGeometryAnimated;
    MDagPath mDagPath;

    AttributesWriterPtr mAttrs;
    Alembic::AbcGeom::OXformSchema mSchema;

    Alembic::AbcGeom::XformSample mSample;

    std::vector < AnimChan > mAnimChanList;
    MPlug mInheritsPlug;

    bool mFilterEulerRotations;
    MEulerRotation mPrevRotateSolution;
    MEulerRotation mPrevRotateAxisSolution;

    size_t mRotateOpIndex[3];
    size_t mRotateAxisOpIndex[3];
};

#endif