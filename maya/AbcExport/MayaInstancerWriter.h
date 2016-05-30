#ifndef __MAYA_INSTANCER_WRITER_H__
#define __MAYA_INSTANCER_WRITER_H__

#include "MayaMeshWriter.h"

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

private:

    bool mIsGeometryAnimated;
    MDagPath mDagPath;

    AttributesWriterPtr mAttrs;
    Alembic::AbcGeom::OXformSchema mSchema;
};

#endif