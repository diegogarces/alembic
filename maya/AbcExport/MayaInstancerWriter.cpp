#include "MayaInstancerWriter.h"
#include "AbcWriteJob.h"
#include <maya/MMatrixArray.h>

MayaInstancerWriter::MayaInstancerWriter(MDagPath & iDag,
    Alembic::Abc::OObject & iParent, Alembic::Util::uint32_t iTimeIndex,
    const JobArgs & iArgs, GetMembersMap& gmMap, const ExportedDagsMap& xpDagMap)
    : mIsGeometryAnimated(false),
    mDagPath(iDag)
{
    mFilterEulerRotations = iArgs.filterEulerRotations;
    mRotateOpIndex[0] = mRotateOpIndex[1] = mRotateOpIndex[2] =
        mRotateAxisOpIndex[0] = mRotateAxisOpIndex[1] = mRotateAxisOpIndex[2] = ~size_t(0);

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

    // Create the alembic object
    MString instancerName = fnInstancer.name();
    instancerName = util::stripNamespaces(instancerName, iArgs.stripNamespace);
    Alembic::AbcGeom::OXform obj(iParent, instancerName.asChar(),
        iTimeIndex);
    mSchema = obj.getSchema();

    // Prepare the attributes for the main transform
    Alembic::Abc::OCompoundProperty cp;
    Alembic::Abc::OCompoundProperty up;
    if (AttributesWriter::hasAnyAttr(fnInstancer, iArgs))
    {
        cp = mSchema.getArbGeomParams();
        up = mSchema.getUserProperties();
    }

    mAttrs = AttributesWriterPtr(new AttributesWriter(cp, up, obj, fnInstancer,
        iTimeIndex, iArgs));

    // Add the main transformation
    if (!iArgs.worldSpace)
    {
        MFnTransform trans(mDagPath, &status);
        pushTransformStack(trans, iTimeIndex == 0);

        // need to look at inheritsTransform
        MFnDagNode dagNode(iDag);
        MPlug inheritPlug = dagNode.findPlug("inheritsTransform");
        if (!inheritPlug.isNull())
        {
            if (util::getSampledType(inheritPlug) != 0)
            {
                mInheritsPlug = inheritPlug;
            }
            mSample.setInheritsXforms(inheritPlug.asBool());
        }

        // no animated inherits plug and no animated samples?
        // then use the default time sampling
        if (mAnimChanList.empty() && mInheritsPlug.isNull())
        {
            mSchema.setTimeSampling(0);
        }
    }
    else
    {
        // copy the dag path because we'll be popping from it
        MDagPath dag(iDag);

        int i;
        int numPaths = dag.length();
        std::vector< MDagPath > dagList;
        for (i = numPaths - 1; i > -1; i--, dag.pop())
        {
            dagList.push_back(dag);

            MFnDagNode dagNode(dag);
            MPlug inheritPlug = dagNode.findPlug("inheritsTransform");

            // if inheritsTransform exists and is set to false, then we
            // don't need to worry about ancestor nodes above this one
            if (!inheritPlug.isNull() && !inheritPlug.asBool())
                break;
        }


        std::vector< MDagPath >::iterator iStart = dagList.begin();

        std::vector< MDagPath >::iterator iCur = dagList.end();
        iCur--;

        // now loop backwards over our dagpath list so we push ancestor nodes
        // first, all the way down to the current node
        for (; iCur != iStart; iCur--)
        {
            // only add it to the stack don't write it yet!

            MFnTransform trans(*iCur);
            pushTransformStack(trans, iTimeIndex == 0);
        }

        // finally add any transform info on the final node and write it
        MFnTransform trans(*iCur);
        pushTransformStack(trans, iTimeIndex == 0);

        // need to look at inheritsTransform
        MFnDagNode dagNode(iDag);
        MPlug inheritPlug = dagNode.findPlug("inheritsTransform");
        if (!inheritPlug.isNull())
        {
            if (util::getSampledType(inheritPlug) != 0)
            {
                mInheritsPlug = inheritPlug;
            }
            mSample.setInheritsXforms(inheritPlug.asBool());
        }

        // no animated inherits plug and no animated samples?
        // then use the default time sampling
        if (mAnimChanList.empty() && mInheritsPlug.isNull())
        {
            mSchema.setTimeSampling(0);
        }
    }

    // Add Child Transforms and instances
    AddInstances(obj, iTimeIndex, iArgs, gmMap, xpDagMap);

    // everything is default, don't write anything
    if (mSample.getNumOps() > 0 || mSample.getInheritsXforms())
    {
        // Set the sample in the schema
        mSchema.set(mSample);
    }
}

void MayaInstancerWriter::write()
{

}

void MayaInstancerWriter::AddInstances(Alembic::Abc::OObject & iParent, Alembic::Util::uint32_t iTimeIndex,
    const JobArgs & iArgs, GetMembersMap& gmMap, const ExportedDagsMap& xpDagMap)
{
    MStatus status;
    // Get all the instances
    MFnInstancer fnInstancer(mDagPath, &status);
    MMatrix invInstancerMat = mDagPath.inclusiveMatrixInverse(&status);
    unsigned int numInstances = fnInstancer.particleCount();
    MDagPathArray instancePaths;
    MMatrixArray instanceMatrices;
    MIntArray instancePathStartIndices;
    MIntArray pathIndices;
    status = fnInstancer.allInstances(instancePaths, instanceMatrices, instancePathStartIndices, pathIndices);
    assert(instancePathStartIndices.length() - 1 == numInstances);

    mInstanceSamples.resize(numInstances);
    mInstanceSchemas.reserve(numInstances);
    for (unsigned int iInstance = 0; iInstance < numInstances; ++iInstance)
    {
        unsigned int startIndex = instancePathStartIndices[iInstance];

        // Create a OXform
        // Create the alembic object
        MString instanceName = "Instance";
        instanceName += iInstance;
        Alembic::AbcGeom::OXform obj(iParent, instanceName.asChar(),
            iTimeIndex);
        mInstanceSchemas.push_back(obj.getSchema());
        //Alembic::AbcGeom::OXformSchema  instanceSchema = obj.getSchema();

        // Add the transformation
        MMatrix instanceMatrix = instanceMatrices[iInstance] * invInstancerMat;
        
        // This includes the instancer transformation
        // To obtain the final position of the instance, this matrix has to be multiplied
        // by the instance transform
        const MDagPath& firstdagInstance = instancePaths[pathIndices[startIndex]];
        MMatrix instancedDagMatrix = firstdagInstance.inclusiveMatrix(&status);
        instancedDagMatrix *= instanceMatrix;
        pushTransformStack(instancedDagMatrix, mInstanceSamples[iInstance]);
        
        // DGC: COMPLETE

        // Add the instance
        unsigned int endIndex = instancePathStartIndices[iInstance + 1];
        int i = 0;
        for (unsigned int pathIndex = startIndex; pathIndex < endIndex; ++pathIndex, ++i)
        {
            unsigned int instanceIndex = pathIndices[pathIndex];
            const MDagPath& instanceDagPath = instancePaths[instanceIndex];
            MString instanceDagName = instanceDagPath.partialPathName();

            Alembic::Abc::OObject iTarget;
            auto it = xpDagMap.find(instanceDagPath);
            if (it != xpDagMap.end())
            {
                // Found the instanced node
                iTarget = it->second.mAlembicObject;
            }
            else
            {
                fprintf(stderr, "Instanced node %s not found, invalid child instanced added", instanceDagPath.partialPathName().asChar());
            }

            MString instanceShapeName = "InstanceShape";
            instanceShapeName += i;
            obj.addChildInstance(iTarget, instanceShapeName.asChar());
        }
        mInstanceSchemas[iInstance].set(mInstanceSamples[iInstance]);
    }
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

void MayaInstancerWriter::pushTransformStack(const MMatrix & matrix, Alembic::AbcGeom::XformSample& sample)
{
    // DGC: COMPLETE
    addTranslate(matrix, Alembic::AbcGeom::kTranslateHint, false, false, false, sample);
    addRotate(matrix, Alembic::AbcGeom::kRotateHint, false, false, false, sample, mRotateAxisOpIndex);
    addScale(matrix, false, false, false, sample);
}

void MayaInstancerWriter::pushTransformStack(const MFnTransform & iTrans,
    bool iForceStatic)
{

    // inspect the translate
    addTranslate(iTrans, "translate", "translateX", "translateY", "translateZ",
        Alembic::AbcGeom::kTranslateHint, false, iForceStatic, false, mSample,
        mAnimChanList);


    // inspect the rotate pivot translate
    addTranslate(iTrans, "rotatePivotTranslate", "rotatePivotTranslateX",
        "rotatePivotTranslateY", "rotatePivotTranslateZ",
        Alembic::AbcGeom::kRotatePivotTranslationHint, false,
        iForceStatic, false, mSample, mAnimChanList);

    // inspect the rotate pivot
    addTranslate(iTrans, "rotatePivot", "rotatePivotX", "rotatePivotY",
        "rotatePivotZ", Alembic::AbcGeom::kRotatePivotPointHint,
        false, iForceStatic, false, mSample, mAnimChanList);

    // inspect rotate names
    MString rotateNames[3];
    rotateNames[0] = "rotateX";
    rotateNames[1] = "rotateY";
    rotateNames[2] = "rotateZ";

    unsigned int rotOrder[3];

    // if this returns false then the rotation order was kInvalid or kLast
    MTransformationMatrix::RotationOrder eRotOrder(iTrans.rotationOrder());
    if (util::getRotOrder(eRotOrder, rotOrder[0], rotOrder[1],
        rotOrder[2]))
    {
        addRotate(iTrans, "rotate", rotateNames, rotOrder,
            Alembic::AbcGeom::kRotateHint, iForceStatic, false,
            mSample, mAnimChanList, mRotateOpIndex);
    }

    // now look at the rotation orientation, aka rotate axis
    rotateNames[0] = "rotateAxisX";
    rotateNames[1] = "rotateAxisY";
    rotateNames[2] = "rotateAxisZ";
    rotOrder[0] = 0;
    rotOrder[1] = 1;
    rotOrder[2] = 2;
    addRotate(iTrans, "rotateAxis", rotateNames, rotOrder,
        Alembic::AbcGeom::kRotateOrientationHint, iForceStatic, false,
        mSample, mAnimChanList, mRotateAxisOpIndex);

    // invert the rotate pivot if necessary
    addTranslate(iTrans, "rotatePivot", "rotatePivotX", "rotatePivotY",
        "rotatePivotZ", Alembic::AbcGeom::kRotatePivotPointHint,
        true, iForceStatic, false, mSample, mAnimChanList);

    // inspect the scale pivot translation
    addTranslate(iTrans, "scalePivotTranslate", "scalePivotTranslateX",
        "scalePivotTranslateY", "scalePivotTranslateZ",
        Alembic::AbcGeom::kScalePivotTranslationHint, false, iForceStatic,
        false, mSample, mAnimChanList);

    // inspect the scale pivot point
    addTranslate(iTrans, "scalePivot", "scalePivotX", "scalePivotY",
        "scalePivotZ", Alembic::AbcGeom::kScalePivotPointHint, false,
        iForceStatic, false, mSample, mAnimChanList);

    // inspect the shear
    addShear(iTrans, iForceStatic, mSample, mAnimChanList);

    // add the scale
    addScale(iTrans, "scale", "scaleX", "scaleY", "scaleZ", false,
        iForceStatic, false, mSample, mAnimChanList);

    // inverse the scale pivot point if necessary
    addTranslate(iTrans, "scalePivot", "scalePivotX", "scalePivotY",
        "scalePivotZ", Alembic::AbcGeom::kScalePivotPointHint, true,
        iForceStatic, false, mSample, mAnimChanList);

    // remember current rotation
    if (mFilterEulerRotations)
    {
        double xx(0), yy(0), zz(0);

        // there are 2 rotation order enum definitions:
        //     MEulerRotation::RotationOrder = MTransformationMatrix::RotationOrder-1
        if (getSampledRotation(mSample, mRotateOpIndex, xx, yy, zz))
        {
            mPrevRotateSolution.setValue(xx, yy, zz, (MEulerRotation::RotationOrder)(eRotOrder - 1));
        }

        if (getSampledRotation(mSample, mRotateAxisOpIndex, xx, yy, zz))
        {
            mPrevRotateAxisSolution.setValue(xx, yy, zz, MEulerRotation::kXYZ);
        }
    }

}