//-*****************************************************************************
//
// Copyright (c) 2009-2012,
//  Sony Pictures Imageworks Inc. and
//  Industrial Light & Magic, a division of Lucasfilm Entertainment Company Ltd.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Sony Pictures Imageworks, nor
// Industrial Light & Magic, nor the names of their contributors may be used
// to endorse or promote products derived from this software without specific
// prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-*****************************************************************************

#include "MayaTransformWriter.h"
#include "MayaUtility.h"


MayaTransformWriter::MayaTransformWriter(Alembic::AbcGeom::OObject & iParent,
    MDagPath & iDag, Alembic::Util::uint32_t iTimeIndex, const JobArgs & iArgs)
{
    mFilterEulerRotations = iArgs.filterEulerRotations;
    mJointOrientOpIndex[0] = mJointOrientOpIndex[1] = mJointOrientOpIndex[2] =
    mRotateOpIndex[0]      = mRotateOpIndex[1]      = mRotateOpIndex[2]      =
    mRotateAxisOpIndex[0]  = mRotateAxisOpIndex[1]  = mRotateAxisOpIndex[2]  = ~size_t(0);

    if (iDag.hasFn(MFn::kJoint))
    {
        MFnIkJoint joint(iDag);
        MString jointName = joint.name();

        jointName = util::stripNamespaces(jointName, iArgs.stripNamespace);

        Alembic::AbcGeom::OXform obj(iParent, jointName.asChar(),
            iTimeIndex);
        mSchema = obj.getSchema();

        Alembic::Abc::OCompoundProperty cp;
        Alembic::Abc::OCompoundProperty up;
        if (AttributesWriter::hasAnyAttr(joint, iArgs))
        {
            cp = mSchema.getArbGeomParams();
            up = mSchema.getUserProperties();
        }

        mAttrs = AttributesWriterPtr(new AttributesWriter(cp, up, obj, joint,
            iTimeIndex, iArgs, false));

        if (!iArgs.worldSpace)
        {
            pushTransformStack(joint, iTimeIndex == 0);

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

            // everything is default, don't write anything
            if (mSample.getNumOps() == 0 && mSample.getInheritsXforms())
            {
                return;
            }

            mSchema.set(mSample);
            return;
        }
    }
    else
    {
        MFnTransform trans(iDag);
        MString transName = trans.name();

        transName = util::stripNamespaces(transName, iArgs.stripNamespace);

        Alembic::AbcGeom::OXform obj(iParent, transName.asChar(),
            iTimeIndex);
        mSchema = obj.getSchema();

        Alembic::Abc::OCompoundProperty cp;
        Alembic::Abc::OCompoundProperty up;
        if (AttributesWriter::hasAnyAttr(trans, iArgs))
        {
            cp = mSchema.getArbGeomParams();
            up = mSchema.getUserProperties();
        }

        mAttrs = AttributesWriterPtr(new AttributesWriter(cp, up, obj, trans,
            iTimeIndex, iArgs, false));

        if (!iArgs.worldSpace)
        {
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

            // everything is default, don't write anything
            if (mSample.getNumOps() == 0 && mSample.getInheritsXforms())
            {
                return;
            }

            mSchema.set(mSample);
            return;
        }
    }

    // if we didn't bail early then we need to add all the transform
    // information at the current node and above

    // copy the dag path because we'll be popping from it
    MDagPath dag(iDag);

    int i;
    int numPaths = dag.length();
    std::vector< MDagPath > dagList;
    for (i = numPaths - 1; i > -1; i--, dag.pop())
    {
        dagList.push_back(dag);

        // inheritsTransform exists on both joints and transforms
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

        if (iCur->hasFn(MFn::kJoint))
        {
            MFnIkJoint joint(*iCur);
            pushTransformStack(joint, iTimeIndex == 0);
        }
        else
        {
            MFnTransform trans(*iCur);
            pushTransformStack(trans, iTimeIndex == 0);
        }
    }

    // finally add any transform info on the final node and write it
    if (iCur->hasFn(MFn::kJoint))
    {
        MFnIkJoint joint(*iCur);
        pushTransformStack(joint, iTimeIndex == 0);
    }
    else
    {
        MFnTransform trans(*iCur);
        pushTransformStack(trans, iTimeIndex == 0);
    }

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

    // everything is default, don't write anything and use the default
    // time sampling
    if (mSample.getNumOps() == 0 && mSample.getInheritsXforms())
    {
        return;
    }

    mSchema.set(mSample);

}

MayaTransformWriter::MayaTransformWriter(MayaTransformWriter & iParent,
    MDagPath & iDag, Alembic::Util::uint32_t iTimeIndex, const JobArgs & iArgs)
{
    mFilterEulerRotations = iArgs.filterEulerRotations;
    mJointOrientOpIndex[0] = mJointOrientOpIndex[1] = mJointOrientOpIndex[2] =
    mRotateOpIndex[0]      = mRotateOpIndex[1]      = mRotateOpIndex[2]      =
    mRotateAxisOpIndex[0]  = mRotateAxisOpIndex[1]  = mRotateAxisOpIndex[2]  = ~size_t(0);

    if (iDag.hasFn(MFn::kJoint))
    {
        MFnIkJoint joint(iDag);
        MString jointName = joint.name();

        jointName = util::stripNamespaces(jointName, iArgs.stripNamespace);

        Alembic::AbcGeom::OXform obj(iParent.getObject(), jointName.asChar(),
            iTimeIndex);
        mSchema = obj.getSchema();

        Alembic::Abc::OCompoundProperty cp;
        Alembic::Abc::OCompoundProperty up;
        if (AttributesWriter::hasAnyAttr(joint, iArgs))
        {
            cp = mSchema.getArbGeomParams();
            up = mSchema.getUserProperties();
        }

        mAttrs = AttributesWriterPtr(new AttributesWriter(cp, up, obj, joint,
            iTimeIndex, iArgs, false));

        pushTransformStack(joint, iTimeIndex == 0);
    }
    else
    {
        MFnTransform trans(iDag);
        MString transName = trans.name();

        transName = util::stripNamespaces(transName, iArgs.stripNamespace);

        Alembic::AbcGeom::OXform obj(iParent.getObject(), transName.asChar(),
            iTimeIndex);
        mSchema = obj.getSchema();

        Alembic::Abc::OCompoundProperty cp;
        Alembic::Abc::OCompoundProperty up;
        if (AttributesWriter::hasAnyAttr(trans, iArgs))
        {
            cp = mSchema.getArbGeomParams();
            up = mSchema.getUserProperties();
        }

        mAttrs = AttributesWriterPtr(new AttributesWriter(cp, up, obj, trans,
            iTimeIndex, iArgs, false));

        pushTransformStack(trans, iTimeIndex == 0);
    }


    // need to look at inheritsTransform
    MFnDagNode dagNode(iDag);
    MPlug inheritPlug = dagNode.findPlug("inheritsTransform");
    if (!inheritPlug.isNull())
    {
        if (util::getSampledType(inheritPlug) != 0)
            mInheritsPlug = inheritPlug;

        mSample.setInheritsXforms(inheritPlug.asBool());
    }

    // everything is default, don't write anything
    if (mSample.getNumOps() == 0 && mSample.getInheritsXforms())
        return;

    mSchema.set(mSample);
}


MayaTransformWriter::~MayaTransformWriter()
{
}

void MayaTransformWriter::write()
{
    size_t numSamples = mAnimChanList.size();
    if (numSamples > 0)
    {
        std::vector < AnimChan >::iterator it, itEnd;

        for (it = mAnimChanList.begin(), itEnd = mAnimChanList.end();
            it != itEnd; ++it)
        {
            double val = it->plug.asDouble();

            if (it->scale == -std::numeric_limits<double>::infinity())
                val = util::inverseScale(val);
            else if (it->scale != 1.0)
                val *= it->scale;

            mSample[it->opNum].setChannelValue(it->channelNum, val);
        }

        if (!mInheritsPlug.isNull())
        {
            mSample.setInheritsXforms(mInheritsPlug.asBool());
        }

        if (mFilterEulerRotations)
        {
            double xx(0), yy(0), zz(0);

            if (getSampledRotation(mSample, mJointOrientOpIndex, xx, yy, zz))
            {
                MEulerRotation euler(xx, yy, zz, mPrevJointOrientSolution.order);
                euler.setToClosestSolution(mPrevJointOrientSolution);

                // update sample with new solution
                setSampledRotation(mSample, mJointOrientOpIndex, euler.x, euler.y, euler.z);
                mPrevJointOrientSolution = euler;
            }

            if (getSampledRotation(mSample, mRotateOpIndex, xx, yy, zz))
            {
                MEulerRotation euler(xx, yy, zz, mPrevRotateSolution.order);
                euler.setToClosestSolution(mPrevRotateSolution);

                // update sample with new solution
                setSampledRotation(mSample, mRotateOpIndex, euler.x, euler.y, euler.z);
                mPrevRotateSolution = euler;
            }

            if (getSampledRotation(mSample, mRotateAxisOpIndex, xx, yy, zz))
            {
                MEulerRotation euler(xx, yy, zz, mPrevRotateAxisSolution.order);
                euler.setToClosestSolution(mPrevRotateAxisSolution);

                // update sample with new solution
                setSampledRotation(mSample, mRotateAxisOpIndex, euler.x, euler.y, euler.z);
                mPrevRotateAxisSolution = euler;
            }
        }

        mSchema.set(mSample);
    }
}

bool MayaTransformWriter::isAnimated() const
{
    return mAnimChanList.size() > 0 || !mInheritsPlug.isNull();
}

void MayaTransformWriter::pushTransformStack(const MFnTransform & iTrans,
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
        "rotatePivotZ",  Alembic::AbcGeom::kRotatePivotPointHint,
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
        if (getSampledRotation( mSample, mRotateOpIndex, xx, yy, zz ))
        {
            mPrevRotateSolution.setValue(xx, yy, zz, (MEulerRotation::RotationOrder)(eRotOrder-1));
        }

        if (getSampledRotation( mSample, mRotateAxisOpIndex, xx, yy, zz ))
        {
            mPrevRotateAxisSolution.setValue(xx, yy, zz, MEulerRotation::kXYZ);
        }
    }

}

void MayaTransformWriter::pushTransformStack(const MFnIkJoint & iJoint,
    bool iForceStatic)
{
    // Some special cases that the joint is animated but has no input connections.
    bool forceAnimated = util::isDrivenByFBIK(iJoint) || util::isDrivenBySplineIK(iJoint);

    // inspect the translate
    addTranslate(iJoint, "translate", "translateX", "translateY", "translateZ",
        Alembic::AbcGeom::kTranslateHint, false, iForceStatic, forceAnimated,
        mSample, mAnimChanList);

    // inspect the inverseParent scale
    // [IS] is ignored when Segment Scale Compensate is false
    MPlug scaleCompensatePlug = iJoint.findPlug("segmentScaleCompensate");
    if (scaleCompensatePlug.asBool())
    {
        addScale(iJoint, "inverseScale", "inverseScaleX", "inverseScaleY",
            "inverseScaleZ", true, iForceStatic, forceAnimated, mSample, mAnimChanList);
    }

    MTransformationMatrix::RotationOrder eJointOrientOrder, eRotOrder, eRotateAxisOrder;
    double vals[3];

    // for reordering rotate names
    MString rotateNames[3];
    unsigned int rotOrder[3];

    // now look at the joint orientation
    rotateNames[0] = "jointOrientX";
    rotateNames[1] = "jointOrientY";
    rotateNames[2] = "jointOrientZ";

    iJoint.getOrientation(vals, eJointOrientOrder);
    if (util::getRotOrder(eJointOrientOrder, rotOrder[0], rotOrder[1], rotOrder[2]))
    {
        addRotate(iJoint, "jointOrient", rotateNames, rotOrder,
            Alembic::AbcGeom::kRotateHint, iForceStatic, true,
            mSample, mAnimChanList, mJointOrientOpIndex);
    }

    rotateNames[0] = "rotateX";
    rotateNames[1] = "rotateY";
    rotateNames[2] = "rotateZ";

    // if this returns false then the rotation order was kInvalid or kLast
    eRotOrder = iJoint.rotationOrder();
    if (util::getRotOrder(eRotOrder, rotOrder[0], rotOrder[1],
        rotOrder[2]))
    {
        addRotate(iJoint, "rotate", rotateNames, rotOrder,
            Alembic::AbcGeom::kRotateHint, iForceStatic, true,
            mSample, mAnimChanList, mRotateOpIndex);
    }

    // now look at the rotation orientation, aka rotate axis
    rotateNames[0] = "rotateAxisX";
    rotateNames[1] = "rotateAxisY";
    rotateNames[2] = "rotateAxisZ";

    iJoint.getScaleOrientation(vals, eRotateAxisOrder);
    if (util::getRotOrder(eRotateAxisOrder, rotOrder[0], rotOrder[1], rotOrder[2]))
    {
        addRotate(iJoint, "rotateAxis", rotateNames, rotOrder,
            Alembic::AbcGeom::kRotateOrientationHint, iForceStatic, true,
            mSample, mAnimChanList, mRotateAxisOpIndex);
    }

    // inspect the scale
    addScale(iJoint, "scale", "scaleX", "scaleY", "scaleZ", false,
        iForceStatic, forceAnimated, mSample, mAnimChanList);

    // remember current rotation
    if (mFilterEulerRotations)
    {
        double xx(0), yy(0), zz(0);

        // there are 2 rotation order enum definitions:
        //     MEulerRotation::RotationOrder = MTransformationMatrix::RotationOrder-1
        if (getSampledRotation( mSample, mJointOrientOpIndex, xx, yy, zz ))
        {
            mPrevJointOrientSolution.setValue(xx, yy, zz, (MEulerRotation::RotationOrder)(eJointOrientOrder-1));
        }

        if (getSampledRotation( mSample, mRotateOpIndex, xx, yy, zz ))
        {
            mPrevRotateSolution.setValue(xx, yy, zz, (MEulerRotation::RotationOrder)(eRotOrder-1));
        }

        if (getSampledRotation( mSample, mRotateAxisOpIndex, xx, yy, zz ))
        {
            mPrevRotateAxisSolution.setValue(xx, yy, zz, (MEulerRotation::RotationOrder)(eRotateAxisOrder-1));
        }
    }
}
