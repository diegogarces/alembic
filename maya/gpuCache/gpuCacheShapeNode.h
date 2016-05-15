#ifndef _gpuCacheShapeNode_h_
#define _gpuCacheShapeNode_h_

//-
//**************************************************************************/
// Copyright 2012 Autodesk, Inc. All rights reserved. 
//
// Use of this software is subject to the terms of the Autodesk 
// license agreement provided at the time of installation or download, 
// or which otherwise accompanies this software in either electronic 
// or hard copy form.
//**************************************************************************/
//+

#include <gpuCacheGeometry.h>
#include <gpuCacheMaterial.h>
#include <CacheReader.h>

#include <maya/MPxSurfaceShape.h>
#include <maya/MPxSurfaceShapeUI.h>
#include <maya/MMessage.h>
#include <maya/MPxGeometryData.h>

#include <boost/shared_ptr.hpp>
#include <gpuCacheSpatialSubdivision.h>
#include <vector>


namespace{

	using namespace GPUCache;

	/*
	This class is used to cache pointers to vertex/index buffers
	and other necessary geometry information to be used by the 
	acceleration structures ( required by the make live functionality)
	*/
	class BufferCache{
	public:
		BufferCache(double seconds){fBufferReadTime = seconds; fUseCachedBuffers = false; fTotalNumTris=0; fTotalNumVerts=0; fNumShapes=0;}
		~BufferCache(){
		}

		std::vector<MMatrix> fXFormMatrix;
		std::vector<MMatrix> fXFormMatrixInverse;

		std::vector<IndexBuffer::ReadInterfacePtr> fTriangleVertIndices;
		std::vector<IndexBuffer::ReadInterfacePtr> fEdgeVertIndices;
		std::vector<VertexBuffer::ReadInterfacePtr> fPositions;
		std::vector<size_t> fNumTriangles;
		std::vector<size_t> fNumEdges;
		std::vector<MBoundingBox> fBoundingBoxes;

		unsigned int fNumShapes;
		unsigned int fTotalNumTris;
		unsigned int fTotalNumVerts;
		bool fUseCachedBuffers;
		double fBufferReadTime;
	};
}

namespace GPUCache {


///////////////////////////////////////////////////////////////////////////////
//
// ShapeNode
//
// Keeps track of animated shapes held in a memory cache.
//
////////////////////////////////////////////////////////////////////////////////


class ShapeNode : public MPxSurfaceShape
{
public:

    static void* creator();
    static MStatus initialize();
    static MStatus uninitialize();
    static MStatus init3dViewPostRenderCallbacks();

    static const MTypeId id;
    static const MString drawDbClassificationGeometry;
    static const MString drawDbClassificationSubScene;
    static const MString drawRegistrantId;
    static MObject aCacheFileName;
	static MObject aRenderFileNames;
    static MObject aCacheGeomPath;
    static MObject aBackfaceCullingEnabled;
    static MObject aCacheTime;
	static MObject aOffsetTime;
    static MObject aScalePivot;
    static MObject aRotatePivot;
    static MObject aScalePivotTranslate;
    static MObject aRotatePivotTranslate;
	static MObject aScaleBoundingBox;
    static MObject aOverrideOpaque;
	static const char* nodeTypeName;
	static const char* selectionMaskName;
    bool fBackfaceCullingEnabled;

    enum BackgroundReadingState {
        kReadingHierarchyInProgress,
        kReadingShapesInProgress,
        kReadingDone
    };

    ShapeNode();
    ~ShapeNode();

    virtual void postConstructor();

    virtual bool isBounded() const;
    virtual MBoundingBox boundingBox() const;

	virtual bool getInternalValueInContext(const MPlug& plug,
        MDataHandle& dataHandle, MDGContext& ctx);
    virtual bool setInternalValueInContext(const MPlug& plug,
        const MDataHandle& dataHandle, MDGContext& ctx);
    virtual void copyInternalData(MPxNode* source);

    virtual MStringArray	getFilesToArchive( bool shortName = false,
                                               bool unresolvedName = false,
                                               bool markCouldBeImageSequence = false ) const;
	virtual bool match( const MSelectionMask & mask,
		const MObjectArray& componentList ) const;

	virtual MSelectionMask getShapeSelectionMask() const;

	virtual bool excludeAsPluginShape() const;

    void refreshCachedGeometry();
    
    const GPUCache::SubNode::Ptr& getCachedGeometry() const;

    const GPUCache::MaterialGraphMap::Ptr& getCachedMaterial() const;

    const BackgroundReadingState backgroundReadingState() const;
    
    // Callback when the gpuCache node is removed from model (delete)
    void removedFromModelCB();

	bool canMakeLive() const;

	bool readBuffers(const SubNode::Ptr subNode, double seconds)const;

	void closestPoint(const MPoint &toThisPoint, MPoint &theClosestPoint, double tolerance = 0.1);
	MStatus closestIntersectWithNorm (const MPoint &raySource, const MVector &rayDirection, MPoint &point, MVector &normal );

	bool closestPoint( const MPoint &raySource, const MVector &rayDirection, MPoint &theClosestPoint, MVector &theClosestNormal, bool findClosestOnMiss, double tolerance=MPoint_kTol);

	unsigned int getIntersectionAccelerator(const gpuCacheIsectAccelParams& accelParams, double seconds) const;

    void applyPivots(GPUCache::Pivots::Ptr pivots) const;

private:
    // Prohibited and not implemented.
    ShapeNode(const ShapeNode& obj);
    const ShapeNode& operator=(const ShapeNode& obj);

    bool setInternalValues(const MString& newFileName,
                           const MString& newGeomPath,
                           const bool newBackFaceCullingEnabled);
	virtual	void getExternalContent(MExternalContentInfoTable& table) const;
	virtual	void setExternalContent(const MExternalContentLocationTable& table);
	
	bool	getEdgeSnapPoint(const MPoint &rayPoint, const MVector &rayDirection, MPoint &theClosestPoint);
    void applyPivotsDeffered(GPUCache::Pivots::Ptr pivots) const;

    mutable MString fCacheFileName;
    mutable MString fCacheGeomPath;
	
	mutable BufferCache* fBufferCache;
	mutable std::vector<gpuCacheSpatialSubdivision *> fSpatialSub;

    mutable GPUCache::SubNode::Ptr                   fCachedGeometry;
    mutable GPUCache::MaterialGraphMap::Ptr          fCachedMaterial;
    mutable GPUCache::Pivots::Ptr                    fCachedPivots;
    mutable GlobalReaderCache::CacheReaderProxy::Ptr fCacheReaderProxy;
    mutable BackgroundReadingState                   fBackgroundReadingState;

    MCallbackId fRemoveFromModelCallbackId;
    
    static MCallbackId fsModelEditorChangedCallbackId;
};


///////////////////////////////////////////////////////////////////////////////
//
// DisplayPref
//
// Keeps track of the display preference.
//
////////////////////////////////////////////////////////////////////////////////

class DisplayPref
{
public:
    enum WireframeOnShadedMode {
        kWireframeOnShadedFull,
        kWireframeOnShadedReduced,
        kWireframeOnShadedNone
    };

    static WireframeOnShadedMode wireframeOnShadedMode();

    static MStatus initCallback();
    static MStatus removeCallback();

private:
    static void displayPrefChanged(void*);

    static WireframeOnShadedMode fsWireframeOnShadedMode;
    static MCallbackId fsDisplayPrefChangedCallbackId;
};


///////////////////////////////////////////////////////////////////////////////
//
// ShapeUI
//
// Displays animated shapes held in a memory cache.
//
////////////////////////////////////////////////////////////////////////////////

class ShapeUI : public MPxSurfaceShapeUI
{
public:

    static void* creator();

    ShapeUI();
    ~ShapeUI();

    virtual void getDrawRequests(const MDrawInfo & info,
                                 bool objectAndActiveOnly,
                                 MDrawRequestQueue & queue);

    // Viewport 1.0 draw
    virtual void draw(const MDrawRequest & request, M3dView & view) const;
    
	virtual bool snap(MSelectInfo &snapInfo) const;
	virtual bool select(MSelectInfo &selectInfo, MSelectionList &selectionList,
                        MPointArray &worldSpaceSelectPts ) const;


private:
    // Prohibited and not implemented.
	ShapeUI(const ShapeUI& obj);
	const ShapeUI& operator=(const ShapeUI& obj);

	static MPoint getPointAtDepth( MSelectInfo &selectInfo,double depth);

    // Helper functions for the viewport 1.0 drawing purposes.
    void drawBoundingBox(const MDrawRequest & request, M3dView & view ) const;
    void drawWireframe(const MDrawRequest & request, M3dView & view ) const;
    void drawShaded(const MDrawRequest & request, M3dView & view, bool depthOffset ) const;
    
	// Draw Tokens
	enum DrawToken {
        kBoundingBox,
		kDrawWireframe,
        kDrawWireframeOnShaded,
		kDrawSmoothShaded,
		kDrawSmoothShadedDepthOffset
	};
};

}

#endif
