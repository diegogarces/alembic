//-*****************************************************************************
//
// Copyright (c) 2009-2010, Industrial Light & Magic,
//   a division of Lucasfilm Entertainment Company Ltd.
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
// *       Neither the name of Industrial Light & Magic nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
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

#ifndef _AlembicTako_PolyMesh_h_
#define _AlembicTako_PolyMesh_h_

#include <AlembicTako/Foundation.h>

namespace AlembicTako {

//-*****************************************************************************
// See Transform.h for commentary on Alembic vs. Tako structures.
//-*****************************************************************************

//-*****************************************************************************
// Make string traits.
ALEMBIC_STRING_TRAIT( PolyMeshProtocolStrait, "TakoPolyMesh_v0001" );
ALEMBIC_STRING_TRAIT( PolyMeshPointsStrait, ".points" );
ALEMBIC_STRING_TRAIT( PolyMeshNormalsStrait, ".normals" );
ALEMBIC_STRING_TRAIT( PolyMeshFacePointsStrait, ".facePoints" );
ALEMBIC_STRING_TRAIT( PolyMeshFaceListStrait, ".faceList" );

//-*****************************************************************************
// Make property traits.
ALEMBIC_PROPERTY_TRAIT( PolyMeshPointsTrait,
                        V3fArrayProperty,
                        PolyMeshPointsStrait );

ALEMBIC_PROPERTY_TRAIT( PolyMeshNormalsTrait,
                        V3fArrayProperty,
                        PolyMeshNormalsStrait );

ALEMBIC_PROPERTY_TRAIT( PolyMeshFacePointsTrait,
                        IntArrayProperty,
                        PolyMeshFacePointsStrait );

ALEMBIC_PROPERTY_TRAIT( PolyMeshFaceListTrait,
                        IntArrayProperty,
                        PolyMeshFaceListStrait );

//-*****************************************************************************
// Make amalgams.
ALEMBIC_AMALGAM_TRAIT4( PolyMeshTrait,
                        PolyMeshPointsTrait, points,
                        PolyMeshNormalsTrait, normals,
                        PolyMeshFacePointsTrait, facePoints,
                        PolyMeshFaceListTrait, faceList );

//-*****************************************************************************
// Make trait objects.
ALEMBIC_TRAIT_OBJECT( PolyMeshObj,
                      PolyMeshTrait,
                      PolyMeshProtocolStrait );


} // End namespace AlembicTako

#endif