//-*****************************************************************************
//
// Copyright (c) 2009-2010,
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

#ifndef _Alembic_MD5Hash_Foundation_h_
#define _Alembic_MD5Hash_Foundation_h_

#include <boost/static_assert.hpp>
#include <boost/detail/endian.hpp>
#include <boost/spirit/home/support/detail/integer/endian.hpp>
#include <boost/cstdint.hpp>
#include <boost/format.hpp>

#include <functional>

// These seem not to be installed.
// #include <unordered_map>
// #include <unordered_set>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <half.h>
#include <utility>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <limits>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

namespace Alembic {
namespace MD5Hash {

//-*****************************************************************************
typedef boost::uint64_t UINT8;
typedef boost::uint32_t UINT4;
typedef boost::uint8_t UCHAR;

//-*****************************************************************************
template <class POD_TYPE>
inline void StoreLittleEndian( void *destination,
                               const POD_TYPE &source )
{
    boost::detail::store_little_endian<POD_TYPE,sizeof(POD_TYPE)>(
        destination, source );
}

//-*****************************************************************************
#define MD5HASH_STATIC_ASSERT BOOST_STATIC_ASSERT

//-*****************************************************************************
#ifdef BOOST_BIG_ENDIAN

#define MD5HASH_BIG_ENDIAN
#undef MD5HASH_LITTLE_ENDIAN

#else

#define MD5HASH_LITTLE_ENDIAN
#undef MD5HASH_BIG_ENDIAN

#endif


} // End namespace MD5Hash
} // End namespace Alembic

#endif