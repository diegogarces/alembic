cmake -G "Visual Studio 14 2015 Win64" -DCMAKE_INSTALL_PREFIX=..\deploy -DBUILD_SHARED_LIBS=OFF -DPYTHON_LIBRARY=C:\Python27\libs\python27.lib -DPYTHON_INCLUDE_DIR=C:\Python27\include -DBOOST_ROOT=Z:\code\third_party\boost_1_55_0 -DBOOST_LIBRARYDIR=Z:\code\third_party\boost_1_55_0\stage\lib -DUSE_STATIC_BOOST=ON -DILMBASE_ROOT=Z:\code\third_party\alembic\openexr\IlmBase\deploy -DBUILD_SHARED_LIBS=OFF -DALEMBIC_SHARED_LIBS=OFF -DUSE_PYALEMBIC=ON -DPYILMBASE_ROOT=Z:\code\third_party\alembic\openexr\PyIlmBase\deploy -DALEMBIC_PYILMBASE_MODULE_DIRECTORY=Z:\code\third_party\alembic\openexr\PyIlmBase\deploy\lib\python2.7\site-packages -DALEMBIC_ILMBASE_LINK_STATIC=ON -DUSE_TESTS=OFF -DRUNTIME_STATIC=ON ..