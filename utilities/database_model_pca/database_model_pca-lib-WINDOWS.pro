
opencv{
QMAKE_LIBDIR += $(OPENCV2)/lib/
    dbg{
    LIBS += cxcore210d.lib cv210d.lib cvaux210d.lib
  }else{
    LIBS += cxcore210.lib cv210.lib cvaux210.lib 
  }
}

openmesh{
  dbg{
QMAKE_LIBDIR += $(OPENMESH)/OpenMesh/Win/msvc7/Core/Debug/
   LIBS += OpenMeshCored.lib
  }else{
  QMAKE_LIBDIR += $(OPENMESH)/OpenMesh/Win/msvc7/Core/Release/
  LIBS += OpenMeshCore.lib
  }
}
