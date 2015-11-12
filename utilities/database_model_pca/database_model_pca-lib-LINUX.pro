opencv{
QMAKE_LIBDIR += $(OPENCV2)/lib/
    dbg{
    LIBS += -lcxcore -lcv -lcvaux -lhighgui -lml
  }else{
    LIBS += -lcxcore -lcv -lcvaux -lhighgui -lml
  }
}

openmesh{
  dbg{
QMAKE_LIBDIR += $(OPENMESH)/OpenMesh/Core/Debian_gcc4.4_dbg/
   LIBS += -lOpenMesh_Core
  }else{
  QMAKE_LIBDIR += $(OPENMESH)/OpenMesh/Core/Debian_gcc4.4_max/
  LIBS += -lOpenMesh_Core
  }
}
