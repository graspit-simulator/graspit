# Try to find SoQt
# Once done this will define
#
# SOQT_LIBRARY_FOUND - if Soqt3d is found
# SOQT_CXXFLAGS - extra flags
# SOQT_INCLUDE_DIRS - include directories
# SOQT_LINK_DIRS - link directories
# SOQT_LIBRARY_RELEASE - the relase version
# SOQT_LIBRARY_DEBUG - the debug version
# SOQT_LIBRARY - a default library, with priority debug.



# --- First, try to find relevant headers with find_path and find_library,
# in order to cause a cmake failure if libraries are not there. 

find_path(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
	${CMAKE_INCLUDE_PATH}
	$ENV{COIN3DDIR}/include
	/usr/local/soqt/include
	/usr/local/SoQt/include
	/usr/local/include
	/usr/include
	$ENV{ProgramFiles}/SoQt-1/include
	$ENV{ProgramFiles}/Coin3D-2/include
	$ENV{COINDIR}/include
)

if (SOQT_INCLUDE_DIR)
	message(STATUS "Looking for SoQt headers -- found " ${SOQT_INCLUDE_DIR}/Inventor/Qt/SoQt.h)
		set(SOQT_INCLUDE_DIR_FOUND 1 CACHE INTERNAL "SoQt headers found")
else (SOQT_INCLUDE_DIR)
	message(SEND_ERROR 
	"Looking for SoQt headers -- not found"
	"Please install SoQt http://www.coin3d.org/ or adjust CMAKE_INCLUDE_PATH"
	"e.g. cmake -DCMAKE_INCLUDE_PATH=/path-to-SoQt/include ...")
endif (SOQT_INCLUDE_DIR)


find_library(SOQT_LIBRARY_RELEASE
	NAMES SoQt4 soqt1 SoQt
	PATHS
	${CMAKE_LIBRARY_PATH}
	$ENV{COIN3DDIR}/lib
	/usr/local/soqt/lib
	/usr/local/SoQt/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramFiles}/SoQt-1/lib
	$ENV{ProgramFiles}/Coin3D-2/lib
	$ENV{COINDIR}/lib
)

find_library(SOQT_LIBRARY_DEBUG
    NAMES SoQt4d SoQtd soqt1d 
    PATHS
    ${CMAKE_LIBRARY_PATH}
    $ENV{COIN3DDIR}/lib
    /usr/local/soqt/lib
    /usr/local/SoQt/lib
    /usr/local/lib
    /usr/lib/debug
    $ENV{ProgramFiles}/SoQt-1/lib
    $ENV{ProgramFiles}/Coin3D-2/lib
    $ENV{COINDIR}/lib
)

if (SOQT_LIBRARY_RELEASE)
	message(STATUS "Looking for SoQt library -- found " ${SOQT_LIBRARY_RELEASE})
else (SOQT_LIBRARY_RELEASE)
 	message(SENDL_ERROR 
	"Looking for SoQt library -- not found"
	"Please install SoQt http://www.coin3d.org/ or adjust CMAKE_LIBRARY_PATH"
	"e.g. cmake -DCMAKE_LIBRARY_PATH=/path-to-SoQt/lib ..."
    "or try COIN3DDIR with the correct value")
endif (SOQT_LIBRARY_RELEASE)

if (SOQT_LIBRARY_DEBUG)
    message(STATUS "Looking for SoQt library debug -- found " ${SOQT_LIBRARY_DEBUG})
else (SOQT_LIBRARY_DEBUG)
    message(STATUS 
    "Looking for SoQt library debug -- not found")
endif (SOQT_LIBRARY_DEBUG)





#--- now, use soqt-config in order to get all settings right:


# use soqt-config
find_program(SOQT_CONFIG_EXECUTABLE NAMES soqt-config DOC "soqt-config executable")
mark_as_advanced(SOQT_CONFIG_EXECUTABLE)

if(SOQT_CONFIG_EXECUTABLE)
  set(SOQT_LIBRARY_FOUND 1)

  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --cppflags
    OUTPUT_VARIABLE _soqtconfig_cppflags
    RESULT_VARIABLE _soqtconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _soqtconfig_cppflags "${_soqtconfig_cppflags}")
  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --includedir
    OUTPUT_VARIABLE _soqtconfig_includedir
    RESULT_VARIABLE _soqtconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _soqtconfig_includedir "${_soqtconfig_includedir}")
  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --ldflags
    OUTPUT_VARIABLE _soqtconfig_ldflags
    RESULT_VARIABLE _soqtconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _soqtconfig_ldflags "${_soqtconfig_ldflags}")
  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --libs
    OUTPUT_VARIABLE _soqtconfig_libs
    RESULT_VARIABLE _soqtconfig_failed)

  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _soqtconfig_ldirs "${_soqtconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L" "" _soqtconfig_ldirs "${_soqtconfig_ldirs}")
  
  # on Mac OSX -L is in both ldflags and libs:
  # $ soqt-config --libs
  # -lSoQt -L/opt/local/lib -lQtOpenGL -lQtGui -lQtCore -lCoin -lpthread
  # $ soqt-config --ldflags
  # -L/usr/local/lib -Wl,-framework,OpenGL -Wl,-multiply_defined,suppress
  # $ ls /opt/local/lib/ | grep "QtO"
  # libQtOpenGL.4.7.4.dylib
  # libQtOpenGL.4.7.dylib
  # libQtOpenGL.4.dylib
  # libQtOpenGL.dylib
  # libQtOpenGL.la
  # libQtOpenGL.prl
  # $ ls /usr/local/lib/ | grep "QtO"
  # QtOpenGL.framework
  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _soqtconfig_ldirs2 "${_soqtconfig_libs}")
  string(REGEX REPLACE "(^| )-L" "" _soqtconfig_ldirs2 "${_soqtconfig_ldirs2}")

  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _soqtconfig_libs "${_soqtconfig_libs}")
  string(REGEX REPLACE "(^| )-l" "" _soqtconfig_libs "${_soqtconfig_libs}")

  string(REGEX REPLACE "(^| )-l([./+-_\\a-zA-Z]*)" " " _soqtconfig_ldflags "${_soqtconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L([./+-_\\a-zA-Z]*)" " " _soqtconfig_ldflags "${_soqtconfig_ldflags}")

  separate_arguments(_soqtconfig_includedir)

  set( SOQT_CXXFLAGS "${_soqtconfig_cppflags}" )
  set( SOQT_LINK_FLAGS "${_soqtconfig_ldflags}" )
  set( SOQT_INCLUDE_DIRS ${_soqtconfig_includedir})
  set( SOQT_LINK_DIRS ${_soqtconfig_ldirs} ${_soqtconfig_ldirs2})
  set( SOQT_LIBRARY ${_soqtconfig_libs})
  set( SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY})
  set( SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY})
else(SOQT_CONFIG_EXECUTABLE)
  # soqt include files in local directory
  if( MSVC )
    set(SOQT_LIBRARY_FOUND 1)
    set( SOQT_CXXFLAGS "-DQT3_SUPPORT -DSOQT_DLL")
    set( SOQT_LINK_FLAGS "")
    set( SOQT_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/msvc_soqt/include")
    set( SOQT_LINK_DIRS "${CMAKE_SOURCE_DIR}/msvc_soqt/lib" )

    if( MSVC70 OR MSVC71 OR MSVC80 )
      set(SOQT_LIBRARY soqt1.5-vc80-mt)
	  elseif(MSVC90)
      set(SOQT_LIBRARY soqt1.5-vc90-mt)
    else() # vc100+
      set(SOQT_LIBRARY soqt1.5-vc100-mt)
    endif()
    set( SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY})
    set( SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY})
  else( MSVC )
    set(SOQT_LIBRARY_FOUND 0)
  endif( MSVC )
endif(SOQT_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
    SOQT_LIBRARY_FOUND
    SOQT_CXXFLAGS
    SOQT_LINK_FLAGS
    SOQT_INCLUDE_DIRS
    SOQT_LINK_DIRS
    SOQT_LIBRARY
    SOQT_LIBRARY_RELEASE
    SOQT_LIBRARY_DEBUG
)
