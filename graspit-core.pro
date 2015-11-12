# list of core source files for GraspIt!
# included in both graspit-WINDOWS.pro and graspit-LINUX.pro
# not for stand-alone use.
# ----------------------general settings--------------------
QT += qt3support network
CONFIG += qt \
    warn_on \
    exceptions

# assistant
INCLUDEPATH += $(QTDIR)/include \
    $(COINDIR)/include \
    qhull
DESTDIR = bin
UI_DIR = ui

# ---------------------- Graspit source code ----------------------------------------------
INCLUDEPATH += src \
    src/Collision \
    include \
    include/math \
    include/Planner \
    include/EGPlanner \
    ui \
    ui/Planner \
    ui/EGPlanner
DEPENDPATH += src \
    src/Collision \
    include \
    include/math \
    include/Planner \
    include/EGPlanner \
    ui \
    ui/Planner \
    ui/EGPlanner
HEADERS += src/marchingcube/CIsoSurface.h \
    src/marchingcube/Vectors.h \
    include/EGPlanner/egPlannerUtils.h \
    include/EGPlanner/egPlannerUtilsImpl.h \
    include/barrett.h \
    include/body.h \
    include/bbox.h \
    include/bbox_inl.h \
    include/contact.h \
    include/contactSetting.h \
    include/debug.h \
    include/dof.h \
    include/dynamics.h \
    include/eigenGrasp.h \
    include/gloveInterface.h \
    include/grasp.h \
    include/graspControllers.h \
    include/graspRecord.h \
    include/gws.h \
    include/gwsprojection.h \
    include/ivmgr.h \
    include/jacobian.h \
    include/joint.h \
    include/kinematicChain.h \
    include/lmiOptimizer.h \
    include/material.h \
    include/matvec3D.h \
    include/matvecIO.h \
    include/maxdet.h \
    include/mcGrip.h \
    include/mytools.h \
    include/profiling.h \
    include/puma560.h \
    include/qhull_mutex.h \
    include/quality.h \
    include/pr2Gripper.h \
    include/m7.h \
    include/m7tool.h \
    include/robonaut.h \
    include/robot.h \
    include/humanHand.h \
    include/SensorInterface.h \
    include/snake.h \
    include/snakeGripper.h \
    include/SoArrow.h \
    include/SoComplexShape.h \
    include/SoTorquePointer.h \
    include/scanSimulator.h \
    include/triangle_inl.h \
    include/triangle.h \
    include/worldElement.h \
    include/world.h \
    include/graspitGUI.h \
    include/graspitServer.h \
    include/graspPlanningService.h \
    include/graspitApp.h \
    include/dynJoint.h \
    include/arch.h \
    include/math/matrix.h \
    src/Collision\collisionInterface.h \
    src/Collision\collisionStructures.h \
    include/Planner/grasp_visualization.h \
    include/Planner/grasp_tester.h \
    include/Planner/grasp_preshape.h \
    include/Planner/grasp_presenter.h \
    include/Planner/grasp_planner.h \
    include/Planner/grasp_manager.h \
    include/Planner/grasp_grasps.h \
    include/Planner/grasp_directions.h \
    include/Planner/grasp_coordinates.h \
    include/EGPlanner/search.h \
    include/EGPlanner/simAnn.h \
    include/EGPlanner/searchState.h \
    include/EGPlanner/searchStateImpl.h \
    include/EGPlanner/searchEnergy.h \
    include/EGPlanner/onLinePlanner.h \
    include/EGPlanner/egPlanner.h \
    include/EGPlanner/simAnnPlanner.h \
    include/EGPlanner/guidedPlanner.h \
    include/EGPlanner/loopPlanner.h \
    include/EGPlanner/timeTest.h \
    include/EGPlanner/graspTesterThread.h \
    include/EGPlanner/onLineGraspInterface.h \
    include/EGPlanner/listPlanner.h \
    include/FitParabola.h \
    include/shadow.h \
    src/SemanticPlanner/SemanticMap.h \
    src/SemanticPlanner/SemanticGrasp.h \
    include/ContactPDModels.h
SOURCES += src/marchingcube/CIsoSurface.cpp \
    src/marchingcube/Vectors.cpp \
    src/EGPlanner/egPlannerUtils.cpp \
    src/arch.cpp \
    src/barrett.cpp \
    src/bbox.cpp \
    src/body.cpp \
    src/contact.cpp \
    src/contactSetting.cpp \
    src/dof.cpp \
    src/dynamics.cpp \
    src/dynJoint.cpp \
    src/eigenGrasp.cpp \
    src/gloveInterface.cpp \
    src/grasp.cpp \
    src/graspitGUI.cpp \
    src/graspitServer.cpp \
    src/graspitApp.cpp \
    src/graspControllers.cpp \
    src/graspPlanningService.cpp \
    src/graspRecord.cpp \
    src/gws.cpp \
    src/gwsprojection.cpp \
    src/humanHand.cpp \
    src/ivmgr.cpp \
    src/jacobian.cpp \
    src/joint.cpp \
    src/kinematicChain.cpp \
    src/lmiOptimizer.cpp \
    src/main.cpp \
    src/material.cpp \
    src/matvec.cpp \
    src/matvecIO.cpp \
    src/maxdet_src.cpp \
    src/mcGrip.cpp \
    src/mytools.cpp \
    src/profiling.cpp \
    src/pr2Gripper.cpp \
    src/m7.cpp \
    src/m7tool.cpp \
    src/puma560.cpp \
    src/quality.cpp \
    src/robonaut.cpp \
    src/robot.cpp \
    src/scanSimulator.cpp \
    src/shadow.cpp \
    src/SensorInterface.cpp \
    src/snake.cpp \
    src/snakeGripper.cpp \
    src/SoArrow.cpp \
    src/SoComplexShape.cpp \
    src/SoTorquePointer.cpp \
    src/triangle.cpp \
    src/world.cpp \
    src/worldElement.cpp \
    src/math/matrix.cpp \
    src/Collision/collisionInterface.cpp \
    src/Planner/grasp_visualization.cpp \
    src/Planner/grasp_tester.cpp \
    src/Planner/grasp_preshape.cpp \
    src/Planner/grasp_presenter.cpp \
    src/Planner/grasp_planner.cpp \
    src/Planner/grasp_manager.cpp \
    src/Planner/grasp_grasps.cpp \
    src/Planner/grasp_directions.cpp \
    src/Planner/grasp_coordinates.cpp \
    src/EGPlanner/simAnn.cpp \
    src/EGPlanner/searchState.cpp \
    src/EGPlanner/searchStateImpl.cpp \
    src/EGPlanner/searchEnergy.cpp \
    src/EGPlanner/onLinePlanner.cpp \
    src/EGPlanner/egPlanner.cpp \
    src/EGPlanner/simAnnPlanner.cpp \
    src/EGPlanner/guidedPlanner.cpp \
    src/EGPlanner/loopPlanner.cpp \
    src/EGPlanner/timeTest.cpp \
    src/EGPlanner/graspTesterThread.cpp \
    src/EGPlanner/onLineGraspInterface.cpp \
    src/EGPlanner/listPlanner.cpp \
    src/SemanticPlanner/SemanticMap.cpp \
    src/SemanticPlanner/SemanticGrasp.cpp \
    src/ContactPDModels.cpp

# --------------------------------------- Implementations of the collision interface ---------------------------------
pqp_collision { 
    DEFINES += PQP_COLLISION
    INCLUDEPATH += src/Collision/PQP \
        PQP-VCOLLIDE/src \
        PQP-VCOLLIDE/PQP_v1.1/src \
        PQP-VCOLLIDE/ivcollide
    DEPENDPATH += PQP-VCOLLIDE/include \
        PQP-VCOLLIDE/PQP_v1.1/src \
        PQP-VCOLLIDE/ivcollide
    HEADERS += src/Collision/PQP/PQPCollision.h
    SOURCES += src/Collision/PQP/PQPCollision.cpp \
        PQP-VCOLLIDE/src/VCollide.cpp \
        PQP-VCOLLIDE/src/VInternal.cpp \
        PQP-VCOLLIDE/src/NBody.cpp \
        PQP-VCOLLIDE/src/PairData.cpp \
        PQP-VCOLLIDE/PQP_v1.1/src/Build.cpp \
        PQP-VCOLLIDE/PQP_v1.1/src/BV.cpp \
        PQP-VCOLLIDE/PQP_v1.1/src/PQP.cpp \
        PQP-VCOLLIDE/PQP_v1.1/src/TriDist.cpp \
        PQP-VCOLLIDE/PQP_v1.1/src/Tri.cpp
}
else:graspit_collision { 
    DEFINES += GRASPIT_COLLISION
    INCLUDEPATH += src/Collision/Graspit
    DEPENDPATH += src/Collision/Graspit
    HEADERS += src/Collision/Graspit/collisionModel.h \
        src/Collision/Graspit/collisionAlgorithms.h \
        src/Collision/Graspit/collisionAlgorithms_inl.h \
        src/Collision/Graspit/graspitCollision.h
    SOURCES += src/Collision/Graspit/collisionModel.cpp \
        src/Collision/Graspit/collisionAlgorithms.cpp \
        src/Collision/Graspit/graspitCollision.cpp
}
else:error("Collision detection method not specified")

# --------------------------------------- User interface: main window and dialogs ---------------------------------
INCLUDEPATH += ui \
    ui/Planner \
    ui/EGPlanner
DEPENDPATH += ui \
    ui/Planner \
    ui/EGPlanner
FORMS += ui/mainWindow.ui \
    ui/about.ui \
    ui/archBuilderDlg.ui \
    ui/barrettHandDlg.ui \
    ui/bodyPropDlg.ui \
    ui/contactExaminerDlg.ui \
    ui/eigenGraspDlg.ui \
    ui/gfoDlg.ui \
    ui/gloveCalibrationDlg.ui \
    ui/graspCaptureDlg.ui \
    ui/gwsProjDlgBase.ui \
    ui/qmDlg.ui \
    ui/qualityIndicator.ui \
    ui/settingsDlg.ui \
    ui/Planner/plannerdlg.ui \
    ui/EGPlanner/egPlannerDlg.ui \
    ui/EGPlanner/compliantPlannerDlg.ui \
    ui/taskControlDlg.ui
HEADERS += ui/mainWindow.h \
    ui/archBuilderDlg.h \
    ui/barrettHandDlg.h \
    ui/bodyPropDlg.h \
    ui/contactExaminerDlg.h \
    ui/eigenGraspDlg.h \
    ui/gfoDlg.h \
    ui/gloveCalibrationDlg.h \
    ui/graspCaptureDlg.h \
    ui/gwsProjDlg.h \
    ui/settingsDlg.h \
    ui/qmDlg.h \
    ui/Planner/plannerdlg.h \
    ui/EGPlanner/egPlannerDlg.h \
    ui/EGPlanner/compliantPlannerDlg.h \
    ui/taskControlDlg.h
SOURCES += ui/mainWindow.cpp \
    ui/archBuilderDlg.cpp \
    ui/barrettHandDlg.cpp \
    ui/bodyPropDlg.cpp \
    ui/contactExaminerDlg.cpp \
    ui/eigenGraspDlg.cpp \
    ui/gfoDlg.cpp \
    ui/gloveCalibrationDlg.cpp \
    ui/graspCaptureDlg.cpp \
    ui/gwsProjDlg.cpp \
    ui/qmDlg.cpp \
    ui/settingsDlg.cpp \
    ui/Planner/plannerdlg.cpp \
    ui/EGPlanner/egPlannerDlg.cpp \
    ui/EGPlanner/compliantPlannerDlg.cpp \
    ui/taskControlDlg.cpp

# -------------------------------------- images and resources -------------------------------------------------------
IMAGES = src/images/play.xpm \
    src/images/pause.xpm \
    src/images/splash.jpg \
    src/images/logo.png \
    src/images/nocollide.xpm \
    src/images/collide.xpm \
    src/images/translateTool.xpm \
    src/images/selectTool.xpm \
    src/images/rotateTool.xpm \
    src/images/mark.xpm \
    src/images/prevMark.xpm \
    src/images/filenew.xpm \
    src/images/fileopen.xpm \
    src/images/filesave.xpm \
    src/images/filenew \
    src/images/fileopen \
    src/images/filesave \
    src/images/print \
    src/images/undo \
    src/images/redo \
    src/images/editcut \
    src/images/editcopy \
    src/images/editpaste \
    src/images/searchfind

# -------------------------------------- The TinyXML XML parser ---------------------------------------------------
SOURCES += tinyxml/tinyxmlparser.cpp
HEADERS += tinyxml/tinyxml.h \
    tinyxml/tinystr.h
INCLUDEPATH += tinyxml

# -------------------------------------- PLY file input (non-commercial license) ---------------------------------------
exists(ply) { 
    DEFINES += PLY_READER
    INCLUDEPATH += ply
    DEPENDPATH += ply
    SOURCES += ply/ply.c \
        ply/mesh_loader.cpp
    HEADERS += ply/ply.h \
        ply/mesh_loader.h
}

# -------------------------------------- The Columbia Grasp Database ---------------------------------------------------
cgdb { 
    # library linking for cgdb is in platform specific files
    !exists(src/DBase):error("CGDB interface code not found")
    !exists(src/DBase/DBPlanner):error("DBPlanner code not found")
    !exists($(CGDB_MODEL_ROOT)):error("CGDB_MODEL_ROOT not set, or directory not found")
    QT += sql
    INCLUDEPATH += src/DBase \
        src/DBase/DBPlanner
    DEPENDPATH += src/DBase \
        src/DBase/DBPlanner
    SOURCES += src/DBase/dbaseDlg.cpp \
        src/DBase/dbasePlannerDlg.cpp \
        src/DBase/dbase_grasp.cpp \
        src/DBase/graspit_db_model.cpp \
        src/DBase/graspit_db_grasp.cpp \
        src/DBase/DBPlanner/database.cpp \
        src/DBase/DBPlanner/sql_database_manager.cpp \
        src/DBase/graspit_db_planner.cpp \
        src/DBase/dbaseUtilDlg.cpp \
        src\DBase\TaskDispatcher.cpp \
        src\DBase\GraspPlanningTask.cpp \
        src\DBase\scan_manager.cpp
    HEADERS += src/DBase/dbaseDlg.h \
        src/DBase/dbasePlannerDlg.h \
        src/DBase/dbase_grasp.h \
        src/DBase/graspit_db_model.h \
        src/DBase/graspit_db_grasp.h \
        src/DBase/DBPlanner/grasp.h \
        src/DBase/DBPlanner/model.h \
        src/DBase/DBPlanner/db_manager.h \
        src/DBase/DBPlanner/database.h \
        src/DBase/DBPlanner/sql_database_manager.h \
        src/DBase/graspit_db_planner.h \
        src/DBase/dbaseUtilDlg.h \
        src\DBase\TaskDispatcher.h \
        src\DBase\GraspPlanningTask.h \
        src\DBase\scan_manager.h
    FORMS += src/DBase/dbaseDlg.ui \
        src/DBase/dbasePlannerDlg.ui \
        src/DBase/dbaseUtilDlg.ui
    
    # you can also define BATCH_PROCESSING in order to supress error output which requires user attention
    DEFINES += CGDB_ENABLED \
        BATCH_PROCESSING
}

# --------------------------------------- Arizona Project ---------------------------------------------
arizona { 
    SOURCES += src/arizona/Arizona_Process.cpp \
        src/arizona/Arizona_Raw_Exp.cpp \
        src/arizona/Arizona_Test.cpp \
        src/arizona/arizonaProjectDlg.cpp
    HEADERS += src/arizona/Arizona_Process.h \
        src/arizona/Arizona_Raw_Exp.h \
        src/arizona/Arizona_Test.h \
        src/arizona/arizonaProjectDlg.h
    FORMS += src/arizona/arizonaProjectDlg.ui
    DEFINES += ARIZONA_PROJECT_ENABLED
}

# -------------------------------------- Interfaces to QP solvers ------------------------------------------------------
cgal_qp { 
    DEFINES += CGAL_QP
    SOURCES += src/math/cgal_qp.cpp
    HEADERS += src/math/cgal_qp.h
}
mosek { 
    DEFINES += MOSEK_QP
    SOURCES += src/math/mosek_qp.cpp
    HEADERS += src/math/mosek_qp.h
}

# -------------------------------------- Eigengrids ---------------------
eigengrids { 
    !exists(eigengrids):error(Eigengrids code not present)
    DEFINES += EIGENGRIDS
    INCLUDEPATH += eigengrids
    DEPENDPATH += eigengrids
    
    # dialogs
    FORMS += eigengrids/scanSimDlg.ui \
        eigengrids/eigenGridsDlg.ui
    HEADERS += eigengrids/scanSimDlg.h \
        eigengrids/eigenGridsDlg.h
    SOURCES += eigengrids/scanSimDlg.cpp \
        eigengrids/eigenGridsDlg.cpp
    
    # and the octree itself
    HEADERS += eigengrids/octree.h \
        eigengrids/octreeNodes.h \
        eigengrids/dataTypes.h \
        eigengrids/intersection_obb.h \
        eigengrids/intersection_sphere.h \
        eigengrids/intersection_triangle.h
}

# -------------------------------------- Optimizer ---------------------
SOURCES += optimizer/optimizerDlg.cpp
HEADERS += optimizer/optimizerDlg.h
cgdb { 
    SOURCES += optimizer/eigenTorques.cpp
    HEADERS += optimizer/eigenTorques.h
}
FORMS += optimizer/optimizerDlg.ui
INCLUDEPATH += optimizer
DEPENDPATH += optimizer

# ------------------------------------- Staubli Control ----------------
staubli { 
    hardwarelib { 
        DEFINES += STAUBLI_CONTROL_ENABLED
        SOURCES += Staubli/staubliControlDlg.cpp
        HEADERS += Staubli/staubliControlDlg.h
        FORMS += Staubli/staubliControlDlg.ui
    }
    else:error ("Please define hardware lib")
}

# ------------------------------------- Semantic Planner -----------------
semantic_planner { 
    QT += opengl
    DEFINES += SEMANTIC_PLANNER_ENABLED
    INCLUDEPATH += "c:/dev/OpenMesh-2.0-RC5/src" \
        		   "c:/dev/opencv/2.4/build/include" \
			   "c:/dev/opencv/2.4/build/include/opencv"
    FORMS += src/SemanticPlanner/semanticPlannerDlg.ui
    SOURCES += src\SemanticPlanner\Views.cpp \ # src/SemanticPlanner/SemanticMap.cpp \
    # src/SemanticPlanner/SemanticGrasp.cpp \
        src/SemanticPlanner/semanticPlannerDlg.cpp \
src/SemanticPlanner/shapeCorrespondence.cpp \
    src/graspviewer.cpp

    HEADERS += src\SemanticPlanner\Utilities.h \
        src\SemanticPlanner\Camera_Frustrum.h \
        src\SemanticPlanner\Views.h \ # src/SemanticPlanner/SemanticMap.h \
        src/SemanticPlanner/SemanticGrasp.h \
        src/SemanticPlanner/semanticPlannerDlg.h \
src/SemanticPlanner/shapeCorrespondence.h \
	   include/graspviewer.h

    QMAKE_LFLAGS += "/LIBPATH:C:/dev/opencv/2.4/build/x86/vc9/lib"
    LIBS += opencv_core240.lib \
		 opencv_flann240.lib \
	       opencv_highgui240.lib
    CONFIG(debug, debug|release) { 
        LIBS += "c:/dev/OpenMesh-2.0-RC5/lib/libOpenMeshCored.lib"
        DESTDIR = debug
    }
    CONFIG(release, debug|release) { 
        LIBS += "c:/dev/OpenMesh-2.0-RC5/lib/libOpenMeshCore.lib"
        DESTDIR = release
    }
    DEFINES += _USE_MATH_DEFINES
}
blind_planner { 
    DEFINES += BLIND_PLANNER_ENABLED
    FORMS += src/BlindPlanner/blindPlannerDlg.ui
    SOURCES += src/BlindPlanner/blindPlannerUtil.cpp \
        src/BlindPlanner/handAdjust.cpp \
        src/BlindPlanner/blindPlannerDlg.cpp \
        src/BlindPlanner/graspExperienceBase.cpp \
        src/BlindPlanner/graspExperienceEntry.cpp \
        src/qualityEstimator.cpp \
        src/handControlAPI.cpp \
        src/svm.cpp \
        src/DBase/DBPlanner/model_utils.cpp
        #src/BlindPlanner/localExplore.cpp \
        #src/BlindPlanner/fitSurface.cpp
    HEADERS += src/BlindPlanner/blindPlannerUtil.h \
        src/BlindPlanner/handAdjust.h \
        src/BlindPlanner/blindPlannerDlg.h \
        src/BlindPlanner/graspExperienceBase.h \
        src/BlindPlanner/graspExperienceEntry.h \
        include/qualityEstimator.h \
        include/handControlAPI.h \
        include/svm.h \
        src/DBase/DBPlanner/model_utils.h
        #src/BlindPlanner/localExplore.h \
        #src/BlindPlanner/fitSurface.h

	local_explore {
		DEFINES += LOCAL_EXPLORE_ENABLED
		SOURCES += src/BlindPlanner/localExplore.cpp \
       			src/BlindPlanner/fitSurface.cpp
		HEADERS += src/BlindPlanner/localExplore.h \
				src/BlindPlanner/fitSurface.h
	}

}
uncertainty_planner { 
    DEFINES += UNCERTAINTY_PLANNER_ENABLED
    FORMS += src/UncertaintyPlanner/uncertaintyPlannerDlg.ui
    SOURCES += src/UncertaintyPlanner/uncertaintyPlannerDlg.cpp
    HEADERS += src/UncertaintyPlanner/uncertaintyPlannerUtil.h \
        src/UncertaintyPlanner/uncertaintyPlannerDlg.h
}
strategy_planner {
    DEFINES += STRATEGY_PLANNER_ENABLED
    FORMS += src/StrategyPlanner/strategyPlannerDlg.ui
    SOURCES += src/StrategyPlanner/strategyPlannerDlg.cpp \
			src/StrategyPlanner/environment.cpp
    HEADERS += src/StrategyPlanner/strategyPlannerDlg.h \
			src/StrategyPlanner/environment.h
}

