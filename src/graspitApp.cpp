//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Andrew T. Miller 
//
// $Id: graspitApp.cpp,v 1.5 2009/03/30 20:42:02 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements GraspItApp, a subclass of QApplication.
*/

#include "graspitApp.h"
#include <QSettings>
#include <QLabel>
#include <QFrame>
#include <QDesktopWidget>

#include "mytools.h"

/*! A pointer to the splash screen, implemented as a QLabel. */
static QLabel *splash = 0;

/*! 
  Reads the registry to determine the location of the main window, and whether
  or not to display the splash screen.  If so, it shows the GraspIt! logo
  splash screen in the center of the screen.
 */
void
GraspItApp::showSplash()
{
    QRect screen = QApplication::desktop()->screenGeometry();
    QSettings config;
    config.insertSearchPath( QSettings::Windows, "/Columbia" );

    QRect mainRect;
    QString keybase = "/GraspIt/0.9/";
    bool show = config.readBoolEntry( keybase + "SplashScreen", TRUE );
    mainRect.setX( config.readNumEntry( keybase + "Geometries/MainwindowX", 0 ) );
    mainRect.setY( config.readNumEntry( keybase + "Geometries/MainwindowY", 0 ) );
    mainRect.setWidth( config.readNumEntry( keybase + "Geometries/MainwindowWidth", 500 ) );
    mainRect.setHeight( config.readNumEntry( keybase + "Geometries/MainwindowHeight", 500 ) );
    screen = QApplication::desktop()->screenGeometry( QApplication::desktop()->screenNumber( mainRect.center() ) );

    if ( show ) {
		splash = new QLabel( 0, "splash",Qt::FramelessWindowHint | Qt::X11BypassWindowManagerHint | Qt::WindowStaysOnTopHint);
	// WStyle_Customize | WStyle_StaysOnTop
	splash->setAttribute(Qt::WA_DeleteOnClose,true);
	splash->setFrameStyle( QFrame::WinPanel | QFrame::Raised );
	splash->setPixmap(load_pixmap( "splash.jpg" ));
	splash->adjustSize();
	splash->setFixedSize(splash->sizeHint());
	splash->setCaption( "GraspIt!" );
	splash->move( screen.center() - QPoint( splash->width() / 2, splash->height() / 2 ) );
	splash->show();
	splash->repaint( FALSE );
	QApplication::flush();
	//	set_splash_status( "Initializing..." );
    }
}

/*! 
  Removes the splash screen, freeing its memory.
*/
void GraspItApp::closeSplash()
{
    if (splash) delete splash;
}
