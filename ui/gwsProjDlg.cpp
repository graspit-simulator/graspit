//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Andrew T. Miller 
//
// $Id: gwsProjDlg.cpp,v 1.3 2009/03/25 22:10:14 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the GWSProjDlg, the Grasp wrench space projection dialog box.
*/

#include "gwsProjDlg.h"

#include <QVariant>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QImage>
#include <QPixmap>

#include "gws.h"

/* // not needed?
static QPixmap uic_load_pixmap_GWSProjDlg( const QString &name )
{
    const QMimeSource *m = QMimeSourceFactory::defaultFactory()->data( name );
    if ( !m )
	return QPixmap();
    QPixmap pix;
    QImageDrag::decode( m, pix );
    return pix;
}
*/

/*! 
  Constructs a GWSProjDlg which is a child of \a parent, with the 
  name \a name and widget flags set to \a fl.

  The dialog will by default be modeless, unless you set \a modal to
  TRUE to construct a modal dialog.

  A combo box, created to allow the user to choose the type of GWS that the
  projection will be created from, is populated with items from the GWS type
  list.

  A button group with 6 check boxes in it is also created to allow the user
  to choose the coordinates that are fixed.
 */
GWSProjDlg::GWSProjDlg( QWidget * parent) : QDialog( parent )
{
  setupUi(this);
  coordButtonGroup = new QButtonGroup(this);
  coordButtonGroup->setExclusive(false);
  QObject::connect(coordButtonGroup, SIGNAL(buttonClicked(int)),this,SLOT(coordBoxClicked(int)));
  coordButtonGroup->addButton(fxCheckBox);
  coordButtonGroup->addButton(fyCheckBox);
  coordButtonGroup->addButton(fzCheckBox);
  coordButtonGroup->addButton(txCheckBox);
  coordButtonGroup->addButton(tyCheckBox);
  coordButtonGroup->addButton(tzCheckBox);
  coordButtonGroup->setId(fxCheckBox,0);
  coordButtonGroup->setId(fyCheckBox,1);
  coordButtonGroup->setId(fzCheckBox,2);
  coordButtonGroup->setId(txCheckBox,3);
  coordButtonGroup->setId(tyCheckBox,4);
  coordButtonGroup->setId(tzCheckBox,5);
  //coordButtonGroup->hide();
  for (int i=0;GWS::TYPE_LIST[i];i++)
    gwsTypeComboBox->insertItem(QString(GWS::TYPE_LIST[i]));
  OKButton->setEnabled(false);  
}

/*!
  When one of the 6 coordinate check boxes is clicked, this checks to see
  how many are already selected.  It only allows a box to be checked if there
  are fewer than 3 boxes already checked.  If there are 3 checked, the OK
  button is enabled, otherwise it is disabled.  \a whichFixed keeps track of
  the indexes of the currently selected check boxes.  
*/
void GWSProjDlg::coordBoxClicked(int buttonNum)
{
  QAbstractButton *b = coordButtonGroup->button(buttonNum);
  if (b->isChecked()) {
    if (whichFixed.size() < 3) {
      whichFixed.insert(buttonNum);
      if (whichFixed.size() == 3) OKButton->setEnabled(true);
    }
    else
      b->toggle();
  }
  else {
    whichFixed.erase(buttonNum);
    OKButton->setEnabled(false);
  }
}

/*!  
  Stub destructor.
 */
GWSProjDlg::~GWSProjDlg()
{
    // no need to delete child widgets, Qt does it all for us
}

