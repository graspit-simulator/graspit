#ifndef _eigengridsdlg_h_
#define _eigengridsdlg_h_

#include "ui_eigenGridsDlg.h"

#include <QDialog>

namespace scan_utils{
	template <typename T> class Octree;
}
class Body;

class EigenGridsDlg : public QDialog, public Ui::EigenGridsDlgUI
{
	Q_OBJECT
private:
	Body *gDisplayBody;
    scan_utils::Octree<float> *mMean;
    scan_utils::Octree<float> *mDisplay;
    std::vector<scan_utils::Octree<float>*> mEigens;

	void init();
	void destroy();
	void reset();
	void updateControls();
	void updateDisplayGrid();
	void showDisplayGrid();
	void createDisplayBody();
	scan_utils::Octree<float>* loadNewOctree();
	void addTriangles(Body *body, std::list<scan_utils::Triangle> *triangles, 
					  float r, float g, float b);

public:
	EigenGridsDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}
	~EigenGridsDlg(){destroy();}

public slots:
	void exitButton_clicked();
	void loadButton_clicked();
	void loadMeanButton_clicked();
	void clearButton_clicked();
	void goButton_clicked();
};

#endif
