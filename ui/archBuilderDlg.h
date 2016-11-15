#ifndef _archbuilderdlg_h_
#define _archbuilderdlg_h_

#include "ui_archBuilderDlg.h"
#include <QDialog>

class ArchBuilderDlg : public QDialog, public Ui::ArchBuilderDlgUI
{
private:
	void init();
public:
	ArchBuilderDlg(QWidget *parent) : QDialog(parent) {
		setupUi(this);
		init();
	}
};

#endif
