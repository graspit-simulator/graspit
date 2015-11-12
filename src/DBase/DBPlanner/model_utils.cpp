#include "model_utils.h"
#include "mytools.h"
#include <QNetworkAccessManager>
#include <QUrl>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QEventLoop>
#include <fstream>
#include <QFile>
#include <QTimer>
#include <QDir>
#include <QProcess>
#define GRASPITDBG
#include "debug.h"

bool modelUtils:: repairModelGeometry(db_planner::Model * m){
	QString relativeModelPath = relativePath(QString(m->GeometryPath().c_str()),getenv("CGDB_MODEL_ROOT"));
	QNetworkAccessManager *nwm = new QNetworkAccessManager();
	QString url(getenv("CGDB_MODEL_ROOT_FTP_URL"));
	//concatenate model path to ftp url
	QNetworkReply* reply = nwm->get(QNetworkRequest(QUrl(url + relativeModelPath)));
	
	//set up event loop to wait for bytes ready signal from qnetworkrequest so that 
	//large files do not cause serious performance issues
	//also return on errors or finished signals
	QTimer t;
	t.setSingleShot(true);
	t.setInterval(200000);
	t.start();
	QEventLoop loop;
	QObject::connect(reply, SIGNAL(readyRead()), &loop, SLOT(quit));
	QObject::connect(reply, SIGNAL(error()), &loop, SLOT(quit));
	QObject::connect(reply, SIGNAL(finished()), &loop, SLOT(quit()));
	QObject::connect(&t, SIGNAL(timeout()), &loop, SLOT(quit()));
	//if there was a problem with the url, return false
	if(reply->error() != QNetworkReply::NoError){
		DBGA("Problem with network request");
		return false;
	}
	//if the desired path doesn't exist, create it
	QString geomDir(m->GeometryPath().substr(0,m->GeometryPath().find_last_of('/')).c_str());
	  if(!QDir().exists(geomDir))
	    QDir().mkpath(geomDir);
	//open the new local file for writing
	QFile newGeomFile(m->GeometryPath().c_str());
	newGeomFile.open(QIODevice::ReadWrite);
	if(newGeomFile.error()){
		DBGA("Opening geometry file failed" << m->GeometryPath());	
		return false;
	}
	//while there is no problem and we have not read the end of the file, read 
	//data from the file
	loop.exec();
	while(reply->bytesAvailable()){
		//sleep until there are bytes available
		
		//write available bytes
		if (reply->bytesAvailable()){
			int numBytes = newGeomFile.write(reply->readAll());
			if(numBytes == -1)
				DBGA("Error Writing"); 
		}
		loop.exec();
	}
	
	if(reply->error() != QNetworkReply::NoError){
		DBGA(reply->errorString().toStdString());
		return false;
	}
	newGeomFile.flush();
	newGeomFile.close();
	//delete reply; -- deleted by deleting nwm
	delete nwm;
	return true;
}


bool modelUtils::repairHandGeometry(){
	QNetworkAccessManager *nwm = new QNetworkAccessManager();
	QString url(getenv("HAND_MODEL_ROOT_FTP_URL"));
	//concatenate model path to ftp url
	QNetworkReply* reply = nwm->get(QNetworkRequest(QUrl(url + "/models.zip")));
	
	//set up event loop to wait for bytes ready signal from qnetworkrequest so that 
	//large files do not cause serious performance issues
	//also return on errors or finished signals
	QTimer t;
	t.setSingleShot(true);
	t.setInterval(20000);
	t.start();
	QEventLoop loop;
	QObject::connect(reply, SIGNAL(readyRead()), &loop, SLOT(quit));
	QObject::connect(reply, SIGNAL(error()), &loop, SLOT(quit));
	QObject::connect(reply, SIGNAL(finished()), &loop, SLOT(quit()));
	QObject::connect(&t, SIGNAL(timeout()), &loop, SLOT(quit()));
	//if there was a problem with the url, return false
	if(reply->error() != QNetworkReply::NoError){
		DBGA("Problem with network request");
		return false;
	}
	QFile newModelZipFile("models.zip");
	newModelZipFile.open(QIODevice::ReadWrite);
	if(newModelZipFile.error()){
	  DBGA("Opening zip file failed");
		return false;
	}
	//while there is no problem and we have not read the end of the file, read 
	//data from the file
	loop.exec();
	while(reply->bytesAvailable()){
		//sleep until there are bytes available
		
		//write available bytes
		if (reply->bytesAvailable()){
			int numBytes = newModelZipFile.write(reply->readAll());
			if(numBytes == -1)
				DBGA("Error Writing"); 
		}
		loop.exec();
	}
	
	if(reply->error() != QNetworkReply::NoError){
		DBGA(reply->errorString().toStdString());
		return false;
	}
	newModelZipFile.flush();
	newModelZipFile.close();
	//now create a QProcess to unzip it
	//delete reply; -- deleted by deleting nwm
	// zip the temporary directory
	QProcess *zipProc=new QProcess ();
	zipProc->start("unzip",QStringList("./models.zip"));
	if (!zipProc->waitForStarted())
	  {   
	    DBGA("unzip process could not be started");
	  } else
	    // zip could be started
	 if(!zipProc->waitForFinished())
	      {
		//zip proc failed to finish?
		DBGA("unzip process could not finish");
	 } else
	 if (zipProc->exitStatus() == QProcess::CrashExit)
		  {
		    DBGA("unzip process failed");
		  }
	 else{
	   return true;
	     }
	   
   // zip could be started
	delete zipProc;
	delete nwm;
	return false;
}
