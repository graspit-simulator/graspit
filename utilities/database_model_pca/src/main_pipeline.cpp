#include <iostream>
#include "meshHandler.h"
#ifdef USE_OPENMESH
#include "openmesh_meshHandler.h"
#endif
#ifdef USE_OPENCV
#include "opencv_pcaFunctor.h"
#endif
#include <sstream>
#include "sql_database_manager.h"
#include "pcaFunctor.h"
#include <map>


using namespace db_planner;

meshHandler * newMesh(){
	return new openmesh_meshHandler();
}

pcaFunctor * newPCAFunctor(){
	return new opencv_pcaFunctor();
}

bool getModelPCA(Model * m, float pca[9]){
	meshHandler * mh = newMesh();	
	meshHandler * zs = newMesh();
	pcaFunctor * pcFunc = newPCAFunctor();
	if(mh->readMesh(m->GeometryPath()))
		(*pcFunc)(mh, zs, pca);
	else{
	  memset(pca, -1, 9*sizeof(float));
	  std::cout<< "failed to read model " << m->GeometryPath() << std::endl;
	}
	delete mh;
	delete zs;
	delete pcFunc;
	return true;
}

bool updateObjectPCA(SqlDatabaseManager * db, Model * m)
{
	float pca[9];
	if(!getModelPCA(m, pca))
		return false;
	//output PCA to database
	//first make array into a string
	return db->SavePCA(*m, pca);

}

void updateModelList(SqlDatabaseManager * db, std::map<std::string, bool> &file_list){
	std::vector<Model*> model_list;
	db->ModelList(&model_list);
	std::cout <<"Model Num: " << model_list.size() << std::endl;


	//In order to operate on only unique file names, we store the filenames that have been used in a map
	for(std::vector<Model *>::iterator mp = model_list.begin(); mp!= model_list.end(); ++mp){
	  if(file_list.find((*mp)->GeometryPath()) == file_list.end()){
	    if (!updateObjectPCA(db,*mp)){
	      std::cout<< "Failed on object "<< (*mp)->ModelName() << std::endl;
	      file_list[(*mp)->GeometryPath()] = false;
	      
	    }
	  file_list[(*mp)->GeometryPath()] = true;

	  }
	}
}

class plainModelAllocator: public ModelAllocator{
public:
	virtual Model * Get()const{
		return new Model();
	};
	plainModelAllocator(){};
};

int main()
{
	std::cout<<"Starting \n";
  SqlDatabaseManager db("borneo.cs.columbia.edu", 5432, "postgres","super","arm_db", new plainModelAllocator(), NULL);
  std::map<std::string,bool> file_list;;
  if(!db.isConnected())
	  std::cout<< "Failed to connect!\n";
  updateModelList(&db, file_list);
  
  return 1;

}
