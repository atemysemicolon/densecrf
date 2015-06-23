


#include "CRFTest.h"


int main()
{

    std::string folder_datasetEne = "/home/prasscvc/Development/workspace/CamVid/FEATS/";
    std::string folder_datasetSeg = "/home/prasscvc/Development/workspace/CamVid/SEGS/";
    std::string folder_datasetim = "/home/prasscvc/Development/workspace/Images/";
    std::string energyex = ".boosted.txt";
    std::string segex = ".seg";
    std::string imex = ".ppm";
    std::string filename = "0001TP_006870";
    bool is_enermat=false;
    std::cout<<"starting";
    CRFTest crf;

    crf.initCRF(720,960,6,6);

    crf.setFolders(folder_datasetEne,folder_datasetSeg,folder_datasetim);// ---> class var folder name
    crf.setExtensions(energyex,segex,imex,is_enermat); //can call setInputSource
    crf.readDatas(filename); //--> segments and energy_sup into Mat objs of class
    crf.loadUnaries();
    crf.setDefaultParams();
    crf.process();
    crf.saveResults("/home/prasscvc/Development/workspace/CRF-Bernat/CRFnew/CRF_results/0001TP_006870");

    return 0;

}