


#include "CRFTrain.h"

#include <iostream>
int main()
{
    std::string folder_datasetEne = "/home/prasscvc/Development/workspace/CamVid/FEATS/";
    std::string folder_datasetSeg = "/home/prasscvc/Development/workspace/CamVid/SEGS/";
    std::string folder_datasetim = "/home/prasscvc/Development/workspace/Images/";
    std::string folder_datanno = "/home/prasscvc/Development/workspace/CamVid/GTTXT/";
    std::string energyex = ".boosted.txt";
    std::string segex = ".seg";
    std::string imex = ".ppm";
    std::string annex= ".txt";
    std::string filename = "0001TP_006870";
    bool is_enermat=false;
    std::cout<<"starting";
    CRFTest crf;
    crf.initCRF(720,960,6,6);
    crf.setFolderCache("home/prasscvc/Development/workspace/cache/");
    crf.setFolders(folder_datasetEne,folder_datasetSeg,folder_datasetim,folder_datanno);
    crf.setExtensions(energyex,segex,imex,annex,is_enermat);
    crf.readDatas(filename);
    crf.loadUnaries();
    crf.setDefaultParams();
    crf.process();
    //crf.saveResults("/home/prasscvc/Development/workspace/CRF-Bernat/CRFnew/CRF_results/0001TP_006870");


    return 0;


}