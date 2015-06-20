


#include "CRFTest.h"


int main()
{

    std::string folder_dataset = "/home/prasscvc/Development/workspace/CamVid/";

    CRFTest crf;
    crf.initCRF(720,960,6,6);
    crf.loadUnaries(folder_dataset+"FEATS/0001TP_006870.boosted.txt",folder_dataset+"SEGS/0001TP_006870.seg","/home/prasscvc/Development/workspace/Images/0001TP_006870.ppm");
    crf.setDefaultParams();
    crf.process();
    crf.saveResults("/home/prasscvc/Development/workspace/CRF-Bernat/CRFnew/CRF_results/0001TP_006870");

    return 0;

}