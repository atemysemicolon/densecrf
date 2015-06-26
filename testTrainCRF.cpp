


#include "CRFTrain.h"

#include <iostream>

std::vector<std::string> filenames ={"0001TP_006690",
                                     "0001TP_006720",
                                     "0001TP_006750",
                                     "0001TP_006780",
                                     "0001TP_006810",
                                     "0001TP_006840",
                                     "0001TP_006870",
                                     "0001TP_006900",
                                     "0001TP_006930",
                                     "0001TP_006960",
                                     "0001TP_006990",
                                     "0001TP_007020",
                                     "0001TP_007050",
                                     "0001TP_007080",
                                     "0001TP_007110",
                                     "0001TP_007140",
                                     "0001TP_007170",
                                     "0001TP_007200",
                                     "0001TP_007230",
                                     "0001TP_007260",
                                     "0001TP_007290",
                                     "0001TP_007320",
                                     "0001TP_007350",
                                     "0001TP_007380",
                                     "0001TP_007410",
                                     "0001TP_007440",
                                     "0001TP_007470",
                                     "0001TP_007500",
                                     "0001TP_007530",
                                     "0001TP_007560",
                                     "0001TP_007590",
                                     "0001TP_007620",
                                     "0001TP_007650",
                                     "0001TP_007680",
                                     "0001TP_007710",
                                     "0001TP_007740",
                                     "0001TP_007770",
                                     "0001TP_007800",
                                     "0001TP_007830",
                                     "0001TP_007860",
                                     "0001TP_007890",
                                     "0001TP_007920",
                                     "0001TP_007950",
                                     "0001TP_007980",
                                     "0001TP_008010",
                                     "0001TP_008040",
                                     "0001TP_008070",
                                     "0001TP_008100",
                                     "0001TP_008130",
                                     "0001TP_008160",
                                     "0001TP_008190",
                                     "0001TP_008220",
                                     "0001TP_008250",
                                     "0001TP_008280",
                                     "0001TP_008310",
                                     "0001TP_008340",
                                     "0001TP_008370",
                                     "0001TP_008400",
                                     "0001TP_008430",
                                     "0001TP_008460",
                                     "0001TP_008490",
                                     "0001TP_008520",
                                     "0001TP_008550",
                                     "0001TP_008580",
                                     "0001TP_008610",
                                     "0001TP_008640",
                                     "0001TP_008670",
                                     "0001TP_008700",
                                     "0001TP_008730",
                                     "0001TP_008760",
                                     "0001TP_008790",
                                     "0001TP_008820",
                                     "0001TP_008850",
                                     "0001TP_008880",
                                     "0001TP_008910",
                                     "0001TP_008940",
                                     "0001TP_008970",
                                     "0001TP_009000",
                                     "0001TP_009030",
                                     "0001TP_009060",
                                     "0001TP_009090",
                                     "0001TP_009120",
                                     "0001TP_009150",
                                     "0001TP_009180",
                                     "0001TP_009210",
                                     "0001TP_009240",
                                     "0001TP_009270",
                                     "0001TP_009300",
                                     "0001TP_009330",
                                     "0001TP_009360",
                                     "0001TP_009390",
                                     "0001TP_009420",
                                     "0001TP_009450",
                                     "0001TP_009480",
                                     "0001TP_009510",
                                     "0001TP_009540",
                                     "0001TP_009570",
                                     "0001TP_009600",
                                     "0001TP_009630",
                                     "0001TP_009660",
                                     "0001TP_009690",
                                     "0001TP_009720",
                                     "0001TP_009750",
                                     "0001TP_009780",
                                     "0001TP_009810",
                                     "0001TP_009840",
                                     "0001TP_009870",
                                     "0001TP_009900",
                                     "0001TP_009930",
                                     "0001TP_009960",
                                     "0001TP_009990",
                                     "0001TP_010020",
                                     "0001TP_010050",
                                     "0001TP_010080",
                                     "0001TP_010110",
                                     "0001TP_010140",
                                     "0001TP_010170",
                                     "0001TP_010200",
                                     "0001TP_010230",
                                     "0001TP_010260",
                                     "0001TP_010290",
                                     "0001TP_010320",
                                     "0001TP_010350",
                                     "0001TP_010380",
                                     "0006R0_f00930",
                                     "0006R0_f00960",
                                     "0006R0_f00990",
                                     "0006R0_f01020",
                                     "0006R0_f01050",
                                     "0006R0_f01080",
                                     "0006R0_f01110",
                                     "0006R0_f01140",
                                     "0006R0_f01170",
                                     "0006R0_f01200",
                                     "0006R0_f01230",
                                     "0006R0_f01260",
                                     "0006R0_f01290",
                                     "0006R0_f01320",
                                     "0006R0_f01350",
                                     "0006R0_f01380",
                                     "0006R0_f01410",
                                     "0006R0_f01440",
                                     "0006R0_f01470",
                                     "0006R0_f01500",
                                     "0006R0_f01530",
                                     "0006R0_f01560",
                                     "0006R0_f01590",
                                     "0006R0_f01620",
                                     "0006R0_f01650",
                                     "0006R0_f01680",
                                     "0006R0_f01710",
                                     "0006R0_f01740",
                                     "0006R0_f01770",
                                     "0006R0_f01800",
                                     "0006R0_f01830",
                                     "0006R0_f01860",
                                     "0006R0_f01890",
                                     "0006R0_f01920",
                                     "0006R0_f01950",
                                     "0006R0_f01980",
                                     "0006R0_f02010",
                                     "0006R0_f02040",
                                     "0006R0_f02070",
                                     "0006R0_f02100",
                                     "0006R0_f02130",
                                     "0006R0_f02160",
                                     "0006R0_f02190",
                                     "0006R0_f02220",
                                     "0006R0_f02250",
                                     "0006R0_f02280",
                                     "0006R0_f02310",
                                     "0006R0_f02340",
                                     "0006R0_f02370",
                                     "0006R0_f02400"
};

int main()
{
    std::string folder_datasetEne = "/home/prassanna/Development/workspace/DCRF_work/CamVid/FEATS/";
    std::string folder_datasetSeg = "/home/prassanna/Development/workspace/DCRF_work/CamVid/SEGS/";
    std::string folder_datasetim = "/home/prassanna/Development/Datasets/CamVid_new/Resplit/Images_ppm/";
    std::string folder_datanno = "/home/prassanna/Development/workspace/DCRF_work/CamVid/GTTXT/";
    std::string folder_cache = "/home/prassanna/Development/workspace/densecrf_bernat/Params_camvid/";
    std::string energyex = ".boosted.txt";
    std::string segex = ".seg";
    std::string imex = ".ppm";
    std::string annex= ".txt";

    CRFTest crf;
    crf.initCRF(720, 960, 6, 6);
    crf.setFolderCache("/home/prassanna/Development/workspace/densecrf_bernat/Params_camvid/");

    for(int i=0;i<filenames.size();i++) {
        std::cout<<"####################################################################"<<std::endl;
        std::cout<<"On FILE : "<<i<<" of "<<filenames.size()<<std::endl;
        std::cout<<"####################################################################"<<std::endl;
        std::string filename = filenames[i];
        bool is_enermat = false;
        std::cout << "starting"<<std::endl;
        crf.setFolders(folder_datasetEne, folder_datasetSeg, folder_datasetim, folder_datanno);
        crf.setExtensions(energyex, segex, imex, annex, is_enermat);
        crf.readDatas(filename);
        crf.loadUnaries();
        crf.setDefaultParams();
        crf.process();
    }
    //crf.saveResults("/home/prasscvc/Development/workspace/CRF-Bernat/CRFnew/CRF_results/0001TP_006870");


    return 0;


}