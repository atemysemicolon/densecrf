//
// Created by prasscvc on 20/06/15.
//

#ifndef CRFNEW_CRFTEST_H
#define CRFNEW_CRFTEST_H

//

//#include <string.h>
//#include <malloc.h>



//#include <dirent.h>


#include "settings.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include "densecrf.h"
#include "common.h"





class CRFTest {


private:
    //definitions:
    int NIT, W, H,M, GW, GH;
    float GT_PROB;
    unsigned char* im;
    MatrixXf unary;
    //DenseCRF2D *crf;
    std::shared_ptr<DenseCRF2D> crf;
    std::string folder_results ="/home/prasscvc/Development/workspace/CRF-Bernat/CRFnew/CRF_results/";
    std::string fn_base;
    std::string folder_energy;
    std::string folder_segment;
    std::string folder_image;
    std::string en_ex;
    std::string seg_ex;
    std::string im_ex;
    VectorXs map;
    bool input_text;
    cv::Mat mseg,mun;


    //auxiliar Functions-------------------
    void saveMap(cv::Mat &prediction, std::string filename);
    cv::Mat rollAnn(cv::Mat &map,int W,int H);
    cv::Mat cvtMap(VectorXs &prediction);
    MatrixXf computeUnary( const cv::Mat energies, int M );
    std::vector<std::string> readmat(std::string filename);
    std::vector<float> parseLine(std::string &line);
    std::vector<cv::Point> getPositions(int superpixel_index, cv::Mat &clusters);
    cv::Mat expandEnergy(cv::Mat &energies, cv::Mat &segments);
    std::vector<std::string> readFile(std::string filename);
    std::vector<std::vector<float>> parseStringFile(std::vector<std::string> &lines);
    cv::Mat parseintomat(std::vector<std::string> &lines);
    void parseStringFile2(std::vector<std::string> &lines, std::vector<std::vector<float>> &vals);



public:
//Main functions----------------------------------------------------------------------------------------------
    void setFolders(std::string fl_energy, std::string fl_segments,std::string fl_image);
    void  setExtensions(std::string enex, std::string segex,std::string imex, bool is_mat);
    void readDatas(std::string filename);
    void loadUnaries();
    void initCRF(int w, int h, int m, int nr_iterations);
    void setDefaultParams();
    void process();
    void saveResults(std::string filename);

};

#endif //CRFNEW_CRFTEST_H
