//
// Created by prasscvc on 20/06/15.
//

#ifndef CRFNEW_CRFTEST_H
#define CRFNEW_CRFTEST_H

//

//#include <string.h>
//#include <malloc.h>



//#include <dirent.h>

#include <iostream>
#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "densecrf.h"
#include "common.h"
#include "optimization.h"
#include <vector>

class CRFEnergy: public EnergyFunction {
protected:
    VectorXf initial_u_param_, initial_lbl_param_, initial_knl_param_;
    DenseCRF & crf_;
    const ObjectiveFunction & objective_;
    int NIT_;
    bool unary_, pairwise_, kernel_;
    float l2_norm_;
public:
    CRFEnergy( DenseCRF & crf, const ObjectiveFunction & objective, int NIT, bool unary=1, bool pairwise=1, bool kernel=1 );
    void setL2Norm( float norm );
    virtual VectorXf initialValue();
    virtual double gradient( const VectorXf & x, VectorXf & dx );
};


class CRFTest {


private:
    //definitions:
    int NIT, W, H,M,N, GW, GH;
    float GT_PROB;
    unsigned char* im;
    MatrixXf unary;
    VectorXs labeling;
    bool verbose;
    //DenseCRF2D *crf;
    //std::string folder_results ="/home/prasscvc/Development/workspace/CRF-Bernat/CRFnew/CRF_results/";
    //std::string fn_base;
    VectorXs map;
    int file_nr;
    bool save_every_it;
    bool load_every_it;
    std::string folder_cache;
    std::string folder_energy;
    std::string folder_segment;
    std::string folder_image;
    std::string folder_anno;
    std::string en_ex;
    std::string seg_ex;
    std::string im_ex;
    std::string ann_ex;
    bool input_text;
    cv::Mat mseg,mun,annMat;

    //auxiliar Functions-------------------
    void saveMap(cv::Mat &prediction, std::string filename);


    cv::Mat rollAnn(cv::Mat &map,int W,int H);
    cv::Mat cvtMap(VectorXs &prediction);
    MatrixXf computeUnary( const cv::Mat energies, int M );
    std::vector<std::string> readmat(std::string filename);


    std::vector<float> parseLine(std::string &line);


    std::vector<cv::Point> getPositions(int superpixel_index, cv::Mat &clusters);


    cv::Mat expandEnergy(cv::Mat &energies, cv::Mat &segments);


    std::vector<std::string> readFile(std::string filename) ;
    std::vector<std::vector<float>> parseStringFile(std::vector<std::string> &lines);
    cv::Mat parseintomat(std::vector<std::string> &lines);

    void parseStringFile2(std::vector<std::string> &lines, std::vector<std::vector<float>> &vals);

    bool saveParams(VectorXf params, std::string s);

    VectorXf loadParams(std::string s);



public:
//Main functions----------------------------------------------------------------------------------------------

    void initCRF(int w, int h, int m, int nr_iterations);

    void setFolderCache(std::string s);

    void setFolders(std::string fl_energy, std::string fl_segments,std::string fl_image, std::string fl_anno);
    void  setExtensions(std::string enex, std::string segex,std::string imex,std::string annex, bool is_mat);
    void readDatas(std::string filename);

    bool loadUnaries();

    void setDefaultParams();

    void process();

    void saveAllParams(DenseCRF2D &crf, std::string foldername);

    void loadAllParams(DenseCRF2D &crf, std::string foldername);



};

#endif //CRFNEW_CRFTEST_H
