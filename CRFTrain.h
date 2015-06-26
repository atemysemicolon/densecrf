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
#include <sstream>
#include <vector>
#include <iterator>
#include <memory>
#include "settings.h"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cmath>
#include "ppm.h"
#include <fstream>
#include "densecrf.h"
#include "common.h"
#include <opencv2/core/eigen.hpp>
#include "optimization.h"

class CRFEnergy: public EnergyFunction {
protected:
    VectorXf initial_u_param_, initial_lbl_param_, initial_knl_param_;
    DenseCRF & crf_;
    const ObjectiveFunction & objective_;
    int NIT_;
    bool unary_, pairwise_, kernel_;
    float l2_norm_;
public:
    CRFEnergy( DenseCRF & crf, const ObjectiveFunction & objective, int NIT, bool unary=1, bool pairwise=1, bool kernel=1 ):crf_(crf),objective_(objective),NIT_(NIT),unary_(unary),pairwise_(pairwise),kernel_(kernel),l2_norm_(0.f){
        initial_u_param_ = crf_.unaryParameters();
        initial_lbl_param_ = crf_.labelCompatibilityParameters();
        initial_knl_param_ = crf_.kernelParameters();
    }
    void setL2Norm( float norm ) {
        l2_norm_ = norm;
    }
    virtual VectorXf initialValue() {
        VectorXf p( unary_*initial_u_param_.rows() + pairwise_*initial_lbl_param_.rows() + kernel_*initial_knl_param_.rows() );
        p << (unary_?initial_u_param_:VectorXf()), (pairwise_?initial_lbl_param_:VectorXf()), (kernel_?initial_knl_param_:VectorXf());
        return p;
    }
    virtual double gradient( const VectorXf & x, VectorXf & dx ) {
        int p = 0;
        if (unary_) {
            crf_.setUnaryParameters( x.segment( p, initial_u_param_.rows() ) );
            p += initial_u_param_.rows();
        }
        if (pairwise_) {
            crf_.setLabelCompatibilityParameters( x.segment( p, initial_lbl_param_.rows() ) );
            p += initial_lbl_param_.rows();
        }
        if (kernel_)
            crf_.setKernelParameters( x.segment( p, initial_knl_param_.rows() ) );

        VectorXf du = 0*initial_u_param_, dl = 0*initial_u_param_, dk = 0*initial_knl_param_;
        double r = crf_.gradient( NIT_, objective_, unary_?&du:NULL, pairwise_?&dl:NULL, kernel_?&dk:NULL );
        dx.resize( unary_*du.rows() + pairwise_*dl.rows() + kernel_*dk.rows() );
        dx << -(unary_?du:VectorXf()), -(pairwise_?dl:VectorXf()), -(kernel_?dk:VectorXf());
        r = -r;
        if( l2_norm_ > 0 ) {
            dx += l2_norm_ * x;
            r += 0.5*l2_norm_ * (x.dot(x));
        }

        return r;
    }
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
    void saveMap(cv::Mat &prediction, std::string filename)
    {
        cv::FileStorage fs;
        fs.open(filename + "_crf.xml", cv::FileStorage::WRITE);
        fs<<"Ann"<<prediction;
        fs.release();

    }


    cv::Mat rollAnn(cv::Mat &map,int W,int H)
    {
        cv::Mat ann = cv::Mat::zeros(H,W,CV_32SC1);
        for(int i=0;i<map.rows;i++)
        {
            int py,px;
            py = (int)(i/W);
            px = (int)(i%W);
            ann.at<int>(py,px) = map.at<int>(i);
        }
        return ann;
    }

    cv::Mat cvtMap(VectorXs &prediction)
    {

        cv::FileStorage fs;
        int nr_rows = prediction.rows();
        int nr_cols = prediction.cols();
        cv::Mat m;
        cv::eigen2cv(prediction,m);
        std::cout<<m.type();
        cv::Mat p;
        m.convertTo(p, CV_32SC1);
        return p.clone();

    }

    MatrixXf computeUnary( const cv::Mat energies, int M )
    {
        const float u_energy = -log( 1.0 / M );
        const float n_energy = -log( (1.0 - GT_PROB) / (M-1) );
        const float p_energy = -log( GT_PROB );
        MatrixXf r( M, energies.rows );


        cv::Mat m;
        cv::transpose(energies, m);
        m=m*-1; //For all scikit-learnt probs



        cv::cv2eigen(m, r);
        std::cout<<"Unaries copied..."<<std::endl;


        return r;

    }
    std::vector<std::string> readmat(std::string filename) //Read file function
    {
        std::string line;
        std::vector<std::string> lines;
        std::ifstream fin;

        fin.open(filename);
        if(fin.is_open())
        {
            while(std::getline(fin,line))
            {
                lines.push_back(line);
            }
        }

        return lines;
    }
        /*
    cv::Mat parsematLine(std::string &line)
    {
        std::istringstream iss(line);
        cv::Mat v;
        std::copy(std::istream_iterator<float>(iss),
                  std::istream_iterator<float>(),
                  std::back_inserter(v));

        return v;
    }

    */


    std::vector<float> parseLine(std::string &line)
    {
        std::istringstream iss(line);
        std::vector<float> v;
        std::copy(std::istream_iterator<float>(iss),
                  std::istream_iterator<float>(),
                  std::back_inserter(v));

        return v;
    }


    std::vector<cv::Point> getPositions(int superpixel_index, cv::Mat &clusters)
    {
        std::vector<cv::Point> p;
        int temp_index = 0;
        for(int y=0;y<clusters.rows;y++)
            for(int x=0;x<clusters.cols;x++)
            {
                temp_index = clusters.at<int>(y,x);

                if(temp_index==superpixel_index)
                {
                    cv::Point pt(x,y);

                    p.push_back(pt);
                }
            }
        return p;
    }


    cv::Mat expandEnergy(cv::Mat &energies, cv::Mat &segments)
    {
        double min;double max;
        cv::minMaxIdx(segments, &min, &max);
        int nr_superpixels = max+1;


        cv::Mat expandedEnergy = cv::Mat::zeros(segments.rows*segments.cols, energies.cols, CV_32FC1);
        std::vector<int> valued_rows;
        valued_rows.resize(segments.rows*segments.cols,0);
        for (int i=0;i<nr_superpixels;i++)
        {
            std::vector<cv::Point> pts = getPositions(i,segments);
            cv::Mat energy_sup = energies.row(i);
            for(int j = 0;j<pts.size();j++)
            {
                cv::Point p = pts[j];
                int rowidx = p.y*segments.cols + p.x;
                energy_sup.copyTo(expandedEnergy.row(rowidx));
                valued_rows[rowidx]=1;
            }

        }


        for (int i=0;i<valued_rows.size();i++)
        {
            if(valued_rows[i]==0)
            {
                int x,y;
                x = i%segments.cols;
                y = i/segments.cols;
            }

        }



        return expandedEnergy.clone();


    }


    std::vector<std::string> readFile(std::string filename) //Read file function
    {
        std::string line;
        std::vector<std::string> lines;
        std::ifstream fin;

        fin.open(filename);
        if(fin.is_open())
        {
            while(std::getline(fin,line))
            {
                lines.push_back(line);
            }
        }

        return lines;
    }
    std::vector<std::vector<float>> parseStringFile(std::vector<std::string> &lines)
    {
        std::vector<std::vector<float>> vals;
        for(int i=0;i<lines.size();i++)
        {
            std::vector<float> v = parseLine(lines[i]);
            vals.push_back(v);
        }

        return vals;
    }
    cv::Mat parseintomat(std::vector<std::string> &lines)
    {
        cv::Mat vals;
        for(int i=0;i<lines.size();i++)
        {
            std::vector<float> v = parseLine(lines[i]);
            vals.push_back(v);
        }

        return vals;
    }

    void parseStringFile2(std::vector<std::string> &lines, std::vector<std::vector<float>> &vals)
    {
        for(int i=0;i<lines.size();i++)
        {
            std::vector<float> v = parseLine(lines[i]);
            vals.push_back(v);
        }
    }

    bool saveParams(VectorXf params, std::string s)
    {
        cv::FileStorage fs;
        cv::Mat m;
        fs.open(s+".xml", cv::FileStorage::WRITE);


        cv::eigen2cv(params,m);
        fs<<"param"<<m;


        fs.release();

        return true;

    }

    VectorXf loadParams(std::string s)
    {
        cv::FileStorage fs;
        cv::Mat m;
        fs.open(s+".xml", cv::FileStorage::READ);
        fs["param"]>>m;
        cv::Mat p;

        m.convertTo(p, CV_32FC1);
        VectorXf parameters(m.rows);

        for(int i=0;i<p.rows;i++)
            parameters[i]=p.at<float>(i,0);


        fs.release();

        return parameters;

    }



public:
//Main functions----------------------------------------------------------------------------------------------

    void initCRF(int w, int h, int m, int nr_iterations){
        H=h;
        W=w;
        M=m;
        N=w*h;
        NIT = nr_iterations;
        file_nr=0;

    }

    void setFolderCache(std::string s)
    {
        folder_cache=s;
    }

    void setFolders(std::string fl_energy, std::string fl_segments,std::string fl_image, std::string fl_anno){
        folder_energy = fl_energy;
        folder_segment=fl_segments;
        folder_image = fl_image;
        folder_anno = fl_anno;

    }
    void  setExtensions(std::string enex, std::string segex,std::string imex,std::string annex, bool is_mat){
        en_ex= enex;
        seg_ex=segex;
        im_ex=imex;
        ann_ex= annex;
        if (is_mat){
            input_text= false;
        }else input_text = true;

    }
    void readDatas(std::string filename){

        std::string image = folder_image +filename+im_ex;
        //std::cout<<image;
        std::vector<std::vector<float>> data;
        std::vector<std::string> lines;
        //image
        im = readPPM(image.c_str(), W, H);
        if (!im) {
            std::cout<<("Failed to load image!\n");
            //return 1;
        }
        //ENergies
        if(input_text){
            lines = readFile(folder_energy+filename+en_ex);
            data = parseStringFile(lines);
        }else{
            lines = readFile(folder_energy+filename+en_ex);
            //mun = parseintomat(lines);
        }
        //Segments
        std::vector<std::string> linesseg = readFile(folder_segment+filename+seg_ex);
        std::vector<std::vector<float> > dataseg;
        parseStringFile2(linesseg,dataseg);
        std::vector<float> v;
        mseg =cv::Mat(dataseg.size(),dataseg[0].size(), CV_32S);
        for(int i=0;i<mseg.rows;i++)
        {
            v=dataseg[i];
            for(int j=0;j<mseg.cols;j++)
                mseg.at<int>(i,j) = v[j];
        }
        if (input_text){
            mun = cv::Mat(data.size(), data[0].size(),CV_32F);
            for(int i=0;i<mun.rows;i++)
            {
                v=data[i];
                for(int j=0;j<mun.cols;j++)
                    mun.at<float>(i,j) = v[j];
            }

        }
        std::vector<std::string> linesann = readFile(folder_anno+filename+ann_ex);
        std::vector<std::vector<float> > datanno;
        parseStringFile2(linesann,datanno);
        std::vector<float> v1;
        annMat =cv::Mat(datanno.size(),datanno[0].size(), CV_32S);
        for(int i=0;i<annMat.rows;i++)
        {
            v1=datanno[i];
            for(int j=0;j<annMat.cols;j++)
                annMat.at<int>(i,j) = v1[j];
        }
        //std::cout<<annMat.size()<<std::endl;
        annMat = annMat.reshape(1,annMat.rows*annMat.cols);
        Matrix<short,Dynamic,1> annEi;
        cv::cv2eigen(annMat, annEi);
        labeling = annEi;


    }

    bool loadUnaries(){

        cv::Mat expandedEnergy = expandEnergy(mun,mseg);

        //M = mun.cols;
        unary = computeUnary( expandedEnergy,  M );

       // return true;

    }

    void setDefaultParams(){

    }


    void process()
    {
        //------------------------ASK------------------------------
        //crf->setLabelCompatibilityParameters(labelParam);
        //crf->setKernelParameters(kernelParam);

        DenseCRF2D crf(W,H,M);
        crf.setUnaryEnergy(unary);
        crf.addPairwiseGaussian( 3, 3, new PottsCompatibility( 3 ) );  //3 terms on addpariwisegaussian?
        crf.addPairwiseBilateral(80, 80, 13, 13, 13, im, new MatrixCompatibility(MatrixXf::Identity(M, M)));

        if(file_nr>0)
            loadAllParams(crf,folder_cache);




        LogLikelihood objective(labeling, 0.01);
        verbose = true;
        MatrixXf learning_params(3, 3);

        // Optimize the CRF in 3 phases:
        //  * First unary only
        //  * Unary and pairwise
        //  * Full CRF
        learning_params << 1, 0, 0,
                1, 1, 0,
                1, 1, 1;

        for (int j = 0; j < learning_params.rows(); j++) {
            std::cout << j << "-> Learning Mode" << std::endl;
            // Setup the energy
            CRFEnergy energy(crf, objective, NIT, learning_params(j, 0), learning_params(j, 1),learning_params(j, 2));

            energy.setL2Norm(1e-3);


            // Minimize the energy
            VectorXf p = minimizeLBFGS(energy, 4, true);

            // Save the values
            int id = 0;
            if (learning_params(j, 0)) {
                crf.setUnaryParameters(p.segment(id, crf.unaryParameters().rows()));
                id += crf.unaryParameters().rows();
            }
            if (learning_params(j, 1)) {
                crf.setLabelCompatibilityParameters(p.segment(id, crf.labelCompatibilityParameters().rows()));
                id += crf.labelCompatibilityParameters().rows();
            }
            if (learning_params(j, 2))
                crf.setKernelParameters(p.segment(id, crf.kernelParameters().rows()));

            //Save at each stage, overwriting previous ones

            std::cout << std::endl;
            std::cout << "Unary parameters: " << crf.unaryParameters().transpose() << std::endl;
            std::cout << "Pairwise parameters: " << crf.labelCompatibilityParameters().transpose() << std::endl;
            std::cout << "Kernel parameters: " << crf.kernelParameters().transpose() << std::endl;

            //if(this->save_every_it)
                saveAllParams(crf,folder_cache);
        }



        file_nr++;


        /*
        crf->setUnaryEnergy(unary);
        std::cout<<"Inference starting...."<<std::endl;;
        MatrixXf Q = crf->startInference(), t1, t2;
        std::cout<<"kl =" << crf->klDivergence(Q) <<std::endl;

        for( int it=0; it<NIT; it++ ) {
            crf->stepInference( Q, t1, t2 );
            std::cout<<"kl = "<< crf->klDivergence(Q) ;
        }
        std::cout<<"Done inference"<<std::endl;
        map = crf->currentMap(Q);
         */

    }

    void saveAllParams(DenseCRF2D &crf, std::string foldername)
    {
        saveParams(crf.unaryParameters(), foldername + "unary_param");
        saveParams(crf.labelCompatibilityParameters(), foldername + "label_param");
        saveParams(crf.kernelParameters(), foldername + "kernel_param");
    }

    void loadAllParams(DenseCRF2D &crf, std::string foldername)
    {

        VectorXf up =loadParams(foldername+"unary_param");
        VectorXf lp =loadParams(foldername+"label_param");
        VectorXf kp =loadParams(foldername+"kernel_param");

        crf.setUnaryParameters(up);
        crf.setLabelCompatibilityParameters(lp);
        crf.setKernelParameters(kp);

    }



};

#endif //CRFNEW_CRFTEST_H
