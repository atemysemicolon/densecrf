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
    VectorXs map;


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

    void parseStringFile2(std::vector<std::string> &lines, std::vector<std::vector<float>> &vals)
    {
        for(int i=0;i<lines.size();i++)
        {
            std::vector<float> v = parseLine(lines[i]);
            vals.push_back(v);
        }
    }



public:
//Main functions----------------------------------------------------------------------------------------------

    bool loadUnaries(std::string fn_energy,std::string fn_segments,std::string  fn_image){
        //Unaries
        std::vector<std::string> lines = readFile(fn_energy);
        std::vector<std::vector<float>> data = parseStringFile(lines);

        //seg

        std::vector<std::string> linesseg = readFile(fn_segments);
        std::vector<std::vector<float> > dataseg;

        cv::Mat mseg,mun;
        parseStringFile2(linesseg,dataseg);

        //image
        im = readPPM(fn_image.c_str(), W, H);
        if (!im) {
            std::cout<<("Failed to load image!\n");
            //return 1;
        }

        //creating unary:
        std::vector<float> v;
        mseg =cv::Mat(dataseg.size(),dataseg[0].size(), CV_32S);
        for(int i=0;i<mseg.rows;i++)
        {
            v=dataseg[i];
            for(int j=0;j<mseg.cols;j++)
                mseg.at<int>(i,j) = v[j];
        }

        mun = cv::Mat(data.size(), data[0].size(),CV_32F);
        for(int i=0;i<mun.rows;i++)
        {
            v=data[i];
            for(int j=0;j<mun.cols;j++)
                mun.at<float>(i,j) = v[j];
        }


        if(!im || mseg.empty() || mun.empty())
            return false;


        cv::Mat expandedEnergy = expandEnergy(mun,mseg);
        std::cout<<expandedEnergy.size();

        //M = mun.cols;
        unary = computeUnary( expandedEnergy,  M );

        return true;

    }
    void initCRF(int w, int h, int m, int nr_iterations){
        H=h;
        W=w;
        M=m;
        NIT = nr_iterations;
        crf= std::shared_ptr<DenseCRF2D>(new DenseCRF2D(W,H,M));

    }
    void setDefaultParams(){
        crf->addPairwiseGaussian( 3, 3, new PottsCompatibility( 3 ) );  //3 terms on addpariwisegaussian?
        crf->addPairwiseBilateral(80, 80, 13, 13, 13, im, new MatrixCompatibility(MatrixXf::Identity(M, M)));
    }


    void process()
    {
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

    }

    void saveResults(std::string filename)
    {
        std::cout<<"Saving Results..."<<std::endl;
        cv::Mat res = cvtMap(map);
        cv::Mat predIdx = rollAnn(res,W,H);
        saveMap( predIdx, filename);
    }

};

#endif //CRFNEW_CRFTEST_H
