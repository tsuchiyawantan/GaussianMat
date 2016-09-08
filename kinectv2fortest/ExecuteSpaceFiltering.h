#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define CIRCLE_RADIUS 2;
using namespace std;


class ExecuteSpaceFiltering{
private:
public:
	vector<double> filter;
	double filtersize;
	cv::Mat image2;
	vector<pair<int, int>> neighbour;

	ExecuteSpaceFiltering(double filter_size){
		filtersize = filter_size;
		createNeighbour(sqrt(filtersize));
	}
	~ExecuteSpaceFiltering(){}

	void createFilter(){
		double filter_value = 1 / filtersize;
		for (int i = 0; i < filtersize; i++)
			filter.push_back(filter_value);
	}
	void createNeighbour(int size){
		// �͈̓`�F�b�N
		if (size < 3) {
			size = 3;
		}
		if (size > 15) {
			size = 15;
		}
		size--;
		size /= 2;

		createFilter();

		for (int y = -size; y <= size; y++) {
			for (int x = -size; x <= size; x++) {
				neighbour.push_back(make_pair(y, x));
			}
		}
	}

	void createNeighbourGaussian(int size, vector<pair<int, int>> &neighbour){
		// �͈̓`�F�b�N
		if (size < 3) {
			size = 3;
		}
		if (size > 15) {
			size = 15;
		}
		size--;
		size /= 2;

		float sigma = 1.0;
		float sum = 0;
		for (int y = -size; y <= size; y++) {
			for (int x = -size; x <= size; x++) {
				double gf = GaussianFunc(y, x, sigma);
				filter.push_back(gf);
				neighbour.push_back(make_pair(y, x));
				sum += gf;
			}
		}
		for (int i = 0; i < filter.size(); i++) {
			filter.at(i) /= sum;
		}

	}

	double GaussianFunc(int y, int x, float sigma) {
		float pi = (float)M_PI;
		float sigma2 = sigma * sigma;
		double gauss_const = 1.0 / (2.0 * pi * sigma * sigma);
		double f = gauss_const * exp(-(x * x + y * y) / (2.0 * sigma * sigma));
		return f;

	}

	void applyFiltering(int y, int x, vector<pair<int, int>> &neighbour, vector<double> &bgr, cv::Mat &srcImg){
		cv::Vec3b* ptr;
		/*double filter = 1 / filtersize;
		for (auto itr = neighbour.begin(); itr != neighbour.end(); ++itr){
			int dy = y + (*itr).first;
			int dx = x + (*itr).second;
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			ptr = srcImg.ptr<cv::Vec3b>(dy);

			bgr.at(0) += ptr[dx][0] * filter;
			bgr.at(1) += ptr[dx][1] * filter;
			bgr.at(2) += ptr[dx][2] * filter;
		}*/
		for (int i = 0; i < neighbour.size(); i++){
			int dy = y + neighbour.at(i).first;
			int dx = x + neighbour.at(i).second;
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			ptr = srcImg.ptr<cv::Vec3b>(dy);

			bgr.at(0) += ptr[dx][0] * filter.at(i);
			bgr.at(1) += ptr[dx][1] * filter.at(i);
			bgr.at(2) += ptr[dx][2] * filter.at(i);
		}
		
	}
	
	//
	// ��ԃt�B���^�����O��p�����摜�����̗�
	//
	void executeSpaceFilteringCircle(cv::Mat &srcImg, cv::Mat &resultImg, cv::Mat &usedPoints, int y, int x) {
		vector<double> bgr(3, 0.0);


		//
		// �e�X�L�������C�����Ƃ�
		//
		int idef = y - 2 - CIRCLE_RADIUS;
		int jdef = x - 2 - CIRCLE_RADIUS;
		int height = y + 2 + CIRCLE_RADIUS;
		int width = x + 2 + CIRCLE_RADIUS;

		for (int i=idef; i <= height; i++) {
			cv::Vec3b* ptrResult;
			//
			// �e��f���Ƃ�
			//
			for (int j=jdef; j <= width; j++) {
				if (i < 0 || i >= srcImg.rows || j < 0 || j >= srcImg.cols) continue;
				bgr = { 0.0, 0.0, 0.0 };

				//��ԃt�B���^�����O�����ςݓ_�Ȃ�Δ�΂�
				unsigned char *p = &usedPoints.at<uchar>(i, j);
				if (*p == 255) continue;

				ptrResult = resultImg.ptr<cv::Vec3b>(i);

				applyFiltering(i, j, neighbour, bgr, srcImg);
				// valueR, valueG, valueB �̒l��0�`255�͈̔͂ɂ���
				if (bgr.at(2) < 0.0) bgr.at(2) = 0.0;
				else if (bgr.at(2) > 255.0) bgr.at(2) = 255.0;
				if (bgr.at(1) < 0.0) bgr.at(1) = 0.0;
				else if (bgr.at(1) > 255.0) bgr.at(1) = 255.0;
				if (bgr.at(0) < 0.0) bgr.at(0) = 0.0;
				else if (bgr.at(0) > 255.0) bgr.at(0) = 255.0;


				ptrResult[j] = cv::Vec3b(bgr.at(0), bgr.at(1), bgr.at(2));

				if (cv::Vec3b(bgr.at(0), bgr.at(1), bgr.at(2)) != cv::Vec3b(bgr.at(0), bgr.at(0), bgr.at(0)))
					*p = 255;
				
			}
		}
	}
};