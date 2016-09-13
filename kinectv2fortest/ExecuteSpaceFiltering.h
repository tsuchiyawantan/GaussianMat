#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Log.h"


#define CIRCLE_RADIUS 2;
using namespace std;


class ExecuteSpaceFiltering{
private:
public:
	double filtersize;
	double filter;
	cv::Mat image2;
	vector<pair<int, int>> neighbour;

	ExecuteSpaceFiltering(double filter_size){
		filtersize = filter_size;
		createNeighbour(sqrt(filtersize));
		filter = 1 / filtersize;
	}
	~ExecuteSpaceFiltering(){}

	void createNeighbour(int size){
		// 範囲チェック
		if (size < 3) {
			size = 3;
		}
		if (size > 15) {
			size = 15;
		}
		size--;
		size /= 2;

		for (int y = -size; y <= size; y++) {
			for (int x = -size; x <= size; x++) {
				neighbour.push_back(make_pair(y, x));
			}
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
		
		for (auto itr = neighbour.begin(); itr != neighbour.end(); ++itr){
			int dy = y + (*itr).first;
			int dx = x + (*itr).second;
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			ptr = srcImg.ptr<cv::Vec3b>(dy);

			bgr.at(0) += ptr[dx][0] * filter;
			bgr.at(1) += ptr[dx][1] * filter;
			bgr.at(2) += ptr[dx][2] * filter;
		}
	}
	
	//
	// 空間フィルタリングを用いた画像処理の例
	//
	void executeSpaceFilteringCircle(cv::Mat &srcImg, cv::Mat &resultImg, cv::Mat &usedPoints, int y, int x) {
		vector<double> bgr(3, 0.0);


		//
		// 各スキャンラインごとに
		//
		int idef = y - 2 - CIRCLE_RADIUS;
		int jdef = x - 2 - CIRCLE_RADIUS;
		int height = y + 2 + CIRCLE_RADIUS;
		int width = x + 2 + CIRCLE_RADIUS;

		for (int i=idef; i <= height; i++) {
			cv::Vec3b* ptrResult;
			//
			// 各画素ごとに
			//
			for (int j=jdef; j <= width; j++) {
				if (i < 0 || i >= srcImg.rows || j < 0 || j >= srcImg.cols) continue;
				bgr = { 0.0, 0.0, 0.0 };

				//空間フィルタリング処理済み点ならば飛ばす
				unsigned char *p = &usedPoints.at<uchar>(i, j);
				if (*p == 255) continue;

				ptrResult = resultImg.ptr<cv::Vec3b>(i);

				applyFiltering(i, j, neighbour, bgr, srcImg);
				// valueR, valueG, valueB の値を0〜255の範囲にする
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

	void executeSpaceFilteringCircle(cv::Mat &srcImg, cv::Mat &resultImg, cv::Mat &usedPoints, int y, int x, Log log) {
		vector<double> bgr(3, 0.0);
		clock_t start = clock();

		//
		// 各スキャンラインごとに
		//
		int idef = y - 2 - CIRCLE_RADIUS;
		int jdef = x - 2 - CIRCLE_RADIUS;
		int height = y + 2 + CIRCLE_RADIUS;
		int width = x + 2 + CIRCLE_RADIUS;

		for (int i = idef; i <= height; i++) {
			cv::Vec3b* ptrResult;
			//
			// 各画素ごとに
			//
			for (int j = jdef; j <= width; j++) {
				if (i < 0 || i >= srcImg.rows || j < 0 || j >= srcImg.cols) continue;
				bgr = { 0.0, 0.0, 0.0 };

				//空間フィルタリング処理済み点ならば飛ばす
				unsigned char *p = &usedPoints.at<uchar>(i, j);
				if (*p == 255) continue;

				ptrResult = resultImg.ptr<cv::Vec3b>(i);

				applyFiltering(i, j, neighbour, bgr, srcImg);
				// valueR, valueG, valueB の値を0〜255の範囲にする
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
		clock_t end = clock();
	}
};