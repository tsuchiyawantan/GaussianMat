#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "NeonDesign.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

class Gaussian{
private:
public:
	vector<float> kernel;
	vector<pair<int, int>> var_yx;
	Gaussian(){}
	~Gaussian(){}

	void createKernel(int size){
		// 範囲チェック
		if (size < 3) {
			size = 3;
		}
		if (size > 15) {
			size = 15;
		}
		size--;
		size /= 2;

		float sigma = 3;
		//kernel.reserve((size * 2 + 1) * (size * 2 + 1));
		for (int y = -size; y <= size; y++) {
			for (int x = -size; x <= size; x++) {
				kernel.push_back(GaussianF(x, y, sigma));
				var_yx.push_back(make_pair(y, x));
			}
		}
	}
	
	// ガウス分布のメソッド
	float GaussianF(int y, int x, float sigma) {
		float pi = (float)M_PI;
		float sigma2 = sigma * sigma;
		return (1 / (2 * pi * sigma2)) * (float)exp(-(x * x + y * y) / (2 * sigma2));
	}
	
	// ガウシアンフィルタを適用する
	void GaussianBlur(cv::Mat &image, int y, int x) {
		for (int i = 0; i < var_yx.size(); i++){
			image.at<cv::Vec3b>(y + var_yx.at(i).first, x + var_yx.at(i).second)[0] *= kernel.at(i); // b; //青
			image.at<cv::Vec3b>(y + var_yx.at(i).first, x + var_yx.at(i).second)[1] *= kernel.at(i); // g; //緑
			image.at<cv::Vec3b>(y + var_yx.at(i).first, x + var_yx.at(i).second)[2] *= kernel.at(i); // r; //赤		}
		}
	}

};