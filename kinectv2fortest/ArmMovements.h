#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "NeonDesign.h"
#include "Gaussian.h"

using namespace std;

class ArmMovements{
private:
public:
	vector<pair<int, int>> yx;
	ArmMovements(){}
	~ArmMovements(){}

	double getC1(int xyf, int xy0, int xy1, double T, double T1, double T12, double T13, double T14, double T15, double tf, double tf5, double l5){
		return 1 / (tf5*T12)*l5*((xyf - xy0)*(300 * T15 - 1200 * T14 + 1600 + T13) + T12*(-720 * xyf + 120 * xy1 + 600 * xy0) + (xy0 - xy1)*(300 * T1 - 200));
	}
	double getPi1(int xyf, int xy0, int xy1, double T, double T1, double T12, double T13, double T14, double T15, double tf, double tf5, double l5){
		return 1 / (tf5*T15*l5)*((xyf - xy0)*(120 * T15 - 300 * T14 + 200 * T13) - 20 * (xy1 - xy0));
	}
	double getXYS(int xyf, int xy0, int xy1, double T, double T3, double T4, double T5, double T1, double T12, double T13, double T14, double T15, double tf, double tf5, double l5){
		return tf5 / 720 * (getPi1(xyf, xy0, xy1, T, T1, T12, T13, T14, T15, tf, tf5, l5)*(T14*(15 * T4 - 30 * T3) + T13*(80 * T3 - 30 * T4) - 60 * T3*T12 + 30 * T4*T1 - 6 * T5) + getC1(xyf, xy0, xy1, T, T1, T12, T13, T14, T15, tf, tf5, l5)*(15 * T4 - 10 * T3 - 6 * T5)) + xy0;
	}
	void drawCurvedArmMove(cv::Mat &image){
		int x0 = 0;
		int y0 = 50;
		int x1 = 10;
		int y1 = 40;
		int xf = 30;
		int yf = 50;
		
		double t = 0;
		double t1 = 1.0;
		double tf = 2.0;

		double tf2 = tf*tf;
		double tf5 = tf2*tf2*tf;
		double T = t / tf;
		double T1 = t1 / tf;
		double T2 = T*T;
		double T3 = T2*T;
		double T4 = T3*T;
		double T5 = T4*T;
		double T12 = T1*T1;
		double T13 = T12 * T1;
		double T14 = T12*T12;
		double T15 = T13*T12;
		double l = (1 - T1);
		double l3 = l*l*l;
		double l5 = l3*l*l;
		double m = (T - T1);
		double m3 = m*m*m;
		double m5 = m3*m*m;
		srand((unsigned int)time(NULL));

		for (t; t < tf; t += 0.2){
			int D = rand() % 10 - 5;
			double xt, yt;
			T = t/tf;
			T2 = T*T;
			T3 = T2*T;
			T4 = T3*T;
			T5 = T4*T;
			m = (T - T1);
			m3 = m*m*m;
			m5 = m3*m*m;
			if (t < t1){
				xt = getXYS(xf, x0, x1, T, T3,T4, T5, T1, T12, T13, T14, T15, tf, tf5, l5);
				yt = getXYS(yf, y0, y1, T, T3, T4, T5, T1, T12, T13, T14, T15, tf, tf5, l5);
			}
			else {
				xt = getXYS(xf, x0, x1, T, T3, T4, T5, T1, T12, T13, T14, T15, tf, tf5, l5) + getPi1(xf, x0, x1, T, T1, T12, T13, T14, T15, tf, tf5, l5)*tf5*m5/120;
				yt = getXYS(yf, y0, y1, T, T3, T4, T5, T1, T12, T13, T14, T15, tf, tf5, l5) + getPi1(yf, y0, y1, T, T1, T12, T13, T14, T15, tf, tf5, l5)*tf5*m5 / 120;
			}
			xt += 300;
			yt += 300;
			image.at<cv::Vec3b>(yt, xt)[0] = 255; // b; //ê¬
			image.at<cv::Vec3b>(yt, xt)[1] = 255; // g; //óŒ
			image.at<cv::Vec3b>(yt, xt)[2] = 0; // r; //ê‘
			yx.push_back(make_pair(yt, xt));
		}
		yx.push_back(make_pair(yf, xf));
	}
	void drawArmMove(cv::Mat &image, vector<pair<int, int>> &armPts, pair<int, int> yx0, pair<int, int> yxf){
		int x0 = yx0.second;
		int y0 = yx0.first;
		int xf = yxf.second;
		int yf = yxf.first;
		double t = 0;
		double tf = 1.0;
		image.at<cv::Vec3b>(y0, x0)[0] = 0; // b; //ê¬
		image.at<cv::Vec3b>(y0, x0)[1] = 0; // g; //óŒ
		image.at<cv::Vec3b>(y0, x0)[2] = 255; // r; //ê‘
		image.at<cv::Vec3b>(yf, xf)[0] = 0; // b; //ê¬
		image.at<cv::Vec3b>(yf, xf)[1] = 0; // g; //óŒ
		image.at<cv::Vec3b>(yf, xf)[2] = 255; // r; //ê‘

		armPts.push_back(make_pair(y0, x0));
		srand((unsigned int)time(NULL));

		for (t; t <= tf; t += 0.5){
			double g = t / tf;
			int D = rand() % 10 - 5;
			double xt = x0 + (x0 - xf)*(15 * g*g*g*g - 6 * g*g*g*g*g - 10 * g*g*g) +D;
			double yt = y0 + (y0 - yf)*(15 * g*g*g*g - 6 * g*g*g*g*g - 10 * g*g*g) +D;
			//image.at<cv::Vec3b>(yt, xt)[0] = 255; // b; //ê¬
			//image.at<cv::Vec3b>(yt, xt)[1] = 255; // g; //óŒ
			//image.at<cv::Vec3b>(yt, xt)[2] = 0; // r; //ê‘
			armPts.push_back(make_pair(yt, xt));
		}
		armPts.push_back(make_pair(yf, xf));
	}
	void drawArmMove(cv::Mat &image){
		int x0 = 100;
		int y0 = 100;
		int xf = 300;
		int yf = 100;
		double t = 0;
		double tf = 2.0;
		image.at<cv::Vec3b>(y0, x0)[0] = 0; // b; //ê¬
		image.at<cv::Vec3b>(y0, x0)[1] = 0; // g; //óŒ
		image.at<cv::Vec3b>(y0, x0)[2] = 255; // r; //ê‘
		image.at<cv::Vec3b>(yf, xf)[0] = 0; // b; //ê¬
		image.at<cv::Vec3b>(yf, xf)[1] = 0; // g; //óŒ
		image.at<cv::Vec3b>(yf, xf)[2] = 255; // r; //ê‘

		//yx.push_back(make_pair(y0, x0));
		srand((unsigned int)time(NULL));

		for (t; t <= tf; t += 0.2){
			double g = t / tf;
			int D = rand() % 10 - 5;
			double xt = x0 + (x0 - xf)*(15 * g*g*g*g - 6 * g*g*g*g*g - 10 * g*g*g) + D;
			double yt = y0 + (y0 - yf)*(15 * g*g*g*g - 6 * g*g*g*g*g - 10 * g*g*g) + D;
			//image.at<cv::Vec3b>(yt, xt)[0] = 255; // b; //ê¬
			//image.at<cv::Vec3b>(yt, xt)[1] = 255; // g; //óŒ
			//image.at<cv::Vec3b>(yt, xt)[2] = 0; // r; //ê‘
			yx.push_back(make_pair(yt, xt));
		}
		yx.push_back(make_pair(yf, xf));
	}
	double catmullRom(double p0, double p1, double p2, double p3, double t){
		double v0 = (p2 - p0) / 2;
		double v1 = (p3 - p1) / 2;
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p0)*t + (2 * p0 - 5 * p1 + 4 * p2 - p3)*t2 + (-p0 + 3 * p1 - 3 * p2 + p3)*t3);
	}
	double catmullRomFL(double p0, double p1, double p2, double p3, double t, double ){
		double v0 = (p2 - p0) / 2;
		double v1 = (p3 - p1) / 2;
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p0)*t + (2 * p0 - 5 * p1 + 4 * p2 - p3)*t2 + (-p0 + 3 * p1 - 3 * p2 + p3)*t3);
	}
	void adjust(vector<pair<int, int>> &yx){
		int j = yx.size() + (4 - yx.size() % 4);
		while (yx.size() < j){
			yx.push_back(make_pair(yx.back().first, yx.back().second));
		}
	}
	void drawInline(cv::Mat &image, vector<pair<int, int>> &yx, int hue){
		//if (yx.size()%4 != 0) adjust(yx);
		NeonDesign design;
		int b = 0, g = 0, r = 0;
		vector<int> bgr = { 0, 0, 0 };
		design.rgb(hue, 255, 255 - 100, bgr, b, g, r);
		for (int i = 0; i < yx.size(); i++){
			int y = yx.at(i).first;
			int x = yx.at(i).second;
			image.at<cv::Vec3b>(y, x)[0] = 255; // b; //ê¬
			image.at<cv::Vec3b>(y, x)[1] = 255; // g; //óŒ
			image.at<cv::Vec3b>(y, x)[2] = 0; // r; //ê‘
			if (i >= yx.size() || i + 1 >= yx.size() || i + 2 >= yx.size() || i + 3 >= yx.size()) break;
			for (double t = 0; t <= 1.0; t += 0.009){
				y = catmullRom(yx.at(i).first, yx.at(i + 1).first, yx.at(i + 2).first, yx.at(i + 3).first, t);
				x = catmullRom(yx.at(i).second, yx.at(i + 1).second, yx.at(i + 2).second, yx.at(i + 3).second, t);
				image.at<cv::Vec3b>(y, x)[0] = 255; // b; //ê¬
				image.at<cv::Vec3b>(y, x)[1] = 255; // g; //óŒ
				image.at<cv::Vec3b>(y, x)[2] = 255; // r; //ê‘
			}
		}
	}
	void drawLine(cv::Mat &image, vector<pair<int, int>> &yx, int hue){
		//if (yx.size()%4 != 0) adjust(yx);
		NeonDesign design;
		int b = 0, g = 0, r = 0;
		vector<int> bgr = { 0, 0, 0 };
		design.rgb(hue, 255, 255 - 100, bgr, b, g, r);
		for (int i = 0; i < yx.size(); i++){
			int y = yx.at(i).first;
			int x = yx.at(i).second;
			image.at<cv::Vec3b>(y, x)[0] = 255; // b; //ê¬
			image.at<cv::Vec3b>(y, x)[1] = 255; // g; //óŒ
			image.at<cv::Vec3b>(y, x)[2] = 0; // r; //ê‘
			if (i >= yx.size() || i + 1 >= yx.size() || i + 2 >= yx.size() || i + 3 >= yx.size()) break;
			for (double t = 0; t <= 1.0; t += 0.009){
				y = catmullRom(yx.at(i).first, yx.at(i + 1).first, yx.at(i + 2).first, yx.at(i + 3).first, t);
				x = catmullRom(yx.at(i).second, yx.at(i + 1).second, yx.at(i + 2).second, yx.at(i + 3).second, t);
				image.at<cv::Vec3b>(y, x)[0] = 255; // b; //ê¬
				image.at<cv::Vec3b>(y, x)[1] = 255; // g; //óŒ
				image.at<cv::Vec3b>(y, x)[2] = 255; // r; //ê‘
				circle(image, cv::Point(x, y), 2, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
			}
		}
	}
};