﻿#include"Som.h"

#define CAM 0 // 0->uvc, 1->pixpro

cv::Mat Img(cv::Size(500, 500), CV_8UC3);

//コンストラクタ
SOM::SOM(){}
SOM::SOM(int width, int height, std::vector<cv::Point2f> &Pos, cv::UMat &src){
	set_size(width, height); //区画の初期化
	set_pos(Pos);			 //4点の座標の初期化
	set_img(src);			 //入力画像の初期化
}

//射影変換前の
void SOM::Init(cv::Mat &src){}

//射影変換前の画像にSOMを適用する
void SOM::Init2(cv::Mat &src){
	std::cout << std::endl << "--------------------------------------------------------------" << std::endl;
	std::cout << std::endl << " 掃引マップ生成開始" << std::endl << std::endl;
	cv::Mat dst;
	int calcNeuronNum; //近傍ニューロンの番号を取得
	std::vector<Neuron> storeNeuron; //ニューロンの格納
	for (int j = 0; j < height + 1; j++){
		for (int i = 0; i < width + 1; i++){
			Neuron neu; //ニューロン構造体
			neu.id = j * (width + 1) + i;
			neu.p = cv::Point2i((i % (width + 1)) * 20 + 200, 300 - (j % (height + 1) * 20)); //ƒjƒ…[ƒƒ“‚Ì‰ŠúˆÊ’u
			for (int k = j - 1; k < j + 2; k++){
				for (int l = i - 1; l < i + 2; l++){
					calcNeuronNum = k * (width + 1) + l;

					//4近傍の調査
					if (k >= 0 && j - 1 == k && i == l){
						neu.link.push_back(calcNeuronNum);
					}
					if (l <= height && j == k && i + 1 == l){
						neu.link.push_back(calcNeuronNum);
					}
					if (k <= height && j + 1 == k && i == l){
						neu.link.push_back(calcNeuronNum);
					}
					if (l >= 0 && j == k && i - 1 == l){
						neu.link.push_back(calcNeuronNum);
					}
				}
			}
			storeNeuron.push_back(neu);
		}
	}

	Imgproc(src, dst);
	calcsom(width, height, storeNeuron, dst, src); //somの計算
	this->som = storeNeuron;
	std::cout << std::endl << std::endl;
	std::cout << " 掃引マップ生成終了" << std::endl << std::endl;

	std::cout << "--------------------------------------------------------------" << std::endl << std::endl;

}


void SOM::InitImg(cv::Mat &img){
	for (int i = 0; i < img.rows; i++){
		for (int j = 0; j < img.cols; j++){
			img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
		}
	}
}

void SOM::edgeline(std::vector<Neuron> &neu, cv::Mat &img){
	for (auto it : neu){
		for (auto nei : it.link)
			line(img, it.p, neu[nei].p, cv::Scalar(0, 0, 0), 1, CV_AA);
	}
}

void SOM::showSOM(int index, std::vector<int> &linked, std::vector<Neuron> &def, int time){
	static cv::Mat im;
	Img.copyTo(im);

	static int maxX = 0;
	static int minX = im.cols;
	static int maxY = 0;
	static int minY = im.rows;

	for (auto it : def){
		if (it.p.x < minX){
			minX = it.p.x;
		}
		if (it.p.x > maxX){
			maxX = it.p.x;
		}
		else{}
		if (it.p.y < minY){
			minY = it.p.y;
		}
		if (it.p.y > maxY){
			maxY = it.p.y;
		}
		else{}
	}
	int r = 0, max_r = 0;
	for (auto it : linked){

		r = abs(sqrt((def[it].p.x - def[index].p.x)*(def[it].p.x - def[index].p.x)
			+ (def[it].p.y - def[index].p.y)*(def[it].p.y - def[index].p.y)));
		if (r > max_r){
			max_r = r;
		}
	}

	cv::Point2f pts1[] = { cv::Point2f(minX, maxY), cv::Point2f(minX, minY),
		cv::Point2f(maxX, minY), cv::Point2f(maxX, maxY) };
	cv::Point2f pts2[] = { cv::Point2f(20, im.rows - 20), cv::Point2f(20, 20),
		cv::Point2f(im.cols - 20, 20), cv::Point2f(im.cols - 20, im.rows - 20) };
	//cv::circle(im, def[index].p, r, cv::Scalar(255, 50, 50), -1, CV_AA);
	edgeline(def, im);
	for (auto it : def){
		if (it.id != index){
			cv::circle(im, it.p, 3, cv::Scalar(50, 50, 255), -1, CV_AA);
		}
	}
	for (auto it : linked){

		cv::circle(im, def[it].p, 3, cv::Scalar(255, 0, 0), -1, CV_AA);

	}

	cv::circle(im, def[index].p, 3, cv::Scalar(0, 255, 255), -1, CV_AA);

	cv::Mat perspective_matrix = getPerspectiveTransform(pts1, pts2);
	cv::warpPerspective(im, im, perspective_matrix, cv::Size(im.cols, im.rows), cv::INTER_LINEAR);


}

void SOM::calcsom(int w, int h, std::vector<Neuron> &som, cv::Mat &src, cv::Mat &origin){

	std::vector<Neuron> defo = som;
	float t = 1;
	static float max_t = 32000;
	int index = 0;
	int count = 0;
	float dist = 0;
	float neuDist = 0;
	int distM = 0;
	static cv::Mat cpimg;
	static cv::Point2f randP;
	float learncoeff = 0;
	cv::Point2f res;
	int ind = 0;
	srand(2);

	std::vector<cv::Point2f> random = storePoint(src);
	std::vector<cv::Point2f> random2 = storeBorderPoint();
	cv::Point pt[10]; //任意の4点を配列に格納
	for (int i = 0; i < P.size(); i++){
		pt[i] = P[i];
	}


	while (t < max_t){
		image.copyTo(cpimg);
		line(cpimg, pt[0], pt[1], cv::Scalar(255, 255, 0), 2, CV_AA);
		line(cpimg, pt[1], pt[2], cv::Scalar(255, 255, 0), 2, CV_AA);
		line(cpimg, pt[2], pt[3], cv::Scalar(255, 255, 0), 2, CV_AA);
		line(cpimg, pt[3], pt[0], cv::Scalar(255, 255, 0), 2, CV_AA);

		dist = 3000;
		index = 0;
		count = 0;
		learncoeff = 1 - float(t / (max_t));
		

		if (CAM == 0){
			// 内側
			if ((0.0 * max_t < t && t < 0.4 * max_t) || (int)t % 10 < 5){
				ind = rand() * rand() % random.size(); //randの範囲拡大
				randP = random[ind];
			}
			// 4隅
			else if (t > 0.95*max_t || (int)t % 10 < 6){
				randP = this->P[rand() % this->P.size()];
			}
			// 外領域
			else{
				ind = rand() % random2.size(); //rand‚ð‚QŒÂ‚©‚¯‚Ä”ÍˆÍ‚ðL‚°‚é

				randP = random2[ind];
			}
		}
		if (CAM == 1){
			// 内側
			//if ((0.0 * max_t < t && t < 0.3 * max_t) || (int)t%10 < 6){
			if ((int)t % 10 < 6 || (0.5* max_t < t && t < 0.6 * max_t)){
				ind = rand() * rand() % random.size(); //rand‚ð‚QŒÂ‚©‚¯‚Ä”ÍˆÍ‚ðL‚°‚é
				randP = random[ind];
				//printf("1!\n");
			}
			// 4隅
			//else if (0.95*max_t < t || (int)t%10 < 7){
			else if ((int)t % 10 < 7){
				randP = this->P[rand() % this->P.size()];
				//printf("22!\n");
			}
			// 外領域
			else{
				ind = rand() % random2.size(); //rand‚ð‚QŒÂ‚©‚¯‚Ä”ÍˆÍ‚ðL‚°‚é
				randP = random2[ind];
				//printf("333!\n");
			}
		}


		cv::circle(cpimg, randP, 4, cv::Scalar(255, 255, 255), -1, CV_AA); //“ü—Í’l
		for (auto it : som){
			neuDist = abs(sqrt((it.p.x - randP.x)*(it.p.x - randP.x) + (it.p.y - randP.y)*(it.p.y - randP.y)));
			if (neuDist < dist){
				dist = neuDist;
				res.x = randP.x - it.p.x;
				res.y = randP.y - it.p.y;
				index = count;
			}
			count++;
		}
		distM = 0; //マンハッタン距離

		std::vector<int> ne;
		std::vector<int> ne2;
		std::vector<int> linked;
		int lik = 0;

		som[index].p.x = som[index].p.x + res.x * learncoeff / 6 * exp(-(distM*distM) / (2 * (learncoeff*learncoeff)));
		som[index].p.y = som[index].p.y + res.y * learncoeff / 6 * exp(-(distM*distM) / (2 * (learncoeff*learncoeff)));
		linked.push_back(index);
		ne = som[index].link;
		distM++;
		int flag = 0;
		int distFlag = 0;
		float r = 0, max_r = 0;

		while (1){

			for (auto it : ne){
				res.x = randP.x - som[it].p.x;
				res.y = randP.y - som[it].p.y;

				som[it].p.x = som[it].p.x + res.x * learncoeff / 6 * exp(-(distM*distM) / (2 * (learncoeff*learncoeff)));
				som[it].p.y = som[it].p.y + res.y * learncoeff / 6 * exp(-(distM*distM) / (2 * (learncoeff*learncoeff)));

				if (learncoeff / 6 * exp(-(distM*distM) / (2 * (learncoeff*learncoeff))) < 0.001){
					if (distFlag == 0){
						distFlag = 1;
					}
				}
				if (distFlag == 1){
					for (auto it : ne){
						r = abs(sqrt((som[it].p.x - som[index].p.x)*(som[it].p.x - som[index].p.x)
							+ (som[it].p.y - som[index].p.y)*(som[it].p.y - som[index].p.y)));
						if (r > max_r){
							max_r = r;
						}
					}
					break;
				}

				for (auto it : ne){
					linked.push_back(it);
				}
				std::sort(linked.begin(), linked.end(), cmp);
				linked.erase(std::unique(linked.begin(), linked.end()), linked.end());

				for (auto iit : som[it].link){
					flag = 0;
					for (auto i : linked){
						if (iit == i){
							flag = 1;
						}
					}
					if (flag == 0){
						ne2.push_back(iit);
					}
				}
			}
			std::sort(ne2.begin(), ne2.end(), cmp);
			ne2.erase(std::unique(ne2.begin(), ne2.end()), ne2.end());

			for (auto it : ne2){
				linked.push_back(it);
			}
			std::sort(linked.begin(), linked.end(), cmp);
			linked.erase(std::unique(linked.begin(), linked.end()), linked.end());

			if (ne2.empty()) {
				break;
			}
			distM++;
			ne.clear();
			for (auto it : ne2){
				ne.push_back(it);
			}
			ne2.clear();
		}
		edgeline(som, cpimg);
		for (auto it : som){
			cv::circle(cpimg, it.p, 3, cv::Scalar(0, 0, 255), -1, CV_AA);
		}

		for (auto it : linked){
			cv::circle(cpimg, som[it].p, 3, cv::Scalar(255, 0, 0), -1, CV_AA);
		}
		cv::circle(cpimg, som[index].p, 5, cv::Scalar(0, 255, 255), -1, CV_AA);
		if (int(t) % 1000 == 0){
			int process;
			std::cout << "*";

			process = t / max_t * 100;

			if (process == 25 || process == 50 || process == 75){
				std::cout << " " << process << "% ";
			}


			//showSOM(index, linked, defo, t);
			//cv::imshow("s", cpimg);
			//cv::waitKey(10);
		}

		t++;
		index = 0;
		count = 0;
		distFlag = 0;
	}
	cv::imshow("s", cpimg);
}

void SOM::Imgproc(cv::Mat &src, cv::Mat &dst){
	cv::cvtColor(src, src, CV_BGR2GRAY);
	cv::threshold(src, dst, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
}

std::vector<cv::Point2f> SOM::storePoint(cv::Mat &img){
	std::vector<cv::Point2f> p;
	for (int j = 0; j < img.rows; j++){
		for (int i = 0; i < img.cols; i++){
			if ((int)img.at<unsigned char>(j, i) > 0){
				cv::Point2f ce(i, j);
				p.push_back(ce);
			}
		}
	}
	return p;
}

std::vector<cv::Point2f> SOM::storeBorderPoint(){
	std::vector<cv::Point2f> p;
	cv::Point2f mid, mid2;

	for (int i = 0; i < P.size(); i++){
		p.push_back(P[i]);
		if (i + 1 < P.size()){
			for (int j = 1; j < 101; j++){
				mid = cv::Point2f(((100 - j)*P[i + 1].x + j * P[i].x) / 100,
					((100 - j)*P[i + 1].y + j * P[i].y) / 100);
				p.push_back(mid);
			}
		}
		else{
			for (int j = 1; j < 101; j++){
				mid = cv::Point2f(((100 - j)*P[0].x + j * P[i].x) / (100),
					((100 - j)*P[0].y + j * P[i].y) / (100));
				p.push_back(mid);
			}
		}
	}
	return p;
}

bool cmp(int A, int B){
	return A < B ? 1 : 0;
}

std::string SOM::setImageName(std::string str, int time){

	char fcount[32];
	std::string data = "./image/";

	sprintf(fcount, "_%dFrame.png", time);

	return data + str + fcount; 
}

cv::Point2f SOM::calc_centerPoint(int n, std::vector<int> &nei){
	//n = 0;
	SOM::Neuron a, b, c, d, cp;
	SOM::Neuron me = this->som[n];
	int flag = 0;

	a = me;
	nei.push_back(a.id);
	for (int i = 0; i < me.link.size(); i++){
		if (this->som[me.link[i]].id > me.id && i + 1 < me.link.size()){
			b = this->som[me.link[i]];
			c = this->som[me.link[i + 1]];
			nei.push_back(b.id);
			nei.push_back(c.id);
			break;
		}
	}
	for (int i = 0; i < b.link.size(); i++){
		for (int j = 0; j < c.link.size(); j++){
			//std::cout << "b: " << som[b.link[i]].id << " c:" << som[c.link[j]].id << std::endl;
			if (som[b.link[i]].id == som[c.link[j]].id){
				if (this->som[b.link[i]].id != me.id){
					d = som[b.link[i]];
					nei.push_back(d.id);
					//std::cout << "a:" << a.id << " b:" << b.id << " c:" << c.id << " d:" << d.id << std::endl;
					flag = 1;
					break;
				}
			}
		}
		if (flag == 1){
			break;
		}
	}


	cp.p = calc_weight(a.p, b.p, c.p, d.p);


	return cp.p;

}


void SOM::set_size(int width, int height){

	this->width = width / 100;
	this->height = height / 100;

}

void SOM::set_pos(std::vector<cv::Point2f> &Pos){

	for (auto it : Pos){
		this->P.push_back(it);
	}

}

void SOM::set_img(cv::UMat &src){

	src.copyTo(this->image);
	InitImg(Img);

}

void SOM::showSOM2(cv::UMat &src, cv::Mat &H, std::vector<int> &v){
	cv::Mat im;
	std::vector<Neuron> S = this->som;
	double a = H.at<double>(0, 0);
	double b = H.at<double>(0, 1);
	double c = H.at<double>(0, 2);
	double d = H.at<double>(1, 0);
	double e = H.at<double>(1, 1);
	double f = H.at<double>(1, 2);
	double g = H.at<double>(2, 0);
	double h = H.at<double>(2, 1);
	double i = H.at<double>(2, 2);


	for (int j = 0; j < S.size(); j++){
		cv::Point2f temp = S[j].p;
		S[j].p.x = (temp.x * a + temp.y * b + c) / (temp.x * g + temp.y * h + i);
		S[j].p.y = (temp.x * d + temp.y * e + f) / (temp.x * g + temp.y * h + i);
		cv::circle(src, S[j].p, 3, cv::Scalar(255, 0, 255), -1, CV_AA);
	}
	for (auto it : S){
		for (auto nei : it.link)
			line(src, it.p, S[nei].p, cv::Scalar(100, 100, 100), 1, CV_AA);
	}
	for (auto it : S){
		cv::circle(src, it.p, 3, cv::Scalar(0, 0, 255), -1, CV_AA);
	}

	line(src, S[v[0]].p, S[v[2]].p, cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, S[v[2]].p, S[v[3]].p, cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, S[v[1]].p, S[v[3]].p, cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, S[v[1]].p, S[v[0]].p, cv::Scalar(255, 0, 255), 2, CV_AA);

	for (auto it : v){
		cv::circle(src, S[it].p, 4, cv::Scalar(0, 255, 255), -1, CV_AA);
	}

}

void SOM::showSOM3(cv::UMat &src, std::vector<int> &v){

	for (auto it : this->som){
		for (auto nei : it.link)
			line(src, it.p, this->som[nei].p, cv::Scalar(100, 100, 100), 1, CV_AA);
	}
	for (auto it : this->som){
		cv::circle(src, it.p, 3, cv::Scalar(0, 0, 255), -1, CV_AA);
	}

	plotNowTargetArea(src, v);

}

cv::Point2f SOM::calc_weight(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d){
	float A = 0, B = 0, C = 0, D = 0;

	A = (d.y - a.y) / (d.x - a.x);
	B = a.y - A * a.x;
	C = (c.y - b.y) / (c.x - b.x);
	D = c.y - C * c.x;

	return cv::Point2f((B - D) / (C - A), (B*C - A*D) / (C - A));
}

std::vector<SOM::Neuron> SOM::get_SOM(){
	return this->som;
}


void SOM::plotNowTargetArea(cv::UMat src, std::vector<int> &v){


	line(src, som[v[0]].p, som[v[2]].p, cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, som[v[2]].p, som[v[3]].p, cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, som[v[1]].p, som[v[3]].p, cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, som[v[1]].p, som[v[0]].p, cv::Scalar(255, 0, 255), 2, CV_AA);

	for (auto it : v){
		cv::circle(src, som[it].p, 4, cv::Scalar(0, 255, 255), -1, CV_AA);
	}
}