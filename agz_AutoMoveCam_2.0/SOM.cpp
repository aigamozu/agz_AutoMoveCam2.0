#include"Som.h"

cv::Mat Img(cv::Size(500, 500), CV_8UC3);

SOM::SOM(int width, int height, std::vector<cv::Point2f> &Pos, cv::UMat &src){
	this->width = width/100;
	this->height = height/100;
	for (auto it : Pos){
		this->P.push_back(it);
	}
	src.copyTo(this->image);
}

void SOM::Init(cv::Mat &src){
	cv::Mat dst;
	int calcNeuronNum; //�ߖT�j���[�����̔ԍ����擾
	std::vector<Neuron> storeNeuron; //�j���[�����̊i�[
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			Neuron neu; //�j���[�����\����
			neu.id = i * width + j;
			neu.p = cv::Point2i((j%width) * 10 + 200, (i % height) * 10 + 150); //�j���[�����̏����ʒu
			for (int k = i - 1; k < i + 2; k++){
				for (int l = j - 1; l < j + 2; l++){
					calcNeuronNum = k * width + l;

					//4�ߖT�̒���
					if (k >= 0 && i - 1 == k && j == l){
						neu.link.push_back(calcNeuronNum);
					}
					if (l <= height - 1 && i == k && j + 1 == l){
						neu.link.push_back(calcNeuronNum);
					}
					if (k <= height - 1 && i + 1 == k && j == l){
						neu.link.push_back(calcNeuronNum);
					}
					if (l >= 0 && i == k && j - 1 == l){
						neu.link.push_back(calcNeuronNum);
					}
				}
			}
			storeNeuron.push_back(neu);
		}
	}
	//�i�[���ꂽ�j���[������\��
	for (auto it : storeNeuron){
		std::cout << "id : " << it.id << " position : " << it.p << std::endl;
	}
	Imgproc(src, dst);
	som(width, height, storeNeuron, dst, src); //som�̌v�Z
	std::cout << "fin" << std::endl;
	cv::waitKey(0); //�v���O�����̏I��
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
			line(img, it.p, neu[nei].p, cv::Scalar(255, 0, 0), 1, CV_AA);
	}
}

void SOM::showSOM(int index, std::vector<Neuron> &def){
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
	cv::Point2f pts1[] = { cv::Point2f(minX, maxY), cv::Point2f(minX, minY),
		cv::Point2f(maxX, minY), cv::Point2f(maxX, maxY) };
	cv::Point2f pts2[] = { cv::Point2f(20, im.rows - 20), cv::Point2f(20, 20),
		cv::Point2f(im.cols - 20, 20), cv::Point2f(im.cols - 20, im.rows - 20) };
	for (auto it : def){
		if (it.id != index){
			cv::circle(im, it.p, 3, cv::Scalar(0, 0, 0), -1, CV_AA);
		}
	}
	edgeline(def, im);
	cv::circle(im, def[index].p, 5, cv::Scalar(0, 0, 255), -1, CV_AA);
	cv::waitKey(10);

	//@comment �����ϊ��s����v�Z
	cv::Mat perspective_matrix = getPerspectiveTransform(pts1, pts2);
	cv::warpPerspective(im, im, perspective_matrix, cv::Size(im.cols, im.rows), cv::INTER_LINEAR);
	cv::imshow("show", im);
	cv::waitKey(1);
}

void SOM::som(int w, int h, std::vector<Neuron> &som, cv::Mat &src, cv::Mat &origin){
	//src.copyTo(Img);
	//InitImg(Img);
	cv::namedWindow("s", 1);
	std::vector<Neuron> defo = som;
	float t = 1;
	static float max_t = 10000;
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
	std::cout << random.size() << std::endl;
	std::cout << random2.size() << std::endl;


	while (t < max_t){
		origin.copyTo(cpimg);
		dist = 10000;
		index = 0;
		count = 0;
		learncoeff = 1 - float(t / max_t);
		if (t > 0 * max_t && t <0.6 * max_t || t > 0.70*max_t && t < 0.84 *max_t ){
			ind = rand()*rand() % random.size() ; //rand���Q�����Ĕ͈͂��L����
			randP = random[ind];
		}
		else if (t > 0.94*max_t){
			randP = this->P[rand() %  this->P.size()];
		}
		else{
			ind = rand() % random2.size(); //rand���Q�����Ĕ͈͂��L����
			//std::cout<<"ind : " << ind << std::endl;
			
			randP = random2[ind];
			std::cout << "ind : "<< ind << "posi : "<< randP << std::endl;
		}
		//std::cout <<"ind : "<<ind << " randP : "<< randP << std::endl;
		//randP = cv::Point2f(rand() % 500 + 1, rand() % 500 + 1); //500x500�̐����`�̃����_���v���b�g
		//if ((randP.x - 250)*(randP.x - 250) + (randP.y - 250)*(randP.y - 250) > 200 * 200) continue;
		cv::circle(cpimg, randP, 4, cv::Scalar(0, 255, 0), -1, CV_AA);
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
		distM = 0; //�}���n�b�^������

		std::vector<int> ne;
		std::vector<int> ne2;
		std::vector<int> linked;

		som[index].p.x = som[index].p.x + res.x * learncoeff/3*exp(-(distM*distM) / learncoeff*learncoeff);
		som[index].p.y = som[index].p.y + res.y * learncoeff/3*exp(-(distM*distM) / learncoeff*learncoeff);
		linked.push_back(index);
		ne = som[index].link;
		distM++;
		int flag = 0;
		//showSOM(index ,defo);
		while (1){
			for (auto it : ne){
				res.x = randP.x - som[it].p.x;
				res.y = randP.y - som[it].p.y;
				som[it].p.x = som[it].p.x + res.x * learncoeff/4 * exp(-(distM*distM) / learncoeff*learncoeff);
				som[it].p.y = som[it].p.y + res.y * learncoeff/4 * exp(-(distM*distM) / learncoeff*learncoeff);
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
		for (auto it : som){
			cv::circle(cpimg, it.p, 3, cv::Scalar(0, 0, 0), -1, CV_AA);
		}
		edgeline(som, cpimg);
		if (int(t) % 100 == 0){

			cv::imshow("s", cpimg);
			cv::waitKey(2);
		}
		t++;
		index = 0;
		count = 0;
	}
}

void SOM::Imgproc(cv::Mat &src, cv::Mat &dst){

	cv::cvtColor(src, src, CV_BGR2GRAY);

	cv::threshold(src, dst, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	imshow("input_image",src);

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
		//std::cout << std::endl;
	}
	//std::cout << p[68684] << std::endl;
	return p;
}

std::vector<cv::Point2f> SOM::storeBorderPoint(){
	std::vector<cv::Point2f> p;
	cv::Point2f mid,mid2;
	//for (int i = 0; i < P.size(); i++){
	//	p.push_back(P[i]);
	//	if (i + 1 < P.size()){
	//		mid = cv::Point2f((P[i+1].x + 2*P[i].x) / 3, (P[i+1].y + 2*P[i].y) / 3);
	//		mid2 = cv::Point2f((2*P[i + 1].x + P[i].x) / 3 , (2*P[i + 1].y + P[i].y) / 3 * 2);
	//		p.push_back(mid);
	//		p.push_back(mid2);
	//	}
	//	else{
	//		mid = cv::Point2f((P[0].x + 2*P[i].x) / 3, (P[0].y + 2*P[i].y) / 3);
	//		mid2 = cv::Point2f((2*P[0].x + P[i].x) / 3 , (2*P[0].y + P[i].y) / 3 );
	//		p.push_back(mid);
	//		p.push_back(mid2);
	//	}
	//}

	for (int i = 0; i < P.size(); i++){
		p.push_back(P[i]);
		if (i+1<P.size()){
			for (int j = 1; j < 101; j++){
					mid = cv::Point2f(((100 - j)*P[i + 1].x + j * P[i].x) / 100,
									  ((100 - j)*P[i + 1].y + j * P[i].y) / 100);
					//mid2 = cv::Point2f((P[i + 1].x + i * P[i].x) / (100), (P[i + 1].y + i * P[i].y) / (100));
					p.push_back(mid);
				}
		}

	
		else{
			for (int j = 1; j < 101; j++){
				
					mid = cv::Point2f(((100 - j)*P[0].x + j * P[i].x) / (100),
						              ((100 - j)*P[0].y + j * P[i].y) / (100));
					//mid2 = cv::Point2f((P[i + 1].x + i * P[i].x) / (100), (P[i + 1].y + i * P[i].y) / (100));
					p.push_back(mid);
			}
		}
	}
	return p;
}

bool cmp(int A, int B){
	return A < B ? 1 : 0;
}