﻿#include "Xbee_com.h"
#include "Img_proc.h"
#include "Control.h"
#include <conio.h>
#include <fstream>
#include <time.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>

//using namespace System
//最終更新 2017/05/04


#define GRAVITY 1             //画像中の領域 : 0  注目領域 : 1
#define CAM_ID 0	          //カメラID
#define Rice_Field_Width  10  //水田横幅(m単位)
#define Rice_Field_Height 10  //水田縦幅(m単位)
#define SOM_XDivide 10	  //SOM分割数 x方向
#define SOM_YDivide 10	  //SOM分割数 y方向

const LPCSTR com = "COM3";		//COMポート名
std::vector<cv::Point2f> Pos;	//水田の四隅の点
std::vector<cv::Point2f> Pos2;

cv::Point2i target, P0[5] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }, P1 = { 0, 0 };
cv::Point2i pre_point; // @comment Point構造体<int型>
int action;			   //ロボットの動作変数 1:前進 2:右旋回 4:左

SOM s = SOM();
SOM s2 = SOM();

//int chval = ;

// データ出力用csvファイル名作成関数　
std::string setFilename(std::string str){
	time_t now = time(NULL);
	struct tm * pnow = localtime(&now);
	char time[32];
	std::string c = ".csv";
	std::string data = "./data/";

	//@comment sprintfを使ってint型をstringに変換
	sprintf(time, "%d_%d_%d_%d_%d", pnow->tm_year + 1900, pnow->tm_mon + 1,
		pnow->tm_mday, pnow->tm_hour, pnow->tm_min);

	return data + str + time + c; //@comment 生成されたファイル名
}

std::string setFilename2(std::string str){
	time_t now = time(NULL);
	struct tm * pnow = localtime(&now);
	char time[32];
	std::string c = ".avi";
	std::string data = "./videos/";

	//@comment sprintfを使ってint型をstringに変換
	sprintf(time, "%d_%d_%d_%d_%d", pnow->tm_year + 1900, pnow->tm_mon + 1,
		pnow->tm_mday, pnow->tm_hour, pnow->tm_min);

	return data + str + time + c; //@comment 生成されたファイル名
}


//PWM値変更関数

void changeFrontPWM(int thres, void*){


	lPwm[1] = chPWM[thres];
	rPwm[1] = chPWM[thres];

	std::cout << "前進 :  左モータ -- " << int(lPwm[1]) << " 右モータ -- " << int(rPwm[1]) << std::endl;
}

void changeLPWM(int thres, void*){

	rPwm[4] = chPWM[thres];
	rPwm[5] = chPWM[thres];
	std::cout << "左旋回 :  左モータ -- " << int(lPwm[4]) << " 右モータ -- " << int(rPwm[4]) << std::endl;
	//std::cout << "L : " << lPwm << "R : " << int(rPwm << std::endl;
}

void changeRPWM(int thres, void*){

	lPwm[2] = chPWM[thres];
	lPwm[3] = chPWM[thres];
	std::cout << "右旋回 :  左モータ -- " << int(lPwm[2]) << " 右モータ -- " << int(rPwm[2]) << std::endl;
	//std::cout << "L : " << lPwm << "R : " << rPwm << std::endl;
}


// マウスクリック取得座標位置に円をプロット
void drawPoint(cv::Mat* img, cv::Point2i point){
	cv::circle(*img, point, 8, cv::Scalar(0, 255, 0), -1, CV_AA);
	cv::imshow("getCoordinates", *img);
}

// 水田領域の座標取得用関数
void getCoordinates(int event, int x, int y, int flags, void* param)
{
	//クリック点描画のための画像生成
	cv::Mat* image = static_cast<cv::Mat*>(param);
	static int count = 0;

	// 4点クリックされた場合は通常処理を行う
	// クリックされない場合はエラー処理を行う

	switch (event) {
		// 左クリックが押された時
	case CV_EVENT_LBUTTONDOWN:
		Pos.push_back(cv::Point2f(x, y));
		drawPoint(image, cv::Point2i(x, y));
		std::cout << " " << ++count << "点目 : " << "x : " << x << ", y : " << y << std::endl;
		if (count > 3){
			cv::imshow("getCoordinates", *image);
			cv::waitKey(200);
			cv::destroyAllWindows();
			break;
		}
		break;

		// 右クリックされたとき
	case CV_EVENT_RBUTTONDOWN:
		if (count < 4){

			std::cout << std::endl;
			std::cout << " 4点以上指定してください" << std::endl << std::endl;
			std::cout << " 何かキーを押してください" << std::endl;

			exit(1);

		}
		cv::destroyAllWindows();
		break;
	}
}


//画像を取得し,水田領域を設定
void setUp(LPCSTR com, HANDLE &hdl, Img_Proc &imp, int w, int h){

	int width = w, height = h;
	cv::Mat field;
	cv::UMat src_frame, dst_img;


	width *= 100;
	height *= 100;


	imp.setField(width, height);
	nm30_init();
	nm30_set_panorama_mode(1, 11); //@comment 魚眼補正

	//@comment 始めの１０フレームは読み飛ばす
	for (int i = 0; i < 10; i++) {
		imp.getFrame().copyTo(src_frame);//@comment 1フレーム取得
	}

	std::cout << std::endl;
	std::cout << " 水田の領域を左下から時計回りになるように４点クリックしてください" << std::endl;
	std::cout << " 間違えた場合はプログラムを再起動してください" << std::endl << std::endl;

	//------------------座標取得-----------------------------------------------
	//画像中からマウスで4 ~ 10点を取得その後右クリックすると変換処理が開始する
	cv::namedWindow("getCoordinates");
	cv::imshow("getCoordinates", src_frame);
	cv::moveWindow("getCoordinates", 550, 0);
	src_frame.copyTo(field);
	cv::setMouseCallback("getCoordinates", getCoordinates, (void *)&field);

	cv::waitKey(0);
	cv::destroyAllWindows();

	//------------------透視変換-----------------------------------------------

	imp.Perspective(src_frame, dst_img, Pos);

	Pos2.push_back(cv::Point(5, dst_img.rows - 5));
	Pos2.push_back(cv::Point(5, 5));
	Pos2.push_back(cv::Point(dst_img.cols - 5, 5));
	Pos2.push_back(cv::Point(dst_img.cols - 5, dst_img.rows - 5));

	cv::Point pt2[10]; //任意の4点を配列に格納
	for (int i = 0; i < Pos2.size(); i++){
		pt2[i] = Pos2[i];
	}
	//s2.set_size(width,height);
	s2.set_size(SOM_XDivide * 100, SOM_XDivide * 100);
	s2.set_pos(Pos2);
	s2.set_img(dst_img);
	cv::Mat_<cv::Vec3b> ss2(dst_img.size());
	for (int j = 0; j < dst_img.rows; j++){
		for (int i = 0; i < dst_img.cols; i++){
			ss2(j, i) = cv::Vec3b(255, 255, 255);
		}
	}
	cv::fillConvexPoly(ss2, pt2, Pos.size(), cv::Scalar(200, 200, 200));//多角形を描画
	s2.Init2(ss2);
	cv::destroyAllWindows();
}

//制御ループ
void Moving(HANDLE &arduino, Xbee_com &xbee, Img_Proc &imp){

	//////////////////////////////////////
	cv::VideoWriter video, video2, video3, video4, video5;
	video.open(setFilename2("Transform"), CV_FOURCC('M', 'J', 'P', 'G'), 4, cv::Size(640, 480), true);
	if (video.isOpened()){
		//std::cout << "open" << std::endl;
	}

	video2.open(setFilename2("Camera"), CV_FOURCC('M', 'J', 'P', 'G'), 4, cv::Size(640, 480), true);
	if (video2.isOpened()){
		//std::cout << "open" << std::endl;
	}

	video3.open(setFilename2("Heatmap"), CV_FOURCC('M', 'J', 'P', 'G'), 4, cv::Size(640, 480), true);
	if (video3.isOpened()){
		//std::cout << "open" << std::endl;
	}
	video4.open(setFilename2("Original_T"), CV_FOURCC('M', 'J', 'P', 'G'), 4, cv::Size(640, 480), true);
	if (video4.isOpened()){
		//std::cout << "open" << std::endl;
	}
	//video5.open(setFilename2("Binary"), CV_FOURCC('M', 'J', 'P', 'G'), 4, cv::Size(640, 480), true);
	//if (video5.isOpened()){
	//	//std::cout << "open" << std::endl;
	//}
	//////////////////////////////////////
	cv::Mat heat;
	cv::Mat element = cv::Mat::ones(3, 3, CV_8UC1); //2値画像膨張用行列
	cv::Mat heatmap_img(cv::Size(500, 500), CV_8UC3, cv::Scalar(255, 255, 255));
	int frameNum = 0;								//フレーム数保持変数
	cv::UMat src, dst, colorExtra, pImg, binari_2, copyImg, copyImg2;
	cv::Point2f sz = imp.getField();
	Control control(sz.x, sz.y);
	control.set_target(s2);
	char command = 's';
	int ypos;
	int ydef = 0;	//補正なし重心座標値
	int num = 0;	// ターゲットの訪問回数更新
	std::ofstream ofs(setFilename("Coordinate")); // @comment　ロボット座標用csv
	int chLval = 8, chRval = 8, chFrontval = 10;
	std::string pwmparaStr = "PWMパラメータ設定画面";
	cv::namedWindow(pwmparaStr, 1);


	//ロボット座標csv
	ofs << imp.getField().x << ", " << imp.getField().y << std::endl;
	//ofs << Rice_Field_Width << ", " << Rice_Field_Height << std::endl;
	ofs << "x軸, y軸（補正なし）, ypos（補正あり）" << std::endl;



	std::cout << " a: 自動掃引" << std::endl;
	std::cout << " s: 停止" << std::endl;
	std::cout << " q もしくはウィンドウ右上の×ボタン : 終了" << std::endl;

	while (1){
		imp.getFrame().copyTo(src);
		if (frameNum % 1 == 0){
			if (_kbhit()){
				command = _getch();
				if (command == 's') {
					xbee.sentManualCommand(byte(0x00), arduino);
					std::cout << " 停止" << std::endl << std::endl;
				}
				if (command == 'q') {
					//xbee.sentManualCommand(byte(0x01), arduino);
					std::cout << " プログラムを終了します " << std::endl << std::endl;
					cv::destroyAllWindows();
					ofs.close(); //@comment ファイルストリームの解放

					exit(0);
					system("exit");
				}
				if (command == 'a') {
					xbee.sentManualCommand(byte(0x01), arduino);
					std::cout << " 自動掃引" << std::endl << std::endl;
				}
			}

			cv::createTrackbar("右旋回", pwmparaStr, &chLval, 16, changeLPWM);
			cv::createTrackbar("左旋回", pwmparaStr, &chRval, 16, changeRPWM);
			cv::createTrackbar("前進", pwmparaStr, &chFrontval, 16, changeFrontPWM);

			/*	if (command == 'a'){
			xbee.sentAigamoCommand(byte(0x01), arduino,lPwm,rPwm);
			}*/


			//std::cout << "L : " << int(lPwm[1]) << "R : " << int(rPwm[1]) << std::endl;

			//@comment 画像をリサイズ(大きすぎるとディスプレイに入りらないため)
			src.copyTo(copyImg);
			cv::warpPerspective(src, dst, imp.getPersMat(), cv::Size(src.cols, src.rows), CV_INTER_LINEAR);
			dst.copyTo(copyImg2);

			//@comment hsvを利用して赤色を抽出
			//入力画像、出力画像、変換、h最小値、h最大値、s最小値、s最大値、v最小値、v最大値 h:(0-180)実際の1/2
			//imp.colorExtraction(dst, colorExtra, CV_BGR2HSV, h_value, h_value2, s_value, 255, v_value, 255);
			imp.colorExtraction(dst, colorExtra, CV_BGR2HSV, 160, 10, 70, 255, 70, 255);
			colorExtra.copyTo(pImg);
			cv::cvtColor(colorExtra, colorExtra, CV_BGR2GRAY);//@comment グレースケールに変換

			//----------------------二値化-----------------------------------------------
			cv::threshold(colorExtra, binari_2, 0, 255, CV_THRESH_BINARY);
			cv::dilate(binari_2, binari_2, element, cv::Point(-1, -1), 5); //膨張処理3回 最後の引数で回数を設定
			cv::erode(binari_2, binari_2, element, cv::Point(-1, -1), 5); //膨張処理3回 最後の引数で回数を設定
			imshow("bin_img", binari_2);
			//---------------------面積計算,重心取得-----------------------------------------------
			//取得した領域の中で一番面積の大きいものを対象としてその対象の重心を求める。
			cv::Point2i point = imp.serchMaxArea(binari_2, pImg);
			if (!GRAVITY)
			{
				point = imp.calculate_center(binari_2);//@comment momentで白色部分の重心を求める
				//std::cout << "posion: " << point.x << " " << point.y << std::endl;//@comment 重心点の表示
			}


			//ypos : 補正あり,  ydef : 補正なし (y座標) : matlabでの処理の際に使用
			if (point.x != 0) {
				ypos = 480 - (point.y + 6 * ((1000 / point.y) + 1));
				ydef = 480 - point.y;//@comment 補正なしｙ重心 
				//std::cout << point.x << " " << ypos << std::endl; //@comment 変換画像中でのロボットの座標(重心)
				ydef = float(ydef * 900 / 480);
				//std::cout << "x : " << float(point.x * 900 / 640) << " ydef : " << ydef << std::endl;
				ofs << float(point.x * 900 / 640) << ", " << ydef << ", " << point.y << std::endl; //@comment 変換
			}

			//---------------------ロボットの動作取得------------------------------------
			if (frameNum % 2 == 0){
				//P1 : 
				P1 = cv::Point2f(point.x, point.y);
				if (P1.x != 0 && P1.y != 0) {
					// ターゲットの更新
					if (control.is_updateTarget()){
						//std::cout << "target number : " << control.get_target() << std::endl;
					}
					// 現在のロボットの位置情報の更新
					control.set_point(P1);

					// ロボットの動作決定
					action = control.robot_action(P0[4]);

					// ターゲットの訪問回数更新
					//num = control.target_count();
					cv::Point2f hPoint = control.area_count();


					//test
					/*hPoint.x = 0.0;
					hPoint.y = 0.0;*/



					//std::cout << "hPoint : " << hPoint.x <<" : " <<hPoint.y << std::endl;
					heat = control.heatmap(cv::Point(hPoint.x, hPoint.y)
						, &heatmap_img, &imp.makeColorbar());
					//control.heatmap(control.area_count(), &heatmap_img, &imp.makeColorbar());
					// 内外判定
					control.is_out();
					for (int i = 1; i < 5; i++){
						P0[i] = P0[i - 1];
					}
				}

				else{
					action = 0;
					heat = control.heatmap(cv::Point(0, 0)
						, &heatmap_img, &imp.makeColorbar());
				}
				P0[0] = P1;
			} //ロボットの動作取得

			if (command == 'a'){
				xbee.sentAigamoCommand(action, arduino, lPwm, rPwm);
			}

			cv::UMat test, test2;
			src.copyTo(test);
			dst.copyTo(test2);

			cv::warpPerspective(test2, test, imp.getInvPerse(), cv::Size(test.cols, test.rows), CV_INTER_LINEAR);


			//-------------------重心点のプロット----------------------------------------- 
			if (!point.y == 0) { //@comment point.y == 0の場合はexceptionが起こる( 0除算 )

				circle(copyImg, imp.calcHomoPoint(cv::Point2f(point)), 8, cv::Scalar(0, 0, 255), -1, CV_AA);
				circle(dst, cv::Point(point.x, point.y), 8, cv::Scalar(0, 0, 255), -1, CV_AA);
			}

			//------------------ターゲットのプロット--------------------------------------

			control.plot_target(dst, P0[4]);
			control.plot_transform_target(copyImg, P0[4], imp.getInvPerse());

			//------------------マス, 直進領域のプロット--------------------------------------

			s2.showSOM3(dst, control.get_nowTargetArea());
			s2.showSOM2(copyImg, imp.getInvPerse(), control.get_nowTargetArea());

			//---------------------表示部分----------------------------------------------

			///////////////////////////////////
			cv::Mat vi, vi2, vi3, v4, transform_original;
			dst.copyTo(vi);
			copyImg.copyTo(vi2);
			heat.copyTo(vi3);
			copyImg2.copyTo(transform_original);
			//binari_2.copyTo(v4);
			//std::cout << "copy done" << std::endl;
			video.write(vi);
			if (!video.isOpened()){
				std::cout << "not Open 1" << std::endl;
			}
			//std::cout << " video1" << std::endl;
			video2.write(vi2);
			if (!video2.isOpened()){
				std::cout << "not Open 2" << std::endl;
			}
			//std::cout << " video2" << std::endl;
			cv::resize(vi3, vi3, cv::Size(640, 480));
			video3.write(vi3);
			if (!video3.isOpened()){
				std::cout << "not Open 2" << std::endl;
			}

			cv::resize(transform_original, transform_original, cv::Size(640, 480));
			video4.write(transform_original);
			if (!video4.isOpened()){
				std::cout << "not Open 2" << std::endl;
			}

			//////////////////////////////////////////

			cv::resize(copyImg, copyImg, cv::Size(400, 400));
			cv::resize(dst, dst, cv::Size(400, 400));
			cv::imshow("camera_image", copyImg);//@comment 出力画像
			cv::imshow("Transform_Img", dst);
			cv::imshow("heatmap", heat);
			if (frameNum == 0){
				cv::moveWindow("camera_image", 400, 0);
				cv::moveWindow("Transform_Img", 800, 0);
			}
			cv::waitKey(1);
		}
		frameNum++;
	}
	ofs.close(); //@comment ファイルストリームの解放
}

//ロボット自動制御
void AutoMove(){
	HANDLE hdl;							//COMポート通信ハンドル
	Img_Proc imp = Img_Proc(CAM_ID);    //画像処理用クラス
	Xbee_com xbee = Xbee_com(com, hdl); 	//xbee通信の初期化
	setUp(com, hdl, imp, Rice_Field_Width, Rice_Field_Height); //初期セットアップ
	Moving(hdl, xbee, imp);				//自動制御ループ
}

int main(int argc, char *argv[]){
	AutoMove();
	return 0;
}
