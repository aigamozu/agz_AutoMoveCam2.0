#include "Control.h"

#define PROGRAM 2


/////////////////////////////////////////////////////////////////////////////////
//
//	Constructor
//	
////////////////////////////////////////////////////////////////////////////////
Control::Control(int w, int h){
	//水田の縦横幅を格納
	width = w;
	height = h;

	// ヒートマップ用配列のメモリ動的確保
	small_area = new int*[width / 100 * 5];
	for (int i = 0; i < width / 100 * 5; i++){
		small_area[i] = new int[height / 100 * 5];
	}

	// ヒートマップ用配列の初期化
	for (int i = 0; i < width / 100 * 5; i++){
		for (int j = 0; j < height / 100 * 5; j++){
			small_area[i][j] = 0;
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////
//
//	is_updateTarget
//	
//	ロボットがターゲットの半径５０cm以内に入ったとき, 次のターゲットに移る.
// 
// return : true or false　		( true : ターゲットに入った, false : 入っていない )
////////////////////////////////////////////////////////////////////////////////
bool Control::is_updateTarget(void){

	bool result = false;

	if (PROGRAM == 0 || PROGRAM == 2){
		int dx = nowPoint.x - nowTarget_itr->point.x;
		int dy = nowPoint.y - nowTarget_itr->point.y;
		double d = sqrt(dx * dx + dy * dy);

		// ターゲットの半径５０cm以内の領域に入ったら訪問完了->ターゲットを移す
		if (d < 50.0) {
			result = true;
			nowTarget_itr++;
		}
	}

	if (PROGRAM == 1){
		// 右側のターゲットに向かっている場合
		if (nowTarget_itr->n % 2){
			// 右側のターゲット区間に入ったとき
			if (width - 200 < nowPoint.x && nowPoint.x < width - 100){
				if (50 < nowPoint.y && nowPoint.y < height - 50){
					result = true;
					// ターゲットの更新
					nowTarget_itr++;
				}
			}
		}
		// 左側のターゲットに向かっている場合
		else{
			// 右側のターゲット区間に入ったとき
			if (100 < nowPoint.x && nowPoint.x < 200){
				if (50 < nowPoint.y && nowPoint.y < height - 50){
					result = true;
					// ターゲットの更新
					nowTarget_itr++;
				}
			}
		}
	}

	// 最後のターゲットまで訪問したら最初のターゲットに戻る
	if (nowTarget_itr == allTarget.end()){
		nowTarget_itr = allTarget.begin();
	}

	return result;

}

/////////////////////////////////////////////////////////////////////////////////
//
//	robot_action
//	
//	ロボットの動作を決定する.
//
//  return : int ( ロボットの動作番号 )
////////////////////////////////////////////////////////////////////////////////
int Control::robot_action(cv::Point2i Previous){

	// ベクトル
	cv::Point2i P0 = nowPoint - Previous;
	cv::Point2i P1 = nowTarget_itr->point - nowPoint;

	// P0とP1のなす角を内積を用いて求める
	double angle = acos(P0.dot(P1) / (sqrt(P0.x * P0.x + P0.y * P0.y) * sqrt(P1.x * P1.x + P1.y * P1.y))) / CV_PI * 180;

	// ロボットの進行方向に対してターゲットが左方向にあるとき
	if (P0.cross(P1) < 0) {
		angle = -angle;
	}

	// ロボットの進行方向に対して前方向にターゲットがある
	if (-30 < angle && angle < 30) {
		action = "f";
		return 1;
	}
	// ロボットの進行方向に対して右方向にターゲットがある
	else if (angle >= 30) {
		action = "r";
		return 2;
	}
	// ロボットの進行方向に対して左方向にターゲットがある
	else {
		action = "l";
		return 4;
	}
}

/////////////////////////////////////////////////////////////////////////////////
//
//	area_count
//	
//	ロボットの訪問回数を求める.
//
//  return : cv::Point2i ( ロボットの座標 )
////////////////////////////////////////////////////////////////////////////////
cv::Point2i Control::area_count(void){

	cv::Point2i p;
	// 位置情報 nowPointから配列の添え字番号を求める
	for (int i = 0; i < width / 100 * 5; i++){
		if (i * 20 <= nowPoint.x && nowPoint.x < (i + 1) * 20){
			p.x = i;
			break;
		}
	}
	for (int j = 0; j < height / 100 * 5; j++){
		if (j * 20 <= nowPoint.y && nowPoint.y < (j + 1) * 20){
			p.y = j;
			break;
		}
	}
	// 訪問回数の更新
	small_area[p.x][p.y] ++;
	return p;
}

/////////////////////////////////////////////////////////////////////////////////
//
//	is_out
//	
//	ロボットが内側領域内にいるかいないか調べる.
//
//	return : void
////////////////////////////////////////////////////////////////////////////////
void Control::is_out(void){
	// 四隅の座標
	cv::Point2i A = { 100, height - 100 }, B = { 100, 100 }, C = { height - 100, 100 }, D = { height - 100, height - 100 };
	// ベクトル
	cv::Point2i BA = A - B, BC = C - B, BP = nowPoint - B;
	cv::Point2i DC = C - D, DA = A - D, DP = nowPoint - D;

	int c1, c2, c3, c4;
	bool flag1 = false, flag2 = false;

	// 外積の計算
	c1 = BA.cross(BP);
	c2 = BP.cross(BC);
	c3 = DC.cross(DP);
	c4 = DP.cross(DA);

	if (c1 < 0 && c2 < 0) {
		flag1 = true;
	}
	if (c3 < 0 && c4 < 0) {
		flag2 = true;
	}
	if (flag1 && flag2) {
		out = "IN";
	}
	else out = "OUT";
}

/////////////////////////////////////////////////////////////////////////////////
//
//	plot_target
//	
//	ターゲットとロボットの状態をプロットする.
//
//  return : void 
////////////////////////////////////////////////////////////////////////////////
void Control::plot_target(cv::UMat &img, cv::Point2i Previous ){

	// すべてのターゲットのプロット（水色）
	for (std::vector<target>::iterator itr = allTarget.begin(); itr != allTarget.end(); itr++) {
		cv::putText(img, std::to_string(itr->n), cv::Point(itr->point), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 0.5, CV_AA);
		if(itr->point.x >= width || itr->point.y >= height){
		printf("[%d]  %d, %d\n", itr->n, itr->point.x, itr->point.y);
		}
	}

	// 1フレーム前の座標との移動量を線で表現
	line(img, nowPoint, Previous, cv::Scalar(255, 0, 0), 2, CV_AA);

	// 内外判定結果の表示
//	cv::putText(img, out, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

	// ロボットの動作の表示
//	cv::putText(img, action, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

}

/////////////////////////////////////////////////////////////////////////////////
//
//	plot_transform_target
//	
//  逆射影変換後のSOMを描画
// 
//  return : void 
////////////////////////////////////////////////////////////////////////////////

void Control::plot_transform_target(cv::UMat &img, cv::Point2i Previous, cv::Mat H){
	std::vector<target> t = allTarget;
	
	// H = [3,3] 行列
	double a = H.at<double>(0, 0);
	double b = H.at<double>(0, 1);
	double c = H.at<double>(0, 2);
	double d = H.at<double>(1, 0);
	double e = H.at<double>(1, 1);
	double f = H.at<double>(1, 2);
	double g = H.at<double>(2, 0);
	double h = H.at<double>(2, 1);
	double i = H.at<double>(2, 2);


	//SOMの全ニューロンの座標を逆変換
	for (int j = 0; j < t.size(); j++){
		
		cv::Point2f temp = t[j].point;
		t[j].point.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
		t[j].point.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);
		if (nowTarget_itr->n != t[j].n){

		}
		cv::putText(img, std::to_string(t[j].n), cv::Point(t[j].point), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 0.5, CV_AA);
	}


	// ※ターゲットに対しても座標変換を適用

	// 現在向かうべきターゲットのプロット（黒）
	cv::Point2f temp = nowTarget_itr->point;
	cv::Point2f target, pre, now;
	target.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
	target.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);

	temp = Previous;
	pre.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
	pre.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);

	temp = nowPoint;
	now.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
	now.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);
	line(img, now, pre, cv::Scalar(255, 0, 0), 2, CV_AA);

	// 内外判定結果の表示

//	cv::putText(img, out, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

	// ロボットの動作の表示
//	cv::putText(img, action, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

}

/////////////////////////////////////////////////////////////////////////////////
//
//	heatmap
//	
//	ヒートマップ作成
// 
//  return : void
////////////////////////////////////////////////////////////////////////////////

void Control::heatmap(cv::Point2i pos, cv::Mat *img, cv::Mat *bar){


	//	１マスのピクセル数 10x10
	int size_x = 10 * 500 / width, size_y = 10 * 500 / height;
	static int max_count = 50;

	// 配列の添え字番号から座標を求め、２で割る
	int x = (pos.x * 20 + 10) * 500 / width;
	int y = (pos.y * 20 + 10) * 500 / height;
	// 訪問回数
	int count = small_area[pos.x][pos.y];

	// カウントの最大値の更新
	if (count > max_count){
		max_count = count;
	}

	// 画像のサイズ分だけループを回す
	for (int i = y - size_y; i < y + size_y; i++){
		cv::Vec3b* ptr = img->ptr<cv::Vec3b>(i); // i行目のポインタを取得
		for (int j = x - size_x; j < x + size_x; j++){
			// countを色相に変換
			int brightness = count;
			float h = 240.0 - 240.0 / max_count * (float)brightness;

			// HSV -> BGR に変換
			float s = 1.0;
			float v = 1.0;
			const float u = v * 255.0;
			const int id = (int)floor(h / 60.0) % 6;
			const float fr = (h / 60.0) - id;
			const float p = u * (1.0f - s);
			const float q = u * (1.0f - s*fr);
			const float t = u * (1.0f - s*(1.0f - fr));

			cv::Vec3b bgr = ptr[j];
			
			
			//訪問回数に応じてプロットの色を変化
			switch (id){
			case 0:
				ptr[j] = cv::Vec3b(p, t, u);

				break;
			case 1:
				ptr[j] = cv::Vec3b(p, u, q);

				break;
			case 2:
				ptr[j] = cv::Vec3b(t, u, p);

				break;
			case 3:
				ptr[j] = cv::Vec3b(u, q, p);

				break;
			case 4:
				ptr[j] = cv::Vec3b(u, p, t);

				break;
			default:
				ptr[j] = cv::Vec3b(q, p, u);

				break;

			}
		}
	}

	// 区画のプロット
	for (int i = 0; i <= width; i += 100) {
		for (int j = 0; j <= height; j += 100) {
			line(*img, cv::Point(i * 500 / width, j * 500 / height), cv::Point(i * 500 / width, 500), cv::Scalar(200, 200, 200), 2);
			line(*img, cv::Point(i * 500 / width, j * 500 / height), cv::Point(500, j * 500 / height), cv::Scalar(200, 200, 200), 2);
		}
	}
	// 画像の結合 bar : カラーバー, concat_img : ヒートマップ
	vconcat(*bar, *img, concat_img);

	int value = max_count / 5;
	for (int i = 0; i < 5; i++){
		cv::putText(concat_img, std::to_string(i*value), cv::Point(0 + 100 * i, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 1.0, CV_AA);
	}
	//cv::imshow("heatmap", concat_img);
}


////////////////////////////////////////////////
//
//	set function
//
//  掃引方針の初期設定
//
////////////////////////////////////////////////

void Control::set_target(SOM som) {
	target t;

	allTarget.clear();

	int num = 0;
	if (PROGRAM == 0){
		for (int j = 0; j < (height / 100) - 2; j++) {
			// 左から右へターゲットを設定する
			if (j % 2 == 0) {
				for (int i = 0; i < (width / 100) - 2; i++) {
					t.point = { (i + 1) * width / (width / 100) + 50, height - (j + 1) * height / (height / 100) - 50 };
					t.n = num;
					allTarget.push_back(t);
					num++;
				}
			}
			// 右から左へターゲットを設定する
			else {
				for (int i = (width / 100) - 3; i >= 0; i--) {
					t.point = { (i + 1) * width / (width / 100) + 50, height - (j + 1) * height / (height / 100) - 50 };
					t.n = num;
					allTarget.push_back(t);
					num++;
				}
			}
		}
	}

	if (PROGRAM == 1){
		for (int i = 0; i < (height / 100) - 2; i++){
			// 左側のターゲット設定
			t.point = { 150, height - (i + 1) * height / (height / 100) - 50 };
			t.n = num;
			allTarget.push_back(t);
			num++;

			// 右側のターゲット設定
			t.point = { width - 150, height - (i + 1) * height / (height / 100) - 50 };
			t.n = num;
			allTarget.push_back(t);
			num++;
		}
	}

	if (PROGRAM == 2){
		for (int j = 1; j < (height / 100)-1; j++) {
			// 左から右へターゲットを設定する
			if (j % 2 != 0){
				for (int i = 1; i < (width / 100)-1; i++) {

					t.point = som.calc_centerPoint((width / 100 + 1)*i + j,t.neighbor);
					t.n = num;
					allTarget.push_back(t);
					t.neighbor.clear();
					num++;

				}
			}
			// 右から左へターゲットを設定する
			else {
				for (int i = (width / 100) - 2; i >= 1; i--) {
					t.point = som.calc_centerPoint((width / 100 + 1)*i + j,t.neighbor);
					t.n = num;
					allTarget.push_back(t);
					t.neighbor.clear();
					num++;

				}
			}

			t.neighbor.clear();
		}
	}

	nowTarget_itr = allTarget.begin();
}

////////////////////////////////////////////////
//
//	set function
//
//  現在の座標の設定
////////////////////////////////////////////////
void Control::set_point(cv::Point2i p){
	nowPoint = p;
}

////////////////////////////////////////////////
//
//	get function
//
//  現在のターゲットのイテレータ取得
////////////////////////////////////////////////
int Control::get_target(void){
	return nowTarget_itr->n;
}

////////////////////////////////////////////////
//
//	get function
//
//  現在のターゲットの近傍情報を取得
////////////////////////////////////////////////
std::vector<int> Control::get_nowTargetArea(void){
	return nowTarget_itr->neighbor;
}
