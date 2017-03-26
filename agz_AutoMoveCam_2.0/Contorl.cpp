#include "Control.h"

#define PROGRAM 2



//test
/////////////////////////////////////////////////////////////////////////////////
//
//	Constructor
//	
////////////////////////////////////////////////////////////////////////////////
Control::Control(int w, int h){
	width = w;
	height = h;

	// �q�[�g�}�b�v�p�z��̃��������I�m��
	small_area = new int*[width / 100 * 5];
	for (int i = 0; i < width / 100 * 5; i++){
		small_area[i] = new int[height / 100 * 5];
	}

	// �z��̏�����
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
//	���{�b�g���^�[�Q�b�g�̔��a�R�Ocm�ȓ��ɓ������Ƃ�, ���̃^�[�Q�b�g�Ɉڂ�.
////////////////////////////////////////////////////////////////////////////////
bool Control::is_updateTarget(void){

	bool result = false;

	if (PROGRAM == 0 || PROGRAM == 2){
		int dx = nowPoint.x - nowTarget_itr->point.x;
		int dy = nowPoint.y - nowTarget_itr->point.y;
		double d = sqrt(dx * dx + dy * dy);

		// �^�[�Q�b�g�̔��a�R�Ocm�ȓ��̗̈�ɓ�������K�⊮��->�^�[�Q�b�g���ڂ�
		if (d < 50.0) {
			result = true;
			nowTarget_itr++;
		}
	}

	if (PROGRAM == 1){
		// �E���̃^�[�Q�b�g�Ɍ������Ă���ꍇ
		if (nowTarget_itr->n % 2){
			// �E���̃^�[�Q�b�g��Ԃɓ������Ƃ�
			if (width - 200 < nowPoint.x && nowPoint.x < width - 100){
				if (50 < nowPoint.y && nowPoint.y < height - 50){
					result = true;
					// �^�[�Q�b�g�̍X�V
					nowTarget_itr++;
				}
			}
		}
		// �����̃^�[�Q�b�g�Ɍ������Ă���ꍇ
		else{
			// �E���̃^�[�Q�b�g��Ԃɓ������Ƃ�
			if (100 < nowPoint.x && nowPoint.x < 200){
				if (50 < nowPoint.y && nowPoint.y < height - 50){
					result = true;
					// �^�[�Q�b�g�̍X�V
					nowTarget_itr++;
				}
			}
		}
	}

	// �Ō�̃^�[�Q�b�g�܂ŖK�₵����ŏ��̃^�[�Q�b�g�ɖ߂�
	if (nowTarget_itr == allTarget.end()){
		nowTarget_itr = allTarget.begin();
	}

	return result;

}

/////////////////////////////////////////////////////////////////////////////////
//
//	robot_action
//	
//	���{�b�g�̓�������肷��.
////////////////////////////////////////////////////////////////////////////////
int Control::robot_action(cv::Point2i Previous){

	// �x�N�g��
	cv::Point2i P0 = nowPoint - Previous;
	cv::Point2i P1 = nowTarget_itr->point - nowPoint;

	// P0��P1�̂Ȃ��p����ς�p���ċ��߂�
	double angle = acos(P0.dot(P1) / (sqrt(P0.x * P0.x + P0.y * P0.y) * sqrt(P1.x * P1.x + P1.y * P1.y))) / CV_PI * 180;

	// ���{�b�g�̐i�s�����ɑ΂��ă^�[�Q�b�g���������ɂ���Ƃ�
	if (P0.cross(P1) < 0) {
		angle = -angle;
	}

	// ���{�b�g�̐i�s�����ɑ΂��đO�����Ƀ^�[�Q�b�g������
	if (-30 < angle && angle < 30) {
		action = "f";
		return 1;
	}
	// ���{�b�g�̐i�s�����ɑ΂��ĉE�����Ƀ^�[�Q�b�g������
	else if (angle >= 30) {
		action = "r";
		return 2;
	}
	// ���{�b�g�̐i�s�����ɑ΂��č������Ƀ^�[�Q�b�g������
	else {
		action = "l";
		return 4;
	}
}

/////////////////////////////////////////////////////////////////////////////////
//
//	area_count
//	
//	���{�b�g�̖K��񐔂����߂�.
////////////////////////////////////////////////////////////////////////////////
cv::Point2i Control::area_count(void){

	cv::Point2i p;
	// �ʒu��� nowPoint����z��̓Y�����ԍ������߂�
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
	// �K��񐔂̍X�V
	small_area[p.x][p.y] ++;
	return p;
}

/////////////////////////////////////////////////////////////////////////////////
//
//	is_out
//	
//	���{�b�g�������̈���ɂ��邩���Ȃ������ׂ�.
////////////////////////////////////////////////////////////////////////////////
void Control::is_out(void){
	// �l���̍��W
	cv::Point2i A = { 100, height - 100 }, B = { 100, 100 }, C = { height - 100, 100 }, D = { height - 100, height - 100 };
	// �x�N�g��
	cv::Point2i BA = A - B, BC = C - B, BP = nowPoint - B;
	cv::Point2i DC = C - D, DA = A - D, DP = nowPoint - D;

	int c1, c2, c3, c4;
	bool flag1 = false, flag2 = false;

	// �O�ς̌v�Z
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
//	�^�[�Q�b�g�ƃ��{�b�g�̏�Ԃ��v���b�g����.
////////////////////////////////////////////////////////////////////////////////
void Control::plot_target(cv::UMat &img, cv::Point2i Previous ){

	// ���ׂẴ^�[�Q�b�g�̃v���b�g�i���F�j
	for (std::vector<target>::iterator itr = allTarget.begin(); itr != allTarget.end(); itr++) {

		//cv::circle(img, cv::Point(itr->point), 20, cv::Scalar(255, 255, 0), 3, 4);
		cv::putText(img, std::to_string(itr->n), cv::Point(itr->point), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 0.5, CV_AA);
	}
	
	// ���݌������ׂ��^�[�Q�b�g�̃v���b�g�i���j
	//cv::circle(img, cv::Point(nowTarget_itr->point), 20, cv::Scalar(0, 0, 0), 3, 4);

	line(img, nowPoint, Previous, cv::Scalar(255, 0, 0), 2, CV_AA);

	// ���O���茋�ʂ̕\��

	cv::putText(img, out, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

	// ���{�b�g�̓���̕\��
	cv::putText(img, action, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);



}

/////////////////////////////////////////////////////////////////////////////////
//
//	plot_trans_target
//	
//	�^�[�Q�b�g�ƃ��{�b�g�̏�Ԃ��v���b�g����.
////////////////////////////////////////////////////////////////////////////////

void Control::plot_transform_target(cv::UMat &img, cv::Point2i Previous, cv::Mat H){
	std::vector<target> t = allTarget;
	double a = H.at<double>(0, 0);
	double b = H.at<double>(0, 1);
	double c = H.at<double>(0, 2);
	double d = H.at<double>(1, 0);
	double e = H.at<double>(1, 1);
	double f = H.at<double>(1, 2);
	double g = H.at<double>(2, 0);
	double h = H.at<double>(2, 1);
	double i = H.at<double>(2, 2);


	for (int j = 0; j < t.size(); j++){
		cv::Point2f temp = t[j].point;
		t[j].point.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
		t[j].point.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);
		if (nowTarget_itr->n != t[j].n){
			//cv::circle(img, t[j].point, 20, cv::Scalar(255, 255, 0), 3, 4);
		}
		cv::putText(img, std::to_string(t[j].n), cv::Point(t[j].point), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 0.5, CV_AA);
	}


	// ���݌������ׂ��^�[�Q�b�g�̃v���b�g�i���j
	cv::Point2f temp = nowTarget_itr->point;
	cv::Point2f target, pre, now;
	target.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
	target.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);
	//cv::circle(img, target, 20, cv::Scalar(0, 0, 0), 3, 4);

	temp = Previous;
	pre.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
	pre.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);

	temp = nowPoint;
	now.x = (temp.x * a + temp.y * b + c) / float(temp.x * g + temp.y * h + i);
	now.y = (temp.x * d + temp.y * e + f) / float(temp.x * g + temp.y * h + i);
	line(img, now, pre, cv::Scalar(255, 0, 0), 2, CV_AA);

	// ���O���茋�ʂ̕\��

	cv::putText(img, out, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

	// ���{�b�g�̓���̕\��
	cv::putText(img, action, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);

}

/////////////////////////////////////////////////////////////////////////////////
//
//	heatmap
//	
//	�q�[�g�}�b�v�쐬
////////////////////////////////////////////////////////////////////////////////

void Control::heatmap(cv::Point2i pos, cv::Mat *img, cv::Mat *bar){


	//	�P�}�X�̃s�N�Z���� 10x10
	int size_x = 10 * 500 / width, size_y = 10 * 500 / height;
	static int max_count = 50;

	// �z��̓Y�����ԍ�������W�����߁A�Q�Ŋ���
	int x = (pos.x * 20 + 10) * 500 / width;
	int y = (pos.y * 20 + 10) * 500 / height;
	// �K���
	int count = small_area[pos.x][pos.y];

	// �J�E���g�̍ő�l�̍X�V
	if (count > max_count){
		max_count = count;
	}

	// �摜�̃T�C�Y���������[�v����
	for (int i = y - size_y; i < y + size_y; i++){
		cv::Vec3b* ptr = img->ptr<cv::Vec3b>(i); // i�s�ڂ̃|�C���^���擾
		for (int j = x - size_x; j < x + size_x; j++){
			// count��F���ɕϊ�
			int brightness = count;
			float h = 240.0 - 240.0 / max_count * (float)brightness;

			// HSV -> BGR �ɕϊ�
			float s = 1.0;
			float v = 1.0;
			const float u = v * 255.0;
			const int id = (int)floor(h / 60.0) % 6;
			const float fr = (h / 60.0) - id;
			const float p = u * (1.0f - s);
			const float q = u * (1.0f - s*fr);
			const float t = u * (1.0f - s*(1.0f - fr));

			cv::Vec3b bgr = ptr[j];
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

	// ���̃v���b�g
	for (int i = 0; i <= width; i += 100) {
		for (int j = 0; j <= height; j += 100) {
			line(*img, cv::Point(i * 500 / width, j * 500 / height), cv::Point(i * 500 / width, 500), cv::Scalar(200, 200, 200), 2);
			line(*img, cv::Point(i * 500 / width, j * 500 / height), cv::Point(500, j * 500 / height), cv::Scalar(200, 200, 200), 2);
		}
	}

	vconcat(*bar, *img, concat_img);

	int value = max_count / 5;
	for (int i = 0; i < 5; i++){
		cv::putText(concat_img, std::to_string(i*value), cv::Point(0 + 100 * i, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 1.0, CV_AA);
	}
	cv::imshow("heatmap", concat_img);
}


////////////////////////////////////////////////
//
//	set function
//
////////////////////////////////////////////////

void Control::set_target(SOM som) {
	target t;

	allTarget.clear();

	int num = 0;
	if (PROGRAM == 0){
		for (int j = 0; j < (height / 100) - 2; j++) {
			// ������E�փ^�[�Q�b�g��ݒ肷��
			if (j % 2 == 0) {
				for (int i = 0; i < (width / 100) - 2; i++) {
					t.point = { (i + 1) * width / (width / 100) + 50, height - (j + 1) * height / (height / 100) - 50 };
					t.n = num;
					allTarget.push_back(t);
					num++;
				}
			}
			// �E���獶�փ^�[�Q�b�g��ݒ肷��
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
			// �����̃^�[�Q�b�g�ݒ�
			t.point = { 150, height - (i + 1) * height / (height / 100) - 50 };
			t.n = num;
			allTarget.push_back(t);
			num++;

			// �E���̃^�[�Q�b�g�ݒ�
			t.point = { width - 150, height - (i + 1) * height / (height / 100) - 50 };
			t.n = num;
			allTarget.push_back(t);
			num++;
		}
	}

	if (PROGRAM == 2){
		for (int i = 0; i < (height / 100); i++) {
			// ������E�փ^�[�Q�b�g��ݒ肷��
			if (i % 2 == 0){
				for (int j = 0; j < (width / 100); j++) {

					t.point = som.calc_centerPoint((width / 100 + 1)*i + j,t.neighbor);
					t.n = num;
					allTarget.push_back(t);
					t.neighbor.clear();
					num++;

				}
			}
			// �E���獶�փ^�[�Q�b�g��ݒ肷��
			else {
				for (int j = (width / 100) - 1; j >= 0; j--) {
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

void Control::set_point(cv::Point2i p){
	nowPoint = p;
}

////////////////////////////////////////////////
//
//	get function
//
////////////////////////////////////////////////
int Control::get_target(void){
	return nowTarget_itr->n;
}

std::vector<int> Control::get_nowTargetArea(void){
	return nowTarget_itr->neighbor;
}