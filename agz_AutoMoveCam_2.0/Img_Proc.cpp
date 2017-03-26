#include "Img_Proc.h"


//�J�����̐ݒ���s���R���X�g���N�^
Img_Proc::Img_Proc(int camId){
	this->cap = cv::VideoCapture(camId);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); //@comment web�J�����̉�����ݒ�
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); //@comment web�J�����̏c����ݒ�

	//@comment �Ăяo���~�X������ΏI��
	if (!this->cap.isOpened()){
		system("PAUSE");
		exit(0);
	}
}

//�̈�̃T�C�Y���i�[
void Img_Proc::setField(int w, int h){
	this->field.x = w;
	this->field.y = h;
}

cv::Mat Img_Proc::makeColorbar(void){
	//////////////////////////////////////////////////////
	//@comment �J���[�o�[�쐬
	//
	/////////////////////////////////////////////////////
	int w = 500;
	int h = 50;
	cv::Mat_<cv::Vec3b> bar(h, w);
	for (int j = 0; j < h; j++)
	{
		for (int i = 0; i < w; i++)
		{
			bar(j, i) = calcPseudoColor(double(i) / (w - 1));
		}
	}
	return bar;
}
cv::Vec3b Img_Proc::calcPseudoColor(double phase){
	double shift = 0.0;
	phase = std::max(std::min(phase, 1.0), 0.0); //0����1��
	shift += CV_PI + CV_PI / 4;     //����Ԃ�
	return cv::Vec3b
		(
		uchar(255 * (sin(1.5*CV_PI*phase + shift + CV_PI) + 1) / 2.0),
		uchar(255 * (sin(1.5*CV_PI*phase + shift + CV_PI / 2) + 1) / 2.0),
		uchar(255 * (sin(1.5*CV_PI*phase + shift) + 1) / 2.0)
		);


}
//�̈�̃T�C�Y���擾
cv::Point2i Img_Proc::getField(){
	return field;
}

//�����ϊ��s��̐ݒ�
cv::Mat Img_Proc::getPersMat(){
	return this->perspective_matrix;
}

//�F���o�摜������ő�ʐς�L����̈�����߂�
cv::Point2i Img_Proc::serchMaxArea(cv::UMat &src, cv::UMat &plot){
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	double max_area = 0;
	int max_area_contour = -1;
	double x = 0;
	double y = 0;
	if (!contours.empty()){
		for (int j = 0; j < contours.size(); j++){
			double area = contourArea(contours.at(j));
			if (max_area < area){
				max_area = area;
				max_area_contour = j;
			}
		}
		int count = contours.at(max_area_contour).size();
		for (int k = 0; k < count; k++){
			x += contours.at(max_area_contour).at(k).x;
			y += contours.at(max_area_contour).at(k).y;
		}
		x /= count;
		y /= count;
		cv::circle(plot, cv::Point(x, y), 50, cv::Scalar(0, 255, 255), 3, 4);
		cv::circle(plot, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), -1, CV_AA);
	}
	return cv::Point2i(x, y);
}

//@comment �d�S�擾�p�֐�
cv::Point2i Img_Proc::calculate_center(cv::UMat &gray)
{
	cv::Point2i center = cv::Point2i(0, 0);
	cv::Moments moment = moments(gray, true);

	if (moment.m00 != 0)
	{
		center.x = (int)(moment.m10 / moment.m00);
		center.y = (int)(moment.m01 / moment.m00);
	}

	return center;
}


//@comment �J�����L�����u���[�V�����p�֐�(gopro�p)
cv::Mat Img_Proc::undist(cv::Mat src_img)
{
	cv::Mat dst_img;

	//@comment �J�����}�g���b�N�X(gopro)
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 469.96, 0, 400, 0, 467.68, 300, 0, 0, 1);
	//@comment �c�ݍs��(gopro)
	cv::Mat distcoeffs = (cv::Mat_<double>(1, 5) << -0.18957, 0.037319, 0, 0, -0.00337);

	undistort(src_img, dst_img, cameraMatrix, distcoeffs);
	return dst_img;
}


//@comment �F���o�p�֐� 
void Img_Proc::colorExtraction(cv::UMat &src, cv::UMat &dst,
	int code,
	int ch1Lower, int ch1Upper, //@comment H(�F��)�@�ŏ��A�ő�
	int ch2Lower, int ch2Upper, //@comment S(�ʓx)�@�ŏ��A�ő�
	int ch3Lower, int ch3Upper  //@comment V(���x)�@�ŏ��A�ő�
	)
{
	cv::Mat colorImage;
	int lower[3];
	int upper[3];

	cv::Mat lut = cv::Mat(256, 1, CV_8UC3);

	cv::cvtColor(src, colorImage, code);

	lower[0] = ch1Lower;
	lower[1] = ch2Lower;
	lower[2] = ch3Lower;

	upper[0] = ch1Upper;
	upper[1] = ch2Upper;
	upper[2] = ch3Upper;

	for (int i = 0; i < 256; i++) {
		for (int k = 0; k < 3; k++) {
			if (lower[k] <= upper[k]) {
				if ((lower[k] <= i) && (i <= upper[k])) {
					lut.data[i*lut.step + k] = 255;
				}
				else {
					lut.data[i*lut.step + k] = 0;
				}
			}
			else {
				if ((i <= upper[k]) || (lower[k] <= i)) {
					lut.data[i*lut.step + k] = 255;
				}
				else {
					lut.data[i*lut.step + k] = 0;
				}
			}
		}
	}
	//@comment LUT���g�p���ē�l��
	cv::LUT(colorImage, lut, colorImage);

	//@comment Channel���ɕ���
	std::vector<cv::Mat> planes;
	cv::split(colorImage, planes);

	//@comment �}�X�N���쐬
	cv::Mat maskImage;
	cv::bitwise_and(planes[0], planes[1], maskImage);
	cv::bitwise_and(maskImage, planes[2], maskImage);

	//@comemnt �o��
	cv::Mat maskedImage;
	src.copyTo(maskedImage, maskImage);
	maskedImage.copyTo(dst);

}

//�J�����摜�擾�p�֐�
cv::UMat Img_Proc::getFrame(){
	cv::Mat src;
	cap >> src; //@comment 1�t���[���擾
	src.copyTo(capImg);
	return capImg;
}

//�����ϊ����s���֐�
void Img_Proc::Perspective(cv::UMat &src, cv::UMat &dst, std::vector<cv::Point2f> &p){
	cv::Point2f pts1[] = { p[0], p[1], p[2], p[3] };

	cv::Point2f pts2[] = { cv::Point2f(0, src.rows), cv::Point2f(0, 0),
		cv::Point2f(src.cols, 0), cv::Point2f(src.cols, src.rows) };

	//@comment �����ϊ��s����v�Z
	this->perspective_matrix = cv::getPerspectiveTransform(pts1, pts2);
	this->InvPerse_matrix = cv::getPerspectiveTransform(pts2, pts1);
	//@comment �ϊ�(���`�⊮)
	cv::warpPerspective(src, dst, this->perspective_matrix, cv::Size(dst.cols, dst.rows), CV_INTER_LINEAR);

	//@comment �ϊ��O��̍��W��`��
	line(src, pts1[0], pts1[1], cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, pts1[1], pts1[2], cv::Scalar(255, 255, 0), 2, CV_AA);
	line(src, pts1[2], pts1[3], cv::Scalar(255, 255, 0), 2, CV_AA);
	line(src, pts1[3], pts1[0], cv::Scalar(255, 255, 0), 2, CV_AA);
	line(src, pts2[0], pts2[1], cv::Scalar(255, 0, 255), 2, CV_AA);
	line(src, pts2[1], pts2[2], cv::Scalar(255, 255, 0), 2, CV_AA);
	line(src, pts2[2], pts2[3], cv::Scalar(255, 255, 0), 2, CV_AA);
	line(src, pts2[3], pts2[0], cv::Scalar(255, 255, 0), 2, CV_AA);

	//cv::imshow("coo",dst); //�e�X�g�p


}

cv::Mat Img_Proc::getInvPerse(){
	return this->InvPerse_matrix;
}

//�̈�̃v���b�g
void Img_Proc::plot_field(cv::UMat &src, cv::Point2f sz){
	//------------------�}�X�̃v���b�g--------------------------------------
	for (int i = 0; i <= sz.x; i += 100) {
		for (int j = 0; j <= sz.y; j += 100) {
			cv::line(src, cv::Point(i, j), cv::Point(i, sz.x), cv::Scalar(200, 200, 200), 3);
			cv::line(src, cv::Point(i, j), cv::Point(sz.y, j), cv::Scalar(200, 200, 200), 3);
		}
	}
	//------------------���i�̈�̃v���b�g--------------------------------------
	cv::rectangle(src, cv::Point(100, 100), cv::Point(sz.x - 100, sz.y - 100), cv::Scalar(200, 0, 0), 3);
}

cv::Point2f Img_Proc::calcHomoPoint(cv::Point2f &p){
	cv::Point2f point;
	double a = this->InvPerse_matrix.at<double>(0, 0);
	double b = this->InvPerse_matrix.at<double>(0, 1);
	double c = this->InvPerse_matrix.at<double>(0, 2);
	double d = this->InvPerse_matrix.at<double>(1, 0);
	double e = this->InvPerse_matrix.at<double>(1, 1);
	double f = this->InvPerse_matrix.at<double>(1, 2);
	double g = this->InvPerse_matrix.at<double>(2, 0);
	double h = this->InvPerse_matrix.at<double>(2, 1);
	double i = this->InvPerse_matrix.at<double>(2, 2);

	point.x = (p.x * a + p.y * b + c) / (p.x * g + p.y * h + i);
	point.y = (p.x * d + p.y * e + f) / (p.x * g + p.y * h + i);
	return point;
}

