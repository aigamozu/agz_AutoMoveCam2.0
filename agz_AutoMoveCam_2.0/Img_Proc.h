#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
//#include "SOM.h"

class Img_Proc
{
private:
	//�|���̈�
	cv::Point2i field;

	//�����ϊ��s��
	cv::Mat perspective_matrix;

	//�J�����X�g���[��
	cv::VideoCapture cap;

	//�擾�摜
	cv::UMat capImg;

	//�z���O���t�B�[�s��
	cv::Mat InvPerse_matrix;

	cv::Mat Homo;
public:
	//�J�����̐ݒ�
	Img_Proc(int camId);

	//���c�̈�̐ݒ�
	void setField(int w, int h);

	//�J���[�o�[�̍쐬
	cv::Mat makeColorbar();

	//�J���[�o�[�̃p�����[�^����
	cv::Vec3b calcPseudoColor(double phase);

	//�̈�̎擾
	cv::Point2i getField();

	//�����ϊ��s��̎擾
	cv::Mat getPersMat();

	//�ő�̈�̎擾
	cv::Point2i serchMaxArea(cv::UMat &src, cv::UMat &plot);

	//���S�̎Z�o
	cv::Point2i calculate_center(cv::UMat &);

	//�c�ݕ␳(Gopro)
	cv::Mat undist(cv::Mat); //@comment �c�ݕ␳(gopro)

	//�F���o�p�֐�
	void colorExtraction(cv::UMat &src, cv::UMat &dst,
		int code,
		int ch1Lower, int ch1Upper,
		int ch2Lower, int ch2Upper,
		int ch3Lower, int ch3Upper
		);

	//�摜�擾�֐�
	cv::UMat getFrame();

	//�����ϊ��֐�
	void Perspective(cv::UMat &src, cv::UMat &dst, std::vector<cv::Point2f> &p);

	//�̈�̃v���b�g
	void plot_field(cv::UMat &src, cv::Point2f sz);

	cv::Mat Img_Proc::getInvPerse();
	//void plot_SOM(cv::UMat &src, SOM &som);

	cv::Point2f calcHomoPoint(cv::Point2f &p);
	cv::Mat calcHomo(cv::UMat &img, cv::UMat &img2);

	void setH(cv::Mat H);

	std::vector<cv::Point2f> getPerseArea(std::vector<cv::Point2f> &Pos);
	//void perseSOM(cv::UMat &img, SOM &som);
};