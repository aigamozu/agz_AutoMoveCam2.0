#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "SOM.h"

class Control{
private:
	int height;
	int width;

	// ���{�b�g�̌��݈ʒu
	cv::Point2i nowPoint;
	// ���{�b�g�̓���
	std::string action;
	// ���O���茋��
	std::string out;

	struct target{
		// �^�[�Q�b�g���W
		cv::Point2i point;
		// �^�[�Q�b�g�ԍ�
		int n;
	};

	// ���ׂẴ^�[�Q�b�g
	std::vector<target> allTarget;

	//�ϊ����ꂽ���ׂẴ^�[�Q�b�g
	std::vector<target> allTransformedTraget;

	// ���Ɍ������^�[�Q�b�g
	std::vector<target>::iterator nowTarget_itr;

	// �q�[�g�}�b�v�p�z�� ���{�b�g�̖K��񐔂��i�[����
	int** small_area;

	//@comment �J���[�o�[�A�q�[�g�}�b�v�����p�摜����
	cv::Mat concat_img = cv::Mat(cv::Size(700, 800), CV_8UC3, cv::Scalar(255, 255, 255));


public:
	// Constructor
	Control(int width, int height);

	// �^�[�Q�b�g�̍X�V
	bool is_updateTarget(void);

	// ���{�b�g�̓��쌈��
	int robot_action(cv::Point2i Previous);

	// �^�[�Q�b�g�G���A�̖K��񐔂̍X�V
	int target_count(void);

	// ���̈��Ԃ̖K��񐔂̍X�V
	cv::Point2i area_count(void);

	// ���O����
	void is_out(void);

	// �v���b�g
	void plot_target(cv::UMat &img, cv::Point2i Previous);

	void plot_transform_target(cv::UMat &img, cv::Point2i Previous, cv::Mat H);

	//@comment �q�[�g�}�b�v�쐬
	void heatmap(cv::Point2i pos, cv::Mat *img, cv::Mat *bar);

	//	set function
	void set_target(SOM som);
	void set_point(cv::Point2i p);


	// get function
	int get_target(void);
};