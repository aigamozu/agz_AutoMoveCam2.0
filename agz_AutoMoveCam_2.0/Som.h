#include<iostream>
#include<vector>
#include<string>
#include<time.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>

class SOM{
private:
	int width,height;
	cv::UMat image;
	std::vector<cv::Point2f> P;

	/* �\���� Neuron ���` */
	typedef struct {
		int id;					/*�j���[����ID*/
		cv::Point2f p;			/*�j���[�������W*/
		std::vector<int>link;	/*�אڏ��*/
		cv::Point2f weight;		/*�j���[�����̉׏d*/
	} Neuron;


	std::vector<Neuron> som;

public:
	SOM();
	SOM(int width, int height, std::vector<cv::Point2f> &Pos , cv::UMat &src);
	/**
	* @fn
	* SOM(���ȑg�D���}�b�v)�̐������s���֐�
	* @brief �����ŗ^����ꂽ�T�C�Y��SOM���쐬����
	* @param (int w) SOM�̃T�C�Y(��)
	* @param (int h) SOM�̃T�C�Y(�c)
	* @return void
	* @sa     -
	* @detail -
	*/
	void Init(cv::Mat &src);


	void Init2(cv::Mat &src);
	/**
	* @fn
	* �摜�̏��������s���֐�
	* @brief �i���F�ŉ摜������������j
	* @param (cv::Mat &img)�摜���Q�ƂŎ󂯂�
	* @return void
	* @sa     -
	* @detail -
	*/
	void InitImg(cv::Mat &img);

	/**
	* @fn
	* �j���[�����ʂ��𒼐��Ō��Ԋ֐�
	* @brief �אڃj���[�����̍��W���璼��������
	* @param (std::vector<Neuron> &neu)�j���[�����̏��(�\����)���Q��
	* @param (cv::Mat &img)�摜���Q�ƂŎ󂯂�
	* @return void
	* @sa     -
	* @detail -
	*/
	void edgeline(std::vector<Neuron> &neu, cv::Mat &img);

	/**
	* @fn
	* SOM�̏������s���֐�
	* @brief �}�b�v�̍쐬�A�`��
	* @param (int w) �摜�̉���
	* @param (int h) �摜�̏c��
	* @param (std::vector<Neuron> &neu)�j���[�����̏��(�\����)���Q��
	* @return void
	* @sa     -
	* @detail -
	*/
	void calcsom(int w, int h, std::vector<Neuron> &som, cv::Mat &src, cv::Mat &origin);

	void calcsom2(int w,int h, std::vector<Neuron> &som, cv::Mat &src, cv::Mat &origin); 

	/**
	* @fn
	* SOM��\��
	* @brief �召��r
	* @param (int index)���΃j���[�����̃C���f�b�N�X
	* @param (std::vector<Neuron> &def) �j���[�����̏��(�\����)���Q��
	* @return void
	* @sa     -
	* @detail -
	*/
	void showSOM(int index,std::vector<int> &linked, std::vector<Neuron> &def, int time);

	void Imgproc(cv::Mat &src, cv::Mat &dst);

	std::vector<cv::Point2f> storePoint(cv::Mat &img);

	std::vector<cv::Point2f> storeBorderPoint();

	std::string setImageName(std::string str, int time);

	cv::Point2f calc_centerPoint(int id);

	void set_size(int width, int height);

	void set_pos(std::vector<cv::Point2f> &Pos);

	void set_img(cv::UMat &src);

	void showSOM2(cv::UMat &src, cv::Mat &H);

	cv::Point2f calc_weight(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);

	std::vector<Neuron> get_SOM();

	void showSOM3(cv::UMat &src);
};


bool cmp(int A, int B);

