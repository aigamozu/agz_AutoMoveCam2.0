#include<iostream>
#include<vector>
#include<string>
#include<time.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>


/* �\���� Neuron ���` */
typedef struct {
	int id;					/*�j���[����ID*/
	cv::Point2f p;			/*�j���[�������W*/
	std::vector<int>link;	/*�אڏ��*/
	cv::Point2f weight;		/*�j���[�����̉׏d*/
} Neuron;


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
void Init(int w, int h, cv::Mat &src);

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
void som(int w, int h, std::vector<Neuron> &som, cv::Mat &src, cv::Mat &origin);

/**
* @fn
* �召�̔�r���s���֐�
* @brief �召��r
* @param (int A) int�^�̒l
* @param (int B) int�^�̒l
* @return bool
* @sa     -
* @detail -
*/
bool cmp(int A, int B);

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
void showSOM(int index, std::vector<Neuron> &def);

void Imgproc(cv::Mat &src, cv::Mat &dst);

std::vector<cv::Point2f> storePoint(cv::Mat &img);