#include <iostream>
#include <windows.h>
#include <math.h>
#include <time.h>
#include "SSK.h"


//���M���Xbee�̃A�h���X
byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00),
byte(0x40), byte(0x99), byte(0x37), byte(0x03) };

//�e�����o�C�g
byte const  A = byte(0x41), B = byte(0x42), C = byte(0x43), D = byte(0x44), E = byte(0x45), F = byte(0x46),
G = byte(0x47), H = byte(0x48), I = byte(0x49), J = byte(0x4a), K = byte(0x4b), L = byte(0x4c),
M = byte(0x4d), N = byte(0x4e), O = byte(0x4f), P = byte(0x50), Q = byte(0x51), R = byte(0x52),
S = byte(0x53), T = byte(0x54), U = byte(0x55), V = byte(0x56), W = byte(0x57), X = byte(0x58),
Y = byte(0x59), Z = byte(0x5a);

//���[�h���Ƃ�left��right��pwm
byte const lPwm[] = { byte(0x00), byte(0x18), byte(0x12), byte(0x12), byte(0x08), byte(0x08),
byte(0x10), byte(0x10), byte(0x0c), byte(0x0c) };
byte const rPwm[] = { byte(0x00), byte(0x18), byte(0x08), byte(0x08), byte(0x30), byte(0x30),
byte(0x0c), byte(0x08), byte(0x0c), byte(0x08) };




class Xbee_com{
private:
	//xbee�ʐM�ݒ�p�\����
	DCB dcb;
public:

	Xbee_com(LPCSTR com, HANDLE &arduino);
	
	//���[�h�ؑ�
	void sentManualCommand(byte command, HANDLE &arduino);
	
	//�A�C�K������ؑ�
	void sentAigamoCommand(int command, HANDLE &arduino);
	
	//COM�|�[�g�I�[�v��
	void openCOM(LPCSTR com, HANDLE &arduino);

	//COM�|�[�g�N���[�Y
	void closeCOM(HANDLE &arduino);
};

