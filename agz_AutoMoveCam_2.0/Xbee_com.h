
#include <iostream>
#include <windows.h>
#include <math.h>
#include <time.h>
#include "SSK.h"


//送信先(robot)のXbeeのアドレス
//以下のリストにない場合は追加してください

//byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00),byte(0x40), byte(0x99), byte(0x37), byte(0x03) };
//robot id 13 
//byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00),byte(0x40), byte(0x9E), byte(0x5C), byte(0x42) };

//robot id 10
byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00), byte(0x40), byte(0x99), byte(0x37), byte(0x08) };

//robot id 5
//byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00), byte(0x40), byte(0x99), byte(0x35), byte(0xb5) };

//robot id 11
//byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00), byte(0x40), byte(0x99), byte(0x37), byte(0x7A) };


//各文字バイトの定義
byte const  A = byte(0x41), B = byte(0x42), C = byte(0x43), D = byte(0x44), E = byte(0x45), F = byte(0x46),
G = byte(0x47), H = byte(0x48), I = byte(0x49), J = byte(0x4a), K = byte(0x4b), L = byte(0x4c),
M = byte(0x4d), N = byte(0x4e), O = byte(0x4f), P = byte(0x50), Q = byte(0x51), R = byte(0x52),
S = byte(0x53), T = byte(0x54), U = byte(0x55), V = byte(0x56), W = byte(0x57), X = byte(0x58),
Y = byte(0x59), Z = byte(0x5a);


//モードごとの左と右のpwm ([0]:停止時, [1]:前進, [2]:右旋回, [3]:右旋回, [4]:左旋回, [5]:左旋回,
//								残りは変更の必要なし)

//PWM値用配列
byte const chPWM[] = { byte(0x00), byte(0x08), byte(0x10), byte(0x18), byte(0x20), byte(0x28), byte(0x30),
byte(0x38), byte(0x40), byte(0x48), byte(0x50), byte(0x58), byte(0x60),
byte(0x68), byte(0x70), byte(0x78), byte(0x80) };

//左モータ、右モータ用PWM配列
//デフォルト 前進 : 64, 左,右旋回 : 48
static byte lPwm[] = { byte(0x00), chPWM[8], chPWM[6], chPWM[6], byte(0x08), byte(0x08),
byte(0x10), byte(0x10), byte(0x0c), byte(0x0c) };
static byte  rPwm[] = { byte(0x00), chPWM[8], byte(0x08), byte(0x08), chPWM[6], chPWM[6],
byte(0x0c), byte(0x08), byte(0x0c), byte(0x08) };

class Xbee_com{
private:
	//xbee通信設定用構造体
	DCB dcb;

public:
	//コンストラクタ
	Xbee_com(LPCSTR com, HANDLE &arduino);

	//モード切替
	void sentManualCommand(byte command, HANDLE &arduino);

	//アイガモ動作切替
	void sentAigamoCommand(int command, HANDLE &arduino, byte LPWM[], byte RPWM[]);

	//COMポートオープン
	void openCOM(LPCSTR com, HANDLE &arduino);

	//COMポートクローズ
	void closeCOM(HANDLE &arduino);
};

