#include "Xbee_com.h"

//xbee�̏����ݒ�
Xbee_com::Xbee_com(LPCSTR com, HANDLE &arduino){
	// COM�|�[�g�I�[�v��
	openCOM(com, arduino);

	//����M�o�b�t�@������
	if (!SetupComm(arduino, 1024, 1024)) {
		printf("SET UP FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}
	//COM�|�[�g������ɏ�������Ȃ��ꍇ�̏���
	if (!PurgeComm(arduino, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR)) {
		printf("CLEAR FAILED\n");
		CloseHandle(arduino);
		exit(0);
	}

	//��{�ʐM�����̐ݒ�
	DCB dcb;
	GetCommState(arduino, &this->dcb);  /*COM�|�[�g�̏�Ԏ擾*/
	this->dcb.DCBlength = sizeof(DCB);  /*DCB�̃T�C�Y�̓��I�m��*/
	this->dcb.BaudRate = 57600;			/*�{�[���[�g�̐ݒ�*/
	this->dcb.fBinary = TRUE;			/*�o�C�i���[���[�h�̐ݒ�*/
	this->dcb.ByteSize = 8;				/*�o�C�g�T�C�Y�̐ݒ�*/
	this->dcb.fParity = NOPARITY;		/*�p���e�B�[�r�b�g�̐ݒ�*/
	this->dcb.StopBits = ONESTOPBIT;	/*�X�g�b�v�r�b�g�̐ݒ�*/

	if (!SetCommState(arduino, &this->dcb)) {
		printf("SetCommState FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}
}

//���[�h�̐؂�ւ��֐�
void Xbee_com::sentManualCommand(byte command, HANDLE &arduino){
	bool Ret;
	DWORD dwSendSize;
	byte checksum = 0;

	//�p�P�b�g����
	byte requestPacket[] = { byte(0x7E), byte(0x00), byte(0x1A), byte(0x10), byte(0x01),
		robotAddr[0], robotAddr[1], robotAddr[2], robotAddr[3],
		robotAddr[4], robotAddr[5], robotAddr[6], robotAddr[7],
		byte(0xFF), byte(0xFE), byte(0x00), byte(0x00), A, G, S, C, F, A, T, A, command, A, G, E, byte(0x00) };

	//�`�F�b�N�T���̌v�Z
	for (int i = 3; i < 29; i++) {
		checksum += requestPacket[i];
	}
	checksum = 0xFF - (checksum & 0x00FF);
	requestPacket[29] = byte(checksum);

	//�p�P�b�g�̑��M
	Ret = WriteFile(arduino, requestPacket, sizeof(requestPacket), &dwSendSize, NULL);
	if (!Ret) {
		printf("SEND FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}
}

//����p�^�[���ύX�֐�
void Xbee_com::sentAigamoCommand(int command, HANDLE &arduino) {
	bool Ret;
	DWORD dwSendSize;
	DWORD dwErrorMask;
	byte checksum = 0;

	//�p�P�b�g����
	byte requestPacket[] = { byte(0x7E), byte(0x00), byte(0x1F), byte(0x10), byte(0x01),
		robotAddr[0], robotAddr[1], robotAddr[2], robotAddr[3],
		robotAddr[4], robotAddr[5], robotAddr[6], robotAddr[7],
		byte(0xFF), byte(0xFE), byte(0x00), byte(0x00), A, G, S,
		M, F, A, T, A, L, 1, lPwm[byte(command)], R, 1, rPwm[byte(command)], A, G, E, byte(0x00) };

	//�`�F�b�N�T���̌v�Z
	for (int i = 3; i < 34; i++) {
		checksum += requestPacket[i];
	}
	checksum = 0xFF - (checksum & 0x00FF);
	requestPacket[34] = byte(checksum);

	//�p�P�b�g�̑��M
	Ret = WriteFile(arduino, requestPacket, sizeof(requestPacket), &dwSendSize, NULL);
	if (!Ret) {
		printf("SEND FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}
}

//COM�|�[�g�I�[�v��
void Xbee_com::openCOM(LPCSTR com, HANDLE &arduino){
	arduino = CreateFile(com, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (arduino == INVALID_HANDLE_VALUE){
		std::cout << "COM�|�[�g���J���܂���." << std::endl;
		std::cout << "COM�|�[�g���`�F�b�N��, " << std::endl;
		std::cout << "�������|�[�g���w�肵�Ă�������." << std::endl;
		system("PAUSE");
		exit(0);
	}
}

//COM�|�[�g�N���[�Y
void Xbee_com::closeCOM(HANDLE &arduino){
	CloseHandle(arduino);
}