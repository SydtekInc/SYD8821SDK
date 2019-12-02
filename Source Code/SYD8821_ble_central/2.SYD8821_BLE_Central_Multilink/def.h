#ifndef __DEF_H
#define __DEF_H

/*
0:����ɨ�� 
1��ɨ�赽�豸��׼������
2����������
3: ���ӳɹ�  ��Ҫ���ַ���
4�����ַ���ɹ�
5�������ӻ�����
*/
struct ble_scan_device_info_t
{
	uint8_t *scan_name;			// ɨ����豸��
	uint8_t *scan_addr;			// ɨ����豸��ַ
	uint8_t device_cs_num;		// �����豸ʹ�õ�ͨ��
	uint8_t device_status;		// �豸״̬
};


struct ble_scan_cs_num_t
{
	uint8_t idle_num;			// һλ����һ��ͨ��״̬��0:���� 1:ɨ��
	bool wait_for_connected;	// �ȴ�����
};

typedef enum
{
	CS_NUM_0 = 0,
	CS_NUM_1 = 1,
	CS_NUM_2 = 2,
	CS_NUM_3 = 3,
	CS_NUM_4 = 4,
	CS_NUM_5 = 5,
	CS_NUM_6 = 6,
	CS_NUM_7 = 7,
}cs_num_t;


#endif /* __DEF_H */
