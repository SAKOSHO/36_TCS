/* 
 * File:   main.c
 * Author: ���X����
 *
 * Created on 2021/11/07, 11:28
 * Discription�@��������3�̔M����n�̃v���O����
 * �ҏW����
 * 2021/11/07�F����
 * 
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "spi.h"
#include "MCP2515.h"
#include "TCS.h"

// CONFIG1
#pragma config FOSC  = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE  = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP    = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD   = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO  = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP   = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT   = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ      20000000
#define _CAN_BAUDRATE   2


/*--Prtotype--*/
void CONFIG();                                                                  //�����ݒ�
void timer_init();                                                             //�^�C�}�[�̏����ݒ�
void interrupt timer();                                                         //�^�C�}�[�̊��荞�ݏ���
char BtoD(char data);                                                           //2�i����10�i���ɕϊ�

/*--Grobal Variables--*/
volatile char *rx_data;                                                         //��M�f�[�^�̎󂯓����
volatile char *fram_data;                                                       //fram�p�̔�
volatile char rx_status;                                                        //��M�X�e�[�^�X�p�̔�
volatile char rx_int;                                                           //��M�t���O�m�F�p�̔�
volatile char data1[8] = {6, 0, 0, 0, 0, 0, 0, 1};                               //HK�f�[�^
volatile char data2[8] = {6, 0, 0, 0, 0, 0, 0, 2};                               //HK�f�[�^
volatile char data3[8] = {6, 0, 0, 0, 0, 0, 0, 3};                               //HK�f�[�^
volatile char data4[8] = {6, 0, 0, 0, 0, 0, 0, 4};                               //HK�f�[�^
volatile char data5[8] = {6, 0, 0, 0, 0, 0, 0, 5};                               //HK�f�[�^
volatile char data6[8] = {6, 0, 0, 0, 0, 0, 0, 6};                               //HK�f�[�^
volatile char mode = _ChargeMode;                                               //���[�h�t���O�i�������[�h�F�[�d���[�h�j
volatile char size;                                                             //�f�[�^��
volatile int cnt = 0;                                                           //�R���y�A�}�b�`�p�̃J�E���^0
volatile int cnt1 = 0;                                                           //�R���y�A�}�b�`�p�̃J�E���^1
volatile int mode_cnt = 0;


void main(void)
{
    int rate1 = 10;
    int rate2 = 10;
    
    CONFIG();                                                                   //�����ݒ�
    __delay_ms(100);
    
    T1CONbits.TMR1ON = 1;                                                       //�^�C�}�[�J�n
    
    while(1)
    {
        /* ���[�h�J�ڏ��� */
        rx_int = Read(_CANINTF);                                                //���荞�݃t���O
        
        if((rx_int & _Flagbit0) == 0b00000001)
        {   
            rx_data = Read_RX_ID(_F_RXB0SIDH, 13);
            Write(_CANINTF, 0b00000000);
            
            if(rx_data[3] == SIDH_MODE)                                         //���[�h��񂩂��m�F
            {
                if((rx_data[5] & _ChargeMode) == 0b00000001)                    //�[�d���[�h�Ɉڍs
                {
                    /*
                    RB5 = 0;
                    RB6 = 0;
                    RB7 = 1;
                    */
                    mode = _ChargeMode;
                }
            
                if((rx_data[5] & _COMMMode) == 0b00000010)                  //�_�E�������N���[�h�Ɉڍs
                {
                    /*
                    RB5 = 0;
                    RB6 = 1;
                    RB7 = 0;
                    */
                    mode = _COMMMode;
                }
            
                if((rx_data[5] & _StanbyMode) == 0b00000011)                    //�ҋ@���[�h�Ɉڍs
                {
                    /*
                    RB5 = 1;
                    RB6 = 0; 
                    RB7 = 0;
                    */
                    mode = _StanbyMode;
                }
            
                if((rx_data[5] & _MissionMode) == 0b00000100)                   //�~�b�V�������[�h�Ɉڍs
                {
                    /*
                    RB5 = 0;
                    RB6 = 0; 
                    RB7 = 0;
                    */
                    mode = _MissionMode;
                }
            }
        }
        
        if(mode_cnt <= 2000)
        {
            /* �[�d���[�h */
            if(mode == _ChargeMode)
            {
                /* 1Hz */
                if(cnt >= rate1)
                {
                    Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_R, 0, 0);                   //���[�h�t���O�ǂݏo��
                    Load_TX_Data(_F_TXB0D0, 1, 0);                                      //�ǂݏo��������C1byte�̋�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);   //���[�h��������ID�̐ݒ�
                    Load_TX_Data(_F_TXB0D0, 1, &mode);                                  //���[�h��������
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��
                    
                    cnt = 0;
                }

                if(cnt1 >= rate2)
                {
                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA1, SIDL_W, EID8_DATA1, EID0_DATA1);//�o�b�e���[�̉��x�l��������
                    Load_TX_Data(_F_TXB0D0, 8, data1);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��


                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA2, SIDL_W, EID8_DATA2, EID0_DATA2);//�������̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data2);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA3, SIDL_W, EID8_DATA3, EID0_DATA3); //�p���Z���T�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data3);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA4, SIDL_W, EID8_DATA4, EID0_DATA4);//�p������A�N�`���G�[�^�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data4);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA5, SIDL_W, EID8_DATA5, EID0_DATA5);//�ʐM�@�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data5);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA6, SIDL_W, EID8_DATA6, EID0_DATA6);//�~�b�V�����@��̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data6);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    data1[1] = data1[1] + 1;
                    data2[1] = data2[1] + 1;
                    data3[1] = data3[1] + 1;
                    data4[1] = data4[1] + 1;
                    data5[1] = data5[1] + 1;
                    data6[1] = data6[1] + 1;
                    
                    cnt1 = 0;
                }
            }
            
            /* �ҋ@���[�h */
            if(mode == _StanbyMode)
            {
                /* 1Hz */
                if(cnt >= rate1)
                {
                    Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_R, 0, 0);                   //���[�h�t���O�ǂݏo��
                    Load_TX_Data(_F_TXB0D0, 1, 0);                                      //�ǂݏo��������C1byte�̋�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);   //���[�h��������ID�̐ݒ�
                    Load_TX_Data(_F_TXB0D0, 1, &mode);                                  //���[�h��������
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��
                    
                    cnt = 0;
                }

                if(cnt1 >= rate2)
                {
                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA1, SIDL_W, EID8_DATA1, EID0_DATA1);//�o�b�e���[�̉��x�l��������
                    Load_TX_Data(_F_TXB0D0, 8, data1);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��


                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA2, SIDL_W, EID8_DATA2, EID0_DATA2);//�������̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data2);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA3, SIDL_W, EID8_DATA3, EID0_DATA3); //�p���Z���T�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data3);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA4, SIDL_W, EID8_DATA4, EID0_DATA4);//�p������A�N�`���G�[�^�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data4);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA5, SIDL_W, EID8_DATA5, EID0_DATA5);//�ʐM�@�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data5);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA6, SIDL_W, EID8_DATA6, EID0_DATA6);//�~�b�V�����@��̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data6);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    data1[1] = data1[1] + 1;
                    data2[1] = data2[1] + 1;
                    data3[1] = data3[1] + 1;
                    data4[1] = data4[1] + 1;
                    data5[1] = data5[1] + 1;
                    data6[1] = data6[1] + 1;
                    
                    cnt1 = 0;
                }
            }

            /* �~�b�V�������[�h */
            if(mode == _MissionMode)
            {   
                /* 1Hz */
                if(cnt >= rate1)
                {
                    Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_R, 0, 0);                   //���[�h�t���O�ǂݏo��
                    Load_TX_Data(_F_TXB0D0, 1, 0);                                      //�ǂݏo��������C1byte�̋�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);   //���[�h��������ID�̐ݒ�
                    Load_TX_Data(_F_TXB0D0, 1, &mode);                                  //���[�h��������
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��
                    
                    cnt = 0;
                }

                if(cnt1 >= rate2)
                {
                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA1, SIDL_W, EID8_DATA1, EID0_DATA1);//�o�b�e���[�̉��x�l��������
                    Load_TX_Data(_F_TXB0D0, 8, data1);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA2, SIDL_W, EID8_DATA2, EID0_DATA2);//�������̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data2);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA3, SIDL_W, EID8_DATA3, EID0_DATA3); //�p���Z���T�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data3);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA4, SIDL_W, EID8_DATA4, EID0_DATA4);//�p������A�N�`���G�[�^�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data4);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA5, SIDL_W, EID8_DATA5, EID0_DATA5);//�ʐM�@�̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data5);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_DATA6, SIDL_W, EID8_DATA6, EID0_DATA6);//�~�b�V�����@��̉��x��������
                    Load_TX_Data(_F_TXB0D0, 8, data6);                                   //�f�[�^
                    RTS0(_CAN_BAUDRATE);                                                //���M�v��

                    data1[1] = data1[1] + 1;
                    data2[1] = data2[1] + 1;
                    data3[1] = data3[1] + 1;
                    data4[1] = data4[1] + 1;
                    data5[1] = data5[1] + 1;
                    data6[1] = data6[1] + 1;
                    
                    cnt1 = 0;
                }
            }
        }
        
        /* 20s�ŒʐM���[�h */
        if(mode_cnt > 2000)
        {
            /* 1Hz */
            if(cnt >= rate1)
            {
                Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_R, 0, 0);                   //���[�h�t���O�v��ID�̐ݒ�
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //���[�h�v��
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                Write(_TXB0DLC , 0b00000001);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);   //���[�h��ID�̐ݒ�
                Load_TX_Data(_F_TXB0D0, 1, &mode);                                  //���[�h��
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                cnt = 0;
            }
            
            /* 1Hz */
            if(cnt1 >= 100)
            {
                Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_DATA1, SIDL_W, EID8_DATA1, EID0_DATA1);//�o�b�e���[�̉��x�l��������
                Load_TX_Data(_F_TXB0D0, 8, data1);                                   //�f�[�^
                RTS0(_CAN_BAUDRATE);                                                //���M�v��


                Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_DATA2, SIDL_W, EID8_DATA2, EID0_DATA2);//�������̉��x��������
                Load_TX_Data(_F_TXB0D0, 8, data2);                                   //�f�[�^
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_DATA3, SIDL_W, EID8_DATA3, EID0_DATA3); //�p���Z���T�̉��x��������
                Load_TX_Data(_F_TXB0D0, 8, data3);                                   //�f�[�^
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_DATA4, SIDL_W, EID8_DATA4, EID0_DATA4);//�p������A�N�`���G�[�^�̉��x��������
                Load_TX_Data(_F_TXB0D0, 8, data4);                                   //�f�[�^
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_DATA5, SIDL_W, EID8_DATA5, EID0_DATA5);//�ʐM�@�̉��x��������
                Load_TX_Data(_F_TXB0D0, 8, data5);                                   //�f�[�^
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                Write(_TXB0DLC , 0b00001000);                                       //���b�Z�[�W�T�C�Y1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_DATA6, SIDL_W, EID8_DATA6, EID0_DATA6);//�~�b�V�����@��̉��x��������
                Load_TX_Data(_F_TXB0D0, 8, data6);                                   //�f�[�^
                RTS0(_CAN_BAUDRATE);                                                //���M�v��

                data1[1] = data1[1] + 1;
                data2[1] = data2[1] + 1;
                data3[1] = data3[1] + 1;
                data4[1] = data4[1] + 1;
                data5[1] = data5[1] + 1;
                data6[1] = data6[1] + 1;

                cnt1 = 0;
            }
            
            if(mode_cnt > 3000) mode_cnt = 0;
        }
    }
}

void CONFIG()
{
    OSCCON = 0b01101000;
    ANSEL  = 0b00000000;
    ANSELH = 0b00000000;
    TRISB  = 0b00000000;
    TRISC  = 0b00000000;
    PORTB  = 0b00000000;
    PORTC  = 0b00000000;
    
    INTCON = 0b11000000;                                                        //���荞�ݐݒ�
    
    spi_init();
    timer_init();
    __delay_ms(100);
    
    MCP2515_init(_CAN_BAUDRATE);                                                //�Ƃ肠�����C���삵�Ă���2�ɂ����D�����͂܂�
    Write(_TXB0DLC , 0b00000001);                                               //���b�Z�[�W�T�C�Y8byte
    Write(_RXM0SIDH, 0b11111111);                                               //�}�X�N�ݒ��ʑS��v
    Write(_RXM0SIDL, 0b11111111);                                               //�}�X�N�ݒ艺�ʑS��v
    Write(_RXM0EID8, 0b11111111);                                               //�}�X�N�ݒ�g����ʑS��v
    Write(_RXM0EID0, 0b11110000);                                               //�}�X�N�ݒ�g�����ʏ��4bit��v
    Write(_RXF0SIDH, 0b00000000);                                               //��M�t�B���^��ʃr�b�g�̐ݒ�
    Write(_RXF0SIDL, 0b00001000);                                               //��M�t�B���^���ʃr�b�g�̐ݒ�
    Write(_RXF0EID8, 0b00000000);                                               //��M�t�B���^�g����ʃr�b�g�̐ݒ�
    Write(_RXF0EID0, Sub_Filt);                                                 //��M�t�B���^�g�����ʃr�b�g�̐ݒ�
    MCP2515_Open(0);                                                            //�Ƃ肠�����C0�ɂ����D�����͂܂�
}

void timer_init()
{
    T1CON = 0b00000000;
    TMR1H = 0b00111100;                                                         //500000�ŃI�[�o�[�t���[������
    TMR1L = 0b10110000;
    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
}

void interrupt timer()
{
    PIR1bits.TMR1IF = 0;
    TMR1H = 0b00111100;                                                         //500000�ŃI�[�o�[�t���[������
    TMR1L = 0b10110000;
    
    cnt++;
    cnt1++;
    mode_cnt++;
}

char BtoD(char data)
{
    char  binary;
    char decimal = 0;
    char bas = 1;
            
    binary = data & 0b00001111;
    
    while(binary>0)
    {
        decimal = decimal + (binary % 10) * bas;
        binary = binary / 10;
        bas = bas * 2;
    }
    
    return decimal;
}



