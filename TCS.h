/* 
 * File:   TCS.h
 * Author: ç≤ÅXñÿ„ƒ
 *
 * Created on 2021/06/30, 17:38
 */

#ifndef TCS_H
#define	TCS_H

/*---TCS CAN Message---*/
/* SIDH */
#define SIDH_READ   0b11100000
#define SIDH_MODE   0b11100001
#define SIDH_DATA1  0b11110011
#define SIDH_DATA2  0b11110100
#define SIDH_DATA3  0b11110101
#define SIDH_DATA4  0b11110110
#define SIDH_DATA5  0b11110111
#define SIDH_DATA6  0b11111000

/* SIDL */
#define SIDL_W      0b00001001                                                  //WÇÃéûÇÃID
#define SIDL_R      0b00001000                                                  //RÇÃéûÇÃID

/* EID8 */
#define EID8_READ   0b00000000
#define EID8_MODE   0b00000000
#define EID8_DATA1  0b00000000
#define EID8_DATA2  0b00000000
#define EID8_DATA3  0b00000000
#define EID8_DATA4  0b00000000
#define EID8_DATA5  0b00000000
#define EID8_DATA6  0b00000000

/* EID0 */
#define EID0_READ   0b00000000
#define EID0_MODE   0b00000110
#define EID0_DATA1  0b00011000
#define EID0_DATA2  0b00011001
#define EID0_DATA3  0b00011010
#define EID0_DATA4  0b00011011
#define EID0_DATA5  0b00011100
#define EID0_DATA6  0b00011101

        
/* Filter */
#define Sub_Filt    0b11100000



/*---Modebit---*/
#define _ChargeMode                          0b00000001
#define _COMMMode                            0b00000010
#define _StanbyMode                          0b00000011
#define _MissionMode                         0b00000100
#define _SafetyMode                          0b00000101


#endif


