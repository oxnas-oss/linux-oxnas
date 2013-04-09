#ifndef __ZDHW_C__
#define __ZDHW_C__

#include "zdtypes.h"
#include "zdequates.h"
#include "zdapi.h"
#include "zdhw.h"
#include "zddebug.h"
#include "zd1211.h"
#include "zd1205.h"
extern struct net_device *g_dev;
extern u8 mMacMode;
extern u8 a_OSC_get_cal_int( u8 ch, u32 rate, u8 *intValue, u8 *calValue);
extern u8 *mTxOFDMType;
extern const U16 dot11A_Channel[];
extern const U16 dot11A_Channel_Amount;

u8 LastSetChannel=1;
u8 LastMacMode=0;

U32 GRF5101T[] = {
                         0x1A0000,   //Null
                         0x1A0000,   //Ch 1
                         0x1A8000,   //Ch 2
                         0x1A4000,   //Ch 3
                         0x1AC000,   //Ch 4
                         0x1A2000,   //Ch 5
                         0x1AA000,   //Ch 6
                         0x1A6000,   //Ch 7
                         0x1AE000,   //Ch 8
                         0x1A1000,   //Ch 9
                         0x1A9000,   //Ch 10
                         0x1A5000,   //Ch 11
                         0x1AD000,   //Ch 12
                         0x1A3000,   //Ch 13
                         0x1AB000    //Ch 14
                 };


U32 AL2210TB[] = {
                         0x2396c0,   //;Null
                         0x0196c0,   //;Ch 1
                         0x019710,   //;Ch 2
                         0x019760,   //;Ch 3
                         0x0197b0,   //;Ch 4
                         0x019800,   //;Ch 5
                         0x019850,   //;Ch 6
                         0x0198a0,   //;Ch 7
                         0x0198f0,   //;Ch 8
                         0x019940,   //;Ch 9
                         0x019990,   //;Ch 10
                         0x0199e0,   //;Ch 11
                         0x019a30,   //;Ch 12
                         0x019a80,   //;Ch 13
                         0x019b40    //;Ch 14
                 };


U32	M2827BF[] = {
                        0x0ccd4,    //;Null
                        0x0ccd4,    //;Ch 1
                        0x22224,    //;Ch 2
                        0x37774,    //;Ch 3
                        0x0ccd4,    //;Ch 4
                        0x22224,    //;Ch 5
                        0x37774,    //;Ch 6
                        0x0ccd4,    //;Ch 7
                        0x22224,    //;Ch 8
                        0x37774,    //;Ch 9
                        0x0ccd4,    //;Ch 10
                        0x22224,    //;Ch 11
                        0x37774,    //;Ch 12
                        0x0ccd4,    //;Ch 13
                        0x199a4    //;Ch 14
                };


U32	M2827BN[] = {
                        0x30a03,    //;Null
                        0x30a03,    //;Ch 1
                        0x00a13,    //;Ch 2
                        0x10a13,    //;Ch 3
                        0x30a13,    //;Ch 4
                        0x00a23,    //;Ch 5
                        0x10a23,    //;Ch 6
                        0x30a23,    //;Ch 7
                        0x00a33,    //;Ch 8
                        0x10a33,    //;Ch 9
                        0x30a33,    //;Ch 10
                        0x00a43,    //;Ch 11
                        0x10a43,    //;Ch 12
                        0x30a43,    //;Ch 13
                        0x20a53    //;Ch 14
                };


U32	M2827BF2[] = {
                         0x33334,    //;Null
                         0x33334,    //;Ch 1
                         0x08884,    //;Ch 2
                         0x1ddd4,    //;Ch 3
                         0x33334,    //;Ch 4
                         0x08884,    //;Ch 5
                         0x1ddd4,    //;Ch 6
                         0x33334,    //;Ch 7
                         0x08884,    //;Ch 8
                         0x1ddd4,    //;Ch 9
                         0x33334,    //;Ch 10
                         0x08884,    //;Ch 11
                         0x1ddd4,    //;Ch 12
                         0x33334,    //;Ch 13
                         0x26664    //;Ch 14
                 };

U32	M2827BN2[] = {
                         0x10a03,    //;Null
                         0x10a03,    //;Ch 1
                         0x20a13,    //;Ch 2
                         0x30a13,    //;Ch 3
                         0x10a13,    //;Ch 4
                         0x20a23,    //;Ch 5
                         0x30a23,    //;Ch 6
                         0x10a23,    //;Ch 7
                         0x20a33,    //;Ch 8
                         0x30a33,    //;Ch 9
                         0x10a33,    //;Ch 10
                         0x20a43,    //;Ch 11
                         0x30a43,    //;Ch 12
                         0x10a43,    //;Ch 13
                         0x20a53    //;Ch 14
                 };

U32 AL2232TB[] =
        {
                0x03f790, 0x033331, 0x00000d,                 //;Null
                0x03f790, 0x033331, 0x00000d,                 //;Ch 1
                0x03f790, 0x0b3331, 0x00000d,                 //;Ch 2
                0x03e790, 0x033331, 0x00000d,                 //;Ch 3
                0x03e790, 0x0b3331, 0x00000d,                 //;Ch 4
                0x03f7a0, 0x033331, 0x00000d,                 //;Ch 5
                0x03f7a0, 0x0b3331, 0x00000d,                 //;Ch 6
                0x03e7a0, 0x033331, 0x00000d,                 //;Ch 7
                0x03e7a0, 0x0b3331, 0x00000d,                 //;Ch 8
                0x03f7b0, 0x033331, 0x00000d,                 //;Ch 9
                0x03f7b0, 0x0b3331, 0x00000d,                 //;Ch 10
                0x03E7b0, 0x033331, 0x00000d,                 //;Ch 11
                0x03e7b0, 0x0b3331, 0x00000d,                 //;Ch 12
                0x03f7c0, 0x033331, 0x00000d,                 //;Ch 13
                0x03e7c0, 0x066661, 0x00000d                  //;Ch 14
        };

U32 AL2230TB[] = {
                         0x03f790, 0x033331, 0x00000d,   //;Null
                         0x03f790, 0x033331, 0x00000d,   //;Ch 1
                         0x03f790, 0x0b3331, 0x00000d,  //;Ch 2
                         0x03e790, 0x033331, 0x00000d,  //;Ch 3
                         0x03e790, 0x0b3331, 0x00000d,  //;Ch 4
                         0x03f7a0, 0x033331, 0x00000d,  //;Ch 5
                         0x03f7a0, 0x0b3331, 0x00000d,  //;Ch 6
                         0x03e7a0, 0x033331, 0x00000d,  //;Ch 7
                         0x03e7a0, 0x0b3331, 0x00000d,  //;Ch 8
                         0x03f7b0, 0x033331, 0x00000d,  //;Ch 9
                         0x03f7b0, 0x0b3331, 0x00000d,  //;Ch 10
                         0x03E7b0, 0x033331, 0x00000d,  //;Ch 11
                         0x03e7b0, 0x0b3331, 0x00000d,  //;Ch 12
                         0x03f7c0, 0x033331, 0x00000d,  //;Ch 13
                         0x03e7c0, 0x066661, 0x00000d   //;Ch 14
                 };
U32 AL7230BTB[] = {
                          0x09ec04, 0x8cccc8,   //;Null
                          0x09ec00, 0x8cccc8,   //;Ch 1
                          0x09ec00, 0x8cccd8,   //;Ch 2
                          0x09ec00, 0x8cccc0,   //;Ch 3
                          0x09ec00, 0x8cccd0,   //;Ch 4
                          0x05ec00, 0x8cccc8,   //;Ch 5
                          0x05ec00, 0x8cccd8,   //;Ch 6
                          0x05ec00, 0x8cccc0,   //;Ch 7
                          0x05ec00, 0x8cccd0,   //;Ch 8
                          0x0dec00, 0x8cccc8,   //;Ch 9
                          0x0dec00, 0x8cccd8,   //;Ch 10
                          0x0dec00, 0x8cccc0,   //;Ch 11
                          0x0dec00, 0x8cccd0,   //;Ch 12
                          0x03ec00, 0x8cccc8,   //;Ch 13
                          0x03ec00, 0x866660    //;Ch 14
                  };

U32 AL7230BTB_a[] = {
                            0x06aff4, 0x855550, 0x47f8a2, 0x21ebfe,   //;Null
                            0x02aff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;CH 8 , 5040MHz
                            0x02aff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;CH 12, 5060MHz
                            0x0aaff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;CH 16, 5080MHz
                            0x06aff0, 0x8aaaa0, 0x47f8a2, 0x21ebfe,   //;CH 34, 5170MHz
                            0x06aff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 36, 5180MHz
                            0x0eaff0, 0x800008, 0x47f8a2, 0x21ebfe,   //;Ch 38, 5190MHz
                            0x0eaff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 40, 5200MHz
                            0x0eaff0, 0x855558, 0x47f8a2, 0x21ebfe,   //;Ch 42, 5210MHz
                            0x0eaff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 44, 5220MHz, current support
                            0x0eaff0, 0x8aaaa0, 0x47f8a2, 0x21ebfe,   //;Ch 46, 5230MHz
                            0x0eaff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 48, 5240MHz
                            0x01aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 52, 5260MHz
                            0x01aff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 56, 5280MHz, current support
                            0x01aff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 60, 5300MHz
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 64, 5320MHz
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 68, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 72, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 76, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 80, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 84, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 88, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 92, 5320MHz,dummy
                            0x09aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 96, 5320MHz,dummy
                            0x03aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 100, 5500MHz
                            0x03aff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 104, 5520MHz
                            0x03aff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 108, 5540MHz
                            0x0baff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 112, 5560MHz
                            0x0baff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 116, 5580MHz
                            0x0baff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 120, 5600MHz
                            0x07aff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 124, 5620MHz
                            0x07aff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 128, 5640MHz
                            0x07aff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 132, 5660MHz
                            0x0faff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 136, 5680MHz
                            0x0faff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 140, 5700MHz
                            0x0faff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 144, 5700MHz, dummy
                            0x006ff0, 0x800018, 0x47f8a2, 0x21ebfe,   //;Ch 149, 5745MHz
                            0x006ff0, 0x855540, 0x47f8a2, 0x21ebfe,   //;Ch 153, 5765MHz
                            0x006ff0, 0x8aaab0, 0x47f8a2, 0x21ebfe,   //;Ch 157, 5785MHz
                            0x086ff0, 0x800018, 0x47f8a2, 0x21ebfe,   //;Ch 161, 5805MHz
                            0x086ff0, 0x855540, 0x47f8a2, 0x21ebfe,   //;Ch 165, 5825MHz
                            0x086ff0, 0x8d5540, 0x47f8a2, 0x21ebfe,   //;Ch 168, 5825MHz,dummy
                            0x086ff0, 0x8d5540, 0x47f8a2, 0x21ebfe,   //;Ch 172, 5825MHz,dummy
                            0x086ff0, 0x8d5540, 0x47f8a2, 0x21ebfe,   //;Ch 176, 5825MHz,dummy
                            0x086ff0, 0x8d5540, 0x47f8a2, 0x21ebfe,   //;Ch 180, 5825MHz,dummy
                            0x04aff0, 0x800000, 0x47f8a2, 0x21ebf6,   //;Ch 184, 4920MHz
                            0x04aff0, 0x855550, 0x47f8a2, 0x21ebfe,   //;Ch 188, 4940MHz
                            0x0caff0, 0x8aaaa8, 0x47f8a2, 0x21ebfe,   //;Ch 192, 4960MHz
                            0x0caff0, 0x800000, 0x47f8a2, 0x21ebf6     //;Ch 196, 4980MHz
                    };

U32 RFMD2958t[] = {
                          0x1422BD,   //Null
                          0x185D17,   //Null
                          0x181979,   //Ch 1
                          0x1e6666,   //Ch 1
                          0x181989,   //Ch 2
                          0x1e6666,   //Ch 2
                          0x181999,   //Ch 3
                          0x1e6666,   //Ch 3
                          0x1819a9,   //Ch 4
                          0x1e6666,   //Ch 4
                          0x1819b9,   //Ch 5
                          0x1e6666,   //Ch 5
                          0x1819c9,   //Ch 6
                          0x1e6666,   //Ch 6
                          0x1819d9,   //Ch 7
                          0x1e6666,   //Ch 7
                          0x1819e9,   //Ch 8
                          0x1e6666,   //Ch 8
                          0x1819f9,   //Ch 9
                          0x1e6666,   //Ch 9
                          0x181a09,   //Ch 10
                          0x1e6666,   //Ch 10
                          0x181a19,   //Ch 11
                          0x1e6666,   //Ch 11
                          0x181a29,   //Ch 12
                          0x1e6666,   //Ch 12
                          0x181a39,   //Ch 13
                          0x1e6666,   //Ch 13
                          0x181a60,   //Ch 14
                          0x1c0000    //Ch 14
                  };



#if 0
int HW_HTP(zd_80211Obj_t *pObj)
{
        void *reg = pObj->reg;
        int i, ret = 0;
        U32 tmpkey, tmpvalue, regvalue, seed;

        dbg_pline_1("\r\nHW_HTP Starting....");

        // PHY CR Registers Read/Write Test
        dbg_pline_1("\r\nPHY CR Registers Read/Write Test Starting....");

        seed = pObj->GetReg(reg, ZD_TSF_LowPart);
        srand(seed);
        LockPhyReg(pObj);

        for (i=0; i<0x0200; i+=4) {
                if ( (i==0x00) || ((i>=0xc8) && (i<=0xfc)) ||
                                ((i>=0x1cc) && (i<=0x1d8)) || ((i>=0x1e0) && (i<=0x1ec))) {
                        // Skip Read Only Register
                        continue;
                }
                tmpkey = (U8)rand();
                pObj->SetReg(reg, i, tmpkey);
                tmpvalue = pObj->GetReg(reg, i);
                if (tmpvalue != tmpkey) {
                        //printf("CR %x Failed (Wr: %x, Rd: %x)\n", i, tmpkey, tmpvalue);
                        dbg_plinew_1("\r\nCR ", i);
                        dbg_pline_1(" Failed ");
                        dbg_plineb_1("(Wr: ", (U8)tmpkey);
                        dbg_plineb_1(", Rd: ", (U8)tmpvalue);
                        dbg_pline_1(")");

                        UnLockPhyReg(pObj);
                        ret = 1;
                } else {
                        //printf("CR %x Success (Wr: %x, Rd: %x)\n", i, tmpkey, tmpvalue);
                        dbg_plinew_1("\r\nCR ", i);
                        dbg_pline_1(" Success ");
                        dbg_plineb_1("(Wr: ", (U8)tmpkey);
                        dbg_plineb_1(", Rd: ", (U8)tmpvalue);
                        dbg_pline_1(")");
                }
        }

        UnLockPhyReg(pObj);
        dbg_pline_1("\r\nPHY CR Registers Read/Write Test End");
        dbg_pline_1("\r\n");


#if 1
        // MAC Registers Read/Write Test
        dbg_pline_1("\r\nMAC Registers Read/Write Test Starting....");
        //to test 0x408, 0x410, 0x42c must set 0x418 to 0
        pObj->SetReg(reg, ZD_GPI_EN, 0);
        seed = pObj->GetReg(reg, ZD_TSF_LowPart);
        srand(seed);
        for (i=0; i<NUM_REG_MASK; i++) {
                tmpkey = (U32)rand();
                tmpkey |= (tmpkey << 16);
                tmpkey &= MacRegMaskTab[i].ReadWriteMask;

                if (MacRegMaskTab[i].Address == 0x42c) {
                        pObj->SetReg(reg, ZD_GPI_EN, 0);

                }



                pObj->SetReg(reg, MacRegMaskTab[i].Address, tmpkey);
                tmpvalue = pObj->GetReg(reg, MacRegMaskTab[i].Address);
                tmpvalue &= MacRegMaskTab[i].ReadWriteMask;
                if (tmpvalue != tmpkey) {
                        //printf("MAC %x Failed (Wr: %x, Rd: %x)\n", MacRegMaskTab[i].Address, tmpkey, tmpvalue);
                        dbg_plinew_1("\r\nMAC ", MacRegMaskTab[i].Address);
                        dbg_pline_1(" Failed ");
                        dbg_plinel_1("(Wr: ", tmpkey);
                        dbg_plinel_1(", Rd: ", tmpvalue);
                        dbg_pline_1(")");
                        ret = 2;
                }
                else {
                        //printf("MAC %x Success (Wr: %x, Rd: %x)\n", MacRegMaskTab[i].Address, tmpkey, tmpvalue);
                        dbg_plinew_1("\r\nMAC ", MacRegMaskTab[i].Address);
                        dbg_pline_1(" Success ");
                        dbg_plinel_1("(Wr: ", tmpkey);
                        dbg_plinel_1(", Rd: ", tmpvalue);
                        dbg_pline_1(")");
                }

        }
        dbg_pline_1("\r\nMAC Registers Read/Write Test End");
        dbg_pline_1("\r\n");
#endif

#if 0
        // EEPROM Read/Write Test
        dbg_pline_1("\r\nEEPROM Read/Write Testing...........");
        seed = pObj->GetReg(reg, ZD_TSF_LowPart);
        srand(seed);

        //for (tmpvalue=0; tmpvalue<1; tmpvalue++){
        {
                tmpkey = (U32)rand();
                tmpkey |= (tmpkey << 16);
                for (i=0; i<256; i++)
                {
                        //if (i == 1)
                        //tmpkey = 0x89;
                        pObj->SetReg(reg, ZD_E2P_SUBID+(i*4), tmpkey);
                }
                // Write to EEPROM
                pObj->SetReg(reg, ZD_EEPROM_PROTECT0, 0x55aa44bb);
                pObj->SetReg(reg, ZD_EEPROM_PROTECT1, 0x33cc22dd);
                pObj->SetReg(reg, ZD_ROMDIR, 0x422);

                // Sleep
                //for (i=0; i<1000; i++)
                //	pObj->DelayUs(5000);
                delay1ms(5);


                // Reset Registers
                for (i=0; i<256; i++)
                {
                        pObj->SetReg(reg, ZD_E2P_SUBID+(i*4), 0);
                }

                // Reload EEPROM
                pObj->SetReg(reg, ZD_ROMDIR, 0x424);

                // Sleep
                //for (i=0; i<1000; i++)
                //	pObj->DelayUs(5000);
                delay1ms(5);

                // Check if right
                for (i=0; i<256; i++)
                {
                        regvalue = pObj->GetReg(reg, ZD_E2P_SUBID+(i*4));
                        if (regvalue != tmpkey) {
                                //printf("EEPROM Addr (%x) error (Wr: %x, Rd: %x)\n", ZD_E2P_SUBID+(i*4), tmpkey, regvalue);
                                dbg_plinew_1("\r\nEEPROM Addr ", ZD_E2P_SUBID+(i*4));
                                dbg_pline_1(" error ");
                                dbg_plinel_1("(Wr: ", tmpkey);
                                dbg_plinel_1(",Rd: ", regvalue);
                                dbg_pline_1(")");
                                ret = 3;
                        }
                }
        }
#endif

        //dbg_pline_1("\r\nDigital Loopback Testing...........");

        dbg_pline_1("\r\nHW_HTP End");
        dbg_pline_1("\r\n");
        return 0;
}
#endif

#define SET_IF_SYNTHESIZER(macp, InputValue)       \
{                                                     \
	mFILL_WRITE_REGISTER( ZD_CR244, (U8) ((InputValue & 0xff0000)>>16));               \
	mFILL_WRITE_REGISTER( ZD_CR243, (U8) ((InputValue & 0xff00) >> 8));      \
	mFILL_WRITE_REGISTER( ZD_CR242, (U8) ((InputValue & 0xff)));   \
}
#define mFILL_WRITE_REGISTER(addr0, value0) \
{                                           \
    WriteAddr[WriteIndex] = addr0;          \
    WriteData[WriteIndex ++] = value0;      \
}


#ifndef HOST_IF_USB
void
HW_Set_IF_Synthesizer(zd_80211Obj_t *pObj, U32 InputValue)
{
        U32	S_bit_cnt;
        U32 tmpvalue;
        void *reg = pObj->reg;
        int i;


        S_bit_cnt = pObj->S_bit_cnt;

        InputValue = InputValue << (31 - S_bit_cnt);

#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )

        pObj->SetReg(reg, ZD_LE2, 0);
        pObj->SetReg(reg, ZD_RF_IF_CLK, 0);

        while(S_bit_cnt) {
                InputValue = InputValue << 1;
                if (InputValue & 0x80000000) {
                        pObj->SetReg(reg, ZD_RF_IF_DATA, 1);
                } else {
                        pObj->SetReg(reg, ZD_RF_IF_DATA, 0);
                }
                pObj->SetReg(reg, ZD_RF_IF_CLK, 1);
                //pObj->DelayUs(50);
                pObj->SetReg(reg, ZD_RF_IF_CLK, 0);

                //pObj->DelayUs(50);
                S_bit_cnt --;
        }

        pObj->SetReg(reg, ZD_LE2, 1);

        if (pObj->S_bit_cnt == 20) {			//Is it Intersil's chipset
                pObj->SetReg(reg, ZD_LE2, 0);
        }
        return;
#else

        LockPhyReg(pObj);
        tmpvalue = pObj->GetReg(reg, ZD_CR203);
        tmpvalue &= ~BIT_1;
        pObj->SetReg(reg, ZD_CR203, tmpvalue);

        tmpvalue = pObj->GetReg(reg, ZD_CR240);
        tmpvalue = 0x80;
        if (tmpvalue & BIT_7) {		// Configure RF by Software
                tmpvalue = pObj->GetReg(reg, ZD_CR203);
                tmpvalue &= ~BIT_2;
                pObj->SetReg(reg, ZD_CR203, tmpvalue);


                while(S_bit_cnt) {
                        InputValue = InputValue << 1;
                        if (InputValue & 0x80000000) {
                                tmpvalue = pObj->GetReg(reg, ZD_CR203);
                                tmpvalue |= BIT_3;
                                pObj->SetReg(reg, ZD_CR203, tmpvalue);
                        } else {
                                tmpvalue = pObj->GetReg(reg, ZD_CR203);
                                tmpvalue &= ~BIT_3;
                                pObj->SetReg(reg, ZD_CR203, tmpvalue);
                        }

                        tmpvalue = pObj->GetReg(reg, ZD_CR203);
                        tmpvalue |= BIT_2;
                        pObj->SetReg(reg, ZD_CR203, tmpvalue);

                        tmpvalue = pObj->GetReg(reg, ZD_CR203);

                        tmpvalue &= ~BIT_2;
                        pObj->SetReg(reg, ZD_CR203, tmpvalue);
                        S_bit_cnt --;
                }
        } else {		// Configure RF by Hardware
                // Make Bit-reverse to meet hardware requirement.
                tmpvalue = 0;
                for (i=0; i<S_bit_cnt; i++) {
                        InputValue = InputValue << 1;
                        if (InputValue & 0x80000000) {
                                tmpvalue |= (0x1 << i);
                        }
                }
                InputValue = tmpvalue;

                // Setup Command-Length
                // wait until command-queue is available
                tmpvalue = pObj->GetReg(reg, ZD_CR241);
                while(tmpvalue & BIT_0) {
                        pObj->DelayUs(1);
                        FPRINT("Command-Queue busy...");
                }

                // write command (from high-byte to low-byte)
                pObj->SetReg(reg, ZD_CR245, InputValue >> 24);
                pObj->SetReg(reg, ZD_CR244, InputValue >> 16);
                pObj->SetReg(reg, ZD_CR243, InputValue >> 8);
                pObj->SetReg(reg, ZD_CR242, InputValue);
        }


        tmpvalue = pObj->GetReg(reg, ZD_CR203);
        tmpvalue |= BIT_1;
        pObj->SetReg(reg, ZD_CR203, tmpvalue);

        if (pObj->S_bit_cnt == 20) {			//Is it Intersil's chipset
                tmpvalue = pObj->GetReg(reg, ZD_CR203);
                tmpvalue &= ~BIT_1;
                pObj->SetReg(reg, ZD_CR203, tmpvalue);
        }

        UnLockPhyReg(pObj);
        return;
#endif
}
#endif


void
LockPhyReg(zd_80211Obj_t *pObj)
{
#ifndef fQuickPhySet

        void *reg = pObj->reg;
        U32	tmpvalue;

        tmpvalue = pObj->GetReg(reg, ZD_CtlReg1);
        tmpvalue &= ~0x80;
        pObj->SetReg(reg, ZD_CtlReg1, tmpvalue);
#endif
}


void
UnLockPhyReg(zd_80211Obj_t *pObj)
{
#ifndef fQuickPhySet
        void *reg = pObj->reg;
        U32	tmpvalue;

        tmpvalue = pObj->GetReg(reg, ZD_CtlReg1);
        tmpvalue |= 0x80;
        pObj->SetReg(reg, ZD_CtlReg1, tmpvalue);
#endif
}


void
HW_Set_Maxim_New_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        void *reg = pObj->reg;
        U32 tmpvalue;

        LockPhyReg(pObj);

#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR23, 0x40);
        pObj->SetReg(reg, ZD_CR15, 0x20);
        pObj->SetReg(reg, ZD_CR28, 0x3e);
        pObj->SetReg(reg, ZD_CR29, 0x00);
        pObj->SetReg(reg, ZD_CR26, 0x11);
        pObj->SetReg(reg, ZD_CR44, 0x33);
        pObj->SetReg(reg, ZD_CR106, 0x2a);
        pObj->SetReg(reg, ZD_CR107, 0x1a);

        pObj->SetReg(reg, ZD_CR109, 0x2b);
        pObj->SetReg(reg, ZD_CR110, 0x2b);
        pObj->SetReg(reg, ZD_CR111, 0x2b);
        pObj->SetReg(reg, ZD_CR112, 0x2b);
        pObj->SetReg(reg, ZD_CR10, 0x89);
        pObj->SetReg(reg, ZD_CR17, 0x20);
        pObj->SetReg(reg, ZD_CR26, 0x93);
        pObj->SetReg(reg, ZD_CR34, 0x30);
        pObj->SetReg(reg, ZD_CR35, 0x40);
        pObj->SetReg(reg, ZD_CR41, 0x24);
        pObj->SetReg(reg, ZD_CR44, 0x32);
        pObj->SetReg(reg, ZD_CR46, 0x90);
        pObj->SetReg(reg, ZD_CR89, 0x18);
        pObj->SetReg(reg, ZD_CR92, 0x0a);

        pObj->SetReg(reg, ZD_CR101, 0x13);
        pObj->SetReg(reg, ZD_CR102, 0x27);
        pObj->SetReg(reg, ZD_CR106, 0x20);
        pObj->SetReg(reg, ZD_CR107, 0x24);
        pObj->SetReg(reg, ZD_CR109, 0x09);
        pObj->SetReg(reg, ZD_CR110, 0x13);
        pObj->SetReg(reg, ZD_CR111, 0x13);
        pObj->SetReg(reg, ZD_CR112, 0x13);
        pObj->SetReg(reg, ZD_CR113, 0x27);
        pObj->SetReg(reg, ZD_CR114, 0x27);
        pObj->SetReg(reg, ZD_CR115, 0x24);
        pObj->SetReg(reg, ZD_CR116, 0x24);
        pObj->SetReg(reg, ZD_CR117, 0xf4);
        pObj->SetReg(reg, ZD_CR118, 0xfa);
        pObj->SetReg(reg, ZD_CR120, 0x4f);
        pObj->SetReg(reg, ZD_CR121, 0x77);
        pObj->SetReg(reg, ZD_CR122, 0xfe);
#else

        pObj->SetReg(reg, ZD_CR23, 0x40);
        pObj->SetReg(reg, ZD_CR15, 0x20);
        pObj->SetReg(reg, ZD_CR28, 0x3e);
        pObj->SetReg(reg, ZD_CR29, 0x00);
        pObj->SetReg(reg, ZD_CR26, 0x11);
        pObj->SetReg(reg, ZD_CR44, 0x34); //4112
        pObj->SetReg(reg, ZD_CR106, 0x2a);
        pObj->SetReg(reg, ZD_CR107, 0x1a);
        pObj->SetReg(reg, ZD_CR109, 0x2b);
        pObj->SetReg(reg, ZD_CR110, 0x2b);
        pObj->SetReg(reg, ZD_CR111, 0x2b);
        pObj->SetReg(reg, ZD_CR112, 0x2b);

#if (defined(GCCK) && defined(OFDM))

        pObj->SetReg(reg, ZD_CR10, 0x89);
        pObj->SetReg(reg, ZD_CR17, 0x20);
        pObj->SetReg(reg, ZD_CR26, 0x93);
        pObj->SetReg(reg, ZD_CR34, 0x30);
        pObj->SetReg(reg, ZD_CR35, 0x40);

        pObj->SetReg(reg, ZD_CR41, 0x24);
        pObj->SetReg(reg, ZD_CR44, 0x32);
        pObj->SetReg(reg, ZD_CR46, 0x90);
        pObj->SetReg(reg, ZD_CR89, 0x18);
        pObj->SetReg(reg, ZD_CR92, 0x0a);
        pObj->SetReg(reg, ZD_CR101, 0x13);
        pObj->SetReg(reg, ZD_CR102, 0x27);
        pObj->SetReg(reg, ZD_CR106, 0x20);
        pObj->SetReg(reg, ZD_CR107, 0x24);
        //pObj->SetReg(reg, ZD_CR109, 0x09);
        //pObj->SetReg(reg, ZD_CR110, 0x13);
        //pObj->SetReg(reg, ZD_CR111, 0x13);




        pObj->SetReg(reg, ZD_CR109, 0x13); //4326
        pObj->SetReg(reg, ZD_CR110, 0x27); //4326
        pObj->SetReg(reg, ZD_CR111, 0x27); //4326
        pObj->SetReg(reg, ZD_CR112, 0x13);
        pObj->SetReg(reg, ZD_CR113, 0x27);
        pObj->SetReg(reg, ZD_CR114, 0x27);
        pObj->SetReg(reg, ZD_CR115, 0x24);
        pObj->SetReg(reg, ZD_CR116, 0x24);
        pObj->SetReg(reg, ZD_CR117, 0xf4);
        //pObj->SetReg(reg, ZD_CR118, 0xfa);
        pObj->SetReg(reg, ZD_CR118, 0x00); //4326
        pObj->SetReg(reg, ZD_CR120, 0x4f);
        //pObj->SetReg(reg, ZD_CR121, 0x77); //3n12
        //pObj->SetReg(reg, ZD_CR121, 0x13); //3d24
        pObj->SetReg(reg, ZD_CR121, 0x06); //4326
        pObj->SetReg(reg, ZD_CR122, 0xfe);
        pObj->SetReg(reg, ZD_CR150, 0x0d); //4407

#elif (defined(ECCK_60_5))

        pObj->SetReg(reg, ZD_CR26, 0x91);
        pObj->SetReg(reg, ZD_CR47, 0x1E);
        pObj->SetReg(reg, ZD_CR106, 0x44);
        pObj->SetReg(reg, ZD_CR107, 0x00);
        pObj->SetReg(reg, ZD_CR14, 0x80);
        pObj->SetReg(reg, ZD_CR10, 0x89);
        pObj->SetReg(reg, ZD_CR11, 0x00);
        pObj->SetReg(reg, ZD_CR24, 0x0e);
        pObj->SetReg(reg, ZD_CR41, 0x24);
        pObj->SetReg(reg, ZD_CR159, 0x93);
        pObj->SetReg(reg, ZD_CR160, 0xfc);
        pObj->SetReg(reg, ZD_CR161, 0x1e);
        pObj->SetReg(reg, ZD_CR162, 0x24);
#endif
#endif

        pObj->CR122Flag = 2;
        pObj->CR31Flag = 2;

        //UnLockPhyReg(pObj);

#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )

        pObj->SetReg(reg, ZD_PE1_PE2, 0x02);
#else
        //LockPhyReg(pObj);
        tmpvalue = pObj->GetReg(reg, ZD_CR203);
        tmpvalue &= ~BIT_4;
        pObj->SetReg(reg, ZD_CR203, tmpvalue);
        //UnLockPhyReg(pObj);
#endif

        HW_Set_IF_Synthesizer(pObj, M2827BF[ChannelNo]);
        HW_Set_IF_Synthesizer(pObj, M2827BN[ChannelNo]);
        HW_Set_IF_Synthesizer(pObj, 0x00400);
        HW_Set_IF_Synthesizer(pObj, 0x00ca1);
        HW_Set_IF_Synthesizer(pObj, 0x10072);
        HW_Set_IF_Synthesizer(pObj, 0x18645);
        HW_Set_IF_Synthesizer(pObj, 0x04006);
        HW_Set_IF_Synthesizer(pObj, 0x000a7);
        HW_Set_IF_Synthesizer(pObj, 0x08258);
        HW_Set_IF_Synthesizer(pObj, 0x03fc9);
        HW_Set_IF_Synthesizer(pObj, 0x0040a);
        HW_Set_IF_Synthesizer(pObj, 0x0000b);
        HW_Set_IF_Synthesizer(pObj, 0x0026c);
#if	defined(ECCK_60_5)

        HW_Set_IF_Synthesizer(pObj, 0x04258);
#endif

#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )

        pObj->SetReg(reg, ZD_PE1_PE2, 0x03);
#else
        //LockPhyReg(pObj);
        tmpvalue = pObj->GetReg(reg, ZD_CR203);
        tmpvalue |= BIT_4;

        pObj->SetReg(reg, ZD_CR203, tmpvalue);
        ;
        //UnLockPhyReg(pObj);
#endif

        UnLockPhyReg(pObj);
}



void
HW_Set_Maxim_New_Chips2(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        void *reg = pObj->reg;
        U32	tmpvalue;

        // Get Phy-Config permission
        LockPhyReg(pObj);

#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR23, 0x40);
        pObj->SetReg(reg, ZD_CR15, 0x20);
        pObj->SetReg(reg, ZD_CR28, 0x3e);
        pObj->SetReg(reg, ZD_CR29, 0x00);
        pObj->SetReg(reg, ZD_CR26, 0x11);
        pObj->SetReg(reg, ZD_CR44, 0x33);
        pObj->SetReg(reg, ZD_CR106, 0x2a);
        pObj->SetReg(reg, ZD_CR107, 0x1a);
        pObj->SetReg(reg, ZD_CR109, 0x2b);
        pObj->SetReg(reg, ZD_CR110, 0x2b);
        pObj->SetReg(reg, ZD_CR111, 0x2b);
        pObj->SetReg(reg, ZD_CR112, 0x2b);
        pObj->SetReg(reg, ZD_CR10, 0x89);
        pObj->SetReg(reg, ZD_CR17, 0x20);
        pObj->SetReg(reg, ZD_CR26, 0x93);
        pObj->SetReg(reg, ZD_CR34, 0x30);
        pObj->SetReg(reg, ZD_CR35, 0x40);
        pObj->SetReg(reg, ZD_CR41, 0x24);
        pObj->SetReg(reg, ZD_CR44, 0x32);
        pObj->SetReg(reg, ZD_CR46, 0x90);
        pObj->SetReg(reg, ZD_CR89, 0x18);
        pObj->SetReg(reg, ZD_CR92, 0x0a);
        pObj->SetReg(reg, ZD_CR101, 0x13);
        pObj->SetReg(reg, ZD_CR102, 0x27);
        pObj->SetReg(reg, ZD_CR106, 0x20);
        pObj->SetReg(reg, ZD_CR107, 0x24);
        pObj->SetReg(reg, ZD_CR109, 0x09);
        pObj->SetReg(reg, ZD_CR110, 0x13);
        pObj->SetReg(reg, ZD_CR111, 0x13);
        pObj->SetReg(reg, ZD_CR112, 0x13);
        pObj->SetReg(reg, ZD_CR113, 0x27);
        pObj->SetReg(reg, ZD_CR114, 0x27);
        pObj->SetReg(reg, ZD_CR115, 0x24);
        pObj->SetReg(reg, ZD_CR116, 0x24);

        pObj->SetReg(reg, ZD_CR117, 0xf4);
        pObj->SetReg(reg, ZD_CR118, 0xfa);
        pObj->SetReg(reg, ZD_CR120, 0x4f);

        pObj->SetReg(reg, ZD_CR121, 0x77);


        pObj->SetReg(reg, ZD_CR122, 0xfe);

#else

        pObj->SetReg(reg, ZD_CR23, 0x40);
        pObj->SetReg(reg, ZD_CR15, 0x20);
        pObj->SetReg(reg, ZD_CR28, 0x3e);
        pObj->SetReg(reg, ZD_CR29, 0x00);
        pObj->SetReg(reg, ZD_CR26, 0x11);
        pObj->SetReg(reg, ZD_CR44, 0x33);
        pObj->SetReg(reg, ZD_CR106, 0x2a);
        pObj->SetReg(reg, ZD_CR107, 0x1a);
        pObj->SetReg(reg, ZD_CR109, 0x2b);
        pObj->SetReg(reg, ZD_CR110, 0x2b);
        pObj->SetReg(reg, ZD_CR111, 0x2b);
        pObj->SetReg(reg, ZD_CR112, 0x2b);

#if (defined(GCCK) && defined(OFDM))

        pObj->SetReg(reg, ZD_CR10, 0x89);
        pObj->SetReg(reg, ZD_CR17, 0x20);
        pObj->SetReg(reg, ZD_CR26, 0x93);
        pObj->SetReg(reg, ZD_CR34, 0x30);
        pObj->SetReg(reg, ZD_CR35, 0x40);
        pObj->SetReg(reg, ZD_CR41, 0x24);
        pObj->SetReg(reg, ZD_CR44, 0x32);

        pObj->SetReg(reg, ZD_CR46, 0x90);
        pObj->SetReg(reg, ZD_CR79, 0x58); //for Atheros compability 4415
        pObj->SetReg(reg, ZD_CR80, 0x30); //for Atheros compability
        pObj->SetReg(reg, ZD_CR81, 0x30); //for Atheros compability
        pObj->SetReg(reg, ZD_CR89, 0x18);







        pObj->SetReg(reg, ZD_CR92, 0x0a);
        pObj->SetReg(reg, ZD_CR101, 0x13);
        pObj->SetReg(reg, ZD_CR102, 0x27);
        pObj->SetReg(reg, ZD_CR106, 0x20);
        pObj->SetReg(reg, ZD_CR107, 0x24);
        pObj->SetReg(reg, ZD_CR109, 0x09);
        pObj->SetReg(reg, ZD_CR110, 0x13);
        pObj->SetReg(reg, ZD_CR111, 0x13);
        pObj->SetReg(reg, ZD_CR112, 0x13);
        pObj->SetReg(reg, ZD_CR113, 0x27);
        pObj->SetReg(reg, ZD_CR114, 0x27);
        pObj->SetReg(reg, ZD_CR115, 0x24);
        pObj->SetReg(reg, ZD_CR116, 0x24);
        pObj->SetReg(reg, ZD_CR117, 0xf4);
        //pObj->SetReg(reg, ZD_CR118, 0xfa);
        pObj->SetReg(reg, ZD_CR118, 0x00); //4326
        pObj->SetReg(reg, ZD_CR120, 0x4f);
        //pObj->SetReg(reg, ZD_CR121, 0x77); //3n12
        //pObj->SetReg(reg, ZD_CR121, 0x13); //3d24
        pObj->SetReg(reg, ZD_CR121, 0x06); //4326
        pObj->SetReg(reg, ZD_CR122, 0xfe);
#elif (defined(ECCK_60_5))

        pObj->SetReg(reg, ZD_CR47, 0x1E);
        pObj->SetReg(reg, ZD_CR106, 0x04);
        pObj->SetReg(reg, ZD_CR107, 0x00);
        pObj->SetReg(reg, ZD_CR14, 0x80);
        pObj->SetReg(reg, ZD_CR10, 0x89);

        pObj->SetReg(reg, ZD_CR11, 0x00);
        pObj->SetReg(reg, ZD_CR161, 0x28);
        pObj->SetReg(reg, ZD_CR162, 0x26);

        pObj->SetReg(reg, ZD_CR24, 0x0e);
        pObj->SetReg(reg, ZD_CR41, 0x24);
        pObj->SetReg(reg, ZD_CR159, 0x93);
        pObj->SetReg(reg, ZD_CR160, 0xfc);
        pObj->SetReg(reg, ZD_CR161, 0x20);
        pObj->SetReg(reg, ZD_CR162, 0x26);
#endif
#endif

        pObj->CR122Flag = 2;
        pObj->CR31Flag = 2;

        //UnLockPhyReg(pObj);

#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )

        pObj->SetReg(reg, ZD_PE1_PE2, 2);
#else
        //LockPhyReg(pObj);
        tmpvalue = pObj->GetReg(reg, ZD_CR203);
        tmpvalue &= ~BIT_4;
        pObj->SetReg(reg, ZD_CR203, tmpvalue);
        //UnLockPhyReg(pObj);
#endif

        HW_Set_IF_Synthesizer(pObj, M2827BF2[ChannelNo]);
        HW_Set_IF_Synthesizer(pObj, M2827BN2[ChannelNo]);
        HW_Set_IF_Synthesizer(pObj, 0x00400);
        HW_Set_IF_Synthesizer(pObj, 0x00ca1);
        HW_Set_IF_Synthesizer(pObj, 0x10072);
        HW_Set_IF_Synthesizer(pObj, 0x18645);
        HW_Set_IF_Synthesizer(pObj, 0x04006);
        HW_Set_IF_Synthesizer(pObj, 0x000a7);
        HW_Set_IF_Synthesizer(pObj, 0x08258);

        HW_Set_IF_Synthesizer(pObj, 0x03fc9);
        HW_Set_IF_Synthesizer(pObj, 0x0040a);
        HW_Set_IF_Synthesizer(pObj, 0x0000b);
        HW_Set_IF_Synthesizer(pObj, 0x0026c);
#if	defined(ECCK_60_5)

        HW_Set_IF_Synthesizer(pObj, 0x04258);
#endif

#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )

        pObj->SetReg(reg, ZD_PE1_PE2, 3);

#else
        //LockPhyReg(pObj);
        tmpvalue = pObj->GetReg(reg, ZD_CR203);
        tmpvalue |= BIT_4;
        pObj->SetReg(reg, ZD_CR203, tmpvalue);
        //UnLockPhyReg(pObj);
#endif

        UnLockPhyReg(pObj);
}


void
HW_Set_GCT_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        void *reg = pObj->reg;

        if (!InitChOnly) {
                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                pObj->SetReg(reg, ZD_CR15, 0xdc);
                pObj->SetReg(reg, ZD_CR113, 0xc0); //3910
                pObj->SetReg(reg, ZD_CR20, 0x0c);
                pObj->SetReg(reg, ZD_CR17, 0x65);
                pObj->SetReg(reg, ZD_CR34, 0x04);
                pObj->SetReg(reg, ZD_CR35, 0x35);
                pObj->SetReg(reg, ZD_CR24, 0x20);
                pObj->SetReg(reg, ZD_CR9, 0xe0);
                pObj->SetReg(reg, ZD_CR127, 0x02);
                pObj->SetReg(reg, ZD_CR10, 0x91);
                pObj->SetReg(reg, ZD_CR23, 0x7f);
                pObj->SetReg(reg, ZD_CR27, 0x10);
                pObj->SetReg(reg, ZD_CR28, 0x7a);
                pObj->SetReg(reg, ZD_CR79, 0xb5);
                pObj->SetReg(reg, ZD_CR64, 0x80);
                //++ Enable D.C cancellation (CR33 Bit_5) to avoid
                //	 CCA always high.
                pObj->SetReg(reg, ZD_CR33, 0x28);
                //--

                pObj->SetReg(reg, ZD_CR38, 0x30);

                UnLockPhyReg(pObj);

                HW_Set_IF_Synthesizer(pObj, 0x1F0000);
                HW_Set_IF_Synthesizer(pObj, 0x1F0000);
                HW_Set_IF_Synthesizer(pObj, 0x1F0200);
                HW_Set_IF_Synthesizer(pObj, 0x1F0600);
                HW_Set_IF_Synthesizer(pObj, 0x1F8600);
                HW_Set_IF_Synthesizer(pObj, 0x1F8600);
                HW_Set_IF_Synthesizer(pObj, 0x002050);
                HW_Set_IF_Synthesizer(pObj, 0x1F8000);
                HW_Set_IF_Synthesizer(pObj, 0x1F8200);
                HW_Set_IF_Synthesizer(pObj, 0x1F8600);
                HW_Set_IF_Synthesizer(pObj, 0x1c0000);
                HW_Set_IF_Synthesizer(pObj, 0x10c458);

                HW_Set_IF_Synthesizer(pObj, 0x088e92);
                HW_Set_IF_Synthesizer(pObj, 0x187b82);
                HW_Set_IF_Synthesizer(pObj, 0x0401b4);
                HW_Set_IF_Synthesizer(pObj, 0x140816);
                HW_Set_IF_Synthesizer(pObj, 0x0c7000);
                HW_Set_IF_Synthesizer(pObj, 0x1c0000);
                HW_Set_IF_Synthesizer(pObj, 0x02ccae);
                HW_Set_IF_Synthesizer(pObj, 0x128023);
                HW_Set_IF_Synthesizer(pObj, 0x0a0000);
                HW_Set_IF_Synthesizer(pObj, GRF5101T[ChannelNo]);
                HW_Set_IF_Synthesizer(pObj, 0x06e380);
                HW_Set_IF_Synthesizer(pObj, 0x16cb94);
                HW_Set_IF_Synthesizer(pObj, 0x0e1740);
                HW_Set_IF_Synthesizer(pObj, 0x014980);
                HW_Set_IF_Synthesizer(pObj, 0x116240);
                HW_Set_IF_Synthesizer(pObj, 0x090000);
                HW_Set_IF_Synthesizer(pObj, 0x192304);
                HW_Set_IF_Synthesizer(pObj, 0x05112f);
                HW_Set_IF_Synthesizer(pObj, 0x0d54a8);
                HW_Set_IF_Synthesizer(pObj, 0x0f8000);
                HW_Set_IF_Synthesizer(pObj, 0x1c0008);
                HW_Set_IF_Synthesizer(pObj, 0x1c0000);

                HW_Set_IF_Synthesizer(pObj, GRF5101T[ChannelNo]);
                HW_Set_IF_Synthesizer(pObj, 0x1c0008);
                HW_Set_IF_Synthesizer(pObj, 0x150000);
                HW_Set_IF_Synthesizer(pObj, 0x0c7000);
                HW_Set_IF_Synthesizer(pObj, 0x150800);
                HW_Set_IF_Synthesizer(pObj, 0x150000);
        } else {
                HW_Set_IF_Synthesizer(pObj, 0x1c0000);
                HW_Set_IF_Synthesizer(pObj, GRF5101T[ChannelNo]);

                HW_Set_IF_Synthesizer(pObj, 0x1c0008);
        }
}


void
HW_Set_AL2210MPVB_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        void *reg = pObj->reg;
        U32	tmpvalue;

        pObj->SetReg(reg, ZD_PE1_PE2, 2);

        if (!InitChOnly) {
                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR9, 0xe0);
                pObj->SetReg(reg, ZD_CR10, 0x91);
                pObj->SetReg(reg, ZD_CR12, 0x90);
                pObj->SetReg(reg, ZD_CR15, 0xd0);
                pObj->SetReg(reg, ZD_CR16, 0x40);
                pObj->SetReg(reg, ZD_CR17, 0x58);
                pObj->SetReg(reg, ZD_CR18, 0x04);
                pObj->SetReg(reg, ZD_CR23, 0x66);
                pObj->SetReg(reg, ZD_CR24, 0x14);
                pObj->SetReg(reg, ZD_CR26, 0x90);
                pObj->SetReg(reg, ZD_CR27, 0x30);
                pObj->SetReg(reg, ZD_CR31, 0x80);
                pObj->SetReg(reg, ZD_CR34, 0x06);
                pObj->SetReg(reg, ZD_CR35, 0x3e);
                pObj->SetReg(reg, ZD_CR38, 0x38);
                pObj->SetReg(reg, ZD_CR46, 0x90);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                pObj->SetReg(reg, ZD_CR64, 0x64);
                pObj->SetReg(reg, ZD_CR79, 0xb5);
                pObj->SetReg(reg, ZD_CR80, 0x38);
                pObj->SetReg(reg, ZD_CR81, 0x30);
                pObj->SetReg(reg, ZD_CR113, 0xc0);
                pObj->SetReg(reg, ZD_CR127, 0x03);
                UnLockPhyReg(pObj);

                HW_Set_IF_Synthesizer(pObj, AL2210TB[ChannelNo]);
                HW_Set_IF_Synthesizer(pObj, 0x00fcb1);
                HW_Set_IF_Synthesizer(pObj, 0x358132);
                HW_Set_IF_Synthesizer(pObj, 0x0108b3);
                HW_Set_IF_Synthesizer(pObj, 0xc77804);
                HW_Set_IF_Synthesizer(pObj, 0x456415);
                HW_Set_IF_Synthesizer(pObj, 0xff2226);
                HW_Set_IF_Synthesizer(pObj, 0x806667);
                HW_Set_IF_Synthesizer(pObj, 0x7860f8);
                HW_Set_IF_Synthesizer(pObj, 0xbb01c9);
                HW_Set_IF_Synthesizer(pObj, 0x00000A);
                HW_Set_IF_Synthesizer(pObj, 0x00000B);

                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                tmpvalue = pObj->GetReg(reg, ZD_RADIO_PD);

                tmpvalue &= ~BIT_0;
                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                tmpvalue |= BIT_0;
                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                pObj->SetReg(reg, ZD_RFCFG, 0x5);
                pObj->DelayUs(100);
                pObj->SetReg(reg, ZD_RFCFG, 0x0);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                UnLockPhyReg(pObj);
        } else {
                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                tmpvalue = pObj->GetReg(reg, ZD_RADIO_PD);
                tmpvalue &= ~BIT_0;

                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                tmpvalue |= BIT_0;



                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                pObj->SetReg(reg, ZD_RFCFG, 0x5);
                pObj->DelayUs(100);
                pObj->SetReg(reg, ZD_RFCFG, 0x0);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                UnLockPhyReg(pObj);
                HW_Set_IF_Synthesizer(pObj, AL2210TB[ChannelNo]);
        }

        pObj->SetReg(reg, ZD_PE1_PE2, 3);
}


void
HW_Set_AL2210_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)

{
        void *reg = pObj->reg;
        U32	tmpvalue;

        pObj->SetReg(reg, ZD_PE1_PE2, 2);

        if (!InitChOnly) {
                LockPhyReg(pObj);



                pObj->SetReg(reg, ZD_CR9, 0xe0);
                pObj->SetReg(reg, ZD_CR10, 0x91);
                pObj->SetReg(reg, ZD_CR12, 0x90);
                pObj->SetReg(reg, ZD_CR15, 0xd0);
                pObj->SetReg(reg, ZD_CR16, 0x40);
                pObj->SetReg(reg, ZD_CR17, 0x58);
                pObj->SetReg(reg, ZD_CR18, 0x04);
                pObj->SetReg(reg, ZD_CR23, 0x66);
                pObj->SetReg(reg, ZD_CR24, 0x14);

                pObj->SetReg(reg, ZD_CR26, 0x90);

                pObj->SetReg(reg, ZD_CR31, 0x80);
                pObj->SetReg(reg, ZD_CR34, 0x06);
                pObj->SetReg(reg, ZD_CR35, 0x3e);
                pObj->SetReg(reg, ZD_CR38, 0x38);
                pObj->SetReg(reg, ZD_CR46, 0x90);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                pObj->SetReg(reg, ZD_CR64, 0x64);
                pObj->SetReg(reg, ZD_CR79, 0xb5);
                pObj->SetReg(reg, ZD_CR80, 0x38);
                pObj->SetReg(reg, ZD_CR81, 0x30);
                pObj->SetReg(reg, ZD_CR113, 0xc0);
                pObj->SetReg(reg, ZD_CR127, 0x3);
                UnLockPhyReg(pObj);

                HW_Set_IF_Synthesizer(pObj, AL2210TB[ChannelNo]);

                HW_Set_IF_Synthesizer(pObj, 0x00fcb1);
                HW_Set_IF_Synthesizer(pObj, 0x358132);
                HW_Set_IF_Synthesizer(pObj, 0x0108b3);
                HW_Set_IF_Synthesizer(pObj, 0xc77804);
                HW_Set_IF_Synthesizer(pObj, 0x456415);
                HW_Set_IF_Synthesizer(pObj, 0xff2226);
                HW_Set_IF_Synthesizer(pObj, 0x806667);
                HW_Set_IF_Synthesizer(pObj, 0x7860f8);
                HW_Set_IF_Synthesizer(pObj, 0xbb01c9);
                HW_Set_IF_Synthesizer(pObj, 0x00000A);
                HW_Set_IF_Synthesizer(pObj, 0x00000B);

                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                tmpvalue = pObj->GetReg(reg, ZD_RADIO_PD);
                tmpvalue &= ~BIT_0;
                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);

                tmpvalue |= BIT_0;
                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                pObj->SetReg(reg, ZD_RFCFG, 0x5);
                pObj->DelayUs(100);
                pObj->SetReg(reg, ZD_RFCFG, 0x0);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                UnLockPhyReg(pObj);
        } else {
                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                tmpvalue = pObj->GetReg(reg, ZD_RADIO_PD);
                tmpvalue &= ~BIT_0;
                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                tmpvalue |= BIT_0;
                pObj->SetReg(reg, ZD_RADIO_PD, tmpvalue);
                pObj->SetReg(reg, ZD_RFCFG, 0x5);
                pObj->DelayUs(100);
                pObj->SetReg(reg, ZD_RFCFG, 0x0);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                UnLockPhyReg(pObj);
                HW_Set_IF_Synthesizer(pObj, AL2210TB[ChannelNo]);
        }

        pObj->SetReg(reg, ZD_PE1_PE2, 3);
}
//------------------------------------------------------------------------------
// Procedure:	 HW_Set_AL7230B_Chips
//
// Description:
//
// Arguments:
//		Adapter - ptr to Adapter object instance
//		ChannelNo
//		Initial Channel only
//
// Returns:		(none)
//
// Note:
//-------------------------------------------------------------------------------
#ifndef ZD1211B
// 1211
void HW_Set_AL7230B_RF_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly, U8 MAC_Mode)
{
        void	*reg = pObj->reg;
        U32		tmpValue;
        U32		ChannelNo_temp;
        //int     i;
        static u8 mOldMacMode = MIXED_MODE;
        U16     WriteAddr[256];
        U16     WriteData[256];
        U16     WriteIndex = 0;

        LockPhyReg(pObj);
        pObj->SetReg(reg, ZD_CR240, 0x57);
        UnLockPhyReg(pObj);

        tmpValue = pObj->GetReg(reg, CtlReg1);
        tmpValue &= ~0x80;
        pObj->SetReg(reg, CtlReg1, tmpValue);

        if (!InitChOnly) {
                mFILL_WRITE_REGISTER( ZD_CR15, 0x20);
                mFILL_WRITE_REGISTER( ZD_CR23, 0x40);
                mFILL_WRITE_REGISTER( ZD_CR24, 0x20);
                mFILL_WRITE_REGISTER( ZD_CR26, 0x11);
                mFILL_WRITE_REGISTER( ZD_CR28, 0x3e);
                mFILL_WRITE_REGISTER( ZD_CR29, 0x00);
                mFILL_WRITE_REGISTER( ZD_CR44, 0x33);
                mFILL_WRITE_REGISTER( ZD_CR106, 0x22);  //from 0x2a to 0x22 for AL7230B
                mFILL_WRITE_REGISTER( ZD_CR107, 0x1a);
                mFILL_WRITE_REGISTER( ZD_CR109, 0x9);
                mFILL_WRITE_REGISTER( ZD_CR110, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR111, 0x2b);
                mFILL_WRITE_REGISTER( ZD_CR112, 0x2b);
                mFILL_WRITE_REGISTER( ZD_CR119, 0xa);
                mFILL_WRITE_REGISTER( ZD_CR122, 0xfc); //from /e0 to fc for AL7230B
                mFILL_WRITE_REGISTER( ZD_CR10, 0x89);
                mFILL_WRITE_REGISTER( ZD_CR17, 0x28);
                mFILL_WRITE_REGISTER( ZD_CR26, 0x93);
                mFILL_WRITE_REGISTER( ZD_CR34, 0x30);
                mFILL_WRITE_REGISTER( ZD_CR35, 0x3E);
                mFILL_WRITE_REGISTER( ZD_CR41, 0x24);
                mFILL_WRITE_REGISTER( ZD_CR44, 0x32);
                mFILL_WRITE_REGISTER( ZD_CR46, 0x96);
                mFILL_WRITE_REGISTER( ZD_CR47, 0x1e);
                mFILL_WRITE_REGISTER( ZD_CR79, 0x58);
                mFILL_WRITE_REGISTER( ZD_CR80, 0x30);
                mFILL_WRITE_REGISTER( ZD_CR81, 0x30);
                mFILL_WRITE_REGISTER( ZD_CR87, 0x0A);
                mFILL_WRITE_REGISTER( ZD_CR89, 0x04);
                mFILL_WRITE_REGISTER( ZD_CR92, 0x0a);
                mFILL_WRITE_REGISTER( ZD_CR99, 0x28);
                mFILL_WRITE_REGISTER( ZD_CR100, 0x02);
                mFILL_WRITE_REGISTER( ZD_CR101, 0x13);
                mFILL_WRITE_REGISTER( ZD_CR102, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR106, 0x22); //from 0x20 to 0x22 for AL7230B
                mFILL_WRITE_REGISTER( ZD_CR107, 0x3f);
                mFILL_WRITE_REGISTER( ZD_CR109, 0x09);
                mFILL_WRITE_REGISTER( ZD_CR110, 0x1f);
                mFILL_WRITE_REGISTER( ZD_CR111, 0x1f);
                mFILL_WRITE_REGISTER( ZD_CR112, 0x1f);
                mFILL_WRITE_REGISTER( ZD_CR113, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR114, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR115, 0x24);
                mFILL_WRITE_REGISTER( ZD_CR116, 0x3f);
                mFILL_WRITE_REGISTER( ZD_CR117, 0xfa);
                mFILL_WRITE_REGISTER( ZD_CR118, 0xfc);
                mFILL_WRITE_REGISTER( ZD_CR119, 0x10);
                mFILL_WRITE_REGISTER( ZD_CR120, 0x4f);
                mFILL_WRITE_REGISTER( ZD_CR121, 0x77);
                mFILL_WRITE_REGISTER( ZD_CR137, 0x88);
                mFILL_WRITE_REGISTER( ZD_CR138, 0xa8);
                mFILL_WRITE_REGISTER( ZD_CR252, 0x34);
                mFILL_WRITE_REGISTER( ZD_CR253, 0x34);
                // mFILL_WRITE_REGISTER( ZD_CR240, 0x57);

                if( MAC_Mode != PURE_A_MODE ) {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f);  //PLL_OFF
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2]);
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2+1]);
                        //SET_IF_SYNTHESIZER(macp, 0x8cccd0);
                        SET_IF_SYNTHESIZER(macp, 0x4ff821);
                        SET_IF_SYNTHESIZER(macp, 0xc5fbfc);
                        SET_IF_SYNTHESIZER(macp, 0x21ebfe);
                        SET_IF_SYNTHESIZER(macp, 0xaad401);
                        SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                        SET_IF_SYNTHESIZER(macp, 0xe04073);
                        SET_IF_SYNTHESIZER(macp, 0x193d76);
                        SET_IF_SYNTHESIZER(macp, 0x9dd844);
                        SET_IF_SYNTHESIZER(macp, 0x500007);
                        SET_IF_SYNTHESIZER(macp, 0xd8c010);
                        SET_IF_SYNTHESIZER(macp, 0x3c900);
                        //Adapter->AL7230CCKSetFlag=0;
                        SET_IF_SYNTHESIZER(macp, 0xbfffff);
                        SET_IF_SYNTHESIZER(macp, 0x700000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d58);
                        //AcquireCtrOfPhyReg(Adapter);
                        //ZD1205_WRITE_REGISTER(Adapter, CR251, 0x2f);  //PLL_OFF
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x3f);  //PLL_ON
                        mFILL_WRITE_REGISTER( ZD_CR128, 0x14);
                        mFILL_WRITE_REGISTER( ZD_CR129, 0x12);
                        mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                        mFILL_WRITE_REGISTER( ZD_CR38, 0x38);
                        mFILL_WRITE_REGISTER( ZD_CR136, 0xdf);
                        ///ZD1211_WRITE_MULTI_REG(Adapter, WriteAddr, WriteData, &WriteIndex);
                        ///NdisStallExecution(1000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d59);
                        ///ZD1211_WRITE_MULTI_REG(Adapter, WriteAddr, WriteData, &WriteIndex);
                        ///NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d5c);
                        ///ZD1211_WRITE_MULTI_REG(Adapter, WriteAddr, WriteData, &WriteIndex);
                        ///NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d58);
                } else {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f); // shdnb(PLL_ON)=0
                        if((34 <= ChannelNo) && (ChannelNo <= 48)) {
                                ChannelNo_temp=(ChannelNo/2)-13;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                        } else {
                                ChannelNo_temp=(ChannelNo/4)-1;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                        }
                        SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+3]);
                        SET_IF_SYNTHESIZER(macp, 0x47f8a2);
                        SET_IF_SYNTHESIZER(macp, 0xc5fbfa);
                        //SET_IF_SYNTHESIZER(macp, 0x21ebf6);
                        SET_IF_SYNTHESIZER(macp, 0xaafca1);
                        SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                        SET_IF_SYNTHESIZER(macp, 0xe04073);
                        SET_IF_SYNTHESIZER(macp, 0x193d76);
                        SET_IF_SYNTHESIZER(macp, 0x9dd844);
                        SET_IF_SYNTHESIZER(macp, 0x500607);
                        SET_IF_SYNTHESIZER(macp, 0xd8c010);
                        if((48 < ChannelNo) && (ChannelNo < 184)) {
                                SET_IF_SYNTHESIZER(macp, 0x3c2800);
                        } else {
                                SET_IF_SYNTHESIZER(macp, 0x3e2800);
                        }
                        SET_IF_SYNTHESIZER(macp, 0xbfffff);
                        SET_IF_SYNTHESIZER(macp, 0x700000);
                        SET_IF_SYNTHESIZER(macp, 0xf35d48);
                        //AcquireCtrOfPhyReg(Adapter);
                        //ZD1205_WRITE_REGISTER(Adapter, CR251, 0x2f); // shdnb(PLL_ON)=0
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x3f); // shdnb(PLL_ON)=1
                        mFILL_WRITE_REGISTER( ZD_CR128, 0x12);
                        mFILL_WRITE_REGISTER( ZD_CR129, 0x10);
                        mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                        mFILL_WRITE_REGISTER( ZD_CR38, 0x7f);
                        mFILL_WRITE_REGISTER( ZD_CR136, 0x5f);
                        ///ZD1211_WRITE_MULTI_REG(Adapter, WriteAddr, WriteData, &WriteIndex);
                        ///NdisStallExecution(1000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d59);
                        ///ZD1211_WRITE_MULTI_REG(Adapter, WriteAddr, WriteData, &WriteIndex);
                        ///NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d5c);
                        ///ZD1211_WRITE_MULTI_REG(Adapter, WriteAddr, WriteData, &WriteIndex);
                        ///NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf35d48);
                }

        } else {
                if( MAC_Mode != PURE_A_MODE ) {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f);  //PLL_OFF
                        //SET_IF_SYNTHESIZER(macp, 0x0b3331);
                        if ( 1 || mOldMacMode != MAC_Mode ) {
                                SET_IF_SYNTHESIZER(macp, 0x4ff821);
                                SET_IF_SYNTHESIZER(macp, 0xc5fbfc);
                                SET_IF_SYNTHESIZER(macp, 0x21ebfe);
                                SET_IF_SYNTHESIZER(macp, 0xaad401);
                                SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                                SET_IF_SYNTHESIZER(macp, 0xe04073);
                                SET_IF_SYNTHESIZER(macp, 0x193d76);
                                SET_IF_SYNTHESIZER(macp, 0x9dd844);
                                SET_IF_SYNTHESIZER(macp, 0x500007);
                                SET_IF_SYNTHESIZER(macp, 0xd8c010);
                                SET_IF_SYNTHESIZER(macp, 0x3c9000);
                                SET_IF_SYNTHESIZER(macp, 0xf15d58);

                                mFILL_WRITE_REGISTER( ZD_CR128, 0x14);
                                mFILL_WRITE_REGISTER( ZD_CR129, 0x12);
                                mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                                mFILL_WRITE_REGISTER( ZD_CR38, 0x38);
                                mFILL_WRITE_REGISTER( ZD_CR136, 0xdf);
                                mOldMacMode = MAC_Mode;
                        }
                        //Adapter->AL7230CCKSetFlag=0;
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2]);
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2+1]);
                        SET_IF_SYNTHESIZER(macp, 0x3c9000);
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x3f);  //PLL_ON
                } else {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f); // shdnb(PLL_ON)=0

                        if ( 1 || mOldMacMode != MAC_Mode ) {
                                SET_IF_SYNTHESIZER(macp, 0x47f8a2);
                                SET_IF_SYNTHESIZER(macp, 0xc5fbfa);
                                SET_IF_SYNTHESIZER(macp, 0xaafca1);
                                SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                                SET_IF_SYNTHESIZER(macp, 0xe04073);
                                SET_IF_SYNTHESIZER(macp, 0x193d76);
                                SET_IF_SYNTHESIZER(macp, 0x9dd844);
                                SET_IF_SYNTHESIZER(macp, 0x500607);
                                SET_IF_SYNTHESIZER(macp, 0xd8c010);
                                SET_IF_SYNTHESIZER(macp, 0xf35d48);
                                mFILL_WRITE_REGISTER( ZD_CR128, 0x12);
                                mFILL_WRITE_REGISTER( ZD_CR129, 0x10);
                                mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                                mFILL_WRITE_REGISTER( ZD_CR38, 0x7f);
                                mFILL_WRITE_REGISTER( ZD_CR136, 0x5f);
                                mOldMacMode = MAC_Mode;
                        }

                        if((48 < ChannelNo) && (ChannelNo < 184)) {
                                SET_IF_SYNTHESIZER(macp, 0x3c2800);
                        } else {
                                SET_IF_SYNTHESIZER(macp, 0x3e2800);
                        }

                        if((34 <= ChannelNo) && (ChannelNo <= 48)) {
                                ChannelNo_temp=(ChannelNo/2)-13;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+3]);
                        } else {
                                ChannelNo_temp=(ChannelNo/4)-1;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+3]);
                        }

                        mFILL_WRITE_REGISTER( ZD_CR251, 0x3f);  //PLL_ON
                }

        }

        mFILL_WRITE_REGISTER( ZD_CR203, 0x06);
        ZD1211_WRITE_MULTI_REG(WriteAddr, WriteData, &WriteIndex);



        tmpValue = pObj->GetReg(reg, CtlReg1);
        tmpValue |= ~0x80;
        pObj->SetReg(reg, CtlReg1, tmpValue);

        LockPhyReg(pObj);
        pObj->SetReg(reg, ZD_CR240, 0x80);

        pObj->CR203Flag = 2;
        if (pObj->HWFeature & BIT_8) //CR47 CCK gain patch
        {
                tmpValue = pObj->GetReg(reg, E2P_PHY_REG);
                pObj->SetReg(reg, ZD_CR47, (tmpValue & 0xff)); //This feature is OK to be overwritten with a lower value by other feature
        }
        if (pObj->HWFeature & BIT_21)  //6321 for FCC regulation, enabled per HWFeature 6M band edge bit (for AL2230, AL2230S)
        {
                if (ChannelNo == 1 || ChannelNo == 11)  //MARK_003, band edge, these may depend on PCB layout
                {
                        pObj->SetReg(reg, ZD_CR128, 0x12);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                } else //(ChannelNo 2 ~ 10, 12 ~ 14)
                {
                        pObj->SetReg(reg, ZD_CR128, 0x14);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                }
        }

        UnLockPhyReg(pObj);

        //	pObj->CR31Flag = 2;
        //   macp->PHY_G_6M_BandEdge_Flag = 0;
        //	if(macp->PHY_36M_Setpoint_Flag != 0)
        //	{
        //		for(i=0;i<16;i++)
        //			macp->a_Calibration_Data[2][i] = macp->PHY_36M_A_Calibration_Setpoint[i];
        //		for(i=0;i<32;i++)
        //			macp->a_Interpolation_Data[2][i] = macp->PHY_36M_A_Interpolation_Setpoint[i];
        //		macp->PHY_36M_Setpoint_Flag = 0;
        //	}

}
#else
// 1215
void HW_Set_AL7230B_RF_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly, U8 MAC_Mode)
{
        void	*reg = pObj->reg;
        U32		tmpValue;
        U32		ChannelNo_temp;
        int     i;
        static  u8 mOldMacMode = MIXED_MODE;
        U16     WriteAddr[256];
        U16     WriteData[256];
        U16     WriteIndex = 0;

        LockPhyReg(pObj);
        pObj->SetReg(reg, ZD_CR240, 0x57);
        pObj->SetReg(reg, ZD_CR9, 0xe0);
        UnLockPhyReg(pObj);

        tmpValue = pObj->GetReg(reg, CtlReg1);
        tmpValue &= ~0x80;
        pObj->SetReg(reg, CtlReg1, tmpValue);


        if (!InitChOnly) {
                //mFILL_WRITE_REGISTER( ZD_CR9, 0xe0);//5119
                mFILL_WRITE_REGISTER( ZD_CR10, 0x89);
                mFILL_WRITE_REGISTER( ZD_CR15, 0x20);
                mFILL_WRITE_REGISTER( ZD_CR17, 0x2B);//for newest(3rd cut)AL2230
                mFILL_WRITE_REGISTER( ZD_CR20, 0x10);//4N25->Stone Request
                mFILL_WRITE_REGISTER( ZD_CR23, 0x40);
                mFILL_WRITE_REGISTER( ZD_CR24, 0x20);
                mFILL_WRITE_REGISTER( ZD_CR26, 0x93);
                mFILL_WRITE_REGISTER( ZD_CR28, 0x3e);
                mFILL_WRITE_REGISTER( ZD_CR29, 0x00);
                mFILL_WRITE_REGISTER( ZD_CR33, 0x28);	//5613
                mFILL_WRITE_REGISTER( ZD_CR34, 0x30);
                mFILL_WRITE_REGISTER( ZD_CR35, 0x3e);  //for newest(3rd cut) AL2230
                mFILL_WRITE_REGISTER( ZD_CR41, 0x24);
                mFILL_WRITE_REGISTER( ZD_CR44, 0x32);
                mFILL_WRITE_REGISTER( ZD_CR46, 0x99);  //for newest(3rd cut) AL2230
                mFILL_WRITE_REGISTER( ZD_CR47, 0x1e);
                mFILL_WRITE_REGISTER( ZD_CR48, 0x00);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR49, 0x00);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR51, 0x01);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR52, 0x80);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR53, 0x7e);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR65, 0x00);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR66, 0x00);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR67, 0x00);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR68, 0x00);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR69, 0x28);	//ZD1215 5610
                mFILL_WRITE_REGISTER( ZD_CR79, 0x58);
                mFILL_WRITE_REGISTER( ZD_CR80, 0x30);
                mFILL_WRITE_REGISTER( ZD_CR81, 0x30);
                mFILL_WRITE_REGISTER( ZD_CR87, 0x0A);
                mFILL_WRITE_REGISTER( ZD_CR89, 0x04);
                mFILL_WRITE_REGISTER( ZD_CR90, 0x58);  //5112
                mFILL_WRITE_REGISTER( ZD_CR91, 0x00);  //5613
                mFILL_WRITE_REGISTER( ZD_CR92, 0x0a);
                mFILL_WRITE_REGISTER( ZD_CR98, 0x8d);  //4804, for 1212 new algorithm
                mFILL_WRITE_REGISTER( ZD_CR99, 0x00);
                mFILL_WRITE_REGISTER( ZD_CR100, 0x02);
                mFILL_WRITE_REGISTER( ZD_CR101, 0x13);
                mFILL_WRITE_REGISTER( ZD_CR102, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR106, 0x20);	// change to 0x24 for AL7230B
                mFILL_WRITE_REGISTER( ZD_CR107, 0x28);
                mFILL_WRITE_REGISTER( ZD_CR109, 0x13);  //4804, for 1212 new algorithm
                mFILL_WRITE_REGISTER( ZD_CR110, 0x1f);  //5127, 0x13->0x1f
                mFILL_WRITE_REGISTER( ZD_CR111, 0x1f);  //0x13 to 0x1f for AL7230B
                mFILL_WRITE_REGISTER( ZD_CR112, 0x1f);
                mFILL_WRITE_REGISTER( ZD_CR113, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR114, 0x27);
                mFILL_WRITE_REGISTER( ZD_CR115, 0x24);
                mFILL_WRITE_REGISTER( ZD_CR116, 0x2a);
                mFILL_WRITE_REGISTER( ZD_CR117, 0xfa);
                mFILL_WRITE_REGISTER( ZD_CR118, 0xfa);
                mFILL_WRITE_REGISTER( ZD_CR119, 0x12);
                mFILL_WRITE_REGISTER( ZD_CR120, 0x4f);
                mFILL_WRITE_REGISTER( ZD_CR121, 0x6c);  //5613
                mFILL_WRITE_REGISTER( ZD_CR122, 0xfc);  // E0->FCh at 4901
                mFILL_WRITE_REGISTER( ZD_CR123, 0x57);  //5613
                mFILL_WRITE_REGISTER( ZD_CR125, 0xad);  //4804, for 1212 new algorithm
                mFILL_WRITE_REGISTER( ZD_CR126, 0x6c);  //5613
                mFILL_WRITE_REGISTER( ZD_CR127, 0x03);  //4804, for 1212 new algorithm
                mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                mFILL_WRITE_REGISTER( ZD_CR131, 0x00);	 //5112
                mFILL_WRITE_REGISTER( ZD_CR137, 0x50);	 //5613
                mFILL_WRITE_REGISTER( ZD_CR138, 0xa8);  //5112
                mFILL_WRITE_REGISTER( ZD_CR144, 0xac);  //5613
                mFILL_WRITE_REGISTER( ZD_CR148, 0x40);	 //5112
                mFILL_WRITE_REGISTER( ZD_CR149, 0x40);  //4O07, 50->40
                mFILL_WRITE_REGISTER( ZD_CR150, 0x1a);  //5112, 0C->1A


                mFILL_WRITE_REGISTER( ZD_CR252, 0x34);
                mFILL_WRITE_REGISTER( ZD_CR253, 0x34);
                //mFILL_WRITE_REGISTER( ZD_CR240, 0x57);
                if( MAC_Mode != PURE_A_MODE ) {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f);  //PLL_OFF
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2]);
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2+1]);
                        //SET_IF_SYNTHESIZER(macp, 0x8cccd0);
                        SET_IF_SYNTHESIZER(macp, 0x4ff821);
                        SET_IF_SYNTHESIZER(macp, 0xc5fbfc);
                        SET_IF_SYNTHESIZER(macp, 0x21ebfe);
                        SET_IF_SYNTHESIZER(macp, 0xaad401);
                        SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                        SET_IF_SYNTHESIZER(macp, 0xe04073);
                        SET_IF_SYNTHESIZER(macp, 0x190d76);
                        SET_IF_SYNTHESIZER(macp, 0x9dd844);
                        SET_IF_SYNTHESIZER(macp, 0x500007);
                        SET_IF_SYNTHESIZER(macp, 0xd8c010);
                        SET_IF_SYNTHESIZER(macp, 0x3c900);
                        //Adapter->AL7230CCKSetFlag=0;
                        SET_IF_SYNTHESIZER(macp, 0xbfffff);
                        SET_IF_SYNTHESIZER(macp, 0x700000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d58);
                        //AcquireCtrOfPhyReg(Adapter);
                        //mFILL_WRITE_REGISTER( ZD_CR251, 0x2f);  //PLL_OFF
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x7f);  //PLL_ON
                        mFILL_WRITE_REGISTER( ZD_CR128, 0x14);
                        mFILL_WRITE_REGISTER( ZD_CR129, 0x12);
                        mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                        mFILL_WRITE_REGISTER( ZD_CR38, 0x38);
                        mFILL_WRITE_REGISTER( ZD_CR136, 0xdf);
                        //NdisStallExecution(1000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d59);
                        //NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d5c);
                        //NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d58);
                } else {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f); // shdnb(PLL_ON)=0
                        if((34 <= ChannelNo) && (ChannelNo <= 48)) {
                                ChannelNo_temp=(ChannelNo/2)-13;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                        } else {
                                ChannelNo_temp=(ChannelNo/4)-1;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                        }
                        SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+3]);
                        SET_IF_SYNTHESIZER(macp, 0x47f8a2);
                        SET_IF_SYNTHESIZER(macp, 0xc5fbfa);
                        //SET_IF_SYNTHESIZER(macp, 0x21ebf6);
                        SET_IF_SYNTHESIZER(macp, 0xaafca1);
                        SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                        SET_IF_SYNTHESIZER(macp, 0xe04073);
                        SET_IF_SYNTHESIZER(macp, 0x190d36);
                        SET_IF_SYNTHESIZER(macp, 0x9dd844);
                        SET_IF_SYNTHESIZER(macp, 0x500607);
                        SET_IF_SYNTHESIZER(macp, 0xd8c010);
                        if((48 < ChannelNo) && (ChannelNo < 184)) {
                                SET_IF_SYNTHESIZER(macp, 0x3c2800);
                        } else {
                                SET_IF_SYNTHESIZER(macp, 0x3e2800);
                        }
                        SET_IF_SYNTHESIZER(macp, 0xbfffff);
                        SET_IF_SYNTHESIZER(macp, 0x700000);
                        SET_IF_SYNTHESIZER(macp, 0xf35d48);
                        //AcquireCtrOfPhyReg(Adapter);
                        //mFILL_WRITE_REGISTER( ZD_CR251, 0x2f); // shdnb(PLL_ON)=0
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x7f); // shdnb(PLL_ON)=1
                        mFILL_WRITE_REGISTER( ZD_CR128, 0x12);
                        mFILL_WRITE_REGISTER( ZD_CR129, 0x10);
                        mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                        mFILL_WRITE_REGISTER( ZD_CR38, 0x7f);
                        mFILL_WRITE_REGISTER( ZD_CR136, 0x5f);
                        //mFILL_WRITE_REGISTER( ZD_CR31, 0x58);

                        //NdisStallExecution(1000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d59);
                        //NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf15d5c);
                        //NdisStallExecution(10000);
                        SET_IF_SYNTHESIZER(macp, 0xf35d48);
                }

        } else {
                if( MAC_Mode != PURE_A_MODE ) {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f);  //PLL_OFF
                        //SET_IF_SYNTHESIZER(macp, 0x8cccd0);
                        if ( 1 || mOldMacMode != MAC_Mode ) {
                                SET_IF_SYNTHESIZER(macp, 0x4ff821);
                                SET_IF_SYNTHESIZER(macp, 0xc5fbfc);
                                SET_IF_SYNTHESIZER(macp, 0x21ebfe);
                                SET_IF_SYNTHESIZER(macp, 0xaad401);
                                SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                                SET_IF_SYNTHESIZER(macp, 0xe04073);
                                SET_IF_SYNTHESIZER(macp, 0x190d76);
                                SET_IF_SYNTHESIZER(macp, 0x9dd844);
                                SET_IF_SYNTHESIZER(macp, 0x500007);
                                SET_IF_SYNTHESIZER(macp, 0xd8c010);
                                SET_IF_SYNTHESIZER(macp, 0x3c9000);
                                SET_IF_SYNTHESIZER(macp, 0xf15d58);
                                //AcquireCtrOfPhyReg(Adapter);
                                //mFILL_WRITE_REGISTER( ZD_CR251, 0x2f);  //PLL_OFF
                                //mFILL_WRITE_REGISTER( ZD_CR251, 0x3f);  //PLL_ON
                                mFILL_WRITE_REGISTER( ZD_CR128, 0x14);
                                mFILL_WRITE_REGISTER( ZD_CR129, 0x12);
                                mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                                mFILL_WRITE_REGISTER( ZD_CR38, 0x38);
                                mFILL_WRITE_REGISTER( ZD_CR136, 0xdf);
                                mOldMacMode = MAC_Mode;
                        }
                        //Adapter->AL7230CCKSetFlag=0;
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2]);
                        SET_IF_SYNTHESIZER(macp, AL7230BTB[ChannelNo*2+1]);
                        SET_IF_SYNTHESIZER(macp, 0x3c9000);
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x7f);  //PLL_ON

                        //NdisStallExecution(10);
                        //SET_IF_SYNTHESIZER(macp, 0xf15d59);
                        //NdisStallExecution(100);
                        //SET_IF_SYNTHESIZER(macp, 0xf15d5c);
                        //NdisStallExecution(100);
                        //SET_IF_SYNTHESIZER(macp, 0xf15d58);
                } else {
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x2f); // shdnb(PLL_ON)=0

                        if ( 1 || mOldMacMode != MAC_Mode ) {
                                SET_IF_SYNTHESIZER(macp, 0x47f8a2);
                                SET_IF_SYNTHESIZER(macp, 0xc5fbfa);
                                //SET_IF_SYNTHESIZER(macp, 0x21ebf6);
                                SET_IF_SYNTHESIZER(macp, 0xaafca1);
                                SET_IF_SYNTHESIZER(macp, 0x6cf56a);
                                SET_IF_SYNTHESIZER(macp, 0xe04073);
                                SET_IF_SYNTHESIZER(macp, 0x190d36);
                                SET_IF_SYNTHESIZER(macp, 0x9dd844);
                                SET_IF_SYNTHESIZER(macp, 0x500607);
                                SET_IF_SYNTHESIZER(macp, 0xd8c010);
                                SET_IF_SYNTHESIZER(macp, 0xf35d48);
                                //AcquireCtrOfPhyReg(Adapter);
                                //mFILL_WRITE_REGISTER( ZD_CR251, 0x2f); // shdnb(PLL_ON)=0
                                //mFILL_WRITE_REGISTER( ZD_CR251, 0x3f); // shdnb(PLL_ON)=1
                                mFILL_WRITE_REGISTER( ZD_CR128, 0x12);
                                mFILL_WRITE_REGISTER( ZD_CR129, 0x10);
                                mFILL_WRITE_REGISTER( ZD_CR130, 0x10);
                                mFILL_WRITE_REGISTER( ZD_CR38, 0x7f);
                                mFILL_WRITE_REGISTER( ZD_CR136, 0x5f);
                                mOldMacMode = MAC_Mode;
                        }
                        if((48 < ChannelNo) && (ChannelNo < 184)) {
                                SET_IF_SYNTHESIZER(macp, 0x3c2800);
                        } else {
                                SET_IF_SYNTHESIZER(macp, 0x3e2800);
                        }
                        if((34 <= ChannelNo) && (ChannelNo <= 48)) {
                                ChannelNo_temp=(ChannelNo/2)-13;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                                //SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+2]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+3]);
                                // SET_IF_SYNTHESIZER(macp, 0x3c2800);
                        } else {
                                ChannelNo_temp=(ChannelNo/4)-1;
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+1]);
                                //SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+2]);
                                SET_IF_SYNTHESIZER(macp, AL7230BTB_a[ChannelNo_temp*4+3]);
                                //SET_IF_SYNTHESIZER(macp, 0x3c2800);
                        }
                        mFILL_WRITE_REGISTER( ZD_CR251, 0x7f);  //PLL_ON
                        //ZD1205_WRITE_REGISTER(Adapter, CR31, 0x58);

                        //NdisStallExecution(10);
                        //SET_IF_SYNTHESIZER(macp, 0xf15d59);
                        //NdisStallExecution(100);
                        //SET_IF_SYNTHESIZER(macp, 0xf15d5c);
                        //NdisStallExecution(100);
                        //SET_IF_SYNTHESIZER(macp, 0xf35d58);
                }

        }
        //ZD1205_WRITE_REGISTER(Adapter,CR138, 0x28);
        mFILL_WRITE_REGISTER( ZD_CR203, 0x06);
        ZD1211_WRITE_MULTI_REG(WriteAddr, WriteData, &WriteIndex);

        tmpValue = pObj->GetReg(reg, CtlReg1);
        tmpValue |= ~0x80;
        pObj->SetReg(reg, CtlReg1, tmpValue);

        LockPhyReg(pObj);
        pObj->SetReg(reg, ZD_CR240, 0x80);
        //if(macp->PHYNEWLayout)
        //	pObj->SetReg(reg, ZD_CR9, 0xe1);
        pObj->SetReg(reg, ZD_CR203, 0x06);


        pObj->CR203Flag = 2;
        pObj->CR31Flag = 2;

        if (pObj->HWFeature & BIT_8) //CR47 CCK gain patch
        {
                tmpValue = pObj->GetReg(reg, E2P_PHY_REG);
                pObj->SetReg(reg, ZD_CR47, (tmpValue & 0xff)); //This feature is OK to be overwritten with a lower value by other feature
        }
        if (pObj->HWFeature & BIT_21)  //6321 for FCC regulation, enabled per HWFeature 6M band edge bit (for AL2230, AL2230S)
        {
                if (ChannelNo == 1 || ChannelNo == 11)  //MARK_003, band edge, these may depend on PCB layout
                {
                        pObj->SetReg(reg, ZD_CR128, 0x12);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                } else //(ChannelNo 2 ~ 10, 12 ~ 14)
                {
                        pObj->SetReg(reg, ZD_CR128, 0x14);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                }
        }

        UnLockPhyReg(pObj);
        //	macp->Change_SetPoint = 2;
        //	macp->PHY_G_6M_BandEdge_Flag = 0;
        //	if(macp->PHY_36M_Setpoint_Flag != 0)
        //	{
        //for(i=0;i<14;i++)
        //	macp->SetPointOFDM[0][i] = macp->PHY_36M_G_Setpoint[i];
        //		for(i=0;i<16;i++)
        //			macp->a_Calibration_Data[2][i] = macp->PHY_36M_A_Calibration_Setpoint[i];
        //		for(i=0;i<32;i++)
        //			macp->a_Interpolation_Data[2][i] = macp->PHY_36M_A_Interpolation_Setpoint[i];
        //		macp->PHY_36M_Setpoint_Flag = 0;
        //	}



}
// end of AL7230B ZD1215
#endif

//------------------------------------------------------------------------------
// Procedure:    HW_Set_AL2232_Chips
//
// Description:
//
// Arguments:
//      pObj - ptr to Adapter object instance
//      ChannelNo
//      Initial Channel only
//
// Returns:     (none)
//
// Note:
//-------------------------------------------------------------------------------
void
HW_Set_AL2232_RF_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        void *reg = pObj->reg;
        // struct zd1205_private *macp = (struct zd1205_private *)g_dev->priv;
        U32 tmpValue;

        if (!InitChOnly) {
                LockPhyReg(pObj);
                pObj->SetReg(reg,ZD_CR9, 0xE0);           //5119
                pObj->SetReg(reg,ZD_CR15, 0x20);
                pObj->SetReg(reg,ZD_CR23, 0x40);
                pObj->SetReg(reg,ZD_CR24, 0x20);
                pObj->SetReg(reg,ZD_CR26, 0x11);
                pObj->SetReg(reg,ZD_CR28, 0x3e);
                pObj->SetReg(reg,ZD_CR29, 0x00);
                pObj->SetReg(reg,ZD_CR44, 0x33);
                pObj->SetReg(reg,ZD_CR106, 0x22);         // 2004/10/19  0x2a -> 0x22
                pObj->SetReg(reg,ZD_CR107, 0x1a);
                pObj->SetReg(reg,ZD_CR109, 0x9);
                pObj->SetReg(reg,ZD_CR110, 0x27);
                pObj->SetReg(reg,ZD_CR111, 0x2b);
                pObj->SetReg(reg,ZD_CR112, 0x2b);
                pObj->SetReg(reg,ZD_CR119, 0xa);

                pObj->SetReg(reg,ZD_CR10, 0x89);
                pObj->SetReg(reg,ZD_CR17, 0x2B);          //for newest(3rd cut) AL2230
                pObj->SetReg(reg,ZD_CR20, 0x12);          //4N25 -> Stone Request
                pObj->SetReg(reg,ZD_CR26, 0x93);
                pObj->SetReg(reg,ZD_CR34, 0x30);
                pObj->SetReg(reg,ZD_CR35, 0x3E);          //for newest(3rd cut) AL2230
                pObj->SetReg(reg,ZD_CR41, 0x24);
                pObj->SetReg(reg,ZD_CR44, 0x32);
                pObj->SetReg(reg,ZD_CR46, 0x99);          //for newest(3rd cut) AL2230
                pObj->SetReg(reg,ZD_CR47, 0x1e);
                pObj->SetReg(reg,ZD_CR79, 0x58);
                pObj->SetReg(reg,ZD_CR80, 0x30);
                pObj->SetReg(reg,ZD_CR81, 0x30);
                pObj->SetReg(reg,ZD_CR87, 0x0A);
                pObj->SetReg(reg,ZD_CR89, 0x04);
                pObj->SetReg(reg,ZD_CR90, 0x58);          //5113
                pObj->SetReg(reg,ZD_CR92, 0x0a);
                pObj->SetReg(reg,ZD_CR98, 0x8d);          //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR99, 0x28);
                pObj->SetReg(reg,ZD_CR100, 0x00);
                pObj->SetReg(reg,ZD_CR101, 0x13);
                pObj->SetReg(reg,ZD_CR102, 0x27);
                pObj->SetReg(reg,ZD_CR106, 0x22);         //for newest(3rd cut) AL2230
                // 2004/10/19  0x2a -> 0x22
                pObj->SetReg(reg,ZD_CR107, 0x2A);
                pObj->SetReg(reg,ZD_CR109, 0x13);         //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR110, 0x1F);         //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR111, 0x1F);         //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR112, 0x1f);
                pObj->SetReg(reg,ZD_CR113, 0x27);
                pObj->SetReg(reg,ZD_CR114, 0x27);
                pObj->SetReg(reg,ZD_CR115, 0x26);         //24->26 at 4901
                pObj->SetReg(reg,ZD_CR116, 0x24);         // 26->24 at 4901
                //rk     pObj->SetReg(reg,ZD_CR117, 0xfa);
                pObj->SetReg(reg,ZD_CR118, 0xf8);         //4O07, fa->f8
                pObj->SetReg(reg,ZD_CR119, 0x10);
                pObj->SetReg(reg,ZD_CR120, 0x4f);
                pObj->SetReg(reg,ZD_CR121, 0x0a);         //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR122, 0xFC);         // E0->FCh at 4901
                pObj->SetReg(reg,ZD_CR125, 0xaD);         //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR127, 0x03);         //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR137, 0x88);
                pObj->SetReg(reg,ZD_CR131, 0x00);         //5113
                pObj->SetReg(reg,ZD_CR148, 0x40);         //5113
                pObj->SetReg(reg,ZD_CR149, 0x40);         //4O07, 50->40
                pObj->SetReg(reg,ZD_CR150, 0x1A);         //5113, 0C->1A

                pObj->SetReg(reg,ZD_CR252, 0x34);
                pObj->SetReg(reg,ZD_CR253, 0x34);

                UnLockPhyReg(pObj);

                HW_Set_IF_Synthesizer(pObj, AL2232TB[ChannelNo*3]);
                HW_Set_IF_Synthesizer(pObj, AL2232TB[ChannelNo*3+1]);
                HW_Set_IF_Synthesizer(pObj, AL2232TB[ChannelNo*3+2]);
                HW_Set_IF_Synthesizer(pObj, 0x0b3331);
                HW_Set_IF_Synthesizer(pObj, 0x01b802);
                HW_Set_IF_Synthesizer(pObj, 0x00fff3);
                HW_Set_IF_Synthesizer(pObj, 0x0005a4);
                HW_Set_IF_Synthesizer(pObj, 0x044dc5);
                HW_Set_IF_Synthesizer(pObj, 0x0805b6);
                HW_Set_IF_Synthesizer(pObj, 0x0146C7);
                HW_Set_IF_Synthesizer(pObj, 0x000688);
                HW_Set_IF_Synthesizer(pObj, 0x0403b9);
                HW_Set_IF_Synthesizer(pObj, 0x00dbba);
                HW_Set_IF_Synthesizer(pObj, 0x00099b);
                HW_Set_IF_Synthesizer(pObj, 0x0bdffc);
                HW_Set_IF_Synthesizer(pObj, 0x00000d);
                HW_Set_IF_Synthesizer(pObj, 0x00580f);

                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR251, 0x2f);        // shdnb(PLL_ON)=0
                pObj->DelayUs(10);
                UnLockPhyReg(pObj);

                //HW_Set_IF_Synthesizer(pObj, 0x000d00f);
                HW_Set_IF_Synthesizer(pObj, 0x000d80f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00780f);
                pObj->DelayUs(100);
                //HW_Set_IF_Synthesizer(pObj, 0x00500f);
                HW_Set_IF_Synthesizer(pObj, 0x00580f);
        } else {
                HW_Set_IF_Synthesizer(pObj, AL2232TB[ChannelNo*3]);
                HW_Set_IF_Synthesizer(pObj, AL2232TB[ChannelNo*3+1]);
                HW_Set_IF_Synthesizer(pObj, AL2232TB[ChannelNo*3+2]);
        }

        LockPhyReg(pObj);
        if (1 )//|| macp->bContinueTx == 0)           // Do not modify CR203 during CAL mode
        {
                pObj->SetReg(reg, ZD_CR203, 0x06);
        }

        if (pObj->HWFeature & BIT_8) //CR47 CCK gain patch
        {
                tmpValue = pObj->GetReg(reg, E2P_PHY_REG);
                pObj->SetReg(reg, ZD_CR47, (tmpValue & 0xff)); //This feature is OK to be overwritten with a lower value by other feature
        }

#if 0 //6321 TO DO
        if (pObj->HWFeature & BIT_22)  //6321 High power band edge for FCC regulation, enabled per HWFeature
        {
                if (ChannelNo == 1 || ChannelNo == 11) //these may depend on PCB layout
                {
                        pObj->SetReg(reg, ZD_CR128, );
                        pObj->SetReg(reg, ZD_CR129, );
                        pObj->SetReg(reg, ZD_CR130, );
                        pObj->SetReg(reg, ZD_CR47, );
                } else //(ChannelNo 2 ~ 10, 12 ~ 14)
                {
                        pObj->SetReg(reg, ZD_CR128, );
                        pObj->SetReg(reg, ZD_CR129, );
                        pObj->SetReg(reg, ZD_CR130, );
                        pObj->SetReg(reg, ZD_CR47, );
                }
        }
#endif

        UnLockPhyReg(pObj);

        pObj->CR203Flag = 2;
        pObj->CR31Flag = 2;                           //cCR31InitialState;

}

#ifdef ZD1211
void
HW_Set_AL2230_RF_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{

        void *reg = pObj->reg;
        U32	tmpvalue;

        LockPhyReg(pObj);

        if (!InitChOnly) {
                //LockPhyReg(pObj);
#ifdef ZD1211B
                pObj->SetReg(reg, ZD_CR10, 0x89);
#endif

                pObj->SetReg(reg, ZD_CR15, 0x20);
#ifdef ZD1211B

                pObj->SetReg(reg, ZD_CR17, 0x2B);
#endif

                pObj->SetReg(reg, ZD_CR23, 0x40);
                pObj->SetReg(reg, ZD_CR24, 0x20);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR26, 0x11);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR26, 0x93);
#endif

                pObj->SetReg(reg, ZD_CR28, 0x3e);
                pObj->SetReg(reg, ZD_CR29, 0x00);
#ifdef ZD1211B

                pObj->SetReg(reg, ZD_CR33, 0x28);
#elif defined(ZD1211)

                pObj->SetReg(reg, ZD_CR44, 0x33);
                pObj->SetReg(reg, ZD_CR106, 0x2a);
                pObj->SetReg(reg, ZD_CR107, 0x1a);
                pObj->SetReg(reg, ZD_CR109, 0x9);
                pObj->SetReg(reg, ZD_CR110, 0x27);
                pObj->SetReg(reg, ZD_CR111, 0x2b);
                pObj->SetReg(reg, ZD_CR112, 0x2b);
                pObj->SetReg(reg, ZD_CR119, 0xa);
#endif

#if (defined(GCCK) && defined(OFDM))

                pObj->SetReg(reg, ZD_CR10, 0x89);
                pObj->SetReg(reg, ZD_CR17, 0x28); //for newest (3rd cut) AL2300
                pObj->SetReg(reg, ZD_CR26, 0x93);
                pObj->SetReg(reg, ZD_CR34, 0x30);
                pObj->SetReg(reg, ZD_CR35, 0x3E); //for newest (3rd cut) AL2300
                pObj->SetReg(reg, ZD_CR41, 0x24);

#ifdef HOST_IF_USB

                pObj->SetReg(reg, ZD_CR44, 0x32);
#else

                pObj->SetReg(reg, ZD_CR44, 0x32);
#endif

                pObj->SetReg(reg, ZD_CR46, 0x96); //for newest (3rd cut) AL2300
                pObj->SetReg(reg, ZD_CR47, 0x1e);
#ifdef ZD1211B

                pObj->SetReg(reg,ZD_CR48, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR49, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR51, 0x01);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR52, 0x80);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR53, 0x7e);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR65, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR66, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR67, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR68, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR69, 0x28);		//ZD1211B 05.06.10
#endif

                pObj->SetReg(reg, ZD_CR79, 0x58);
                pObj->SetReg(reg, ZD_CR80, 0x30);
                pObj->SetReg(reg, ZD_CR81, 0x30);
                pObj->SetReg(reg, ZD_CR87, 0x0A);
                pObj->SetReg(reg, ZD_CR89, 0x04);


                pObj->SetReg(reg, ZD_CR92, 0x0a);
                pObj->SetReg(reg, ZD_CR99, 0x28);
                pObj->SetReg(reg, ZD_CR100, 0x00);
                pObj->SetReg(reg, ZD_CR101, 0x13);
                pObj->SetReg(reg, ZD_CR102, 0x27);
                pObj->SetReg(reg, ZD_CR106, 0x24);
                pObj->SetReg(reg, ZD_CR107, 0x2A);
                pObj->SetReg(reg, ZD_CR109, 0x09);
                pObj->SetReg(reg, ZD_CR110, 0x13);
                pObj->SetReg(reg, ZD_CR111, 0x1f);
                pObj->SetReg(reg, ZD_CR112, 0x1f);
                pObj->SetReg(reg, ZD_CR113, 0x27);
                pObj->SetReg(reg, ZD_CR114, 0x27);
                pObj->SetReg(reg, ZD_CR115, 0x24); //for newest (3rd cut) AL2300
                pObj->SetReg(reg, ZD_CR116, 0x24);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR117, 0xf4);
                pObj->SetReg(reg, ZD_CR118, 0xfc);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR117, 0xfa);
                pObj->SetReg(reg, ZD_CR118, 0xfa);
#endif

                pObj->SetReg(reg, ZD_CR119, 0x10);
                pObj->SetReg(reg, ZD_CR120, 0x4f);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR121, 0x77);
                pObj->SetReg(reg, ZD_CR122, 0xe0);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR121, 0x6c);
                pObj->SetReg(reg, ZD_CR122, 0xfc);
#endif

                pObj->SetReg(reg, ZD_CR137, 0x88);
#ifndef HOST_IF_USB

                pObj->SetReg(reg, ZD_CR150, 0x0D);
#endif
#elif (defined(ECCK_60_5))

                pObj->SetReg(reg, ZD_CR47, 0x1E);
                pObj->SetReg(reg, ZD_CR106, 0x04);
                pObj->SetReg(reg, ZD_CR107, 0x00);
                pObj->SetReg(reg, ZD_CR14, 0x80);
                pObj->SetReg(reg, ZD_CR10, 0x89);
                pObj->SetReg(reg, ZD_CR11, 0x00);
                pObj->SetReg(reg, ZD_CR161, 0x28);
                pObj->SetReg(reg, ZD_CR162, 0x26);

                pObj->SetReg(reg, ZD_CR24, 0x0e);
                pObj->SetReg(reg, ZD_CR41, 0x24);
                pObj->SetReg(reg, ZD_CR159, 0x93);
                pObj->SetReg(reg, ZD_CR160, 0xfc);
                pObj->SetReg(reg, ZD_CR161, 0x20);
                pObj->SetReg(reg, ZD_CR162, 0x26);
#endif

                pObj->SetReg(reg, ZD_CR252, 0xff);
                pObj->SetReg(reg, ZD_CR253, 0xff);

                //UnLockPhyReg(pObj);

                if (pObj->rfMode == AL2230S_RF) {
                        pObj->SetReg(reg, ZD_CR47 , 0x1E); //MARK_002
                        pObj->SetReg(reg, ZD_CR106, 0x22);
                        pObj->SetReg(reg, ZD_CR107, 0x2A); //MARK_002
                        pObj->SetReg(reg, ZD_CR109, 0x13); //MARK_002
                        pObj->SetReg(reg, ZD_CR118, 0xF8); //MARK_002
                        pObj->SetReg(reg, ZD_CR119, 0x12);
                        pObj->SetReg(reg, ZD_CR122, 0xE0);
                        pObj->SetReg(reg, ZD_CR128, 0x10); //MARK_001 from 0xe->0x10
                        pObj->SetReg(reg, ZD_CR129, 0x0E); //MARK_001 from 0xd->0x0e
                        pObj->SetReg(reg, ZD_CR130, 0x10); //MARK_001 from 0xb->0x0d
                }

                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+1]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+2]);
                HW_Set_IF_Synthesizer(pObj, 0x0b3331);
                HW_Set_IF_Synthesizer(pObj, 0x03b812);
                HW_Set_IF_Synthesizer(pObj, 0x00fff3);
                if (pObj->rfMode == AL2230S_RF)
                        HW_Set_IF_Synthesizer(pObj, 0x000824);  //improve band edge for AL2230S
                else
                        HW_Set_IF_Synthesizer(pObj, 0x0005a4);

                HW_Set_IF_Synthesizer(pObj, 0x000da4);
                HW_Set_IF_Synthesizer(pObj, 0x04edc5);
                HW_Set_IF_Synthesizer(pObj, 0x0805b6);
                HW_Set_IF_Synthesizer(pObj, 0x011687);
                HW_Set_IF_Synthesizer(pObj, 0x000688);
                HW_Set_IF_Synthesizer(pObj, 0x0403b9);   //External control TX power (CR31)
                HW_Set_IF_Synthesizer(pObj, 0x00dbba);
                HW_Set_IF_Synthesizer(pObj, 0x00099b);
                HW_Set_IF_Synthesizer(pObj, 0x0bdffc);
                HW_Set_IF_Synthesizer(pObj, 0x00000d);
                HW_Set_IF_Synthesizer(pObj, 0x00500f);

                //LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR251, 0x2f); // shdnb(PLL_ON)=0
                pObj->SetReg(reg, ZD_CR251, 0x3f); // shdnb(PLL_ON)=1
                pObj->DelayUs(10);
                //UnLockPhyReg(pObj);
                HW_Set_IF_Synthesizer(pObj, 0x000d00f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x0004c0f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00540f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00700f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00500f);

        } else {
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+1]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+2]);
        }

        //LockPhyReg(pObj);
        pObj->SetReg(reg, ZD_CR138, 0x28);
        pObj->SetReg(reg, ZD_CR203, 0x06);
        //UnLockPhyReg(pObj);
        pObj->CR203Flag = 2;
        pObj->CR31Flag = 2;
        if (pObj->HWFeature & BIT_8) //CR47 CCK gain patch
        {
                tmpvalue = pObj->GetReg(reg, E2P_PHY_REG);
                pObj->SetReg(reg, ZD_CR47, (tmpvalue & 0xff)); //This feature is OK to be overwritten with a lower value by other feature
        }

        if (pObj->HWFeature & BIT_21)  //6321 for FCC regulation, enabled per HWFeature 6M band edge bit (for AL2230, AL2230S)
        {
                if (ChannelNo == 1 || ChannelNo == 11)  //MARK_003, band edge, these may depend on PCB layout
                {
                        pObj->SetReg(reg, ZD_CR128, 0x12);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                } else //(ChannelNo 2 ~ 10, 12 ~ 14)
                {
                        pObj->SetReg(reg, ZD_CR128, 0x14);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                }
        }


#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )
        pObj->SetReg(reg, ZD_PE1_PE2, 3);
#endif

        UnLockPhyReg(pObj);
}
#elif defined(ZD1211B)
void
HW_Set_AL2230_RF_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        //ZDMacLog("HW_Set_AL2230_RF_Chips\n");

        void *reg = pObj->reg;
        U32 tmpValue;

        LockPhyReg(pObj);


        //++
        //1211b----------------

        if (!InitChOnly) {
                //LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR10, 0x89);
                pObj->SetReg(reg, ZD_CR15, 0x20);
                pObj->SetReg(reg, ZD_CR17, 0x2B);       //for newest(3rd cut) AL2230
                pObj->SetReg(reg, ZD_CR23, 0x40);
                pObj->SetReg(reg, ZD_CR24, 0x20);
                pObj->SetReg(reg, ZD_CR26, 0x93);
                pObj->SetReg(reg, ZD_CR28, 0x3e);
                pObj->SetReg(reg, ZD_CR29, 0x00);
                pObj->SetReg(reg, ZD_CR33, 0x28);   //5621
                pObj->SetReg(reg, ZD_CR34, 0x30);
                pObj->SetReg(reg, ZD_CR35, 0x3e);  //for newest(3rd cut) AL2230
                pObj->SetReg(reg, ZD_CR41, 0x24);
                pObj->SetReg(reg, ZD_CR44, 0x32);
                pObj->SetReg(reg, ZD_CR46, 0x99);  //for newest(3rd cut) AL2230
                pObj->SetReg(reg, ZD_CR47, 0x1e);
                pObj->SetReg(reg, ZD_CR48, 0x00);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR49, 0x00);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR51, 0x01);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR52, 0x80);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR53, 0x7e);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR65, 0x00);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR66, 0x00);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR67, 0x00);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR68, 0x00);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR69, 0x28);       //ZD1211B 05.06.10
                pObj->SetReg(reg, ZD_CR79, 0x58);
                pObj->SetReg(reg, ZD_CR80, 0x30);
                pObj->SetReg(reg, ZD_CR81, 0x30);
                pObj->SetReg(reg, ZD_CR87, 0x0a);
                pObj->SetReg(reg, ZD_CR89, 0x04);
                pObj->SetReg(reg, ZD_CR91, 0x00);   //5621
                pObj->SetReg(reg, ZD_CR92, 0x0a);
                pObj->SetReg(reg, ZD_CR98, 0x8d);  //4804, for 1212 new algorithm
                pObj->SetReg(reg, ZD_CR99, 0x00);  //5621
                pObj->SetReg(reg, ZD_CR101, 0x13);
                pObj->SetReg(reg, ZD_CR102, 0x27);
                pObj->SetReg(reg, ZD_CR106, 0x24);  //for newest(3rd cut) AL2230
                pObj->SetReg(reg, ZD_CR107, 0x2a);
                pObj->SetReg(reg, ZD_CR109, 0x13);  //4804, for 1212 new algorithm
                pObj->SetReg(reg, ZD_CR110, 0x1f);  //4804, for 1212 new algorithm
                pObj->SetReg(reg, ZD_CR111, 0x1f);
                pObj->SetReg(reg, ZD_CR112, 0x1f);
                pObj->SetReg(reg, ZD_CR113, 0x27);
                pObj->SetReg(reg, ZD_CR114, 0x27);
                pObj->SetReg(reg, ZD_CR115, 0x26); //24->26 at 4902 for newest(3rd cut) AL2230
                pObj->SetReg(reg, ZD_CR116, 0x24);
                pObj->SetReg(reg, ZD_CR117, 0xfa); // for 1211b
                pObj->SetReg(reg, ZD_CR118, 0xfa); // for 1211b
                pObj->SetReg(reg, ZD_CR119, 0x10);
                pObj->SetReg(reg, ZD_CR120, 0x4f);
                pObj->SetReg(reg, ZD_CR121, 0x6c); // for 1211b
                pObj->SetReg(reg, ZD_CR122, 0xfc); // E0->FC at 4902
                pObj->SetReg(reg, ZD_CR123, 0x57); //5623
                pObj->SetReg(reg, ZD_CR125, 0xad); //4804, for 1212 new algorithm
                pObj->SetReg(reg, ZD_CR126, 0x6c); //5614
                pObj->SetReg(reg, ZD_CR127, 0x03); //4804, for 1212 new algorithm
                pObj->SetReg(reg, ZD_CR137, 0x50); //5614
                pObj->SetReg(reg, ZD_CR138, 0xa8);
                pObj->SetReg(reg, ZD_CR144, 0xac); //5621
                pObj->SetReg(reg, ZD_CR150, 0x0d);

                //--
                pObj->SetReg(reg, ZD_CR252, 0x00);
                pObj->SetReg(reg, ZD_CR253, 0x00);

                //UnLockPhyReg(pObj);
                if (pObj->rfMode == AL2230S_RF) {
                        pObj->SetReg(reg, ZD_CR47 , 0x1E); //MARK_002
                        pObj->SetReg(reg, ZD_CR106, 0x22);
                        pObj->SetReg(reg, ZD_CR107, 0x2A); //MARK_002
                        pObj->SetReg(reg, ZD_CR109, 0x13); //MARK_002
                        pObj->SetReg(reg, ZD_CR118, 0xF8); //MARK_002
                        pObj->SetReg(reg, ZD_CR119, 0x12);
                        pObj->SetReg(reg, ZD_CR122, 0xE0);
                        pObj->SetReg(reg, ZD_CR128, 0x10); //MARK_001 from 0xe->0x10
                        pObj->SetReg(reg, ZD_CR129, 0x0E); //MARK_001 from 0xd->0x0e
                        pObj->SetReg(reg, ZD_CR130, 0x10); //MARK_001 from 0xb->0x0d
                }


                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+1]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+2]);
                HW_Set_IF_Synthesizer(pObj, 0x0b3331);
                HW_Set_IF_Synthesizer(pObj, 0x03b812);
                HW_Set_IF_Synthesizer(pObj, 0x00fff3);
                if (pObj->rfMode == AL2230S_RF)
                        HW_Set_IF_Synthesizer(pObj, 0x000824);  //improve band edge for AL2230S
                else
                        HW_Set_IF_Synthesizer(pObj, 0x0005a4);

                HW_Set_IF_Synthesizer(pObj, 0x044dc5);
                HW_Set_IF_Synthesizer(pObj, 0x0805b6);
                HW_Set_IF_Synthesizer(pObj, 0x0146c7);
                HW_Set_IF_Synthesizer(pObj, 0x000688);
                HW_Set_IF_Synthesizer(pObj, 0x0403b9);   //External control TX power (CR31)
                HW_Set_IF_Synthesizer(pObj, 0x00dbba);
                HW_Set_IF_Synthesizer(pObj, 0x00099b);
                HW_Set_IF_Synthesizer(pObj, 0x0bdffc);
                HW_Set_IF_Synthesizer(pObj, 0x00000d);
                HW_Set_IF_Synthesizer(pObj, 0x00580f);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                pObj->SetReg(reg, 0x464, 0x3);
                pObj->DelayUs(10);
                HW_Set_IF_Synthesizer(pObj, 0x000880f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00080f);
                pObj->DelayUs(100);

                pObj->SetReg(reg, 0x464, 0x00);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
                //LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR251, 0x7f);
                pObj->DelayUs(10);
                //UnLockPhyReg(pObj);
                HW_Set_IF_Synthesizer(pObj, 0x000d80f);
                pObj->DelayUs(100);
                //HW_Set_IF_Synthesizer(pObj, 0x0004c0f);
                //pObj->DelayUs(100);
                //HW_Set_IF_Synthesizer(pObj, 0x00540f);
                //pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00780f);
                pObj->DelayUs(100);
                HW_Set_IF_Synthesizer(pObj, 0x00580f);


        } else {
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+1]);
                HW_Set_IF_Synthesizer(pObj, AL2230TB[ChannelNo*3+2]);
        }

        //LockPhyReg(pObj);
        pObj->SetReg(reg, ZD_CR138, 0x28);
        pObj->SetReg(reg, ZD_CR203, 0x06);
        //UnLockPhyReg(pObj);
        pObj->CR203Flag = 2;
        pObj->CR31Flag = 2;

        if (pObj->HWFeature & BIT_8) //CR47 CCK gain patch
        {
                tmpValue = pObj->GetReg(reg, E2P_PHY_REG);
                pObj->SetReg(reg, ZD_CR47, (tmpValue & 0xff)); //This feature is OK to be overwritten with a lower value by other feature
        }

        if (pObj->HWFeature & BIT_21)  //6321 for FCC regulation, enabled per HWFeature 6M band edge bit (for AL2230, AL2230S)
        {
                LockPhyReg(pObj);
                if (ChannelNo == 1 || ChannelNo == 11)  //MARK_003, band edge, these may depend on PCB layout
                {
                        pObj->SetReg(reg, ZD_CR128, 0x12);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                } else //(ChannelNo 2 ~ 10, 12 ~ 14)
                {
                        pObj->SetReg(reg, ZD_CR128, 0x14);
                        pObj->SetReg(reg, ZD_CR129, 0x12);
                        pObj->SetReg(reg, ZD_CR130, 0x10);
                        pObj->SetReg(reg, ZD_CR47, 0x1E);
                }
                UnLockPhyReg(pObj);
        }

#if !( (defined(OFDM) && defined(GCCK)) || defined(ECCK_60_5) )
        pObj->SetReg(reg, ZD_PE1_PE2, 3);
#endif

        UnLockPhyReg(pObj);
}

#endif



//2-step LNA for RF2959
void
HW_Set_RFMD_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly)
{
        void *reg = pObj->reg;

        LockPhyReg(pObj);

        // Get Phy-Config permission
        if (!InitChOnly ) {
                //LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR2, 0x1E);
                pObj->SetReg(reg, ZD_CR9, 0x20);
                //pObj->SetReg(reg, ZD_CR10, 0xB1);
                pObj->SetReg(reg, ZD_CR10, 0x89);
                pObj->SetReg(reg, ZD_CR11, 0x00);
                pObj->SetReg(reg, ZD_CR15, 0xD0);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR17, 0x68);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR17, 0x2b);
#endif

                pObj->SetReg(reg, ZD_CR19, 0x4a);
                pObj->SetReg(reg, ZD_CR20, 0x0c);
                pObj->SetReg(reg, ZD_CR21, 0x0E);
                pObj->SetReg(reg, ZD_CR23, 0x48);

                if (pObj->bIsNormalSize)
                        pObj->SetReg(reg, ZD_CR24, 0x14);//cca threshold
                else
                        pObj->SetReg(reg, ZD_CR24, 0x20);//cca threshold

                pObj->SetReg(reg, ZD_CR26, 0x90);
                pObj->SetReg(reg, ZD_CR27, 0x30);
                pObj->SetReg(reg, ZD_CR29, 0x20);
                pObj->SetReg(reg, ZD_CR31, 0xb2);
                //pObj->SetReg(reg, ZD_CR31, 0xaa);
                pObj->SetReg(reg, ZD_CR32, 0x43);
                pObj->SetReg(reg, ZD_CR33, 0x28);
                pObj->SetReg(reg, ZD_CR38, 0x30);
                pObj->SetReg(reg, ZD_CR34, 0x0f);
                pObj->SetReg(reg, ZD_CR35, 0xF0);
                pObj->SetReg(reg, ZD_CR41, 0x2a);
                pObj->SetReg(reg, ZD_CR46, 0x7F);
                pObj->SetReg(reg, ZD_CR47, 0x1E);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR51, 0xc5);
                pObj->SetReg(reg, ZD_CR52, 0xc5);
                pObj->SetReg(reg, ZD_CR53, 0xc5);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR51, 0x01);
                pObj->SetReg(reg, ZD_CR52, 0x80);
                pObj->SetReg(reg, ZD_CR53, 0x7e);

                pObj->SetReg(reg,ZD_CR48, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR49, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR65, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR66, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR67, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR68, 0x00);		//ZD1211B 05.06.10
                pObj->SetReg(reg,ZD_CR69, 0x28);		//ZD1211B 05.06.10

#endif

                pObj->SetReg(reg, ZD_CR79, 0x58);
                pObj->SetReg(reg, ZD_CR80, 0x30);
                pObj->SetReg(reg, ZD_CR81, 0x30);
                pObj->SetReg(reg, ZD_CR82, 0x00);
                pObj->SetReg(reg, ZD_CR83, 0x24);
                pObj->SetReg(reg, ZD_CR84, 0x04);
                pObj->SetReg(reg, ZD_CR85, 0x00);
                pObj->SetReg(reg, ZD_CR86, 0x10);
                pObj->SetReg(reg, ZD_CR87, 0x2A);
                pObj->SetReg(reg, ZD_CR88, 0x10);
                pObj->SetReg(reg, ZD_CR89, 0x24);
                pObj->SetReg(reg, ZD_CR90, 0x18);
                //pObj->SetReg(reg, ZD_CR91, 0x18);
                pObj->SetReg(reg, ZD_CR91, 0x00); // to solve continuous CTS frames problem
                pObj->SetReg(reg, ZD_CR92, 0x0a);
                pObj->SetReg(reg, ZD_CR93, 0x00);
                pObj->SetReg(reg, ZD_CR94, 0x01);
                pObj->SetReg(reg, ZD_CR95, 0x00);
                pObj->SetReg(reg, ZD_CR96, 0x40);

                pObj->SetReg(reg, ZD_CR97, 0x37);
#ifdef HOST_IF_USB
	#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR98, 0x05);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR98, 0x8d);
#endif
#else

                pObj->SetReg(reg, ZD_CR98, 0x0D);
                pObj->SetReg(reg, ZD_CR121, 0x06);
                pObj->SetReg(reg, ZD_CR125, 0xAA);
#endif

                pObj->SetReg(reg, ZD_CR99, 0x28);
                pObj->SetReg(reg, ZD_CR100, 0x00);
                pObj->SetReg(reg, ZD_CR101, 0x13);
                pObj->SetReg(reg, ZD_CR102, 0x27);
                pObj->SetReg(reg, ZD_CR103, 0x27);
                pObj->SetReg(reg, ZD_CR104, 0x18);
                pObj->SetReg(reg, ZD_CR105, 0x12);

                if (pObj->bIsNormalSize)

                        pObj->SetReg(reg, ZD_CR106, 0x1a);
                else
                        pObj->SetReg(reg, ZD_CR106, 0x22);

                pObj->SetReg(reg, ZD_CR107, 0x24);
                pObj->SetReg(reg, ZD_CR108, 0x0a);
                pObj->SetReg(reg, ZD_CR109, 0x13);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR110, 0x2F);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR110, 0x1F);
#endif

                pObj->SetReg(reg, ZD_CR111, 0x27);
                pObj->SetReg(reg, ZD_CR112, 0x27);
                pObj->SetReg(reg, ZD_CR113, 0x27);
                pObj->SetReg(reg, ZD_CR114, 0x27);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR115, 0x40);
                pObj->SetReg(reg, ZD_CR116, 0x40);
                pObj->SetReg(reg, ZD_CR117, 0xF0);
                pObj->SetReg(reg, ZD_CR118, 0xF0);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR115, 0x26);
                pObj->SetReg(reg, ZD_CR116, 0x40);
                pObj->SetReg(reg, ZD_CR117, 0xFA);
                pObj->SetReg(reg, ZD_CR118, 0xFA);
                pObj->SetReg(reg, ZD_CR121, 0x6C);
#endif

                pObj->SetReg(reg, ZD_CR119, 0x16);
                //pObj->SetReg(reg, ZD_CR122, 0xfe);
                if (pObj->bContinueTx)
                        pObj->SetReg(reg, ZD_CR122, 0xff);
                else
                        pObj->SetReg(reg, ZD_CR122, 0x00);
                pObj->CR122Flag = 2;
#ifdef ZD1211B

                pObj->SetReg(reg,ZD_CR125, 0xad);  //4804, for 1212 new algorithm
                pObj->SetReg(reg,ZD_CR126, 0x6c);  //5614

#endif

                pObj->SetReg(reg, ZD_CR127, 0x03);
                pObj->SetReg(reg, ZD_CR131, 0x08);
                pObj->SetReg(reg, ZD_CR138, 0x28);
                pObj->SetReg(reg, ZD_CR148, 0x44);
#ifdef ZD1211

                pObj->SetReg(reg, ZD_CR150, 0x10);
#elif defined(ZD1211B)

                pObj->SetReg(reg, ZD_CR150, 0x14);
#endif

                pObj->SetReg(reg, ZD_CR169, 0xBB);
                pObj->SetReg(reg, ZD_CR170, 0xBB);
                //pObj->SetReg(reg, ZD_CR38, 0x30);
                //UnLockPhyReg(pObj);

                HW_Set_IF_Synthesizer(pObj, 0x000007);  //REG0(CFG1)
                HW_Set_IF_Synthesizer(pObj, 0x07dd43);  //REG1(IFPLL1)
                HW_Set_IF_Synthesizer(pObj, 0x080959);  //REG2(IFPLL2)
                HW_Set_IF_Synthesizer(pObj, 0x0e6666);
                HW_Set_IF_Synthesizer(pObj, 0x116a57);  //REG4
                HW_Set_IF_Synthesizer(pObj, 0x17dd43);  //REG5
                HW_Set_IF_Synthesizer(pObj, 0x1819f9);  //REG6
                HW_Set_IF_Synthesizer(pObj, 0x1e6666);
                HW_Set_IF_Synthesizer(pObj, 0x214554);
                HW_Set_IF_Synthesizer(pObj, 0x25e7fa);
                HW_Set_IF_Synthesizer(pObj, 0x27fffa);
                //HW_Set_IF_Synthesizer(pObj, 0x294128);  //Register control TX power
                // set in Set_RF_Channel( )
                //HW_Set_IF_Synthesizer(pObj, 0x28252c);    //External control TX power (CR31_CCK, CR51_6-36M, CR52_48M, CR53_54M
                HW_Set_IF_Synthesizer(pObj, 0x2c0000);
                HW_Set_IF_Synthesizer(pObj, 0x300000);


                HW_Set_IF_Synthesizer(pObj, 0x340000);  //REG13(0xD)
                HW_Set_IF_Synthesizer(pObj, 0x381e0f);  //REG14(0xE)
                HW_Set_IF_Synthesizer(pObj, 0x6c180f);  //REG27(0x11)
        } else {
                //LockPhyReg(pObj);
                if (pObj->bContinueTx)
                        pObj->SetReg(reg, ZD_CR122, 0xff);
                else
                        pObj->SetReg(reg, ZD_CR122, 0x00);
                //UnLockPhyReg(pObj);

                pObj->CR122Flag = 2;
                pObj->CR31Flag = 2;

                HW_Set_IF_Synthesizer(pObj, RFMD2958t[ChannelNo*2]);
                HW_Set_IF_Synthesizer(pObj, RFMD2958t[ChannelNo*2+1]);

        }


        UnLockPhyReg(pObj);

        return;
}

void HW_EnableBeacon(zd_80211Obj_t *pObj, U16 BeaconInterval, U16 DtimPeriod, U8 BssType)
{
        U32 tmpValue;
        U32 Mode = 0;
        U16 Dtim = 0;

        void *reg = pObj->reg;

        if (BssType == INDEPENDENT_BSS) {
                Mode = IBSS_MODE;
                printk(KERN_ERR "Mode: IBSS_MODE\n");
        } else if (BssType == AP_BSS) {
                Mode = AP_MODE;
                Dtim = DtimPeriod;
                printk(KERN_ERR "Mode: AP_BSS\n");
        }

        tmpValue = BeaconInterval | Mode | (Dtim<<16) ;
        pObj->SetReg(reg, ZD_BCNInterval, tmpValue);
}


void HW_SwitchChannel(zd_80211Obj_t *pObj, U16 channel, U8 InitChOnly, const U8 MAC_Mode)
{
        void *reg = pObj->reg;


        pObj->SetReg(reg, ZD_CONFIGPhilips, 0x0);

        //FPRINT_V("rfMode", pObj->rfMode);

        switch(pObj->rfMode) {
        default:
                FPRINT_V("Invalid RF module parameter", pObj->rfMode);

                break;

        case MAXIM_NEW_RF:
                FPRINT_V("MAXIM_NEW_RF Channel", channel);
                pObj->S_bit_cnt = 18;
                HW_Set_Maxim_New_Chips(pObj, channel, 0);

#ifdef HOST_IF_USB

                HW_UpdateIntegrationValue(pObj, channel, MAC_Mode);
#endif

                break;

        case GCT_RF:
                //	FPRINT_V("GCT Channel", channel);
                pObj->S_bit_cnt = 21;

                pObj->AcquireDoNotSleep();
                if (!pObj->bDeviceInSleep)
                        HW_Set_GCT_Chips(pObj, channel, InitChOnly);
                pObj->ReleaseDoNotSleep();
                //HW_UpdateIntegrationValue(pObj, channel);
                break;

        case AL2230_RF:
        case AL2230S_RF:
                //FPRINT_V("AL2210MPVB_RF Channel", channel);
                pObj->S_bit_cnt = 24;
                HW_Set_AL2230_RF_Chips(pObj, channel, InitChOnly);
                HW_UpdateIntegrationValue(pObj, channel, MAC_Mode);
                break;
        case AL7230B_RF: //For 802.11a/b/g
                FPRINT_V("AL7230B_RF",channel);
                pObj->S_bit_cnt = 24;
                //			printk("Before AL7230BRF:C,%d,M,%d\n\n",channel,MAC_Mode);
                HW_Set_AL7230B_RF_Chips(pObj, channel, InitChOnly,MAC_Mode);
                break;
        case AL2210_RF:
                //FPRINT_V("AL2210_RF Channel", channel);
                pObj->S_bit_cnt = 24;
                HW_Set_AL2210_Chips(pObj, channel, 0);
                break;

        case RALINK_RF:
                FPRINT_V("Ralink Channel", channel);
                break;

        case INTERSIL_RF:
                FPRINT_V("Intersil Channel", channel);
                break;

        case RFMD_RF:
                FPRINT_V("RFMD Channel", channel);
                pObj->S_bit_cnt = 24;
                HW_Set_RFMD_Chips(pObj, channel, InitChOnly);


                if (!InitChOnly)
                        HW_UpdateIntegrationValue(pObj, channel, MAC_Mode);
                break;

        case MAXIM_NEW_RF2:
                FPRINT_V("MAXIM_NEW_RF2 Channel", channel);

                pObj->S_bit_cnt = 18;
                HW_Set_Maxim_New_Chips2(pObj, channel, 0);
                break;

        case PHILIPS_RF:
                FPRINT_V("Philips SA2400 Channel", channel);
                break;
        }

        HW_OverWritePhyRegFromE2P(pObj);

        return;
}



void HW_SetRfChannel(zd_80211Obj_t *pObj, U16 channel, U8 InitChOnly, const U8 MAC_Mode)
{
        void *reg = pObj->reg;
        //FPRINT_V("HW_SetRfChannel", channel);

        // Check if this ChannelNo allowed?
        u32 i;

        if (!((1 << (channel-1)) & pObj->AllowedChannel)) {
                // Not an allowed channel, we use default channel.
                //printk("Channel = %d Not an allowed channel\n", channel);
                //printk("Set default channel = %d\n", (pObj->AllowedChannel >> 16));
                //channel = (pObj->AllowedChannel >> 16);
                if(PURE_A_MODE != MAC_Mode) {
                        //printk("You use a non-allowed channel in HW_setRfChannel(%d)\n",channel);
                        return;
                }
        }

        //Check if channel is valid 2.4G Band
        if(MAC_Mode != PURE_A_MODE) {
                if ((channel > 14 ) || (channel < 1)) { // for the wrong content of the EEPROM
                        printk(KERN_DEBUG "Error Channel Number in HW_SetRfChannel(11b/g)\n");
                        return;
                }
        } else if(MAC_Mode == PURE_A_MODE) {
                //Check is the A Band Channel is valid.
                for(i=0;i<dot11A_Channel_Amount;i++)
                        if(dot11A_Channel[i] == channel)
                                break;
                if(i>=dot11A_Channel_Amount) {
                        printk(KERN_DEBUG "Error Channel Number in HW_SetRfChannel(11a,CH=%d)\n",channel);
                        return;
                }
        }
        if(PURE_A_MODE == MAC_Mode) {
                pObj->SetReg(pObj->reg, ZD_IFS_Value, 0x1147c00a);
                pObj->SetReg(pObj->reg, ZD_RTS_CTS_Rate, 0x01090109);
        } else {
                pObj->SetReg(pObj->reg, ZD_IFS_Value, 0xa47c032);
                pObj->SetReg(pObj->reg, ZD_RTS_CTS_Rate, 0x30000);
        }

        pObj->Channel = channel;
        HW_SwitchChannel(pObj, channel, InitChOnly,MAC_Mode);
        LastSetChannel = channel;
        LastMacMode = MAC_Mode;

        //The UpdateIntegrationValue call should be called immediately
        //after HW_SetRfChannel
        HW_UpdateIntegrationValue(pObj, channel,MAC_Mode);

        // When channnel == 14 , enable Japan spectrum mask
        if (pObj->RegionCode == 0x40) { //Japan
                if (channel == 14) {
                        HW_Set_FilterBand(pObj, pObj->RegionCode);  // for Japan, RegionCode = 0x40
                        if (pObj->rfMode == RFMD_RF) {
                                LockPhyReg(pObj);
                                pObj->SetReg(reg, ZD_CR47, 0x1E);
                                //UnLockPhyReg(pObj);

                                HW_Set_IF_Synthesizer(pObj, 0x28252d);    //External control TX power (CR31_CCK, CR51_6-36M, CR52_48M, CR53_54M
                                UnLockPhyReg(pObj);
                        }

                } else {

                        // For other channels, use default filter.
                        HW_Set_FilterBand(pObj, 0);

                        if (pObj->rfMode == RFMD_RF) {
                                // CR47 has been restored in Init_RF_Chips( ), its value is from EEPROM
                                HW_Set_IF_Synthesizer(pObj, 0x28252d);    //External control TX power (CR31_CCK, CR51_6-36M, CR52_48M, CR53_54M
                        }
                }
        }

        pObj->DelayUs(100);
}

void HW_SetBeaconFIFO(zd_80211Obj_t *pObj, U8 *pBeacon, U16 index)
{
        U32 tmpValue, BCNPlcp;
        U16 j;
        void *reg = pObj->reg;
        U32 count = 0;

        pObj->SetReg(reg, ZD_BCN_FIFO_Semaphore, 0x0);
        tmpValue = pObj->GetReg(reg, ZD_BCN_FIFO_Semaphore);

        while (tmpValue & BIT_1) {
                pObj->DelayUs(1000);
                tmpValue = pObj->GetReg(reg, ZD_BCN_FIFO_Semaphore);

                if ((++count % 100) == 0)
                        printk(KERN_ERR "BCN\n");
        }

        /* Write (Beacon_Len -1) to Beacon-FIFO */
        pObj->SetReg(reg, ZD_BCNFIFO, (index - 1));
#ifdef ZD1211B

        pObj->SetReg(reg,ZD_BCNLENGTH, (index - 1));
#endif

        for (j=0; j<index; j++) {
                pObj->SetReg(reg, ZD_BCNFIFO, pBeacon[j]);
        }
        pObj->SetReg(reg, ZD_BCN_FIFO_Semaphore, 1);

        /* Configure BCNPLCP */
        if(mMacMode == PURE_A_MODE)  {
                BCNPlcp = 0x0000003b; //802.11a 5g OFDM 6Mb
        } else {
                index = (index << 3); //802.11b/g 2.4G CCK 1Mb
                BCNPlcp = 0x00000400;
        }
        BCNPlcp |= (((U32)index) << 16);
        pObj->SetReg(reg, ZD_BCNPLCPCfg, BCNPlcp);
}



void HW_SetSupportedRate(zd_80211Obj_t *pObj, U8 *prates)
{
        U8 HighestBasicRate = SR_1M;
        U8 HighestRate = SR_1M;
        U8 SRate;
        U32 tmpValue;
        U16 j;
        U8 MaxBasicRate;

        void *reg = pObj->reg;
        MaxBasicRate = pObj->BasicRate;


        for (j=0; j<(*(prates+1)); j++) {
                switch((*(prates+2+j)) & 0x7f) {
                case SR_1M:
                        SRate = SR_1M;
#if defined(AMAC)

                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate

                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_0;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
#endif
                        break;

                case SR_2M:
                        SRate = SR_2M;
#if defined(AMAC)

                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_1;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
#endif
                        break;

                case SR_5_5M:
                        SRate = SR_5_5M;
#if defined(AMAC)

                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);

                                tmpValue |= BIT_2;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
#endif
                        break;

                case SR_11M:
                        SRate = SR_11M;
#if defined(AMAC)

                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_3;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }

#endif
                        break;

#if	(defined(GCCK) && defined(OFDM))

                case SR_6M:
                        SRate = SR_6M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_8;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;

                case SR_9M:
                        SRate = SR_9M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_9;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;

                case SR_12M:
                        SRate = SR_12M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_10;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;

                case SR_18M:
                        SRate = SR_18M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_11;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;

                case SR_24M:
                        SRate = SR_24M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_12;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;

                case SR_36M:
                        SRate = SR_36M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_13;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);

                        }
                        break;

                case SR_48M:
                        SRate = SR_48M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_14;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;

                case SR_54M:
                        SRate = SR_54M;
                        if ((*(prates+2+j)) & 0x80) {	//It's a basic rate
                                tmpValue = pObj->GetReg(reg, ZD_BasicRateTbl);
                                tmpValue |= BIT_15;
                                pObj->SetReg(reg, ZD_BasicRateTbl, tmpValue);
                        }
                        break;
#endif

                default:
                        SRate = SR_1M;

                        break;
                }

                if (HighestRate < SRate)
                        HighestRate = SRate;


                if ((*(prates+2+j)) & 0x80) {
                        /* It's a basic rate */
                        if (HighestBasicRate < SRate)
                                HighestBasicRate = SRate;
                }
        }

#if !defined(OFDM)
        tmpValue = pObj->GetReg(reg, ZD_CtlReg1);

        if (pObj->BssType == INDEPENDENT_BSS) {
                if (HighestBasicRate == SR_1M) {
                        // Workaround compatibility issue.
                        // For resonable case, HighestBasicRate should larger than 2M if

                        // short-preamble is supported.
                        HighestBasicRate = SR_2M;
                        pObj->SetReg(reg, ZD_Ack_Timeout_Ext, 0x3f);
                }
        }

        switch(HighestBasicRate) {
        case SR_1M:
                tmpValue &= ~0x1c;
                tmpValue |= 0x00;
                pObj->SetReg(reg, ZD_CtlReg1, tmpValue);
                pObj->BasicRate = 0x0;
                break;

        case SR_2M:
                tmpValue &= ~0x1c;
                tmpValue |= 0x04;
                pObj->SetReg(reg, ZD_CtlReg1, tmpValue);
                pObj->BasicRate = 0x1;
                break;

        case SR_5_5M:
                tmpValue &= ~0x1c;

                tmpValue |= 0x08;
                pObj->SetReg(reg, ZD_CtlReg1, tmpValue);
                pObj->BasicRate = 0x2;
                break;

        case SR_11M:
                tmpValue &= ~0x1c;
                tmpValue |= 0x0c;
                pObj->SetReg(reg, ZD_CtlReg1, tmpValue);
                pObj->BasicRate = 0x3;
                break;

        default:
                break;
        }
#else
        switch(HighestBasicRate) {
        case SR_1M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x0;
                break;


        case SR_2M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x1;
                break;

        case SR_5_5M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x2;
                break;

        case SR_11M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x3;
                break;

        case SR_6M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x4;
                break;

        case SR_9M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x5;
                break;

        case SR_12M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x6;
                break;

        case SR_18M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x7;
                break;

        case SR_24M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x8;
                break;

        case SR_36M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0x9;
                break;

        case SR_48M:
                if (HighestBasicRate >= MaxBasicRate)

                        pObj->BasicRate = 0xa;
                break;

        case SR_54M:
                if (HighestBasicRate >= MaxBasicRate)
                        pObj->BasicRate = 0xb;
                break;

        default:

                break;
        }
#endif

        //FPRINT_V("HighestBasicRate", pObj->BasicRate);
}

extern U16 mBeaconPeriod;

void HW_SetSTA_PS(zd_80211Obj_t *pObj, U8 op)
{
        void *reg = pObj->reg;
        U32 tmpValue;

        tmpValue = pObj->GetReg(reg, ZD_BCNInterval);

        /* Beacon interval check */
        if((tmpValue & 0xffff) != mBeaconPeriod) {
                printk(KERN_ERR "Beacon Interval not match\n");
                return ;
        }

        //if (op)
        //	tmpValue |= STA_PS;
        //else
        tmpValue &= ~STA_PS;

        pObj->SetReg(reg, ZD_BCNInterval, tmpValue);
}


void HW_GetTsf(zd_80211Obj_t *pObj, U32 *loTsf, U32 *hiTsf)
{
        void *reg = pObj->reg;


        *loTsf = pObj->GetReg(reg, ZD_TSF_LowPart);
        *hiTsf = pObj->GetReg(reg, ZD_TSF_HighPart);
}

U32 HW_GetNow(zd_80211Obj_t *pObj)
{
#ifndef HOST_IF_USB
        void *reg = pObj->reg;
        return pObj->GetReg(reg, ZD_TSF_LowPart);  //us unit
#else

        return jiffies; //10ms unit
#endif
}

void HW_RadioOnOff(zd_80211Obj_t *pObj, U8 on)
{
        void *reg = pObj->reg;
        U32	tmpvalue;
        U8 ii;


        if (on) {
                //++ Turn on RF
                switch(pObj->rfMode) {
                case RFMD_RF:
                        if (!(pObj->PhyTest & BIT_2))
                                HW_Set_IF_Synthesizer(pObj, 0x000007);

                        if (!(pObj->PhyTest & BIT_0)) {
                                LockPhyReg(pObj);
                                pObj->SetReg(reg, ZD_CR10, 0x89);

                                pObj->SetReg(reg, ZD_CR11, 0x00);
                                tmpvalue = pObj->GetReg(reg, ZD_CR11);
                                tmpvalue &= 0xFF;
                                if (tmpvalue != 0x00) {
                                        if (pObj->PhyTest & BIT_1) {
                                                for (ii = 0; ii < 10; ii ++) {
                                                        pObj->DelayUs(1000);
                                                        pObj->SetReg(reg, ZD_CR11, 0x00);
                                                        tmpvalue = pObj->GetReg(reg, ZD_CR11);
                                                        if ((tmpvalue & 0xFF) == 0x00)
                                                                break;
                                                }
                                        }
                                }

                                UnLockPhyReg(pObj);
                        }
                        break;

                case AL2230_RF:
                case AL2230S_RF:
                        LockPhyReg(pObj);

                        for (ii = 0; ii < 10; ii ++) {
                                pObj->DelayUs(1000);
                                pObj->SetReg(reg, ZD_CR11, 0x00);
                                tmpvalue = pObj->GetReg(reg, ZD_CR11);
                                if ((tmpvalue & 0xFF) == 0x00)
                                        break;
                        }
#ifdef ZD1211
                        pObj->SetReg(reg, ZD_CR251, 0x3f);
#elif defined(ZD1211B)

                        pObj->SetReg(reg, ZD_CR251, 0x7f);
#else
	#error "You do not define ZD1211 Model"
#endif

                        UnLockPhyReg(pObj);
                        break;


                default:
                        break;
                }
        } else {
                //++ Turn off RF
                switch(pObj->rfMode) {
                case RFMD_RF:
                        if (!(pObj->PhyTest & BIT_0)) {
                                LockPhyReg(pObj);

                                pObj->SetReg(reg, ZD_CR11, 0x15);
                                tmpvalue = pObj->GetReg(reg, ZD_CR11);
                                pObj->SetReg(reg, ZD_CR10, 0x81);
                                UnLockPhyReg(pObj);
                                tmpvalue &= 0xFF;
                        }

                        if (!(pObj->PhyTest & BIT_2)) {
                                LockPhyReg(pObj);
                                HW_Set_IF_Synthesizer(pObj, 0x00000F);
                                UnLockPhyReg(pObj);
                        }
                        break;

                case AL2230_RF:
                case AL2230S_RF:
                        LockPhyReg(pObj);
                        pObj->SetReg(reg, ZD_CR11, 0x04);
                        pObj->SetReg(reg, ZD_CR251, 0x2f);
                        UnLockPhyReg(pObj);
                        break;


                default:
                        break;
                }

        }

}

#ifdef ZD1211
void HW_ResetPhy(zd_80211Obj_t *pObj)
{
        void *reg = pObj->reg;
        U32 phyOverwrite;


        LockPhyReg(pObj);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR0, 0x0a);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR0, 0x14);
#endif

        pObj->SetReg(reg, ZD_CR1, 0x06);
        pObj->SetReg(reg, ZD_CR2, 0x26);
        pObj->SetReg(reg, ZD_CR3, 0x38);
        pObj->SetReg(reg, ZD_CR4, 0x80);
        pObj->SetReg(reg, ZD_CR9, 0xa0);
        pObj->SetReg(reg, ZD_CR10, 0x81);
#if fTX_PWR_CTRL && fTX_GAIN_OFDM
        //tmpvalue = pObj->GetReg(reg, ZD_CR11);
        //tmpvalue |= BIT_6;
        //pObj->SetReg(reg, ZD_CR11, tmpvalue);
        pObj->SetReg(reg, ZD_CR11, BIT_6);
#else

        pObj->SetReg(reg, ZD_CR11, 0x00);
#endif

        pObj->SetReg(reg, ZD_CR12, 0x7f);
        pObj->SetReg(reg, ZD_CR13, 0x8c);
        pObj->SetReg(reg, ZD_CR14, 0x80);
        pObj->SetReg(reg, ZD_CR15, 0x3d);
        pObj->SetReg(reg, ZD_CR16, 0x20);
        pObj->SetReg(reg, ZD_CR17, 0x1e);
        pObj->SetReg(reg, ZD_CR18, 0x0a);
        pObj->SetReg(reg, ZD_CR19, 0x48);
        pObj->SetReg(reg, ZD_CR20, 0x0c);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR21, 0x0c);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR21, 0x0e);
#endif

        pObj->SetReg(reg, ZD_CR22, 0x23);
        pObj->SetReg(reg, ZD_CR23, 0x90);
        pObj->SetReg(reg, ZD_CR24, 0x14);
        pObj->SetReg(reg, ZD_CR25, 0x40);
        pObj->SetReg(reg, ZD_CR26, 0x10);
        pObj->SetReg(reg, ZD_CR27, 0x19);
        pObj->SetReg(reg, ZD_CR28, 0x7f);
        pObj->SetReg(reg, ZD_CR29, 0x80);

#ifndef ASIC

        pObj->SetReg(reg, ZD_CR30, 0x4b);
#else

        pObj->SetReg(reg, ZD_CR30, 0x49);
#endif

        pObj->SetReg(reg, ZD_CR31, 0x60);
        pObj->SetReg(reg, ZD_CR32, 0x43);
        pObj->SetReg(reg, ZD_CR33, 0x08);
        pObj->SetReg(reg, ZD_CR34, 0x06);
        pObj->SetReg(reg, ZD_CR35, 0x0a);
        pObj->SetReg(reg, ZD_CR36, 0x00);
        pObj->SetReg(reg, ZD_CR37, 0x00);
        pObj->SetReg(reg, ZD_CR38, 0x38);
        pObj->SetReg(reg, ZD_CR39, 0x0c);
        pObj->SetReg(reg, ZD_CR40, 0x84);
        pObj->SetReg(reg, ZD_CR41, 0x2a);
        pObj->SetReg(reg, ZD_CR42, 0x80);
        pObj->SetReg(reg, ZD_CR43, 0x10);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR44, 0x12);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR44, 0x33);
#endif

        pObj->SetReg(reg, ZD_CR46, 0xff);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR47, 0x1E);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR47, 0x1E);
#endif

        pObj->SetReg(reg, ZD_CR48, 0x26);
        pObj->SetReg(reg, ZD_CR49, 0x5b);


        pObj->SetReg(reg, ZD_CR64, 0xd0);
        pObj->SetReg(reg, ZD_CR65, 0x04);
        pObj->SetReg(reg, ZD_CR66, 0x58);
        pObj->SetReg(reg, ZD_CR67, 0xc9);
        pObj->SetReg(reg, ZD_CR68, 0x88);
        pObj->SetReg(reg, ZD_CR69, 0x41);
        pObj->SetReg(reg, ZD_CR70, 0x23);
        pObj->SetReg(reg, ZD_CR71, 0x10);
        pObj->SetReg(reg, ZD_CR72, 0xff);
        pObj->SetReg(reg, ZD_CR73, 0x32);
        pObj->SetReg(reg, ZD_CR74, 0x30);
        pObj->SetReg(reg, ZD_CR75, 0x65);

        pObj->SetReg(reg, ZD_CR76, 0x41);
        pObj->SetReg(reg, ZD_CR77, 0x1b);
        pObj->SetReg(reg, ZD_CR78, 0x30);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR79, 0x68);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR79, 0xf0);
#endif

        pObj->SetReg(reg, ZD_CR80, 0x64);
        pObj->SetReg(reg, ZD_CR81, 0x64);
        pObj->SetReg(reg, ZD_CR82, 0x00);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR83, 0x00);
        pObj->SetReg(reg, ZD_CR84, 0x00);
        pObj->SetReg(reg, ZD_CR85, 0x02);
        pObj->SetReg(reg, ZD_CR86, 0x00);
        pObj->SetReg(reg, ZD_CR87, 0x00);
        pObj->SetReg(reg, ZD_CR88, 0xff);
        pObj->SetReg(reg, ZD_CR89, 0xfc);
        pObj->SetReg(reg, ZD_CR90, 0x00);
        pObj->SetReg(reg, ZD_CR91, 0x00);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR83, 0x24);
        pObj->SetReg(reg, ZD_CR84, 0x04);
        pObj->SetReg(reg, ZD_CR85, 0x00);
        pObj->SetReg(reg, ZD_CR86, 0x0c);
        pObj->SetReg(reg, ZD_CR87, 0x12);
        pObj->SetReg(reg, ZD_CR88, 0x0c);
        pObj->SetReg(reg, ZD_CR89, 0x00);
        pObj->SetReg(reg, ZD_CR90, 0x58);
        pObj->SetReg(reg, ZD_CR91, 0x04);
#endif


        pObj->SetReg(reg, ZD_CR92, 0x00);
        pObj->SetReg(reg, ZD_CR93, 0x08);
        pObj->SetReg(reg, ZD_CR94, 0x00);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR95, 0x00);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR95, 0x20);
#endif

        pObj->SetReg(reg, ZD_CR96, 0xff);
        pObj->SetReg(reg, ZD_CR97, 0xe7);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR98, 0x00);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR98, 0x35);
#endif

        pObj->SetReg(reg, ZD_CR99, 0x00);
        pObj->SetReg(reg, ZD_CR100, 0x00);
        pObj->SetReg(reg, ZD_CR101, 0xae);
        pObj->SetReg(reg, ZD_CR102, 0x02);
        pObj->SetReg(reg, ZD_CR103, 0x00);
        pObj->SetReg(reg, ZD_CR104, 0x03);
        pObj->SetReg(reg, ZD_CR105, 0x65);
        pObj->SetReg(reg, ZD_CR106, 0x04);
        pObj->SetReg(reg, ZD_CR107, 0x00);
        pObj->SetReg(reg, ZD_CR108, 0x0a);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR109, 0xaa);
        pObj->SetReg(reg, ZD_CR110, 0xaa);
        pObj->SetReg(reg, ZD_CR111, 0x25);
        pObj->SetReg(reg, ZD_CR112, 0x25);
        pObj->SetReg(reg, ZD_CR113, 0x00);

        pObj->SetReg(reg, ZD_CR119, 0x1e);

        pObj->SetReg(reg, ZD_CR125, 0x90);
        pObj->SetReg(reg, ZD_CR126, 0x00);
        pObj->SetReg(reg, ZD_CR127, 0x00);
#elif defined(ZD1211B)

        pObj->SetReg(reg,ZD_CR109, 0x27);
        pObj->SetReg(reg,ZD_CR110, 0x27);
        pObj->SetReg(reg,ZD_CR111, 0x27);
        pObj->SetReg(reg,ZD_CR112, 0x27);
        pObj->SetReg(reg,ZD_CR113, 0x27);
        pObj->SetReg(reg,ZD_CR114, 0x27);
        pObj->SetReg(reg,ZD_CR115, 0x26);
        pObj->SetReg(reg,ZD_CR116, 0x24);
        pObj->SetReg(reg,ZD_CR117, 0xfc);
        pObj->SetReg(reg,ZD_CR118, 0xfa);
        pObj->SetReg(reg,ZD_CR119, 0x1e);
        pObj->SetReg(reg,ZD_CR125, 0x90);
        pObj->SetReg(reg,ZD_CR126, 0x00);
        pObj->SetReg(reg,ZD_CR127, 0x00);
        pObj->SetReg(reg,ZD_CR128, 0x14);
        pObj->SetReg(reg,ZD_CR129, 0x12);
        pObj->SetReg(reg,ZD_CR130, 0x10);
        pObj->SetReg(reg,ZD_CR131, 0x0c);
        pObj->SetReg(reg,ZD_CR136, 0xdf);
        pObj->SetReg(reg,ZD_CR137, 0xa0);
        pObj->SetReg(reg,ZD_CR138, 0xa8);
        pObj->SetReg(reg,ZD_CR139, 0xb4);
#endif


#if (defined(GCCK) && defined(OFDM))
	#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR5, 0x00);
        pObj->SetReg(reg, ZD_CR6, 0x00);
        pObj->SetReg(reg, ZD_CR7, 0x00);
        pObj->SetReg(reg, ZD_CR8, 0x00);
#endif

        pObj->SetReg(reg, ZD_CR9, 0x20);
        pObj->SetReg(reg, ZD_CR12, 0xf0);
        pObj->SetReg(reg, ZD_CR20, 0x0e);
        pObj->SetReg(reg, ZD_CR21, 0x0e);
        pObj->SetReg(reg, ZD_CR27, 0x10);
#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR44, 0x33);
#else

        pObj->SetReg(reg, ZD_CR44, 0x33);
#endif

        pObj->SetReg(reg, ZD_CR47, 0x1E);
        pObj->SetReg(reg, ZD_CR83, 0x24);
        pObj->SetReg(reg, ZD_CR84, 0x04);
        pObj->SetReg(reg, ZD_CR85, 0x00);
        pObj->SetReg(reg, ZD_CR86, 0x0C);
        pObj->SetReg(reg, ZD_CR87, 0x12);
        pObj->SetReg(reg, ZD_CR88, 0x0C);
        pObj->SetReg(reg, ZD_CR89, 0x00);
        pObj->SetReg(reg, ZD_CR90, 0x10);
        pObj->SetReg(reg, ZD_CR91, 0x08);
        pObj->SetReg(reg, ZD_CR93, 0x00);

        pObj->SetReg(reg, ZD_CR94, 0x01);
#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR95, 0x0);
#else

        pObj->SetReg(reg, ZD_CR95, 0x20); //3d24


#endif

        pObj->SetReg(reg, ZD_CR96, 0x50);
        pObj->SetReg(reg, ZD_CR97, 0x37);
#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR98, 0x35);
#else

        pObj->SetReg(reg, ZD_CR98, 0x8d); //4326
#endif

        pObj->SetReg(reg, ZD_CR101, 0x13);
        pObj->SetReg(reg, ZD_CR102, 0x27);
        pObj->SetReg(reg, ZD_CR103, 0x27);
        pObj->SetReg(reg, ZD_CR104, 0x18);
        pObj->SetReg(reg, ZD_CR105, 0x12);
        pObj->SetReg(reg, ZD_CR109, 0x27);
        pObj->SetReg(reg, ZD_CR110, 0x27);
        pObj->SetReg(reg, ZD_CR111, 0x27);
        pObj->SetReg(reg, ZD_CR112, 0x27);
        pObj->SetReg(reg, ZD_CR113, 0x27);
        pObj->SetReg(reg, ZD_CR114, 0x27);

        pObj->SetReg(reg, ZD_CR115, 0x26);
        pObj->SetReg(reg, ZD_CR116, 0x24);

        pObj->SetReg(reg, ZD_CR117, 0xfc);
        pObj->SetReg(reg, ZD_CR118, 0xfa);
        pObj->SetReg(reg, ZD_CR120, 0x4f); //3d24
#ifndef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR123, 0x27); //3d24
#endif

        pObj->SetReg(reg, ZD_CR125, 0xaa); //4326
        pObj->SetReg(reg, ZD_CR127, 0x03); //4326
        pObj->SetReg(reg, ZD_CR128, 0x14);
        pObj->SetReg(reg, ZD_CR129, 0x12);
        pObj->SetReg(reg, ZD_CR130, 0x10);
        pObj->SetReg(reg, ZD_CR131, 0x0C);
        pObj->SetReg(reg, ZD_CR136, 0xdf);
        pObj->SetReg(reg, ZD_CR137, 0x40);
        pObj->SetReg(reg, ZD_CR138, 0xa0);
        pObj->SetReg(reg, ZD_CR139, 0xb0);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR140, 0x99);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR140, 0x98); //4407
#endif

        pObj->SetReg(reg, ZD_CR141, 0x82);
#ifdef ZD1211

        pObj->SetReg(reg, ZD_CR142, 0x54);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_CR142, 0x53); //4407
#endif

        pObj->SetReg(reg, ZD_CR143, 0x1c);
        pObj->SetReg(reg, ZD_CR144, 0x6c);
        pObj->SetReg(reg, ZD_CR147, 0x07);
        pObj->SetReg(reg, ZD_CR148, 0x4c);
        pObj->SetReg(reg, ZD_CR149, 0x50);
        pObj->SetReg(reg, ZD_CR150, 0x0e);
        pObj->SetReg(reg, ZD_CR151, 0x18);
#ifdef ZD1211B

        pObj->SetReg(reg, ZD_CR159, 0x70); //3d24
#endif

        pObj->SetReg(reg, ZD_CR160, 0xfe);
        pObj->SetReg(reg, ZD_CR161, 0xee);
        pObj->SetReg(reg, ZD_CR162, 0xaa);
        pObj->SetReg(reg, ZD_CR163, 0xfa);
        pObj->SetReg(reg, ZD_CR164, 0xfa);

        pObj->SetReg(reg, ZD_CR165, 0xea);
        pObj->SetReg(reg, ZD_CR166, 0xbe);
        pObj->SetReg(reg, ZD_CR167, 0xbe);
        pObj->SetReg(reg, ZD_CR168, 0x6a);
        pObj->SetReg(reg, ZD_CR169, 0xba);
        pObj->SetReg(reg, ZD_CR170, 0xba);
        pObj->SetReg(reg, ZD_CR171, 0xba);
        // Note: CR204 must lead the CR203
        pObj->SetReg(reg, ZD_CR204, 0x7d);
        pObj->SetReg(reg, ZD_CR203, 0x30);
#ifndef HOST_IF_USB

        pObj->SetReg(reg, ZD_CR240, 0x80);
#endif

#endif
        //if (pObj->ChipVer == ZD_1211)
        {
                if (pObj->HWFeature & BIT_13) //6321 Bin 4 Tx IQ balance for ZD1212 only
                {
                        phyOverwrite = pObj->GetReg(reg, E2P_PHY_REG);
                        pObj->SetReg(reg, ZD_CR157, ((phyOverwrite >> 8) & 0xff)); //make sure no one will overwrite CR157 again
                }
        }



        UnLockPhyReg(pObj);
        return;
}
#elif defined(ZD1211B)
void HW_ResetPhy(zd_80211Obj_t *pObj)
{
        void *reg = pObj->reg;

        // Get Phy-Config permission
        LockPhyReg(pObj);

        pObj->SetReg(reg,ZD_CR0, 0x14);
        pObj->SetReg(reg,ZD_CR1, 0x06);
        pObj->SetReg(reg,ZD_CR2, 0x26);
        pObj->SetReg(reg,ZD_CR3, 0x38);
        pObj->SetReg(reg,ZD_CR4, 0x80);
        pObj->SetReg(reg,ZD_CR9, 0xe0);
        pObj->SetReg(reg,ZD_CR10, 0x81);
        //``JWEI 2003/12/26
#if fTX_PWR_CTRL && fTX_GAIN_OFDM

        pObj->SetReg(reg, ZD_CR11, BIT_6);
#else

pObj->SetReg(reg,ZD_CR11, 0x00);
#endif

        pObj->SetReg(reg,ZD_CR12, 0xf0);
        pObj->SetReg(reg,ZD_CR13, 0x8c);
        pObj->SetReg(reg,ZD_CR14, 0x80);
        pObj->SetReg(reg,ZD_CR15, 0x3d);
        pObj->SetReg(reg,ZD_CR16, 0x20);
        pObj->SetReg(reg,ZD_CR17, 0x1e);
        pObj->SetReg(reg,ZD_CR18, 0x0a);
        pObj->SetReg(reg,ZD_CR19, 0x48);
        pObj->SetReg(reg,ZD_CR20, 0x10);//Org:0x0E,ComTrend:RalLink AP
        pObj->SetReg(reg,ZD_CR21, 0x0e);
        pObj->SetReg(reg,ZD_CR22, 0x23);
        pObj->SetReg(reg,ZD_CR23, 0x90);
        pObj->SetReg(reg,ZD_CR24, 0x14);
        pObj->SetReg(reg,ZD_CR25, 0x40);
        pObj->SetReg(reg,ZD_CR26, 0x10);
        pObj->SetReg(reg,ZD_CR27, 0x10);
        pObj->SetReg(reg,ZD_CR28, 0x7f);
        pObj->SetReg(reg,ZD_CR29, 0x80);
#ifndef ASIC
        // For FWT
        pObj->SetReg(reg,ZD_CR30, 0x4b);
#else
// For Jointly decoder
pObj->SetReg(reg,ZD_CR30, 0x49);
#endif

        pObj->SetReg(reg,ZD_CR31, 0x60);
        pObj->SetReg(reg,ZD_CR32, 0x43);
        pObj->SetReg(reg,ZD_CR33, 0x08);
        pObj->SetReg(reg,ZD_CR34, 0x06);
        pObj->SetReg(reg,ZD_CR35, 0x0a);
        pObj->SetReg(reg,ZD_CR36, 0x00);
        pObj->SetReg(reg,ZD_CR37, 0x00);
        pObj->SetReg(reg,ZD_CR38, 0x38);
        pObj->SetReg(reg,ZD_CR39, 0x0c);
        pObj->SetReg(reg,ZD_CR40, 0x84);
        pObj->SetReg(reg,ZD_CR41, 0x2a);
        pObj->SetReg(reg,ZD_CR42, 0x80);
        pObj->SetReg(reg,ZD_CR43, 0x10);
        pObj->SetReg(reg,ZD_CR44, 0x33);
        pObj->SetReg(reg,ZD_CR46, 0xff);
        pObj->SetReg(reg,ZD_CR47, 0x1E);
        pObj->SetReg(reg,ZD_CR48, 0x26);
        pObj->SetReg(reg,ZD_CR49, 0x5b);
        pObj->SetReg(reg,ZD_CR64, 0xd0);
        pObj->SetReg(reg,ZD_CR65, 0x04);
        pObj->SetReg(reg,ZD_CR66, 0x58);
        pObj->SetReg(reg,ZD_CR67, 0xc9);
        pObj->SetReg(reg,ZD_CR68, 0x88);
        pObj->SetReg(reg,ZD_CR69, 0x41);
        pObj->SetReg(reg,ZD_CR70, 0x23);
        pObj->SetReg(reg,ZD_CR71, 0x10);
        pObj->SetReg(reg,ZD_CR72, 0xff);
        pObj->SetReg(reg,ZD_CR73, 0x32);
        pObj->SetReg(reg,ZD_CR74, 0x30);
        pObj->SetReg(reg,ZD_CR75, 0x65);
        pObj->SetReg(reg,ZD_CR76, 0x41);
        pObj->SetReg(reg,ZD_CR77, 0x1b);
        pObj->SetReg(reg,ZD_CR78, 0x30);
        pObj->SetReg(reg,ZD_CR79, 0xf0);
        pObj->SetReg(reg,ZD_CR80, 0x64);
        pObj->SetReg(reg,ZD_CR81, 0x64);
        pObj->SetReg(reg,ZD_CR82, 0x00);
        pObj->SetReg(reg,ZD_CR83, 0x24);
        pObj->SetReg(reg,ZD_CR84, 0x04);
        pObj->SetReg(reg,ZD_CR85, 0x00);
        pObj->SetReg(reg,ZD_CR86, 0x0c);
        pObj->SetReg(reg,ZD_CR87, 0x12);
        pObj->SetReg(reg,ZD_CR88, 0x0c);
        pObj->SetReg(reg,ZD_CR89, 0x00);
        pObj->SetReg(reg,ZD_CR90, 0x58);
        pObj->SetReg(reg,ZD_CR91, 0x04);
        pObj->SetReg(reg,ZD_CR92, 0x00);
        pObj->SetReg(reg,ZD_CR93, 0x00);
        pObj->SetReg(reg,ZD_CR94, 0x01);
        pObj->SetReg(reg,ZD_CR95, 0x20); // ZD1211B
        pObj->SetReg(reg,ZD_CR96, 0x50);
        pObj->SetReg(reg,ZD_CR97, 0x37);
        pObj->SetReg(reg,ZD_CR98, 0x35);
        pObj->SetReg(reg,ZD_CR99, 0x00);
        pObj->SetReg(reg,ZD_CR100, 0x01);
        pObj->SetReg(reg,ZD_CR101, 0x13);
        pObj->SetReg(reg,ZD_CR102, 0x27);
        pObj->SetReg(reg,ZD_CR103, 0x27);
        pObj->SetReg(reg,ZD_CR104, 0x18);
        pObj->SetReg(reg,ZD_CR105, 0x12);
        pObj->SetReg(reg,ZD_CR106, 0x04);
        pObj->SetReg(reg,ZD_CR107, 0x00);
        pObj->SetReg(reg,ZD_CR108, 0x0a);
        pObj->SetReg(reg,ZD_CR109, 0x27);
        pObj->SetReg(reg,ZD_CR110, 0x27);
        pObj->SetReg(reg,ZD_CR111, 0x27);
        pObj->SetReg(reg,ZD_CR112, 0x27);
        pObj->SetReg(reg,ZD_CR113, 0x27);
        pObj->SetReg(reg,ZD_CR114, 0x27);
        pObj->SetReg(reg,ZD_CR115, 0x26);
        pObj->SetReg(reg,ZD_CR116, 0x24);
        pObj->SetReg(reg,ZD_CR117, 0xfc);
        pObj->SetReg(reg,ZD_CR118, 0xfa);
        pObj->SetReg(reg,ZD_CR119, 0x1e);
        pObj->SetReg(reg,ZD_CR125, 0x90);
        pObj->SetReg(reg,ZD_CR126, 0x00);
        pObj->SetReg(reg,ZD_CR127, 0x00);
        pObj->SetReg(reg,ZD_CR128, 0x14);
        pObj->SetReg(reg,ZD_CR129, 0x12);
        pObj->SetReg(reg,ZD_CR130, 0x10);
        pObj->SetReg(reg,ZD_CR131, 0x0c);
        pObj->SetReg(reg,ZD_CR136, 0xdf);
        pObj->SetReg(reg,ZD_CR137, 0xa0);
        pObj->SetReg(reg,ZD_CR138, 0xa8);
        pObj->SetReg(reg,ZD_CR139, 0xb4);
        pObj->SetReg(reg,ZD_CR140, 0x98);
        pObj->SetReg(reg,ZD_CR141, 0x82);
        pObj->SetReg(reg,ZD_CR142, 0x53);
        pObj->SetReg(reg,ZD_CR143, 0x1c);
        pObj->SetReg(reg,ZD_CR144, 0x6c);
        pObj->SetReg(reg,ZD_CR147, 0x07);
        pObj->SetReg(reg,ZD_CR148, 0x40);
        pObj->SetReg(reg,ZD_CR149, 0x40); // Org:0x50 //ComTrend:RalLink AP
        pObj->SetReg(reg,ZD_CR150, 0x14);//Org:0x0E //ComTrend:RalLink AP
        pObj->SetReg(reg,ZD_CR151, 0x18);
        pObj->SetReg(reg,ZD_CR159, 0x70);
        pObj->SetReg(reg,ZD_CR160, 0xfe);
        pObj->SetReg(reg,ZD_CR161, 0xee);
        pObj->SetReg(reg,ZD_CR162, 0xaa);
        pObj->SetReg(reg,ZD_CR163, 0xfa);
        pObj->SetReg(reg,ZD_CR164, 0xfa);
        pObj->SetReg(reg,ZD_CR165, 0xea);
        pObj->SetReg(reg,ZD_CR166, 0xbe);
        pObj->SetReg(reg,ZD_CR167, 0xbe);
        pObj->SetReg(reg,ZD_CR168, 0x6a);
        pObj->SetReg(reg,ZD_CR169, 0xba);
        pObj->SetReg(reg,ZD_CR170, 0xba);
        pObj->SetReg(reg,ZD_CR171, 0xba);
        // Note: CR204 must lead the CR203
        pObj->SetReg(reg,ZD_CR204, 0x7d);
        pObj->SetReg(reg,ZD_CR203, 0x30);

        // Release Phy-Config permission
        //    if (pObj->ChipVer == ZD_1211)
        //    {
        //        if (pObj->HWFeature & BIT_13) //6321 Bin 4 Tx IQ balance for ZD1212 only
        //        {
        //            phyOverwrite = pObj->GetReg(reg, E2P_PHY_REG);
        //            pObj->SetReg(reg, ZD_CR157, ((phyOverwrite >> 8) & 0xff)); //make sure no one will overwrite CR157 again
        //        }
        //    }

        UnLockPhyReg(pObj);


        return;
}

#endif

void HW_InitHMAC(zd_80211Obj_t *pObj)
{
        void *reg = pObj->reg;

        // Set GPI_EN be zero. ie. Disable GPI (Requested by Ahu)
        //pObj->SetReg(reg, ZD_GPI_EN, 0x00);

        // Use Ack_Timeout_Ext to tolerance some peers that response slowly.
        // The influence is that the retry frame will be less competitive. It's acceptable.
        pObj->SetReg(reg, ZD_Ack_Timeout_Ext, 0x20); //only bit0-bit5 are valid

        pObj->SetReg(reg, ZD_ADDA_MBIAS_WarmTime, 0x30000808);

        /* Set RetryMax 8 */
#ifdef ZD1211

        pObj->SetReg(reg, ZD_RetryMAX, 0x2);
#elif defined(ZD1211B)

        pObj->SetReg(reg, ZD_RetryMAX, 0x02020202);

        pObj->SetReg(reg,0xB0C,0x007f003f);
        pObj->SetReg(reg,0xB08,0x007f003f);
        pObj->SetReg(reg,0xB04,0x003f001f);
        pObj->SetReg(reg,0xB00,0x001f000f);
        //set AIFS AC0 - AC3
        pObj->SetReg(reg,0xB10,0x00280028);
        pObj->SetReg(reg,0xB14,0x008C003C);
        //set TXOP AC0 - AC3
        pObj->SetReg(reg,0xB20,0x01800824);
        //pObj->SetReg(reg,0xB20,0x00800a28);

#endif

        /* Turn off sniffer mode */
        pObj->SetReg(reg, ZD_SnifferOn, 0);

        /* Set Rx filter*/
        // filter Beacon and unfilter PS-Poll
        pObj->SetReg(reg, ZD_Rx_Filter, AP_RX_FILTER);

        /* Set Hashing table */
        pObj->SetReg(reg, ZD_GroupHash_P1, 0x00);
        pObj->SetReg(reg, ZD_GroupHash_P2, 0x80000000);

        pObj->SetReg(reg, ZD_CtlReg1, 0xa4);
        pObj->SetReg(reg, ZD_ADDA_PwrDwn_Ctrl, 0x7f);

        /* Initialize BCNATIM needed registers */
        pObj->SetReg(reg, ZD_BCNPLCPCfg, 0x00f00401);
        pObj->SetReg(reg, ZD_PHYDelay, 0x00);

#if defined(OFDM)
	#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_Ack_Timeout_Ext, 0x80);
        pObj->SetReg(reg, ZD_ADDA_PwrDwn_Ctrl, 0x0);
#endif

        //pObj->SetReg(reg, ZD_AckTime80211, 0x102);
        pObj->SetReg(reg, ZD_AckTime80211, 0x100);
        pObj->SetReg(reg, ZD_IFS_Value, 0x547c032); //0x547c032

        // accept beacon for enable protection mode

        //pObj->SetReg(reg, ZD_Rx_Filter, ((BIT_10 << 16) | (0xffff)));
        //pObj->SetReg(reg, ZD_Rx_Filter, ((BIT_10 << 16) | (0xffff & ~BIT_8))); //for pure G debug

        // Set RX_PE_DELAY 0x10 to enlarge the time for decharging tx power.
        pObj->SetReg(reg, ZD_RX_PE_DELAY, 0x70);

        //pObj->SetReg(reg, ZD_SnifferOn, 0x3000000); //enable HW Rx Retry filter, and HW MIC
        //pObj->SetReg(reg, ZD_Rx_OFFSET, 0x03); //to fit MIC engine's 4 byte alignment

        // Keep 44MHz oscillator always on.
        pObj->SetReg(reg, ZD_PS_Ctrl, 0x10000000);
#endif


#if defined(AMAC)
	#if defined(ECCK_60_5)

        pObj->SetReg(reg, ZD_RTS_CTS_Rate, 0x00);
#elif (defined(GCCK) && defined(OFDM))
        //pObj->SetReg(reg, ZD_RTS_CTS_Rate, 0x30000);
        pObj->SetReg(reg, ZD_RTS_CTS_Rate, 0x2030203);
#endif

        // Set Rx Threshold
        pObj->SetReg(reg, ZD_RX_THRESHOLD, 0x000c0640);

        // Set Tx-Pwr-Control registers
        //pObj->SetReg(reg, ZD_TX_PWR_CTRL_1, 0x7f7f7f7f);
        //pObj->SetReg(reg, ZD_TX_PWR_CTRL_2, 0x7c7f7f7f);
        //pObj->SetReg(reg, ZD_TX_PWR_CTRL_3, 0x6c6c747c);
        //pObj->SetReg(reg, ZD_TX_PWR_CTRL_4, 0x00006064);

#ifdef HOST_IF_USB

        pObj->SetReg(reg, ZD_AfterPNP, 0x1);
        pObj->SetReg(reg, ZD_Wep_Protect, 0x114);
#else

        pObj->SetReg(reg, ZD_AfterPNP, 0x64009);
        pObj->SetReg(reg, ZD_Wep_Protect, 0x118); //4315 for TKIP key mixing
#endif
#endif
}

void HW_OverWritePhyRegFromE2P(zd_80211Obj_t *pObj)
{
        U32 tmpvalue;
        void *reg = pObj->reg;

#ifdef HOST_IF_USB

        if (!pObj->bOverWritePhyRegFromE2P)
                return;

        LockPhyReg(pObj);
        tmpvalue = pObj->GetReg(reg, E2P_PHY_REG);
        pObj->SetReg(reg, ZD_CR47, (tmpvalue & 0xFF));
        UnLockPhyReg(pObj);
        return;
#endif
}

void HW_WritePhyReg(zd_80211Obj_t *pObj, U8 PhyIdx, U8 PhyValue)
{
        U32	IoAddress;
        void *reg = pObj->reg;

        switch(PhyIdx) {
        case 4:
                IoAddress = 0x20;
                break;

        case 5:
                IoAddress = 0x10;
                break;

        case 6:
                IoAddress = 0x14;
                break;

        case 7:
                IoAddress = 0x18;
                break;

        case 8:
                IoAddress = 0x1C;
                break;

        default:
                IoAddress = (((U32)PhyIdx) << 2);

                break;
        }

        LockPhyReg(pObj);
        pObj->SetReg(reg, IoAddress, PhyValue);
        UnLockPhyReg(pObj);
}


void HW_UpdateIntegrationValue(zd_80211Obj_t *pObj, U32 ChannelNo, const U8 MAC_Mode)
{
#ifdef ZD1211B
        void *reg = pObj->reg;
        struct zd1205_private *macp = (struct zd1205_private *) g_dev->priv;
#endif
#ifdef HOST_IF_USB
        U32	tmpvalue;
        u8 Useless_set, intV;

        //tmpvalue = pObj->GetReg(reg, ZD_E2P_PWR_INT_VALUE1+((ChannelNo-1) & 0xc));
        //tmpvalue = (U8) (tmpvalue >> (((ChannelNo - 1) % 4) * 8));
        if(PURE_A_MODE != MAC_Mode)
                tmpvalue = pObj->IntValue[ChannelNo - 1];
        else if(PURE_A_MODE == MAC_Mode) {
                a_OSC_get_cal_int(ChannelNo, RATE_54M,&intV,&Useless_set);
                tmpvalue = intV;
        } else
                VerAssert();
        HW_Write_TxGain1(pObj, (U8) tmpvalue, cTX_CCK);
#endif
#ifdef ZD1211B

        LockPhyReg(pObj);
        if(PURE_A_MODE != MAC_Mode) {
                pObj->SetReg(reg,ZD_CR65,macp->SetPointOFDM[2][ChannelNo-1]);
                pObj->SetReg(reg,ZD_CR66,macp->SetPointOFDM[1][ChannelNo-1]);
                pObj->SetReg(reg,ZD_CR67,macp->SetPointOFDM[0][ChannelNo-1]);
                pObj->SetReg(reg,ZD_CR68,macp->EepSetPoint[ChannelNo-1]);
        } else {
                u8 set36,set48,set54, intValue;
                a_OSC_get_cal_int( ChannelNo, RATE_54M, &intValue, &set54);
                a_OSC_get_cal_int( ChannelNo, RATE_48M, &intValue, &set48);
                a_OSC_get_cal_int( ChannelNo, RATE_36M, &intValue, &set36);
                pObj->SetReg(reg,ZD_CR65,set54);
                pObj->SetReg(reg,ZD_CR66,set48);
                pObj->SetReg(reg,ZD_CR67,set36);
                pObj->SetReg(reg,ZD_CR68,macp->EepSetPoint[ChannelNo-1]);
        }

        pObj->SetReg(reg,ZD_CR69,0x28);
        pObj->SetReg(reg,ZD_CR69,0x2a);

        UnLockPhyReg(pObj);
#endif
}

void HW_Write_TxGain(zd_80211Obj_t *pObj, U32 txgain)
{
        U32	tmpvalue;
        void *reg = pObj->reg;
        U8	i;

        switch(pObj->rfMode) {
        case GCT_RF:
                txgain &= 0x3f;

                //FPRINT_V("Set tx gain", txgain);
                tmpvalue = 0;
                // Perform Bit-Reverse
                for (i=0; i<6; i++) {
                        if (txgain & BIT_0) {
                                tmpvalue |= (0x1 << (15-i));
                        }
                        txgain = (txgain >> 1);
                }
                tmpvalue |= 0x0c0000;
                HW_Set_IF_Synthesizer(pObj, tmpvalue);
                //FPRINT_V("HW_Set_IF_Synthesizer", tmpvalue);
                HW_Set_IF_Synthesizer(pObj, 0x150800);
                HW_Set_IF_Synthesizer(pObj, 0x150000);
                break;

        case AL2210_RF:
        case AL2210MPVB_RF:
                if (txgain > AL2210_MAX_TX_PWR_SET) {
                        txgain = AL2210_MAX_TX_PWR_SET;
                } else if (txgain < AL2210_MIN_TX_PWR_SET) {
                        txgain = AL2210_MIN_TX_PWR_SET;
                }

                LockPhyReg(pObj);
                pObj->SetReg(reg, ZD_CR31, (U8)txgain);
                UnLockPhyReg(pObj);
                break;

        default:
                break;
        }
}


void HW_Write_TxGain0(zd_80211Obj_t *pObj, U8 *pTxGain, U8 TxPwrType)
{
        void *reg = pObj->reg;

        switch (pObj->rfMode) {
        case MAXIM_NEW_RF:
                *pTxGain &= MAXIM2_MAX_TX_PWR_SET;
                LockPhyReg(pObj);

                if (TxPwrType != cTX_OFDM) {
                        pObj->SetReg(reg, ZD_CR31, *pTxGain);
                } else {
#if !fTX_GAIN_OFDM
                        pObj->SetReg(reg, ZD_CR31, *pTxGain);
#else

                        pObj->SetReg(reg, ZD_CR51, *pTxGain);
                        pObj->SetReg(reg, ZD_CR52, *pTxGain);
                        pObj->SetReg(reg, ZD_CR53, *pTxGain);
#endif

                }
                UnLockPhyReg(pObj);
                break;

        case RFMD_RF:
        case AL2230_RF:
        case AL7230B_RF:
        case AL2230S_RF:
                LockPhyReg(pObj);
                if (TxPwrType != cTX_OFDM) {
                        pObj->SetReg(reg, ZD_CR31, *pTxGain);
                } else {
#if !fTX_GAIN_OFDM
                        pObj->SetReg(reg, ZD_CR31, *pTxGain);
#else

                        pObj->SetReg(reg, ZD_CR51, *pTxGain);
                        pObj->SetReg(reg, ZD_CR52, *pTxGain);
                        pObj->SetReg(reg, ZD_CR53, *pTxGain);
#endif

                }
                UnLockPhyReg(pObj);
                break;
        default:
                break;
        }
}

void HW_Write_TxGain1(zd_80211Obj_t *pObj, U8 txgain, U8 TxPwrType)
{
        U8   *pTxGain;

        HW_Write_TxGain0(pObj, &txgain, TxPwrType);

#if fTX_GAIN_OFDM

        if (TxPwrType != cTX_OFDM)
                pTxGain = &(pObj->TxGainSetting);
        else
                pTxGain = &(pObj->TxGainSetting2);
#else

        pTxGain = &(pObj->TxGainSetting);
#endif

        *pTxGain = txgain;
}

void HW_Write_TxGain2(zd_80211Obj_t *pObj, U8 TxPwrType)
{
        U8   *pTxGain;


        if (TxPwrType != cTX_OFDM) {
                pTxGain = &(pObj->TxGainSetting);
        } else {
#if fTX_GAIN_OFDM
                pTxGain = &(pObj->TxGainSetting2);
#else

                pTxGain = &(pObj->TxGainSetting);
#endif

        }

        HW_Write_TxGain0(pObj, pTxGain, TxPwrType);
}

void HW_Set_FilterBand(zd_80211Obj_t *pObj, U32	region_code)
{
        U32	tmpLong;
        void *reg = pObj->reg;

        switch(region_code) {
        case 0x40:	// Japan
                LockPhyReg(pObj);
                //if (pObj->rfMode == MAXIM_NEW_RF)
                {
                        tmpLong = pObj->GetReg(reg, ZD_CR5);
                        tmpLong |= BIT_6;	//japan
                        pObj->SetReg(reg, ZD_CR5, tmpLong);
                }

                UnLockPhyReg(pObj);
                break;

        default:
                LockPhyReg(pObj);
                tmpLong = pObj->GetReg(reg, ZD_CR5);
                tmpLong &= ~BIT_6;//USA

                pObj->SetReg(reg, ZD_CR5, tmpLong);
                UnLockPhyReg(pObj);
                break;
        }
}


void HW_UpdateBcnInterval(zd_80211Obj_t *pObj, U16 BcnInterval)
{
        void *reg = pObj->reg;
        U32	tmpvalue;
        U32	ul_PreTBTT;
        U32	ul_ATIMWnd;


        //++
        // Make sure that BcnInterval > Pre_TBTT > ATIMWnd >= 0
        if (BcnInterval < 5) {
                BcnInterval = 5;
        }

        ul_PreTBTT = pObj->GetReg(reg, ZD_Pre_TBTT);
        if (ul_PreTBTT < 4) {
                ul_PreTBTT = 4;
        }

        if (ul_PreTBTT >= BcnInterval) {
                ul_PreTBTT = BcnInterval-1;

        }
        pObj->SetReg(reg, ZD_Pre_TBTT, ul_PreTBTT);

        ul_ATIMWnd = pObj->GetReg(reg, ZD_ATIMWndPeriod);
        if (ul_ATIMWnd >= ul_PreTBTT) {
                ul_ATIMWnd = ul_PreTBTT-1;
        }
        pObj->SetReg(reg, ZD_ATIMWndPeriod, ul_ATIMWnd);

        tmpvalue = pObj->GetReg(reg, ZD_BCNInterval);
        tmpvalue &= ~0xffff;
        tmpvalue |= BcnInterval;
        pObj->SetReg(reg, ZD_BCNInterval, tmpvalue);

        pObj->BeaconInterval = BcnInterval;
}




void HW_UpdateATIMWindow(zd_80211Obj_t *pObj, U16 AtimWnd)
{
        void *reg = pObj->reg;
        U32	ul_PreTBTT;

        //++
        // Make sure that Pre_TBTT > ATIMWnd >= 0

        ul_PreTBTT = pObj->GetReg(reg, ZD_Pre_TBTT);
        if (AtimWnd >= ul_PreTBTT) {
                AtimWnd = (U16)(ul_PreTBTT-1);
        }
        //--

        pObj->SetReg(reg, ZD_ATIMWndPeriod, AtimWnd);
}


void HW_UpdatePreTBTT(zd_80211Obj_t *pObj, U32 pretbtt)
{
        void *reg = pObj->reg;
        U32	ul_BcnItvl;
        U32	ul_AtimWnd;

        //++
        // Make sure that BcnInterval > Pre_TBTT > ATIMWnd
        ul_BcnItvl = pObj->GetReg(reg, ZD_BCNInterval);
        ul_BcnItvl &= 0xff;
        if (pretbtt >= ul_BcnItvl) {
                pretbtt = ul_BcnItvl-1;
        }

        ul_AtimWnd = pObj->GetReg(reg, ZD_ATIMWndPeriod);
        if (pretbtt <= ul_AtimWnd) {
                pretbtt = ul_AtimWnd+1;
        }
        //--

        pObj->SetReg(reg, ZD_Pre_TBTT, pretbtt);
}

// for AMAC CAM operation
void HW_CAM_Avail(zd_80211Obj_t *pObj)
{
        void *reg = pObj->reg;
        U32 tmpValue;

        tmpValue = pObj->GetReg(reg, ZD_CAM_MODE);
        while(tmpValue & HOST_PEND) {
                pObj->DelayUs(10);
                tmpValue = pObj->GetReg(reg, ZD_CAM_MODE);
        }
}

void HW_CAM_Write(zd_80211Obj_t *pObj, U32 address, U32 data)
{
        void *reg = pObj->reg;

        HW_CAM_Avail(pObj);
        pObj->SetReg(reg, ZD_CAM_DATA, data);
        pObj->SetReg(reg, ZD_CAM_ADDRESS, (address | CAM_WRITE));
}

U32 HW_CAM_Read(zd_80211Obj_t *pObj, U32 address)
{
        void *reg = pObj->reg;
        U32 result;

        HW_CAM_Avail(pObj);
        pObj->SetReg(reg, ZD_CAM_ADDRESS, address);
        HW_CAM_Avail(pObj);
        result = pObj->GetReg(reg, ZD_CAM_DATA);

        return result;
}

void HW_CAM_SetMAC(zd_80211Obj_t *pObj, U16 aid, U8 *pMAC)
{
        U32 userWordAddr;
        U32 userByteOffset;
        U32 tmpValue;
        int i;

        userWordAddr = (aid/4)*6;
        userByteOffset = aid % 4;

        for (i=0; i<MAC_LENGTH; i++) {
                tmpValue = HW_CAM_Read(pObj, (userWordAddr+i));
                tmpValue &= ~(0xff << (userByteOffset*8));
                tmpValue |= pMAC[i]<<(userByteOffset*8);
                HW_CAM_Write(pObj, (userWordAddr+i), tmpValue);
        }
}


void HW_CAM_GetMAC(zd_80211Obj_t *pObj, U16 aid, U8 *pMac)
{
        U32 userWordAddr;
        U32 userByteOffset;
        U32 tmpValue;
        U8 mac[6];
        int i;

        userWordAddr = (aid/4)*6;
        userByteOffset = aid % 4;

        for (i=0; i<MAC_LENGTH; i++) {
                tmpValue = HW_CAM_Read(pObj, (userWordAddr+i));
                mac[i] = (U8)(tmpValue >> (userByteOffset*8)) & 0xFF;
        }

        if (memcmp(mac, pMac, 6) != 0) {
                FPRINT("*****Where is my MAC ????");
        } else
                FPRINT("*****Verify MAC OK!!!");
}


void HW_CAM_SetEncryType(zd_80211Obj_t *pObj, U16 aid, U8 encryType)
{
        U32 encryWordAddr;
        U32 encryByteOffset;
        U32 tmpValue;

        U8 targetByte;

        encryWordAddr = ENCRY_TYPE_START_ADDR + (aid/8);
        encryByteOffset = (aid/2) % 4;

        tmpValue = HW_CAM_Read(pObj, encryWordAddr);
        targetByte = (U8)(tmpValue >> (encryByteOffset*8));
        tmpValue &= ~(0xff << (encryByteOffset*8)); //clear target byte
        if (aid % 2)
                targetByte = (encryType<<4) | (targetByte & 0xf); //set hignt part
        else //low nibble
                targetByte = encryType | (targetByte & 0xf0); //set low part

        tmpValue |= targetByte << (encryByteOffset*8);


        HW_CAM_Write(pObj, encryWordAddr, tmpValue);
}


U8 HW_CAM_GetEncryType(zd_80211Obj_t *pObj, U16 aid)
{
        U32 encryWordAddr;
        U32 encryByteOffset;

        U32 tmpValue;
        U8 keyLength = 0;
        U8 targetByte;

        encryWordAddr = ENCRY_TYPE_START_ADDR + (aid/8);
        encryByteOffset = (aid/2) % 4;

        tmpValue = HW_CAM_Read(pObj, encryWordAddr);
        targetByte = (U8)(tmpValue >> (encryByteOffset*8));
        if (aid % 2)
                targetByte >>= 4; //get hignt part
        else
                targetByte &= 0x0f; //get low part

        switch(targetByte) {
        case NO_WEP: //0
                FPRINT("***No Encryption");
                break;

        case WEP64: //1
                FPRINT("***WEP 64");
                keyLength = 5;
                break;

        case TKIP: //2
                FPRINT("**TKIP");
                keyLength = 16;
                break;

        case AES: //4

                FPRINT("***CCM");
                keyLength = 16;
                break;


        case WEP128: //5
                FPRINT("***WEP 128");
                keyLength = 13;
                break;

        default:
                FPRINT("***Not Supported Encry");
                break;
        }

        return keyLength;

}


void HW_CAM_SetKey(zd_80211Obj_t *pObj, U16 aid, U8 keyLength, U8 *pKey)
{
        U32 keyWordAddr;
        U8 offset;
        U32 tmpValue;
        int i, j, k;

        keyWordAddr = KEY_START_ADDR + (aid*8);

        offset = 0;
        for (i=0; i<8; i++) {
                tmpValue = HW_CAM_Read(pObj, (keyWordAddr+i));
                for (j=offset, k=0; k<4; j++, k++) {
                        tmpValue &= ~(0xff << (k*8));
                        if (offset < keyLength) {
                                tmpValue |= pKey[j] << (k*8);
                        }
                        offset++;
                }
                HW_CAM_Write(pObj, (keyWordAddr+i), tmpValue);
        }

}


void HW_CAM_GetKey(zd_80211Obj_t *pObj, U16 aid, U8 keyLength, U8 *pKey)
{
        U32 keyWordAddr;
        U8 key[32];
        int i, j;
        U32 tmpValue;

        keyWordAddr = KEY_START_ADDR + (aid*8);
        j = 0;
        for (i=0; i<8; i++) {
                tmpValue = HW_CAM_Read(pObj, (keyWordAddr+i));
                key[j] = (U8)(tmpValue);
                j++;
                key[j] = (U8)(tmpValue >> 8);
                j++;
                key[j] = (U8)(tmpValue >> 16);
                j++;
                key[j] = (U8)(tmpValue >> 24);
                j++;
        }

        if (memcmp(&key[0], pKey, keyLength) != 0) {
                FPRINT("*****Where is my Key ????");
        } else
                FPRINT("*****Verify KEY OK!!!");
}


void HW_CAM_UpdateRollTbl(zd_80211Obj_t *pObj, U16 aid)
{
        void *reg = pObj->reg;
        U32 tmpValue;

        if (aid >= 32) {
                tmpValue = pObj->GetReg(reg, ZD_CAM_ROLL_TB_HIGH);
                tmpValue |= BIT_0 << (aid-32);
                pObj->SetReg(reg, ZD_CAM_ROLL_TB_HIGH, tmpValue);
        } else {
                tmpValue = pObj->GetReg(reg, ZD_CAM_ROLL_TB_LOW);
                tmpValue |= (BIT_0 << aid);
                pObj->SetReg(reg, ZD_CAM_ROLL_TB_LOW, tmpValue);
        }
}


void HW_CAM_ResetRollTbl(zd_80211Obj_t *pObj)
{
        void *reg = pObj->reg;

        pObj->SetReg(reg, ZD_CAM_ROLL_TB_LOW, 0);
        pObj->SetReg(reg, ZD_CAM_ROLL_TB_HIGH, 0);
}


void HW_CAM_ClearRollTbl(zd_80211Obj_t *pObj, U16 aid)
{
        void *reg = pObj->reg;
        U32 tmpValue;


        //update roll table
        if (aid > 32) {
                tmpValue = pObj->GetReg(reg, ZD_CAM_ROLL_TB_HIGH);
                tmpValue &= ~(BIT_0 << (aid-32)); //set user invalid
                pObj->SetReg(reg, ZD_CAM_ROLL_TB_HIGH, tmpValue);
        } else {
                tmpValue = pObj->GetReg(reg, ZD_CAM_ROLL_TB_LOW);
                tmpValue &= ~(BIT_0 << aid); //set user invalid
                pObj->SetReg(reg, ZD_CAM_ROLL_TB_LOW, tmpValue);
        }
}
#if 0
void HW_ConfigDynaKey(zd_80211Obj_t *pObj, U16 aid, U8 *pMac, U8 *pKey, U8 keyLength, U8 encryType)
{
        //void *reg = pObj->reg;

        //set MAC address
        HW_CAM_SetMAC(pObj, aid, pMac);
        HW_CAM_SetEncryType(pObj, aid, encryType);
        HW_CAM_SetKey(pObj, aid, keyLength, pKey);
        HW_CAM_UpdateRollTbl(pObj, aid);
}
#endif
void HW_ConfigDynaKey(zd_80211Obj_t *pObj, U16 aid, U8 *pMac, U8 *pKey, U8 keyLength, U8 encryType, U8 change_enc)
{
        //set MAC address
        //	int	flags;
        //flags = pObj->EnterCS();
        HW_CAM_ClearRollTbl(pObj, aid);
        if (change_enc)
                //if (1)
        {
                HW_CAM_SetMAC(pObj, aid, pMac);
                HW_CAM_SetEncryType(pObj, aid, encryType);
        }
        HW_CAM_SetKey(pObj, aid, keyLength, pKey);
        HW_CAM_UpdateRollTbl(pObj, aid);
        //pObj->ExitCS(flags);
}

void HW_ConfigStatKey(zd_80211Obj_t *pObj, U8 *pKey, U8 keyLen, U32 startAddr)
{
        int i, j, k, offset;
        U32 tmpKey = 0;

        j = 0;
        offset = 0;

        while(offset < keyLen) {
                for (i=offset, k=0; k<4; i++, k++) {
                        tmpKey |= pKey[i] << ((k%4)*8);
                        offset++;
                        if (offset == keyLen)
                                goto last_part;
                }
                HW_CAM_Write(pObj, startAddr+j, tmpKey);
                j++;
                tmpKey = 0;
        }

last_part:
        HW_CAM_Write(pObj, startAddr+j, tmpKey);
}


void HW_GetStatKey(zd_80211Obj_t *pObj)
{
        //void *reg = pObj->reg;
        int i, j;
        U8 key[128];
        U32 tmpValue;
        U32 encryType;
        U8	keyLength;

        encryType = HW_CAM_Read(pObj, DEFAULT_ENCRY_TYPE);
        switch(encryType) {
        case WEP64:
                FPRINT("WEP64 Mode");

                keyLength = 5;
                break;

        case WEP128:
                FPRINT("WEP128 Mode");
                keyLength = 13;
                break;

        case WEP256:
                FPRINT("WEP256 Mode");
                keyLength = 29;
                break;

        default:
                FPRINT("Not supported Mode");
                return;

        }

        for (i=0, j=0; i<32; i++) {
                tmpValue = HW_CAM_Read(pObj, (STA_KEY_START_ADDR+i));

                key[j] = (U8)(tmpValue);
                j++;
                key[j] = (U8)(tmpValue >> 8);
                j++;
                key[j] = (U8)(tmpValue >> 16);
                j++;
                key[j] = (U8)(tmpValue >> 24);
                j++;
        }

        zd1205_dump_data("Key 1 = ", (U8 *)&key[0], keyLength);
        zd1205_dump_data("Key 2 = ", (U8 *)&key[32], keyLength);
        zd1205_dump_data("Key 3 = ", (U8 *)&key[2*32], keyLength);
        zd1205_dump_data("Key 4 = ", (U8 *)&key[3*32], keyLength);
        return;
}


void HW_EEPROM_ACCESS(zd_80211Obj_t *pObj, U8 RamAddr, U32 RomAddr, U32 length, U8 bWrite)
{
        void *reg = pObj->reg;
        U32 status;
        U32 access = 0;
        U32 startTime;
        U32 endTime;
        U32 diffTime;
        int count = 0;

        if (bWrite) {
                FPRINT("Write Access");
        } else
                FPRINT("Read Access");


        FPRINT_V("RomAddr", RomAddr);
        FPRINT_V("RamAddr", RamAddr);
        FPRINT_V("Length", length);

        if (bWrite) {
                access = EEPROM_WRITE_ACCESS;
                //unlock write access
                pObj->SetReg(reg, ZD_EPP_KEY_PROT, 0x55aa);
                pObj->SetReg(reg, ZD_EPP_KEY_PROT, 0x44bb);
                pObj->SetReg(reg, ZD_EPP_KEY_PROT, 0x33cc);
                pObj->SetReg(reg, ZD_EPP_KEY_PROT, 0x22dd);
        }

        pObj->SetReg(reg, ZD_EPP_ROM_ADDRESS, RomAddr);
        pObj->SetReg(reg, ZD_EPP_SRAM_ADDRESS, RamAddr);
        pObj->SetReg(reg, ZD_EPP_LENG_DIR, access | length);

        startTime = pObj->GetReg(reg, ZD_TSF_LowPart);

        pObj->DelayUs(2000);
        status = pObj->GetReg(reg, ZD_EPP_CLOCK_DIV);
        while(status & EEPROM_BUSY_FLAG) {
                pObj->DelayUs(1000);
                //FPRINT("EEPROM programming !!!");
                status = pObj->GetReg(reg, ZD_EPP_CLOCK_DIV);

                if (count > 500) {
                        FPRINT("EEPROM Timeout !!!");
                        if (bWrite)
                                pObj->SetReg(reg, ZD_EPP_KEY_PROT, 0x00);
                        return;
                }
                //len = pObj->GetReg(reg, ZD_EPP_LENG_DIR);
                //FPRINT_V("len", len);
                count++;
        }

        endTime = pObj->GetReg(reg, ZD_TSF_LowPart);
        if (endTime > startTime) {
                diffTime = endTime - startTime;
        } else {
                diffTime = 0xffffffff + startTime - endTime;
        }

        //FPRINT_V("Processing Time", diffTime);

        printk("\nProcessing Time = %lu ms\n", diffTime/1000);


        //lock write access
        if (bWrite)
                pObj->SetReg(reg, ZD_EPP_KEY_PROT, 0x00);
}

#endif
