#include "zd80211.h"

#if!defined(AMAC)

/* The Sbox is reduced to 2 16-bit wide tables, each with 256 entries. */
/* The 2nd table is the same as the 1st but with the upper and lower   */
/* bytes swapped. To allow an endian tolerant implementation, the byte */
/* halves have been expressed independently here.                      */
unsigned int Tkip_Sbox_Lower[256] =
        {
                0xA5,0x84,0x99,0x8D,0x0D,0xBD,0xB1,0x54,
                0x50,0x03,0xA9,0x7D,0x19,0x62,0xE6,0x9A,
                0x45,0x9D,0x40,0x87,0x15,0xEB,0xC9,0x0B,
                0xEC,0x67,0xFD,0xEA,0xBF,0xF7,0x96,0x5B,
                0xC2,0x1C,0xAE,0x6A,0x5A,0x41,0x02,0x4F,
                0x5C,0xF4,0x34,0x08,0x93,0x73,0x53,0x3F,
                0x0C,0x52,0x65,0x5E,0x28,0xA1,0x0F,0xB5,
                0x09,0x36,0x9B,0x3D,0x26,0x69,0xCD,0x9F,
                0x1B,0x9E,0x74,0x2E,0x2D,0xB2,0xEE,0xFB,
                0xF6,0x4D,0x61,0xCE,0x7B,0x3E,0x71,0x97,
                0xF5,0x68,0x00,0x2C,0x60,0x1F,0xC8,0xED,
                0xBE,0x46,0xD9,0x4B,0xDE,0xD4,0xE8,0x4A,
                0x6B,0x2A,0xE5,0x16,0xC5,0xD7,0x55,0x94,
                0xCF,0x10,0x06,0x81,0xF0,0x44,0xBA,0xE3,
                0xF3,0xFE,0xC0,0x8A,0xAD,0xBC,0x48,0x04,
                0xDF,0xC1,0x75,0x63,0x30,0x1A,0x0E,0x6D,
                0x4C,0x14,0x35,0x2F,0xE1,0xA2,0xCC,0x39,
                0x57,0xF2,0x82,0x47,0xAC,0xE7,0x2B,0x95,
                0xA0,0x98,0xD1,0x7F,0x66,0x7E,0xAB,0x83,
                0xCA,0x29,0xD3,0x3C,0x79,0xE2,0x1D,0x76,
                0x3B,0x56,0x4E,0x1E,0xDB,0x0A,0x6C,0xE4,
                0x5D,0x6E,0xEF,0xA6,0xA8,0xA4,0x37,0x8B,
                0x32,0x43,0x59,0xB7,0x8C,0x64,0xD2,0xE0,
                0xB4,0xFA,0x07,0x25,0xAF,0x8E,0xE9,0x18,
                0xD5,0x88,0x6F,0x72,0x24,0xF1,0xC7,0x51,
                0x23,0x7C,0x9C,0x21,0xDD,0xDC,0x86,0x85,
                0x90,0x42,0xC4,0xAA,0xD8,0x05,0x01,0x12,
                0xA3,0x5F,0xF9,0xD0,0x91,0x58,0x27,0xB9,
                0x38,0x13,0xB3,0x33,0xBB,0x70,0x89,0xA7,
                0xB6,0x22,0x92,0x20,0x49,0xFF,0x78,0x7A,
                0x8F,0xF8,0x80,0x17,0xDA,0x31,0xC6,0xB8,
                0xC3,0xB0,0x77,0x11,0xCB,0xFC,0xD6,0x3A};


unsigned int Tkip_Sbox_Upper[256] =
        {
                0xC6,0xF8,0xEE,0xF6,0xFF,0xD6,0xDE,0x91,
                0x60,0x02,0xCE,0x56,0xE7,0xB5,0x4D,0xEC,
                0x8F,0x1F,0x89,0xFA,0xEF,0xB2,0x8E,0xFB,
                0x41,0xB3,0x5F,0x45,0x23,0x53,0xE4,0x9B,
                0x75,0xE1,0x3D,0x4C,0x6C,0x7E,0xF5,0x83,
                0x68,0x51,0xD1,0xF9,0xE2,0xAB,0x62,0x2A,
                0x08,0x95,0x46,0x9D,0x30,0x37,0x0A,0x2F,
                0x0E,0x24,0x1B,0xDF,0xCD,0x4E,0x7F,0xEA,
                0x12,0x1D,0x58,0x34,0x36,0xDC,0xB4,0x5B,
                0xA4,0x76,0xB7,0x7D,0x52,0xDD,0x5E,0x13,
                0xA6,0xB9,0x00,0xC1,0x40,0xE3,0x79,0xB6,
                0xD4,0x8D,0x67,0x72,0x94,0x98,0xB0,0x85,
                0xBB,0xC5,0x4F,0xED,0x86,0x9A,0x66,0x11,
                0x8A,0xE9,0x04,0xFE,0xA0,0x78,0x25,0x4B,
                0xA2,0x5D,0x80,0x05,0x3F,0x21,0x70,0xF1,
                0x63,0x77,0xAF,0x42,0x20,0xE5,0xFD,0xBF,
                0x81,0x18,0x26,0xC3,0xBE,0x35,0x88,0x2E,
                0x93,0x55,0xFC,0x7A,0xC8,0xBA,0x32,0xE6,
                0xC0,0x19,0x9E,0xA3,0x44,0x54,0x3B,0x0B,
                0x8C,0xC7,0x6B,0x28,0xA7,0xBC,0x16,0xAD,
                0xDB,0x64,0x74,0x14,0x92,0x0C,0x48,0xB8,
                0x9F,0xBD,0x43,0xC4,0x39,0x31,0xD3,0xF2,
                0xD5,0x8B,0x6E,0xDA,0x01,0xB1,0x9C,0x49,
                0xD8,0xAC,0xF3,0xCF,0xCA,0xF4,0x47,0x10,
                0x6F,0xF0,0x4A,0x5C,0x38,0x57,0x73,0x97,
                0xCB,0xA1,0xE8,0x3E,0x96,0x61,0x0D,0x0F,

                0xE0,0x7C,0x71,0xCC,0x90,0x06,0xF7,0x1C,
                0xC2,0x6A,0xAE,0x69,0x17,0x99,0x3A,0x27,
                0xD9,0xEB,0x2B,0x22,0xD2,0xA9,0x07,0x33,
                0x2D,0x3C,0x15,0xC9,0x87,0xAA,0x50,0xA5,
                0x03,0x59,0x09,0x1A,0x65,0xD7,0x84,0xD0,
                0x82,0x29,0x5A,0x1E,0x7B,0xA8,0x6D,0x2C};


unsigned int tkip_sbox(unsigned int index)
/************************************************************/
/* tkip_sbox()                                              */
/* Returns a 16 bit value from a 64K entry table. The Table */
/* is synthesized from two 256 entry byte wide tables.      */
/************************************************************/
{
        unsigned int index_low;
        unsigned int index_high;
        unsigned int left, right;

        index_low = (index & 0xFF);
        index_high = ((index >> 8) & 0xFF);

        left = Tkip_Sbox_Lower[index_low] + (Tkip_Sbox_Upper[index_low] << 8);
        right = Tkip_Sbox_Upper[index_high] + (Tkip_Sbox_Lower[index_high] << 8);
        return (left ^ right);
}


unsigned int rotr1(unsigned int a)
// rotate right by 1 bit.
{
        unsigned int b;

        if (a & 0x01)
        {
                b = (a >> 1) | 0x8000;
        } else
        {
                b = (a >> 1) & 0x7fff;
        }

        return b;
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_clear                 */
/*                                                                     */
/*   clear all variable used in tkip seed generation function          */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void Tkip_clear(Seedvar *Seed)
{
        memset(Seed, 0, sizeof(Seedvar));
        //	Seed->IV16=1; // According to WPA std Ver2.0 8.3.2.4.4, TSC should be
        // initialized to one.
        return;
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_setkey                */
/*                                                                     */
/*   Set Temporal key(TK) and Transmitter Address(TA)                  */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void Tkip_setkey(U8 * key, U8 *ta, Seedvar *Seed)
{
        memcpy(Seed->TA, ta, 6);
        memcpy(Seed->TK, key, 16);
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_Init                  */
/*                                                                     */
/*   TKIP seed generation function initialization routine              */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void Tkip_Init(U8 *key, U8 *ta, Seedvar *Seed, U8 *initiv)
{
        U16  iv16;
        U32 iv32;
        int i;

        Tkip_clear(Seed);
        Tkip_setkey(key, ta, Seed);

        iv16 = *initiv++;
        iv16 += *initiv << 8;
        initiv++;
        iv32 = 0;

        for (i=0; i<4; i++) {	// initiv is little endian
                iv32 += *initiv << (i*8);
                *initiv++;
        }

        Seed->IV32 = iv32+1; // Force Recaculating on Tkip Phase1
        Tkip_phase1_key_mix(iv32, Seed);

        Seed->IV16 = iv16;
        Seed->IV32 = iv32;
}



/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_phase1_key_mix        */
/*                                                                     */
/*   TKIP seed generation function for phase1 key mixing               */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*   Return value:													   */
/*		Return TRUE if expected IV sequence is correct.	               */
/*                                                                     */
/***********************************************************************/
char Tkip_phase1_key_mix(U32 iv32, Seedvar *Seed)
{
        unsigned short tsc0;
        unsigned short tsc1;
        int i, j;

        if (iv32 == Seed->IV32)    //don't need to proceed this function the same
                return 1;
        else {
                tsc0 = (unsigned short)((iv32 >> 16) & 0xffff); /* msb */
                tsc1 = (unsigned short)(iv32 & 0xffff);

                /* Phase 1, step 1 */
                Seed->TTAK[0] = tsc1;
                Seed->TTAK[1] = tsc0;
                Seed->TTAK[2] = (unsigned short)(Seed->TA[0] + (Seed->TA[1] <<8));
                Seed->TTAK[3] = (unsigned short)(Seed->TA[2] + (Seed->TA[3] <<8));
                Seed->TTAK[4] = (unsigned short)(Seed->TA[4] + (Seed->TA[5] <<8));

                /* Phase 1, step 2 */
                for (i=0; i<8; i++) {
                        j = 2*(i & 1);
                        Seed->TTAK[0] = (Seed->TTAK[0] + tkip_sbox(Seed->TTAK[4]
                                         ^ Mk16(Seed->TK[1+j],Seed->TK[j]))) & 0xffff;
                        Seed->TTAK[1] = (Seed->TTAK[1] + tkip_sbox(Seed->TTAK[0]
                                         ^ Mk16(Seed->TK[5+j],Seed->TK[4+j] ))) & 0xffff;
                        Seed->TTAK[2] = (Seed->TTAK[2] + tkip_sbox(Seed->TTAK[1]
                                         ^ Mk16(Seed->TK[9+j],Seed->TK[8+j] ))) & 0xffff;
                        Seed->TTAK[3] = (Seed->TTAK[3] + tkip_sbox(Seed->TTAK[2]
                                         ^ Mk16(Seed->TK[13+j],Seed->TK[12+j])))& 0xffff;
                        Seed->TTAK[4] = (Seed->TTAK[4] + tkip_sbox(Seed->TTAK[3]
                                         ^ Mk16(Seed->TK[1+j] ,Seed->TK[j]  ))) & 0xffff;
                        Seed->TTAK[4] = (Seed->TTAK[4] + i) & 0xffff;
                }

                if (iv32 == (Seed->IV32+1)) {
                        Seed->iv32tmp = iv32;
                        return 1;
                } else
                        return 0;
        }
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_phase2_key_mix        */
/*                                                                     */
/*   TKIP seed generation function for phase2 key mixing               */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*   Return value:													   */
/*		Return TRUE if expected IV sequence is correct.	               */
/*                                                                     */
/***********************************************************************/
char Tkip_phase2_key_mix(U16 iv16, Seedvar *Seed)
{
        unsigned int tsc2;

        tsc2 = iv16;
        /* Phase 2, Step 1 */
        Seed->ppk[0] = Seed->TTAK[0];
        Seed->ppk[1] = Seed->TTAK[1];
        Seed->ppk[2] = Seed->TTAK[2];
        Seed->ppk[3] = Seed->TTAK[3];
        Seed->ppk[4] = Seed->TTAK[4];
        Seed->ppk[5] = (Seed->TTAK[4] + tsc2) & 0xffff;

        /* Phase2, Step 2 */
        Seed->ppk[0] = Seed->ppk[0] + tkip_sbox(Seed->ppk[5] ^ Mk16(Seed->TK[1],Seed->TK[0])) ;
        Seed->ppk[1] = Seed->ppk[1] + tkip_sbox(Seed->ppk[0] ^ Mk16(Seed->TK[3],Seed->TK[2])) ;
        Seed->ppk[2] = Seed->ppk[2]	+ tkip_sbox(Seed->ppk[1] ^ Mk16(Seed->TK[5],Seed->TK[4])) ;
        Seed->ppk[3] = Seed->ppk[3]	+ tkip_sbox(Seed->ppk[2] ^ Mk16(Seed->TK[7],Seed->TK[6])) ;
        Seed->ppk[4] = Seed->ppk[4]	+ tkip_sbox(Seed->ppk[3] ^ Mk16(Seed->TK[9],Seed->TK[8] )) ;
        Seed->ppk[5] = Seed->ppk[5]	+ tkip_sbox(Seed->ppk[4] ^ Mk16(Seed->TK[11],Seed->TK[10])) ;

        Seed->ppk[0] = Seed->ppk[0]	+ rotr1(Seed->ppk[5] ^ Mk16(Seed->TK[13],Seed->TK[12]));
        Seed->ppk[1] = Seed->ppk[1]	+ rotr1(Seed->ppk[0] ^ Mk16(Seed->TK[15],Seed->TK[14]));
        Seed->ppk[2] = Seed->ppk[2] + rotr1(Seed->ppk[1]);
        Seed->ppk[3] = Seed->ppk[3] + rotr1(Seed->ppk[2]);
        Seed->ppk[4] = Seed->ppk[4] + rotr1(Seed->ppk[3]);
        Seed->ppk[5] = Seed->ppk[5] + rotr1(Seed->ppk[4]);

        if (iv16 == 0) {
                if (Seed->IV16 == 0xffff) {
                        Seed->iv16tmp = 0;
                        return 1;
                } else
                        return 0;
        } else if (iv16 == (Seed->IV16+1)) {
                Seed->iv16tmp = iv16;
                return 1;
        } else
                return 0;
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_getseeds              */
/*                                                                     */

/*   Get RC4Key seeds generated by TKIP                                */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void Tkip_getseeds(U16 iv16, U8 *RC4Key, Seedvar *Seed)
{
        RC4Key[0]  = Hi8(iv16);
        RC4Key[1]  = (Hi8(iv16) | 0x20) & 0x7f;
        RC4Key[2]  = Lo8(iv16);
        RC4Key[3]  = ((Seed->ppk[5] ^ Mk16(Seed->TK[1],Seed->TK[0]))>>1) & 0xff;
        RC4Key[4]  = Seed->ppk[0] & 0xff;
        RC4Key[5]  = Seed->ppk[0] >> 8  ;
        RC4Key[6]  = Seed->ppk[1] & 0xff;
        RC4Key[7]  = Seed->ppk[1] >> 8  ;
        RC4Key[8]  = Seed->ppk[2] & 0xff;
        RC4Key[9]  = Seed->ppk[2] >> 8  ;
        RC4Key[10] = Seed->ppk[3] & 0xff;
        RC4Key[11] = Seed->ppk[3] >> 8  ;
        RC4Key[12] = Seed->ppk[4] & 0xff;
        RC4Key[13] = Seed->ppk[4] >> 8  ;
        RC4Key[14] = Seed->ppk[5] & 0xff;
        RC4Key[15] = Seed->ppk[5] >> 8  ;
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   Tkip_updateiv              */
/*                                                                     */
/*   update stored iv value                                            */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void Tkip_updateiv(Seedvar *Seed)
{
        Seed->IV16 = Seed->iv16tmp;
        Seed->IV32 = Seed->iv32tmp;
}
#endif
