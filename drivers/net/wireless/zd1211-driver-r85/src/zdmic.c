#include "zd80211.h"
//
// Michael.cpp  Reference implementation for Michael
//
// Copyright (c) 2001 by MacFergus BV
// All rights reserved,
//


U32 getUInt32(U8 *p)
// Convert from U8[] to U32 in a portable way
{
        U32 res = 0;
        int i;

        for( i=0; i<4; i++ )
        {
                res |= (*p++) << (8*i);
        }

        return res;
}


void putUInt32(U8 *p, U32 val)
// Convert from U32 to U8[] in a portable way
{
        int i;

        for(i=0; i<4; i++)
        {
                *p++ = (U8)(val & 0xff);
                val >>= 8;
        }
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   MICclear                   */
/*      Initial variable require by MIC computation                    */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void MICclear(MICvar *MIC)
{
        // Reset the state to the empty message.
        MIC->L = MIC->K0;
        MIC->R = MIC->K1;
        MIC->nBytesInM = 0;
        MIC->M = 0;
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   MICsetKey                  */
/*      Set MIC key (Tx or Rx)                                         */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void MICsetKey(U8 *key, MICvar *MIC)
{
        // Set the key
        MIC->K0 = getUInt32(key);
        MIC->K1 = getUInt32(key + 4);
        if (MIC) {
#ifdef WPA_DEBUG
                printk(KERN_ERR "mic->K0= %08x K1=%08x\n", (unsigned)MIC->K0,(unsigned)MIC->K1);
#endif
        } else
                printk(KERN_ERR "pMic is NULL\n");
        // and reset the message
        MICclear(MIC);
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   MICappendByte              */
/*      Compute MIC for adding a single byte                           */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
void MICappendByte(U8 b, MICvar *MIC)
{
        register int nBytesInM = MIC->nBytesInM;
        register U32 M = MIC->M;

        // Append the byte to our word-sized buffer
        M |= b << (8* nBytesInM);
        nBytesInM ++;

        // Process the word if it is full.
        if (nBytesInM > 3) {
                register U32 L = MIC->L;
                register U32 R = MIC->R;

                L ^= M;
                R ^= ROL32(L, 17);
                L += R;
                R ^= ((L & 0xff00ff00) >> 8) | ((L & 0x00ff00ff) << 8);
                L += R;
                R ^= ROL32(L, 3);
                L += R;
                R ^= ROR32(L, 2);
                L += R;

                MIC->L = L;
                MIC->R = R;

                // Clear the buffer
                M = 0;
                nBytesInM = 0;
        }

        MIC->M = M;
        MIC->nBytesInM = nBytesInM;

}


void MICappendArr(U8 *pb, MICvar *MIC, U32 Size)
{
        int	i;
        U8 *pB = pb;
        // Append the byte to our word-sized buffer

        for (i=0; i<Size; i++) {
                MIC->M |= *pB << (8 * MIC->nBytesInM);
                MIC->nBytesInM++;

                // Process the word if it is full.
                if(MIC->nBytesInM >= 4) {
                        MIC->L ^= MIC->M;
                        MIC->R ^= ROL32(MIC->L, 17);
                        MIC->L += MIC->R;
                        MIC->R ^= ((MIC->L & 0xff00ff00) >> 8) | ((MIC->L & 0x00ff00ff) << 8);
                        MIC->L += MIC->R;
                        MIC->R ^= ROL32(MIC->L, 3);
                        MIC->L += MIC->R;
                        MIC->R ^= ROR32(MIC->L, 2);
                        MIC->L += MIC->R;

                        // Clear the buffer
                        MIC->M = 0;
                        MIC->nBytesInM = 0;
                }
                pB++;
        }
}


/***********************************************************************/
/*                                                                     */
/*   FUNCTION DESCRIPTION                   MICappendByte              */
/*      Compute MIC for adding a single byte                           */
/*                                                                     */
/*   AUTHOR                                                            */
/*     Liam,Hwu            ZyDAS Technology   Corporation              */
/*                                                                     */
/***********************************************************************/
U8	MicTailPadding[]={0x5a, 0, 0, 0, 0, 0, 0, 0, 0};

void MICgetMIC(U8 *dst, MICvar *MIC)
{
        // Append the minimum padding
        //MICappendArr(MicTailPadding, MIC, 5+((4 -((MIC->nBytesInM+5) & 3)) & 3));

        MICappendByte(0x5a, MIC);
        MICappendByte(0, MIC);
        MICappendByte(0, MIC);
        MICappendByte(0, MIC);
        MICappendByte(0, MIC);

        // and then zeroes until the length is a multiple of 4
        while( MIC->nBytesInM != 0 ) {
                MICappendByte(0, MIC);
        }

        // The appendByte function has already computed the result.
        putUInt32(dst, MIC->L);
        putUInt32(dst+4, MIC->R);

        // Reset to the empty message.
        MICclear(MIC);
}
