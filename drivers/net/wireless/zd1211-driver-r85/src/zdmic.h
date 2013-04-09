/*
   $Id: zdmic.h,v 1.1.1.1 2005/11/25 08:04:38 mzcai Exp $
*/
/***********************************************************************/
/*          Copyright 2003 by ZyDAS Technology     Corporation         */
/*                                                                     */
/***********************************************************************/
/***********************************************************************/
/*                                                                     */
/*    FILE DESCRIPTION                          MIChael.h              */
/*      Header file required for TKIP  MIC                             */
/*                                                                     */
/*    ROUTINES                                                         */
/*      XXX                                                            */
/*                                                                     */
/*    NOTES                                                            */
/*      XXX                                                            */
/*                                                                     */
/*    MAINTAINER                                                       */
/*      Liam,Hwu            ZyDAS Technology   Corporation     2003    */
/*                                                                     */
/***********************************************************************/
#ifndef _ZDMIC_H_
#define _ZDMIC_H_


// Rotation functions on 32 bit values
#define ROL32(A, n) \
		( ((A) << (n)) | ( ((A)>>(32-(n)))  & ( (1UL << (n)) - 1 ) ) )

#define ROR32(A, n) ROL32( (A), 32-(n) )


//variable.
typedef struct
{
        U32  K0, K1;	// Key
        U32  L, R;   	// Current state
        U32  M;      	// Message accumulator (single word)
        int  nBytesInM; // # bytes in M
}
MICvar;			// variable for MIC


// Clear the internal message,
// resets the object to the state just after construction.
void MICclear(MICvar *MIC);


// Set the key to a new value
void MICsetKey(U8 *key, MICvar *MIC);


// Get the MIC result. Destination should accept 8 bytes of result.
// This also resets the message to empty.

void MICgetMIC(U8 *dst, MICvar *MIC);


// Add a single byte to the internal message
void MICappendByte(U8 b, MICvar *MIC);
void MICappendArr(U8 *pb, MICvar *MIC, U32 Size);


// Get U32 from 4 bytes LSByte first
U32 getUInt32(U8 *p);


// Put U32 into 4 bytes LSByte first
void putUInt32(U8 *p, U32 val);

#endif
