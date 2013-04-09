#ifndef _ZD_ENCRYPT_H
#define _ZD_ENCRYPT_H

#if defined(PHY_1202)

void initWepState(void);

void zd_EncryptData (
        U8		Wep_Key_Len,
        U8*		Wep_Key,
        U8*		Wep_Iv,
        U16		Num_Bytes,
        U8*		Inbuf,
        U8*		Outbuf,
        U32*	Icv);


BOOLEAN zd_DecryptData (
        U8		Wep_Key_Len,
        U8* 	Wep_Key,
        U8* 	Wep_Iv,
        U16 	Num_Bytes,
        U8* 	Inbuf,
        U8* 	Outbuf,
        U32*	Icv);

#endif
#endif
