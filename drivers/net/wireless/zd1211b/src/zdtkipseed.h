#ifndef _ZDTKIPSEED_H_
#define _ZDTKIPSEED_H_


#define Mk16(A, B) 	((A<<8)+B)

#define Lo8(v16)  ((U8)( (v16) & 0xFF))
#define Hi8(v16)  ((U8)(((v16)>>8) & 0xFF))


//variable.
typedef struct _Seedvar
{
        U8		TK[16];		// Key
        U8		TA[6];
        U16		TTAK[5];	// TTAK
        U16		ppk[6];
        U16		IV16, iv16tmp;
        U32		IV32, iv32tmp;
}
Seedvar, *PSeedvar;

#define Tx  0
#define Rx  1

#if defined(PHY_1202)

void Tkip_Init(U8 *key, U8 *ta, Seedvar *Seed, U8 *initiv);
void Tkip_clear(Seedvar *Seed);

// set key and TA
void Tkip_setkey(U8 *key, U8 *ta, Seedvar *Seed);

// phase1 key mixing function
char Tkip_phase1_key_mix(U32 IV32, Seedvar *Seed);

// phase2 key mixing function
char Tkip_phase2_key_mix(U16 IV16, Seedvar *Seed);

// get generated seeds
void Tkip_getseeds(U16 IV16, U8 *RC4Key, Seedvar *Seed);

// update stored IV.
void Tkip_updateiv(Seedvar *Seed);


/************************************************************/
/* tkip_sbox()                                              */
/* Returns a 16 bit value from a 64K entry table. The Table */
/* is synthesized from two 256 entry byte wide tables.      */
/************************************************************/
unsigned int tkip_sbox(unsigned int index);

// rotate right by 1 bit.
unsigned int rotr1(unsigned int a);

#endif
#endif
