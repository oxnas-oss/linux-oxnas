#ifndef _zd1211_wext_h_
#define _zd1211_wext_h_


#if WIRELESS_EXT < 18

#define SIOCSIWGENIE	0x8B30
#define SIOCSIWMLME	0x8B16
#define SIOCSIWAUTH	0x8B32
#define SIOCSIWENCODEEXT 0x8B34

#define IW_AUTH_WPA_VERSION		0
#define IW_AUTH_CIPHER_PAIRWISE		1
#define IW_AUTH_CIPHER_GROUP		2
#define IW_AUTH_KEY_MGMT		3
#define IW_AUTH_TKIP_COUNTERMEASURES	4
#define IW_AUTH_DROP_UNENCRYPTED	5
#define IW_AUTH_80211_AUTH_ALG		6
#define IW_AUTH_WPA_ENABLED		7
#define IW_AUTH_RX_UNENCRYPTED_EAPOL	8
#define IW_AUTH_ROAMING_CONTROL		9
#define IW_AUTH_PRIVACY_INVOKED		10

#define IW_AUTH_WPA_VERSION_DISABLED	0x00000001
#define IW_AUTH_WPA_VERSION_WPA		0x00000002
#define IW_AUTH_WPA_VERSION_WPA2	0x00000004

#define IW_AUTH_CIPHER_NONE	0x00000001
#define IW_AUTH_CIPHER_WEP40	0x00000002
#define IW_AUTH_CIPHER_TKIP	0x00000004
#define IW_AUTH_CIPHER_CCMP	0x00000008
#define IW_AUTH_CIPHER_WEP104	0x00000010

#define IW_AUTH_KEY_MGMT_802_1X	1
#define IW_AUTH_KEY_MGMT_PSK	2

#define IW_AUTH_ALG_OPEN_SYSTEM	0x00000001
#define IW_AUTH_ALG_SHARED_KEY	0x00000002
#define IW_AUTH_ALG_LEAP	0x00000004

#define IW_AUTH_ROAMING_ENABLE	0
#define IW_AUTH_ROAMING_DISABLE	1

#define IW_ENCODE_SEQ_MAX_SIZE	8

#define IW_ENCODE_ALG_NONE	0
#define IW_ENCODE_ALG_WEP	1
#define IW_ENCODE_ALG_TKIP	2
#define IW_ENCODE_ALG_CCMP	3


struct	iw_encode_ext
{
	__u32		ext_flags; /* IW_ENCODE_EXT_* */
	__u8		tx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
	__u8		rx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
	struct sockaddr	addr; /* ff:ff:ff:ff:ff:ff for broadcast/multicast
			       * (group) keys or unicast address for
			       * individual keys */
	__u16		alg; /* IW_ENCODE_ALG_* */
	__u16		key_len;
	__u8		key[0];
};


struct	iw_mlme
{
	__u16		cmd; /* IW_MLME_* */
	__u16		reason_code;
	struct sockaddr	addr;
};

#endif // WIRELESS_EXT < 18


#endif // _zd1211_wext_h_
#ifndef _zd1211_wext_h_
#define _zd1211_wext_h_


#if WIRELESS_EXT < 18

#define SIOCSIWGENIE	0x8B30
#define SIOCSIWMLME	0x8B16
#define SIOCSIWAUTH	0x8B32
#define SIOCSIWENCODEEXT 0x8B34

#define IW_AUTH_WPA_VERSION		0
#define IW_AUTH_CIPHER_PAIRWISE		1
#define IW_AUTH_CIPHER_GROUP		2
#define IW_AUTH_KEY_MGMT		3
#define IW_AUTH_TKIP_COUNTERMEASURES	4
#define IW_AUTH_DROP_UNENCRYPTED	5
#define IW_AUTH_80211_AUTH_ALG		6
#define IW_AUTH_WPA_ENABLED		7
#define IW_AUTH_RX_UNENCRYPTED_EAPOL	8
#define IW_AUTH_ROAMING_CONTROL		9
#define IW_AUTH_PRIVACY_INVOKED		10

#define IW_AUTH_WPA_VERSION_DISABLED	0x00000001
#define IW_AUTH_WPA_VERSION_WPA		0x00000002
#define IW_AUTH_WPA_VERSION_WPA2	0x00000004

#define IW_AUTH_CIPHER_NONE	0x00000001
#define IW_AUTH_CIPHER_WEP40	0x00000002
#define IW_AUTH_CIPHER_TKIP	0x00000004
#define IW_AUTH_CIPHER_CCMP	0x00000008
#define IW_AUTH_CIPHER_WEP104	0x00000010

#define IW_AUTH_KEY_MGMT_802_1X	1
#define IW_AUTH_KEY_MGMT_PSK	2

#define IW_AUTH_ALG_OPEN_SYSTEM	0x00000001
#define IW_AUTH_ALG_SHARED_KEY	0x00000002
#define IW_AUTH_ALG_LEAP	0x00000004

#define IW_AUTH_ROAMING_ENABLE	0
#define IW_AUTH_ROAMING_DISABLE	1

#define IW_ENCODE_SEQ_MAX_SIZE	8

#define IW_ENCODE_ALG_NONE	0
#define IW_ENCODE_ALG_WEP	1
#define IW_ENCODE_ALG_TKIP	2
#define IW_ENCODE_ALG_CCMP	3


struct	iw_encode_ext
{
	__u32		ext_flags; /* IW_ENCODE_EXT_* */
	__u8		tx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
	__u8		rx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
	struct sockaddr	addr; /* ff:ff:ff:ff:ff:ff for broadcast/multicast
			       * (group) keys or unicast address for
			       * individual keys */
	__u16		alg; /* IW_ENCODE_ALG_* */
	__u16		key_len;
	__u8		key[0];
};


struct	iw_mlme
{
	__u16		cmd; /* IW_MLME_* */
	__u16		reason_code;
	struct sockaddr	addr;
};

#endif // WIRELESS_EXT < 18


#endif // _zd1211_wext_h_
