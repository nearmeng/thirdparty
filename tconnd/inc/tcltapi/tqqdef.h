/********************************************************************
**       This head file is generated by program,                   **
**            Please do not change it directly.                    **
********************************************************************/

#ifndef TQQDEF_H_
#define TQQDEF_H_


/*   Define c types.   */
#ifndef TDR_CUSTOM_C_TYPES
#define TDR_CUSTOM_C_TYPES
    #include <stddef.h>
    #include <time.h>
    #include <sys/types.h>

    #if !defined(_WIN32) && !defined(_WIN64)

        #include <stdint.h>
        #include <inttypes.h>

    #else /*if !defined(_WIN32) && !defined(_WIN64)*/

        //The stdint declaras
        typedef  signed char  int8_t;
        typedef  short int16_t;
        typedef  int   int32_t;
        typedef unsigned char  uint8_t;
        typedef unsigned short uint16_t;
        typedef unsigned int   uint32_t;
        #if _MSC_VER >= 1300
            typedef unsigned long long 	uint64_t;
            typedef long long 	int64_t;
        #else /* _MSC_VER */
            typedef unsigned __int64	uint64_t;
            typedef __int64	int64_t;
        #endif /* _MSC_VER */

    #endif /*if !defined(_WIN32) && !defined(_WIN64)*/

    typedef int64_t tdr_longlong;
    typedef uint64_t tdr_ulonglong;
    typedef uint16_t tdr_wchar_t;  /**<Wchar基本数据类型*/
    typedef uint32_t tdr_date_t;	/**<data基本数据类型*/
    typedef uint32_t tdr_time_t;	/**<time基本数据类型*/
    typedef uint64_t tdr_datetime_t; /**<datetime基本数据类型*/
    typedef uint32_t tdr_ip_t;  /**<IPv4数据类型*/
#endif /*TDR_CUSTOM_C_TYPES*/


/*   User defined includes.   */


/*   User defined macros.   */
#define TQQ_KEY_LEN                                      	16
#define TQQ_MAX_NAME_LEN                                 	32
#define TQQ_MAX_IDENT_LEN                                	16
#define TQQ_MAX_SIGN_LEN                                 	128
#define TQQ_MAX_SIGN2_LEN                                	64
#define TQQ_IDENT_LEN                                    	16
#define TQQ_MAX_USERIDENT_LEN                            	64
#define TQQ_SVCBITMAP_LEN                                	12
#define TQQ_SVCBITMAP_EXT_LEN                            	8
#define TQQ_UNIFIED_MAX_SIGN_LEN                         	256
#define TQQ_AP_MAX_SIGN_LEN                              	1024
#define TQQ_UNIFIED_MAX_ENCSIGN_LEN                      	128
#define TQQ_UNIFIED_MAX_ENCSIGN2_LEN                     	32
#define TQQ_UNIFIED_CUSTOMINFO_LEN                       	32


/*   Structs/unions prototype.   */
struct tagTQQGameSig;
typedef struct tagTQQGameSig                                       	TQQGAMESIG;
typedef struct tagTQQGameSig                                       	*LPTQQGAMESIG;

struct tagTQQSigForS2;
typedef struct tagTQQSigForS2                                      	TQQSIGFORS2;
typedef struct tagTQQSigForS2                                      	*LPTQQSIGFORS2;

struct tagTQQUserIdent;
typedef struct tagTQQUserIdent                                     	TQQUSERIDENT;
typedef struct tagTQQUserIdent                                     	*LPTQQUSERIDENT;

struct tagTQQAuthInfo;
typedef struct tagTQQAuthInfo                                      	TQQAUTHINFO;
typedef struct tagTQQAuthInfo                                      	*LPTQQAUTHINFO;

struct tagTQQUnifiedSig;
typedef struct tagTQQUnifiedSig                                    	TQQUNIFIEDSIG;
typedef struct tagTQQUnifiedSig                                    	*LPTQQUNIFIEDSIG;

struct tagTQQUnifiedEncrySig;
typedef struct tagTQQUnifiedEncrySig                               	TQQUNIFIEDENCRYSIG;
typedef struct tagTQQUnifiedEncrySig                               	*LPTQQUNIFIEDENCRYSIG;

struct tagTQQUnifiedAuthInfo;
typedef struct tagTQQUnifiedAuthInfo                               	TQQUNIFIEDAUTHINFO;
typedef struct tagTQQUnifiedAuthInfo                               	*LPTQQUNIFIEDAUTHINFO;

struct tagTApAuthInfo;
typedef struct tagTApAuthInfo                                      	TAPAUTHINFO;
typedef struct tagTApAuthInfo                                      	*LPTAPAUTHINFO;

/*   Define structs/unions.   */
#pragma pack(1)

/* 0x82签名第一段 */
struct tagTQQGameSig
{
    char szGameKey[TQQ_KEY_LEN];                     
    char szSvcBitmap[TQQ_SVCBITMAP_LEN];             
    char szSvcBitmapExt[TQQ_SVCBITMAP_EXT_LEN];      
    uint32_t dwValidateBitmap;                       
    uint32_t dwUin;                                  
    uint32_t dwTime;                                 
    uint32_t dwUinFlag;                              
    uint32_t dwClientIP;                              	/*  Ver.12 客户端IP */
};

/* 0x82签名第二段,一般不用 */
struct tagTQQSigForS2
{
    int8_t chType;                                   
    uint32_t dwValidateBitmap;                       
    uint32_t dwUin;                                  
    uint32_t dwTime;                                 
    uint32_t dwCltIP;                                
};

/* 重连验证字段 */
struct tagTQQUserIdent
{
    uint32_t dwUin;                                  
    int32_t iPos;                                    
    char szIdent[TQQ_IDENT_LEN];                     
};

/* 0X82签名协议 */
struct tagTQQAuthInfo
{
    uint32_t dwUin;                                  
    uint8_t bSignLen;                                
    char szSignData[TQQ_MAX_SIGN_LEN];                	/*   TQQGameSig的密文 */
    uint8_t bSign2Len;                               
    char szSign2Data[TQQ_MAX_SIGN2_LEN];              	/*   一般不用 */
};

/* 0XDE签名协议格式 */
struct tagTQQUnifiedSig
{
    int16_t nVersion;                                
    uint32_t dwTime;                                 
    int16_t nEncryptSignLen;                         
    char szEncryptSignData[TQQ_UNIFIED_MAX_ENCSIGN_LEN]; 	/*   TQQUnifiedEncrySig的密文 */
};

/* 0xDE签名协议加密部分 */
struct tagTQQUnifiedEncrySig
{
    int32_t iRandom;                                 
    int16_t nVersion;                                
    uint32_t dwUin;                                  
    uint32_t dwTime;                                 
    uint32_t dwSSOver;                               
    int32_t iAppID;                                  
    int32_t iAppClientVer;                           
    uint32_t dwClientIP;                             
    char szSessionKey[TQQ_KEY_LEN];                  
    int16_t nUnifiedSig2Len;                         
    char szUnifiedSig2[TQQ_UNIFIED_MAX_ENCSIGN2_LEN];
    int16_t nCustomInfoLen;                          
    char szCustomInfoData[TQQ_UNIFIED_CUSTOMINFO_LEN];
};

/* 0XDE签名协议 */
struct tagTQQUnifiedAuthInfo
{
    uint32_t dwUin;                                  
    uint8_t bLen;                                    
    char szSigInfo[TQQ_UNIFIED_MAX_SIGN_LEN];         	/*   TQQUnifiedSig的网络字节流 */
};

/* 0X66签名协议 */
struct tagTApAuthInfo
{
    uint32_t dwUin;                                  
    uint32_t dwLen;                                  
    uint8_t szSigInfo[TQQ_AP_MAX_SIGN_LEN];          
};

#pragma pack()


#endif /* TQQDEF_H_ */
