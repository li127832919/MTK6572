#include "AudioBTCVSDControl.h"
 
#if defined(__MSBC_CODEC_SUPPORT__)
extern "C" {
   #include "cvsd_codec_exp_v2.h"
   #include "msbc_codec_exp.h"
}   
#endif

#include <utils/Log.h>

#define LOG_TAG  "AudioBTCVSDControl"

namespace android
{

#ifdef BT_SW_CVSD

#define BTSCO_CVSD_SAMPLERATE_DOMAIN 64000
#define BTSCO_MSBC_SAMPLERATE_DOMAIN 16000
#define BTSCO_CVSD_PLC_SAMPLERATE 8000
#define BTSCO_CVSD_CHANNEL_NUM 1
#define BTSCO_MSBC_CHANNEL_NUM 1

static const int32_t  btsco_FilterCoeff_64K[4]={0x07F54A0C, 0xF017CB31, 0x07F2EB36, 0x7F41C0BE};
static const int32_t  btsco_FilterCoeff_8K[4] ={0x07AB676D, 0xF0BB80B9, 0x079933A1, 0x79F9C5B0};

AudioBTCVSDControlInterface *AudioBTCVSDControlInterface::UniqueAudioBTCVSDControlInterface = NULL;

AudioBTCVSDControlInterface *AudioBTCVSDControlInterface::getInstance()
{
    if (UniqueAudioBTCVSDControlInterface == NULL) {
        ALOGD("+AudioBTCVSDControlInterface");
        UniqueAudioBTCVSDControlInterface = new AudioBTCVSDControlInterface();
#if defined(__MSBC_CODEC_SUPPORT__)
        UniqueAudioBTCVSDControlInterface->BTmode = false;
#endif
        ALOGD("-AudioBTCVSDControlInterface");
    }
    ALOGD("AudioBTCVSDControlInterface getInstance()");
    return UniqueAudioBTCVSDControlInterface;
}

void AudioBTCVSDControlInterface::freeInstance()
{
	if (UniqueAudioBTCVSDControlInterface != NULL) {
        delete UniqueAudioBTCVSDControlInterface;
    }
    ALOGD("AudioBTCVSDControlInterface freeInstance()");
}

AudioBTCVSDControlInterface::AudioBTCVSDControlInterface()
{
    ALOGD("AudioBTCVSDControlInterface constructor");    
}

AudioBTCVSDControlInterface::~AudioBTCVSDControlInterface()
{
    ALOGD("AudioBTCVSDControlInterface destructor");

}

void AudioBTCVSDControlInterface::BT_SCO_CVSD_Init(void)
{
	mBTSCOCVSDContext = NULL;
	mBTSCOCVSDContext = (BTSCO_CVSD_Context *)new char[sizeof(BTSCO_CVSD_Context)];
	ASSERT(mBTSCOCVSDContext);
	memset((void *)mBTSCOCVSDContext, 0, sizeof(BTSCO_CVSD_Context));
    ALOGD("BT_SCO_CVSD_Init() allocate mBTSCOCVSDContext");
}

void AudioBTCVSDControlInterface::BT_SCO_CVSD_DeInit(void)
{
	if (mBTSCOCVSDContext) {
    	delete []mBTSCOCVSDContext;
    	mBTSCOCVSDContext = NULL;
		ALOGD("BT_SCO_CVSD_DeInit() release mBTSCOCVSDContext");
	}	
}

#if defined(__MSBC_CODEC_SUPPORT__)
void AudioBTCVSDControlInterface::BT_SCO_SetMode(uint32_t mode)
{
   if (mode == 1) {
      UniqueAudioBTCVSDControlInterface->BTmode = true;
   } else {
      UniqueAudioBTCVSDControlInterface->BTmode = false;
   }
   
   if (mBTSCOCVSDContext)
      mBTSCOCVSDContext->fIsWideBand = UniqueAudioBTCVSDControlInterface->BTmode;
}

bool AudioBTCVSDControlInterface::BT_SCO_isWideBand(void)
{
   return UniqueAudioBTCVSDControlInterface->BTmode;
}
#endif

uint32_t AudioBTCVSDControlInterface::BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MODULE uModule)
{
   uint32_t uSize;
   switch(uModule)
   {
      case BT_SCO_MOD_CVSD_ENCODE: 
         uSize = (uint32_t)CVSD_ENC_GetBufferSize();
         break;
      case BT_SCO_MOD_CVSD_DECODE: 
         uSize = (uint32_t)CVSD_DEC_GetBufferSize();
         break;
      case BT_SCO_MOD_FILTER_TX: 
      case BT_SCO_MOD_FILTER_RX: 
         uSize = (uint32_t)Audio_IIRHPF_GetBufferSize(1);
         break;
#if defined(__MSBC_CODEC_SUPPORT__)
      case BT_SCO_MOD_PLC_NB:
      case BT_SCO_MOD_PLC_WB:
         uSize = (uint32_t)g711plc_GetMemorySize_v2();
         break;
#else
      case BT_SCO_MOD_PLC:
         uSize = (uint32_t)g711plc_GetMemorySize();
         break;
#endif
      case BT_SCO_MOD_CVSD_TX_SRC:
         BLI_GetMemSize(mBTSCOCVSDContext->pTX->uSampleRate,mBTSCOCVSDContext->pTX->uChannelNumber, BTSCO_CVSD_SAMPLERATE_DOMAIN, BTSCO_CVSD_CHANNEL_NUM, &uSize);
         break;
      case BT_SCO_MOD_CVSD_RX_SRC1:
         BLI_GetMemSize(BTSCO_CVSD_SAMPLERATE_DOMAIN, BTSCO_CVSD_CHANNEL_NUM, BTSCO_CVSD_PLC_SAMPLERATE, BTSCO_CVSD_CHANNEL_NUM, &uSize);
         break;
      case BT_SCO_MOD_CVSD_RX_SRC2:
         BLI_GetMemSize(BTSCO_CVSD_PLC_SAMPLERATE, BTSCO_CVSD_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber,&uSize);
         break;
#if defined(__MSBC_CODEC_SUPPORT__)
      case BT_SCO_MOD_MSBC_ENCODE: 
         uSize = (uint32_t)MSBC_ENC_GetBufferSize();
         break;
      case BT_SCO_MOD_MSBC_DECODE: 
         uSize = (uint32_t)MSBC_DEC_GetBufferSize();
         break;
      case BT_SCO_MOD_MSBC_TX_SRC:
         BLI_GetMemSize(mBTSCOCVSDContext->pTX->uSampleRate, mBTSCOCVSDContext->pTX->uChannelNumber, BTSCO_MSBC_SAMPLERATE_DOMAIN, BTSCO_MSBC_CHANNEL_NUM, &uSize);
         break;
      case BT_SCO_MOD_MSBC_RX_SRC:
         BLI_GetMemSize(BTSCO_MSBC_SAMPLERATE_DOMAIN, BTSCO_MSBC_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate, mBTSCOCVSDContext->pRX->uChannelNumber, &uSize);
         break;
#endif
      default:
         ASSERT(0);
   }
   uSize = (uSize+3) & ~3 ;
   ALOGD("BT_SCO_GetMemorySize_4ByteAlign uModule=%d, uSize=%d",uModule,uSize);
   return uSize;
}

void AudioBTCVSDControlInterface::BT_SCO_TX_DestroyModule(void)
{
   if(mBTSCOCVSDContext->pTX)
   {
      if(mBTSCOCVSDContext->pTX->pSRCHandle)
      {
      	#ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
         BLI_Close(mBTSCOCVSDContext->pTX->pSRCHandle, NULL);
		#else
		 BLI_Close(mBTSCOCVSDContext->pTX->pSRCHandle);
		#endif
         mBTSCOCVSDContext->pTX->pSRCHandle = NULL;
      }
   }
}

void AudioBTCVSDControlInterface::BT_SCO_RX_DestroyModule(void)
{
   if(mBTSCOCVSDContext->pRX)
   {
      if(mBTSCOCVSDContext->pRX->pSRCHandle_1)
      {
      	#ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
         BLI_Close(mBTSCOCVSDContext->pRX->pSRCHandle_1, NULL);
		#else
		 BLI_Close(mBTSCOCVSDContext->pRX->pSRCHandle_1);
		#endif
      }
      if(mBTSCOCVSDContext->pRX->pSRCHandle_2)
      {
      	#ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
         BLI_Close(mBTSCOCVSDContext->pRX->pSRCHandle_2, NULL);
		#else
		 BLI_Close(mBTSCOCVSDContext->pRX->pSRCHandle_2);
		#endif
      }
   }
}


void AudioBTCVSDControlInterface::BT_SCO_InitialModule(BT_SCO_MODULE uModule, uint8_t *pBuf)
{
   ASSERT(pBuf);
   switch(uModule)
   {
      case BT_SCO_MOD_CVSD_ENCODE: 
         mBTSCOCVSDContext->pTX->pEncHandle = CVSD_ENC_Init((int8_t*)pBuf);
         break;
      case BT_SCO_MOD_CVSD_DECODE: 
         mBTSCOCVSDContext->pRX->pDecHandle = CVSD_DEC_Init((int8_t*)pBuf);
         break;
      case BT_SCO_MOD_FILTER_TX: 
         mBTSCOCVSDContext->pTX->pHPFHandle = Audio_IIRHPF_Init((int8_t*)pBuf, btsco_FilterCoeff_8K, 1);
         break;
      case BT_SCO_MOD_FILTER_RX: 
         mBTSCOCVSDContext->pRX->pHPFHandle = Audio_IIRHPF_Init((int8_t*)pBuf, btsco_FilterCoeff_64K, 1);
         break;
#if defined(__MSBC_CODEC_SUPPORT__)
      case BT_SCO_MOD_PLC_NB:
         g711plc_construct_v2((void *)pBuf, BTSCO_CVSD_PLC_SAMPLERATE);
         mBTSCOCVSDContext->pRX->pPLCHandle = (void*)pBuf;
         break;
#else
      case BT_SCO_MOD_PLC:
         g711plc_construct((void *)pBuf);
         mBTSCOCVSDContext->pRX->pPLCHandle = (void*)pBuf;
         break;
#endif
      case BT_SCO_MOD_CVSD_TX_SRC:
		 #ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
		 ALOGD("BT_SCO_InitialModule BT_SCO_MOD_CVSD_TX_SRC source: uSampleRate=%d, uChannelNumber=%d",mBTSCOCVSDContext->pTX->uSampleRate,mBTSCOCVSDContext->pTX->uChannelNumber);
         mBTSCOCVSDContext->pTX->pSRCHandle = (void*)BLI_Open(mBTSCOCVSDContext->pTX->uSampleRate,mBTSCOCVSDContext->pTX->uChannelNumber, BTSCO_CVSD_SAMPLERATE_DOMAIN, BTSCO_CVSD_CHANNEL_NUM, (char *)pBuf, NULL);
		 ALOGD("BT_SCO_InitialModule BT_SCO_MOD_CVSD_TX_SRC pTX->pSRCHandle=0x%x",mBTSCOCVSDContext->pTX->pSRCHandle);
		 #else
         mBTSCOCVSDContext->pTX->pSRCHandle = (void*)BLI_Open(mBTSCOCVSDContext->pTX->uSampleRate,mBTSCOCVSDContext->pTX->uChannelNumber, BTSCO_CVSD_SAMPLERATE_DOMAIN, BTSCO_CVSD_CHANNEL_NUM, (char *)pBuf);
		 #endif
         break;
      case BT_SCO_MOD_CVSD_RX_SRC1:
		#ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
		ALOGD("BT_SCO_InitialModule BT_SCO_MOD_CVSD_RX_SRC1 target: uSampleRate=%d, uChannelNumber=%d",mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber);
        mBTSCOCVSDContext->pRX->pSRCHandle_1 = (void*)BLI_Open(BTSCO_CVSD_SAMPLERATE_DOMAIN, BTSCO_CVSD_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber,(char *)pBuf, NULL);
		ALOGD("BT_SCO_InitialModule BT_SCO_MOD_CVSD_RX_SRC1 pRX->pSRCHandle_1=0x%x",mBTSCOCVSDContext->pRX->pSRCHandle_1);
		#else
		mBTSCOCVSDContext->pRX->pSRCHandle_1 = (void*)BLI_Open(BTSCO_CVSD_SAMPLERATE_DOMAIN, BTSCO_CVSD_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber,(char *)pBuf);
		#endif
        break;
      case BT_SCO_MOD_CVSD_RX_SRC2:
		#ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
        mBTSCOCVSDContext->pRX->pSRCHandle_2 = (void*)BLI_Open(BTSCO_CVSD_PLC_SAMPLERATE, BTSCO_CVSD_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber,(char *)pBuf, NULL);
		#else
		mBTSCOCVSDContext->pRX->pSRCHandle_2 = (void*)BLI_Open(BTSCO_CVSD_PLC_SAMPLERATE, BTSCO_CVSD_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber,(char *)pBuf);
		#endif
        break;

#if defined(__MSBC_CODEC_SUPPORT__)
      case BT_SCO_MOD_MSBC_ENCODE: 
         mBTSCOCVSDContext->pTX->pEncHandle = MSBC_ENC_Init((int8_t*)pBuf);
         break;
      case BT_SCO_MOD_MSBC_DECODE: 
         mBTSCOCVSDContext->pRX->pDecHandle = MSBC_DEC_Init((int8_t*)pBuf);
         break;
      case BT_SCO_MOD_PLC_WB:
         g711plc_construct_v2((void *)pBuf, BTSCO_MSBC_SAMPLERATE_DOMAIN);
         mBTSCOCVSDContext->pRX->pPLCHandle = (void*)pBuf;
         break;
      case BT_SCO_MOD_MSBC_TX_SRC:
         ALOGD("BT_SCO_InitialModule BT_SCO_MOD_MSBC_TX_SRC source: uSampleRate=%d, uChannelNumber=%d", mBTSCOCVSDContext->pTX->uSampleRate, mBTSCOCVSDContext->pTX->uChannelNumber);
   #ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
         mBTSCOCVSDContext->pTX->pSRCHandle = (void*)BLI_Open(mBTSCOCVSDContext->pTX->uSampleRate, mBTSCOCVSDContext->pTX->uChannelNumber, BTSCO_MSBC_SAMPLERATE_DOMAIN, BTSCO_MSBC_CHANNEL_NUM, (char *)pBuf, NULL);
   #else
         mBTSCOCVSDContext->pTX->pSRCHandle = (void*)BLI_Open(mBTSCOCVSDContext->pTX->uSampleRate, mBTSCOCVSDContext->pTX->uChannelNumber, BTSCO_MSBC_SAMPLERATE_DOMAIN, BTSCO_MSBC_CHANNEL_NUM, (char *)pBuf);
   #endif
         break;
      case BT_SCO_MOD_MSBC_RX_SRC:
         ALOGD("BT_SCO_InitialModule BT_SCO_MOD_MSBC_RX_SRC target: uSampleRate=%d, uChannelNumber=%d", mBTSCOCVSDContext->pRX->uSampleRate, mBTSCOCVSDContext->pRX->uChannelNumber);
   #ifndef MTK_82_AUDIO_EARLY_PORTING_METHOD
         mBTSCOCVSDContext->pRX->pSRCHandle_1 = (void*)BLI_Open(BTSCO_MSBC_SAMPLERATE_DOMAIN, BTSCO_MSBC_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber, (char *)pBuf, NULL);
   #else
         mBTSCOCVSDContext->pRX->pSRCHandle_1 = (void*)BLI_Open(BTSCO_MSBC_SAMPLERATE_DOMAIN, BTSCO_MSBC_CHANNEL_NUM, mBTSCOCVSDContext->pRX->uSampleRate,mBTSCOCVSDContext->pRX->uChannelNumber, (char *)pBuf);
   #endif
         break;
#endif

      default:
         ASSERT(0);
   }
}


void AudioBTCVSDControlInterface::BT_SCO_SET_TXState(BT_SCO_STATE state)
{
	ALOGD("BT_SCO_SET_TXState state=%d",state);
	mBTSCOCVSDContext->uTXState = state;
}

void AudioBTCVSDControlInterface::BT_SCO_SET_RXState(BT_SCO_STATE state)
{
	ALOGD("BT_SCO_SET_RXState state=%d",state);
	mBTSCOCVSDContext->uRXState = state;
}


void AudioBTCVSDControlInterface::BT_SCO_TX_Open(void)
{
	uint32 uTxMemSize = 0;
	char *pAllocMemory;
	ALOGD("BT_SCO_TX_Open(+)");

	uTxMemSize += (sizeof(BT_SCO_TX)+3)& ~0x3;

	mBTSCOCVSDContext->pTX = (BT_SCO_TX*)new char[uTxMemSize]; 
	ASSERT(mBTSCOCVSDContext->pTX);
	memset((void *)mBTSCOCVSDContext->pTX, 0, uTxMemSize);

	mBTCVSDTXOutBuf = (uint8_t *)new char[BTSCO_CVSD_TX_OUTBUF_SIZE];	
	ALOGD("mBTSCOCVSDContext->uTXState=0x%x",mBTSCOCVSDContext->uTXState);
	ASSERT(mBTSCOCVSDContext->uTXState == BT_SCO_TXSTATE_IDLE);

	#ifdef DUMP_TXSRC_TEST
	mTXSRCPCMDumpFile = NULL;
	mTXSRCPCMDumpFile = AudioOpendumpPCMFile("/sdcard/mtklog/audio_dump/txsrcout.pcm", "txsrc.pcm.dump");
	#endif
	ALOGD("BT_SCO_TX_Open(-)");
}

void AudioBTCVSDControlInterface::BT_SCO_TX_Close(void)
{
	ALOGD("BT_SCO_TX_Close(+)");
	
	if (mBTCVSDTXOutBuf) {
    	delete []mBTCVSDTXOutBuf;
    	mBTCVSDTXOutBuf = NULL;
		ALOGD("BT_SCO_TX_Close() release mBTCVSDTXOutBuf");
	}	
	
	if (mBTSCOCVSDContext->pTX) {
    	delete []mBTSCOCVSDContext->pTX;
    	mBTSCOCVSDContext->pTX = NULL;
		ALOGD("BT_SCO_TX_Close() release mBTSCOCVSDContext->pTX");
	}	

	
#ifdef DUMP_TXSRC_TEST
    if (mTXSRCPCMDumpFile) {
           AudioCloseDumpPCMFile(mTXSRCPCMDumpFile);;
    }
#endif

	ALOGD("BT_SCO_TX_Close(-)");
}

#if defined(__MSBC_CODEC_SUPPORT__)
void AudioBTCVSDControlInterface::btsco_AllocMemory_TX_CVSD(void)
{
	  uint32_t uTotalMemory = 0;
	  uint8_t  *pBuf = NULL;

	  ASSERT(mBTSCOCVSDContext->uTXState==BT_SCO_TXSTATE_READY);   
	  if(mBTSCOCVSDContext->pTX)
	  {
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_ENCODE);
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_TX_SRC);
		  if(mBTSCOCVSDContext->pTX->fEnableFilter)
		  {
			 uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_TX);
		  }
	  }
	   
	  pBuf = new uint8_t[uTotalMemory];
	  mBTSCOCVSDContext->pTXWorkingMemory = pBuf;
	  ASSERT(mBTSCOCVSDContext->pTXWorkingMemory);   
	   
	  if(mBTSCOCVSDContext->pTX)
	  {
		  BT_SCO_InitialModule(BT_SCO_MOD_CVSD_ENCODE, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_ENCODE);
		  BT_SCO_InitialModule(BT_SCO_MOD_CVSD_TX_SRC, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_TX_SRC);
		  if(mBTSCOCVSDContext->pTX->fEnableFilter)
		  {
			 BT_SCO_InitialModule(BT_SCO_MOD_FILTER_TX, pBuf);
			 pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_TX);
		  }
	  }
	  
	  ALOGD("btsco_AllocMemory_TX_CVSD %d", uTotalMemory);
}

void AudioBTCVSDControlInterface::btsco_AllocMemory_TX_MSBC(void)
{
	  uint32_t uTotalMemory = 0;
	  uint8_t  *pBuf = NULL;

	  ASSERT(mBTSCOCVSDContext->uTXState==BT_SCO_TXSTATE_READY);   
	  if(mBTSCOCVSDContext->pTX)
	  {
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_ENCODE);
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_TX_SRC);
		  if(mBTSCOCVSDContext->pTX->fEnableFilter)
		  {
			 uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_TX);
		  }
	  }
	   
	  pBuf = new uint8_t[uTotalMemory];
	  mBTSCOCVSDContext->pTXWorkingMemory = pBuf;
	  ASSERT(mBTSCOCVSDContext->pTXWorkingMemory);   
	   
	  if(mBTSCOCVSDContext->pTX)
	  {
		  BT_SCO_InitialModule(BT_SCO_MOD_MSBC_ENCODE, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_ENCODE);
		  BT_SCO_InitialModule(BT_SCO_MOD_MSBC_TX_SRC, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_TX_SRC);
		  
		  if(mBTSCOCVSDContext->pTX->fEnableFilter)
		  {
			 BT_SCO_InitialModule(BT_SCO_MOD_FILTER_TX, pBuf);
			 pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_TX);
		  }
	  }
	  
	  ALOGD("btsco_AllocMemory_TX_MSBC %d", uTotalMemory);
}

void AudioBTCVSDControlInterface::BT_SCO_TX_Start(void)
{

   ALOGD("BT_SCO_TX_Start() (+)");

   mBTSCOCVSDContext->fIsWideBand = UniqueAudioBTCVSDControlInterface->BTmode;

   if(mBTSCOCVSDContext->fIsWideBand)
   {
      btsco_AllocMemory_TX_MSBC();
   }
   else
   {
      btsco_AllocMemory_TX_CVSD();
   }

   ALOGD("BT_SCO_TX_Start() (-)");
}

#else

void AudioBTCVSDControlInterface::BT_SCO_TX_Start(void)
{
	   uint32_t uTotalMemory = 0;
	   uint8_t  *pBuf = NULL;
	   ALOGD("BT_SCO_TX_Start() (+)");
	   ASSERT(mBTSCOCVSDContext->uTXState==BT_SCO_TXSTATE_READY);   
	   if(mBTSCOCVSDContext->pTX)
	   {
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_ENCODE);
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_TX_SRC);
		  if(mBTSCOCVSDContext->pTX->fEnableFilter)
		  {
			 uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_TX);
		  }
	   }
	   
	   pBuf = new uint8_t[uTotalMemory];
	   mBTSCOCVSDContext->pTXWorkingMemory = pBuf;
	   ASSERT(mBTSCOCVSDContext->pTXWorkingMemory);   
	   
	   if(mBTSCOCVSDContext->pTX)
	   {
		  BT_SCO_InitialModule(BT_SCO_MOD_CVSD_ENCODE, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_ENCODE);
		  BT_SCO_InitialModule(BT_SCO_MOD_CVSD_TX_SRC, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_TX_SRC);
		  if(mBTSCOCVSDContext->pTX->fEnableFilter)
		  {
			 BT_SCO_InitialModule(BT_SCO_MOD_FILTER_TX, pBuf);
			 pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_TX);
		  }
	   }
	   ALOGD("BT_SCO_TX_Start() (-)");
}
#endif

void AudioBTCVSDControlInterface::BT_SCO_TX_Stop(void)
{
	ALOGD("BT_SCO_TX_Stop(+)");
	BT_SCO_TX_DestroyModule();

	if (mBTSCOCVSDContext->pTXWorkingMemory) {
        delete []mBTSCOCVSDContext->pTXWorkingMemory;
        mBTSCOCVSDContext->pTXWorkingMemory = NULL;
    }
	ALOGD("BT_SCO_TX_Stop(-)");
}
int AudioBTCVSDControlInterface::BT_SCO_TX_SetHandle(void(*pCallback)( void *pData ), void *pData, uint32_t uSampleRate, uint32_t uChannelNumber, uint32_t uEnableFilter)
{
	ASSERT(mBTSCOCVSDContext->uTXState==BT_SCO_TXSTATE_INIT);   
	if(uChannelNumber != 1 && uChannelNumber != 2)
	{
	   return -1;
	}
	if( uSampleRate != 8000  && uSampleRate != 11025 && uSampleRate != 12000 && 
		uSampleRate != 16000 && uSampleRate != 22050 && uSampleRate != 24000 && 
		uSampleRate != 32000 && uSampleRate != 44100 && uSampleRate != 48000 )
	{
	   return -2;
	}
	
	ASSERT(mBTSCOCVSDContext->pTX);
	mBTSCOCVSDContext->pTX->pCallback 	 = pCallback;
	mBTSCOCVSDContext->pTX->uSampleRate	 = (uint16_t)uSampleRate;
	mBTSCOCVSDContext->pTX->uChannelNumber = (uint8_t)uChannelNumber;
	mBTSCOCVSDContext->pTX->pUserData 	 = pData;
	mBTSCOCVSDContext->pTX->fEnableFilter  = uEnableFilter;

	return 0;
}

int AudioBTCVSDControlInterface::BT_SCO_RX_SetHandle(void(*pCallback)( void *pData ), void *pData, uint32_t uSampleRate, uint32_t uChannelNumber, uint32_t uEnableFilter)
{

	ALOGD("BT_SCO_RX_SetHandle uSampleRate=%d, uChannelNumber=%d", uSampleRate, uChannelNumber);

	ASSERT(mBTSCOCVSDContext->uRXState==BT_SCO_RXSTATE_INIT);   
	if(uChannelNumber != 1 && uChannelNumber != 2)
	{
	   return -1;
	}
	if( uSampleRate != 8000  && uSampleRate != 11025 && uSampleRate != 12000 && 
		uSampleRate != 16000 && uSampleRate != 22050 && uSampleRate != 24000 && 
		uSampleRate != 32000 && uSampleRate != 44100 && uSampleRate != 48000 )
	{
	   return -2;
	}
	ASSERT(mBTSCOCVSDContext->pRX);
	mBTSCOCVSDContext->pRX->pCallback	  = pCallback;
	mBTSCOCVSDContext->pRX->uSampleRate	  = (uint16_t)uSampleRate;
	mBTSCOCVSDContext->pRX->uChannelNumber = (uint8_t)uChannelNumber;
	mBTSCOCVSDContext->pRX->pUserData	  = pData;
	mBTSCOCVSDContext->pRX->fEnableFilter  = uEnableFilter;
	
#if defined(__MSBC_CODEC_SUPPORT__)
	if (mBTSCOCVSDContext->fIsWideBand)
	{
		if(uSampleRate == BTSCO_MSBC_SAMPLERATE_DOMAIN)
		{
			mBTSCOCVSDContext->pRX->fEnablePLC = true;  // TODO: disable PLC for test
		}

		if(mBTSCOCVSDContext->pRX->uSampleRate != BTSCO_MSBC_SAMPLERATE_DOMAIN || mBTSCOCVSDContext->pRX->uChannelNumber != BTSCO_MSBC_CHANNEL_NUM)
		{
			mBTSCOCVSDContext->pRX->fEnableSRC2 = true;
		}
	} else
#endif
    {
		if(uSampleRate == BTSCO_CVSD_PLC_SAMPLERATE)
		{
			mBTSCOCVSDContext->pRX->fEnablePLC = true;  // TODO: disable PLC for test
		}

		if(mBTSCOCVSDContext->pRX->uSampleRate != BTSCO_CVSD_PLC_SAMPLERATE || mBTSCOCVSDContext->pRX->uChannelNumber != BTSCO_CVSD_CHANNEL_NUM)
		{
			mBTSCOCVSDContext->pRX->fEnableSRC2 = true;
		}
	}

	return 0;
}



/*
btsco_process_RX_CVSD(void *inbuf, uint32 *insize, void *outbuf, uint32 *outsize, void *workbuf, const uint32 workbufsize, uint8_t packetvalid)

void *inbuf                                         : inbuf of CVSD bitstream for decode
uint32 *insize                                     :  in: CVSD bitstream length for decode (i.e. SCO_RX_PLC_SIZE), 
                        						out: consumed length of CVSD bitstream
void *outbuf                                     : outbuf of CVSD decoded pcm data (I.e. PcmBuf_8k)
uint32 *outsize                                 :  in: desired output length of CVSD decoded pcm data (i.e. SCO_RX_PCM8K_BUF_SIZE)
                                                                   out: practical output length of CVSD decoded pcm data 
void *workbuf                                  : working buf during CVSD decode (i.e. PcmBuf_64k)
const uint32 workbufsize              : working buf size during CVSD decode (i.e. SCO_RX_PCM64K_BUF_SIZE)
uint8_t packetvalid			  : Is this SCO_RX_PLC_SIZE packet valid (without packet loss)
*/

#ifdef DUMP_CVSDRX_OUTPUT //for TEST 
static int Rxcnt=0;
static uint8_t RXout_dumpbuf[SCO_RX_PCM64K_BUF_SIZE*2];
#endif

void AudioBTCVSDControlInterface::btsco_process_RX_CVSD(void *inbuf, uint32_t *insize, void *outbuf, uint32_t *outsize, void *workbuf, const uint32_t workbufsize, uint8_t packetvalid)
{
    uint16_t *pDst, *pSrc, i;
    int32_t iOutSample=0, iInByte=0, consumed;

	consumed = *insize;
    iOutSample = workbufsize>>1;
//    ALOGD("btsco_process_RX_CVSD() CVSD_DEC_Process(+) *insize=%d, iOutSample=%d", *insize, iOutSample);
    CVSD_DEC_Process(mBTSCOCVSDContext->pRX->pDecHandle, (char *)inbuf, (int *)insize, (short *)workbuf, (int *)&iOutSample);
//    ALOGD("btsco_process_RX_CVSD() CVSD_DEC_Process(-) remaining=%d, iOutSample=%d", *insize, iOutSample);

	if(iOutSample!=(SCO_RX_PCM64K_BUF_SIZE>>1))
	{
		ALOGE("ERROR!!!btsco_process_RX_CVSD() iOutSample!=(SCO_RX_PCM64K_BUF_SIZE>>1)!!!!,iOutSample=%d",iOutSample);
	}
	
	#ifdef DUMP_CVSDRX_OUTPUT
	memcpy((void *)RXout_dumpbuf+Rxcnt*SCO_RX_PCM64K_BUF_SIZE, (void *)workbuf, iOutSample<<1);	
	if(Rxcnt==1)
	{
		if (mBTCVSDRXDumpFile) {
			fwrite((void *)RXout_dumpbuf, 1, iOutSample<<2, mBTCVSDRXDumpFile);
		}
		Rxcnt = 0;
	}
	else
	{
		Rxcnt++;
	}
	#endif
	
	consumed -= *insize;
	*insize = consumed;
    {
       uint32_t uOutByte=0, uInByte=0, uConsumeByte=0;
       uInByte = iOutSample<<1; //should be SCO_RX_PCM64K_BUF_SIZE!!!
       uOutByte = *outsize;
    
//      ALOGD("btsco_process_RX_CVSD() BLI_Convert(+) uInByte=%d, uOutByte=%d", uInByte, uOutByte);
       uConsumeByte = BLI_Convert(mBTSCOCVSDContext->pRX->pSRCHandle_1, (int16_t *)workbuf, &uInByte, (int16_t *)outbuf, &uOutByte);
//        ALOGD("btsco_process_RX_CVSD() BLI_Convert(-) remaining=%d, uConsumeByte=%d, uOutByte=%d", uInByte, uConsumeByte, uOutByte);
       ASSERT(uConsumeByte == workbufsize);
       //ASSERT(uOutByte == *outsize); //TODO: input 480 bytes, out only 26 bytes. WHY???    if uOutByte is not *outsize, PLC will have problem!!! 
       *outsize = uOutByte;
    }

#if defined(BTCVSD_LOOPBACK_WITH_CODEC)
	if(mBTSCOCVSDContext->pTX)
	{
		if(mBTSCOCVSDContext->pTX->uSampleRate == BTSCO_CVSD_PLC_SAMPLERATE)
		{
			mBTSCOCVSDContext->pRX->fEnablePLC = false;
			if(packetvalid==0)
			{
				ALOGD("btsco_process_RX_CVSD(), packet lost, in loopback mode, no PLC!!!");
			}
		}
	}
#endif

    if(mBTSCOCVSDContext->pRX->fEnablePLC)
    {  //do PLC
       if(packetvalid)
       {  //packet not lost
#if defined(__MSBC_CODEC_SUPPORT__)
          g711plc_addtohistory_v2(mBTSCOCVSDContext->pRX->pPLCHandle, (short *)outbuf, 0);
#else
          g711plc_addtohistory(mBTSCOCVSDContext->pRX->pPLCHandle, (short *)outbuf);
#endif
       }
       else
       {  //packet lost
       		
		   ALOGD("btsco_process_RX_CVSD(), packet lost, do PLC!!!");
			#if 0
			for(i=0;i<SCO_RX_PLC_SIZE;i++)
			{
				ALOGD("%x ",*((char *)inbuf+i));  //should be 0x55
			}
			#endif
			
#if defined(__MSBC_CODEC_SUPPORT__)
          g711plc_dofe_v2(mBTSCOCVSDContext->pRX->pPLCHandle, (short *)outbuf, 0);
#else
          g711plc_dofe(mBTSCOCVSDContext->pRX->pPLCHandle, (short *)outbuf);
#endif
       }
    }
    if(mBTSCOCVSDContext->pRX->fEnableFilter)
    {  //do filter
       int32_t iInSample = *outsize>>1;
       int32_t iOutSample = *outsize>>1;
       Audio_IIRHPF_Process(mBTSCOCVSDContext->pRX->pHPFHandle, (uint16_t*)outbuf, (uint16_t*)&iInSample, (uint16_t*)outbuf, (uint16_t*)&iOutSample);
	   *outsize = iOutSample<<1;
    }
     	
//    ALOGD("btsco_process_RX_CVSD() consumed=%d, *outsize=%d", *insize,*outsize);      
}

/*
btsco_process_RX_MSBC(void *inbuf, uint32 *insize, void *outbuf, uint32 *outsize, void *workbuf, const uint32 workbufsize, uint8_t packetvalid)

void *inbuf                                         : inbuf of MSBC bitstream for decode
uint32 *insize                                     :  in: MSBC bitstream length for decode, 
                        						out: consumed length of MSBC bitstream
void *outbuf                                     : outbuf of CVSD decoded pcm data (I.e. PcmBuf_8k)
uint32 *outsize                                 :  in: desired output length of MSBC decoded pcm data
                                                                   out: practical output length of MSBC decoded pcm data 
void *workbuf                                  : working buf during MSBC decode
const uint32 workbufsize              : working buf size during MSBC decode
uint8_t packetvalid			  : Is this SCO_RX_PLC_SIZE packet valid (without packet loss)
*/

#if defined(__MSBC_CODEC_SUPPORT__)

void AudioBTCVSDControlInterface::btsco_process_RX_MSBC(void *inbuf, uint32_t *insize, void *outbuf, uint32_t *outsize, void *workbuf, const uint32_t workbufsize, uint8_t packetvalid)
{
   uint16_t *pDst, *pSrc, i;
   int32_t iOutSample=0, iInByte=0;
   uint32_t index1=0, index2=0, dwBtEv3HalfBad=0;
   uint8_t *pSrc8;
   int32_t status=0;
   
   pSrc8 = (uint8_t *)inbuf;
   iInByte = MSBC_PACKET_SIZE_BYTE>>1; 
   iOutSample = MSBC_PCM_FRAME_BYTE >> 1;

   // Put into packet buffer
   if (*insize < iInByte) {
      *insize = 0;
      *outsize = 0;
      return;
   }
   index1 = mBTSCOCVSDContext->pRX->iPacket_w & SCO_RX_PACKET_MASK;
   memcpy(mBTSCOCVSDContext->pRX->PacketBuf[index1], inbuf, iInByte);
   mBTSCOCVSDContext->pRX->PacketValid[index1] = packetvalid;
   mBTSCOCVSDContext->pRX->iPacket_w++;

   *insize = iInByte;
   
   // KH TBD: updata the condition of output buffer size
   if ( ((mBTSCOCVSDContext->pRX->iPacket_w - mBTSCOCVSDContext->pRX->iPacket_r)<2) || (*outsize < MSBC_PCM_FRAME_BYTE) )
   {
      *outsize = 0;
      return;
   }
   
   index1 = mBTSCOCVSDContext->pRX->iPacket_r & SCO_RX_PACKET_MASK;
   index2 = (mBTSCOCVSDContext->pRX->iPacket_r+1) & SCO_RX_PACKET_MASK;
   mBTSCOCVSDContext->pRX->iPacket_r++;  // Consume 30 bytes
   
   if ( (mBTSCOCVSDContext->pRX->PacketBuf[index1][0] == 0x1) && ((mBTSCOCVSDContext->pRX->PacketBuf[index1][1]&0xF) == 0x8) )
   {      
      int32_t iInByteTemp = MSBC_BTSTREAM_FRAME_BYTE;
      
      // Copy to bitstream buffer for decoding
      memcpy(mBTSCOCVSDContext->pRX->EntirePacket, &mBTSCOCVSDContext->pRX->PacketBuf[index1][2], 28);
      memcpy(mBTSCOCVSDContext->pRX->EntirePacket+28, &mBTSCOCVSDContext->pRX->PacketBuf[index2][0], 29);
      
//      ALOGD("btsco_process_RX_MSBC() MSBC_DEC_Process(+)");
      status = MSBC_DEC_Process(mBTSCOCVSDContext->pRX->pDecHandle, (char *)mBTSCOCVSDContext->pRX->EntirePacket, &iInByteTemp, (int16_t *)mBTSCOCVSDContext->pRX->PcmBuf_mSBC, &iOutSample);
//      ALOGD("btsco_process_RX_MSBC() MSBC_DEC_Process(-) status=%d", status);
      
      mBTSCOCVSDContext->pRX->iPacket_r++;   // Consume another 30 bytes for entire packet
   } else if (mBTSCOCVSDContext->pRX->PacketValid[index1] == 1) {
      //wrong header, but packet not lost, sync to next 30 byte block (should only happenes on packetsize 30)
      status = -1;
      ALOGD("btsco_process_RX_MSBC() wrong header, but packet not lost, sync to next 30 byte block!!!");
   } else {
      status = -1;
      
      mBTSCOCVSDContext->pRX->iPacket_r++;   // Consume another 30 bytes for entire packet
      ALOGD("btsco_process_RX_MSBC() wrong header, packet invalid!!!");
   }
   
   if (status == MSBC_BTSTREAM_FRAME_BYTE)
   {
      if (mBTSCOCVSDContext->pRX->PacketValid[index1]==1 && mBTSCOCVSDContext->pRX->PacketValid[index2]==0)
      {
         dwBtEv3HalfBad = 1;
      }
      else
      {
         dwBtEv3HalfBad = 0;
      }      
   }
   
   if (mBTSCOCVSDContext->pRX->fEnablePLC)
   {
      if ( (status==MSBC_BTSTREAM_FRAME_BYTE) && (mBTSCOCVSDContext->pRX->PacketValid[index1]==1) && (mBTSCOCVSDContext->pRX->PacketValid[index2]==1) )
      {
         //packet not lost
         g711plc_addtohistory_v2(mBTSCOCVSDContext->pRX->pPLCHandle, (short *)mBTSCOCVSDContext->pRX->PcmBuf_mSBC, 0);
      }
      else
      {
         //packet lost
         ALOGD("btsco_process_RX_MSBC(), packet lost, do PLC!!!");
         g711plc_dofe_v2(mBTSCOCVSDContext->pRX->pPLCHandle, (short *)mBTSCOCVSDContext->pRX->PcmBuf_mSBC, dwBtEv3HalfBad);
      }
   }
   
   if(mBTSCOCVSDContext->pRX->fEnableFilter)
   {  //do filter
      int32_t iInSampleTemp = iOutSample;
      int32_t iOutSampleTemp = iOutSample;
      Audio_IIRHPF_Process(mBTSCOCVSDContext->pRX->pHPFHandle, (uint16_t*)mBTSCOCVSDContext->pRX->PcmBuf_mSBC, (uint16_t*)&iInSampleTemp, (uint16_t*)mBTSCOCVSDContext->pRX->PcmBuf_mSBC, (uint16_t*)&iOutSampleTemp);
	    iOutSample = iOutSampleTemp;
   }
   
   if (mBTSCOCVSDContext->pRX->fEnableSRC2) {
      uint32_t uOutByte=0, uInByte=0, uConsumeByte=0;
      
      uInByte = iOutSample<<1;
      uOutByte = *outsize;
          
//      ALOGD("btsco_process_RX_MSBC() BLI_Convert(+) uInByte=%d, uOutByte=%d", uInByte, uOutByte);
      uConsumeByte = BLI_Convert(mBTSCOCVSDContext->pRX->pSRCHandle_1, (int16_t *)mBTSCOCVSDContext->pRX->PcmBuf_mSBC, &uInByte, (int16_t *)outbuf, &uOutByte);
//      ALOGD("btsco_process_RX_MSBC() BLI_Convert(-) remaining=%d, uConsumeByte=%d, uOutByte=%d", uInByte, uConsumeByte, uOutByte);

      *outsize = uOutByte;
   }
    
//   ALOGD("btsco_process_RX_MSBC() consumed=%d, *outsize=%d", *insize, *outsize);      
}

#endif

/*
btsco_process_TX_CVSD(void *inbuf, uint32 *insize, void *outbuf, uint32 *outsize, void *workbuf, const uint32 workbufsize, uint32 src_fs_s)

void *inbuf                                         : inbuf of pcm data(for SRC to 64k)
uint32 *insize                                     : in: inbuf length of pcm data(for SRC to 64k)
									out: consumed inbuf length of pcm data (i.e. SRC consumed buf length)
void *outbuf                                      : outbuf of CVSD encoded bitstream
uint32 *outsize                                 : in: desired output length of CVSD encoded bitstream (i.e. SCO_TX_ENCODE_SIZE)
                                                        out: practical output length of CVSD encoded bitstream
void *workbuf                                  : workingbuf during CVSD encode (i.e. PcmBuf_64k)
const uint32 workbufsize              : workingbuf size during CVSD encode (i.e. SCO_RX_PCM64K_BUF_SIZE)
unint32 src_fs_s                               : SRC source sample rate(for SRC to 64k)
*/

void AudioBTCVSDControlInterface::btsco_process_TX_CVSD(void *inbuf, uint32_t *insize, void *outbuf, uint32_t *outsize, void *workbuf, const uint32_t workbufsize, uint32_t src_fs_s)
{
	uint32_t src_outsize,iInSample,iOutSample,iOutByte,uConsumeByte,i;

	if(*insize!=0 && *outsize!=0)
	{
		src_outsize = workbufsize;
//		ALOGD("btsco_process_TX_CVSD() BLI_Convert *insize=%d, *outsize=%d",*insize, *outsize);
		uConsumeByte = BLI_Convert(mBTSCOCVSDContext->pTX->pSRCHandle, (int16_t *)inbuf, insize, (int16_t *)workbuf, &src_outsize);
//		ALOGD("btsco_process_TX_CVSD() BLI_Convert consumed=%d, remaining=%d, src_outsize=%d", uConsumeByte, *insize, src_outsize);

		#ifdef DUMP_TXSRC_TEST
	    if (mTXSRCPCMDumpFile) {
	          fwrite((void *)workbuf, 1, src_outsize, mTXSRCPCMDumpFile);
	    }
		#endif

		*insize = uConsumeByte;
		iInSample = src_outsize>>1;

		if(mBTSCOCVSDContext->pTX->fEnableFilter)
		{  //filter
			iInSample = src_outsize>>1;
			iOutSample = src_outsize>>1;
			Audio_IIRHPF_Process(mBTSCOCVSDContext->pTX->pHPFHandle, (uint16_t*)workbuf, (uint16_t*)&iInSample, (uint16_t*)workbuf, (uint16_t*)&iOutSample);
			iInSample = iOutSample;
		}
		//encode
		iOutByte = SCO_TX_ENCODE_SIZE;
		//uint32_t iGain = 0x7FFF;
		
		CVSD_ENC_Process(mBTSCOCVSDContext->pTX->pEncHandle, (short *)workbuf, (int *)&iInSample, (char *)outbuf, (int *)&iOutByte);
//		ALOGD("CVSD_ENC_Process BLI_Convert iInSample=%d, iOutByte=%d", iInSample, iOutByte);
		
		#ifdef TXOUT_RXIN_TEST // use sequence number to replace ENC out
		for(i=0;i<iOutByte;i++)
		{
			*((char *)outbuf+i) = i;
		}
		#endif

		*outsize = iOutByte;
	}
	else
	{
		*insize = 0;
		*outsize = 0;
	}
}

/*
btsco_process_TX_MSBC(void *inbuf, uint32 *insize, void *outbuf, uint32 *outsize, void *workbuf, const uint32 workbufsize, uint32 src_fs_s)

void *inbuf                                         : inbuf of pcm data
uint32 *insize                                     : in: inbuf length of pcm data(for SRC to 64k)
									out: consumed inbuf length of pcm data (i.e. SRC consumed buf length)
void *outbuf                                      : outbuf of CVSD encoded bitstream
uint32 *outsize                                 : in: desired output length of mSBC encoded bitstream
                                                        out: practical output length of mSBC encoded bitstream
void *workbuf                                  : workingbuf during CVSD encode (i.e. PcmBuf_64k)
const uint32 workbufsize              : workingbuf size during mSBC encode
unint32 src_fs_s                               : SRC source sample rate
*/

#if defined(__MSBC_CODEC_SUPPORT__) 

static const uint8_t btsco_MsbcHeader[4] = {0x08, 0x38, 0xc8, 0xf8};

void AudioBTCVSDControlInterface::btsco_process_TX_MSBC(void *inbuf, uint32_t *insize, void *outbuf, uint32_t *outsize, void *workbuf, const uint32_t workbufsize, uint32_t src_fs_s)
{
	 uint32_t src_outsize,iInSample,iOutSample,iOutByte,i,status,index;
   uint32_t uBliConsumeByte, uFreeSpace;
   char *pOutput8 = (char *)outbuf;

   // Calculate free space
   uFreeSpace = MSBC_PCM_FRAME_BYTE - mBTSCOCVSDContext->pTX->uPcmBuf_mSBC_w;
   src_outsize = uFreeSpace;

   // Sampling rate converter to 16kHz. As mSBC input
   uBliConsumeByte = BLI_Convert(mBTSCOCVSDContext->pTX->pSRCHandle, (int16_t *)inbuf, insize, (int16_t *)workbuf, &src_outsize);
   
   // Copy to PCM buffer
   memcpy(mBTSCOCVSDContext->pTX->PcmBuf_mSBC + mBTSCOCVSDContext->pTX->uPcmBuf_mSBC_w, workbuf, src_outsize);
   mBTSCOCVSDContext->pTX->uPcmBuf_mSBC_w += src_outsize;
   
   if( (mBTSCOCVSDContext->pTX->uPcmBuf_mSBC_w < MSBC_PCM_FRAME_BYTE) || (*outsize < SCO_TX_ENCODE_SIZE) )
   {
      *insize = uBliConsumeByte;
      *outsize = 0;
      return;
   }
   
   if(mBTSCOCVSDContext->pTX->fEnableFilter)
   {  
   	  //filter
   	  iInSample = MSBC_PCM_FRAME_BYTE>>1;
   	  iOutSample = MSBC_PCM_FRAME_BYTE>>1;
   	  Audio_IIRHPF_Process(mBTSCOCVSDContext->pTX->pHPFHandle, (uint16_t*)mBTSCOCVSDContext->pTX->PcmBuf_mSBC, (uint16_t*)&iInSample, (uint16_t*)mBTSCOCVSDContext->pTX->PcmBuf_mSBC, (uint16_t*)&iOutSample);
   	  ASSERT(iInSample == iOutSample);
   }
   
   //encode
   iInSample = MSBC_PCM_FRAME_BYTE>>1;
   iOutByte = MSBC_BTSTREAM_FRAME_BYTE;
   
   status = MSBC_ENC_Process(mBTSCOCVSDContext->pTX->pEncHandle, (short *)mBTSCOCVSDContext->pTX->PcmBuf_mSBC, (int *)&iInSample, pOutput8 + 2, (int *)&iOutByte); //out 57 bytes
   ALOGD("MSBC_ENC_Process iInSample=%d, iOutByte=%d, status=%d", iInSample, iOutByte, status);
   
   index = mBTSCOCVSDContext->pTX->iPacket_w++;
   pOutput8[0 ] = 0x01; //header
   pOutput8[1 ] = btsco_MsbcHeader[index&0x3]; //header
   pOutput8[59] = 0; //header
   pOutput8 += SCO_TX_ENCODE_SIZE;
   
   mBTSCOCVSDContext->pTX->uPcmBuf_mSBC_w = 0;
   
   *insize = uBliConsumeByte;
   *outsize = SCO_TX_ENCODE_SIZE;
}

#endif

void AudioBTCVSDControlInterface::BT_SCO_RX_Open(void)
{
	uint32 uRxMemSize = 0;
	char *pAllocMemory;

	ALOGD("BT_SCO_RX_Open(+) mBTSCOCVSDContext->uRXState=%d", mBTSCOCVSDContext->uRXState);

	ASSERT(mBTSCOCVSDContext->uRXState == BT_SCO_RXSTATE_IDLE);

	uRxMemSize += (sizeof(BT_SCO_RX)+3)& ~0x3;

	mBTSCOCVSDContext->pRX = (BT_SCO_RX *)new char[uRxMemSize]; 
	ASSERT(mBTSCOCVSDContext->pRX);
	memset((void *)mBTSCOCVSDContext->pRX, 0, uRxMemSize);

	mBTCVSDRXTempInBuf = (uint8_t *)new char[BTSCO_CVSD_RX_TEMPINPUTBUF_SIZE];
	mBTCVSDRXInBuf = (uint8_t *)new char[BTSCO_CVSD_RX_INBUF_SIZE];

	#ifdef DUMP_CVSDRX_OUTPUT
    mBTCVSDRXDumpFile = NULL;
    mBTCVSDRXDumpFile = AudioOpendumpPCMFile("/sdcard/mtklog/audio_dump/CVSDRXOut.pcm", "CVSDRXOut.pcm.dump");
	#endif
	ALOGD("BT_SCO_RX_Open(-)");
}

void AudioBTCVSDControlInterface::BT_SCO_RX_Close(void)
{
	ALOGD("BT_SCO_RX_Close(+)");
	
	if (mBTCVSDRXTempInBuf) {
    	delete []mBTCVSDRXTempInBuf;
    	mBTCVSDRXTempInBuf = NULL;
		ALOGD("BT_SCO_RX_Close() release mBTCVSDRXTempInBuf");
	}

	if (mBTCVSDRXInBuf) {
    	delete []mBTCVSDRXInBuf;
        mBTCVSDRXInBuf = NULL;
		ALOGD("BT_SCO_RX_Close() release mBTCVSDRXInBuf");		
	}

	if (mBTSCOCVSDContext->pRX) {
    	delete []mBTSCOCVSDContext->pRX;
    	mBTSCOCVSDContext->pRX = NULL;
		ALOGD("BT_SCO_RX_Close(-) release mBTSCOCVSDContext->pRX");
	}	
	#ifdef DUMP_CVSDRX_OUTPUT
	if (mBTCVSDRXDumpFile) {
		AudioCloseDumpPCMFile(mBTCVSDRXDumpFile);
		ALOGD("ClosePcmDumpFile mBTCVSDRXDumpFile");
	}
	#endif
	ALOGD("BT_SCO_RX_Close(-)");
}

uint8_t *AudioBTCVSDControlInterface::BT_SCO_RX_GetCVSDOutBuf(void)
{
	return mBTSCOCVSDContext->pRX->PcmBuf_8k;
}

#if defined(__MSBC_CODEC_SUPPORT__) 
uint8_t *AudioBTCVSDControlInterface::BT_SCO_RX_GetMSBCOutBuf(void)
{
	return mBTSCOCVSDContext->pRX->PcmBuf_mSBC;
}
#endif

uint8_t *AudioBTCVSDControlInterface::BT_SCO_RX_GetCVSDWorkBuf(void)
{
	return mBTSCOCVSDContext->pRX->PcmBuf_64k;
}

#if defined(__MSBC_CODEC_SUPPORT__) 
void AudioBTCVSDControlInterface::btsco_AllocMemory_RX_CVSD(void)
{
   uint32_t uTotalMemory = 0;
   uint8_t  *pBuf = NULL;

   ASSERT(mBTSCOCVSDContext->uRXState==BT_SCO_RXSTATE_READY);	
	   
   if(mBTSCOCVSDContext->pRX)
   {
      uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_DECODE);
      uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC1);
      if(mBTSCOCVSDContext->pRX->fEnableFilter)
      {
         uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_RX);
      }
      if(mBTSCOCVSDContext->pRX->fEnablePLC)
      {
	       uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PLC_NB);
      }
      if(mBTSCOCVSDContext->pRX->fEnableSRC2)
      {
         uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC2);
      }
   }
	   
   pBuf = new uint8_t[uTotalMemory];
   mBTSCOCVSDContext->pRXWorkingMemory = pBuf;
   ASSERT(mBTSCOCVSDContext->pRXWorkingMemory);	
   
   if(mBTSCOCVSDContext->pRX)
   {
      BT_SCO_InitialModule(BT_SCO_MOD_CVSD_DECODE, pBuf);
      pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_DECODE);
      BT_SCO_InitialModule(BT_SCO_MOD_CVSD_RX_SRC1, pBuf);
      pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC1);
      if(mBTSCOCVSDContext->pRX->fEnableFilter)
      {
	       BT_SCO_InitialModule(BT_SCO_MOD_FILTER_RX, pBuf);
	       pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_RX);
      }
      if(mBTSCOCVSDContext->pRX->fEnablePLC)
      {
	       BT_SCO_InitialModule(BT_SCO_MOD_PLC_NB, pBuf);
	       pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PLC_NB);
      }
      if(mBTSCOCVSDContext->pRX->fEnableSRC2)
      {
	       BT_SCO_InitialModule(BT_SCO_MOD_CVSD_RX_SRC2, pBuf);
	       pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC2);
      }
   }
}

void AudioBTCVSDControlInterface::btsco_AllocMemory_RX_MSBC(void)
{
   uint32_t uTotalMemory = 0;
   uint8_t  *pBuf = NULL;

   ASSERT(mBTSCOCVSDContext->uRXState==BT_SCO_RXSTATE_READY);	
	   
   if(mBTSCOCVSDContext->pRX)
   {
      uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_DECODE);
      if(mBTSCOCVSDContext->pRX->fEnableFilter)
      {
         uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_RX);
      }
      if(mBTSCOCVSDContext->pRX->fEnablePLC)
      {
	       uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PLC_WB);
      }
      if(mBTSCOCVSDContext->pRX->fEnableSRC2)
      {
         uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_RX_SRC);
      }
   }
	   
   pBuf = new uint8_t[uTotalMemory];
   mBTSCOCVSDContext->pRXWorkingMemory = pBuf;
   ASSERT(mBTSCOCVSDContext->pRXWorkingMemory);	
   
   if(mBTSCOCVSDContext->pRX)
   {
      BT_SCO_InitialModule(BT_SCO_MOD_MSBC_DECODE, pBuf);
      pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_DECODE);
      if(mBTSCOCVSDContext->pRX->fEnableFilter)
      {
	       BT_SCO_InitialModule(BT_SCO_MOD_FILTER_RX, pBuf);
	       pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_RX);
      }
      if(mBTSCOCVSDContext->pRX->fEnablePLC)
      {
	       BT_SCO_InitialModule(BT_SCO_MOD_PLC_WB, pBuf);
	       pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PLC_WB);
      }
      if(mBTSCOCVSDContext->pRX->fEnableSRC2)
      {
	       BT_SCO_InitialModule(BT_SCO_MOD_MSBC_RX_SRC, pBuf);
	       pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_MSBC_RX_SRC);
      }
   }
}

void AudioBTCVSDControlInterface::BT_SCO_RX_Start(void)
{
   ALOGD("BT_SCO_RX_Start(+)");

   mBTSCOCVSDContext->fIsWideBand = UniqueAudioBTCVSDControlInterface->BTmode;

   if(mBTSCOCVSDContext->fIsWideBand)
   {
      btsco_AllocMemory_RX_MSBC();
   }
   else
   {
      btsco_AllocMemory_RX_CVSD();
   }
   
   ALOGD("BT_SCO_RX_Start(-)");
}

#else

void AudioBTCVSDControlInterface::BT_SCO_RX_Start(void)
{
	   uint32_t uTotalMemory = 0;
	   uint8_t  *pBuf = NULL;
	   ALOGD("BT_SCO_RX_Start(+)");
	   ASSERT(mBTSCOCVSDContext->uRXState==BT_SCO_RXSTATE_READY);	
	   
	   if(mBTSCOCVSDContext->pRX)
	   {
		  //uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PCM_RINGBUF_RX);
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_DECODE);
		  uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC1);
		  if(mBTSCOCVSDContext->pRX->fEnableFilter)
		  {
			 uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_RX);
		  }
		  if(mBTSCOCVSDContext->pRX->fEnablePLC)
		  {
			 uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PLC);
		  }
		  if(mBTSCOCVSDContext->pRX->fEnableSRC2)
		  {
			 uTotalMemory += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC2);
		  }
	   }
	   
	   pBuf = new uint8_t[uTotalMemory];
	   mBTSCOCVSDContext->pRXWorkingMemory = pBuf;
	   ASSERT(mBTSCOCVSDContext->pRXWorkingMemory);	
	   
	   if(mBTSCOCVSDContext->pRX)
	   {
		  BT_SCO_InitialModule(BT_SCO_MOD_CVSD_DECODE, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_DECODE);
		  BT_SCO_InitialModule(BT_SCO_MOD_CVSD_RX_SRC1, pBuf);
		  pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC1);
		  if(mBTSCOCVSDContext->pRX->fEnableFilter)
		  {
			 BT_SCO_InitialModule(BT_SCO_MOD_FILTER_RX, pBuf);
			 pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_FILTER_RX);
		  }
		  if(mBTSCOCVSDContext->pRX->fEnablePLC)
		  {
			 BT_SCO_InitialModule(BT_SCO_MOD_PLC, pBuf);
			 pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_PLC);
		  }
		  if(mBTSCOCVSDContext->pRX->fEnableSRC2)
		  {
			 BT_SCO_InitialModule(BT_SCO_MOD_CVSD_RX_SRC2, pBuf);
			 pBuf += BT_SCO_GetMemorySize_4ByteAlign(BT_SCO_MOD_CVSD_RX_SRC2);
		  }
	   }
	   ALOGD("BT_SCO_RX_Start(-)");
}
#endif


void AudioBTCVSDControlInterface::BT_SCO_RX_Stop(void)
{
	ALOGD("BT_SCO_RX_Stop(+)");

	BT_SCO_RX_DestroyModule();

	if (mBTSCOCVSDContext->pRXWorkingMemory) {
        delete []mBTSCOCVSDContext->pRXWorkingMemory;
        mBTSCOCVSDContext->pRXWorkingMemory = NULL;
    }
	ALOGD("BT_SCO_RX_Stop(-)");
}

uint8_t *AudioBTCVSDControlInterface::BT_SCO_RX_GetCVSDTempInBuf(void)
{
	return mBTCVSDRXTempInBuf;
}

void AudioBTCVSDControlInterface::BT_SCO_RX_SetCVSDTempInBuf(uint8_t *addr)
{
	mBTCVSDRXTempInBuf = addr;
}


uint8_t *AudioBTCVSDControlInterface::BT_SCO_RX_GetCVSDInBuf(void)
{
	return mBTCVSDRXInBuf;
}

void AudioBTCVSDControlInterface::BT_SCO_RX_SetCVSDInBuf(uint8_t *addr)
{
	mBTCVSDRXInBuf = addr;
}

uint8_t *AudioBTCVSDControlInterface::BT_SCO_TX_GetCVSDOutBuf(void)
{
	return mBTCVSDTXOutBuf;
}

void AudioBTCVSDControlInterface::BT_SCO_TX_SetCVSDOutBuf(uint8_t *addr)
{
	mBTCVSDTXOutBuf = addr;
}

uint8_t *AudioBTCVSDControlInterface::BT_SCO_TX_GetCVSDWorkBuf(void)
{
	return mBTSCOCVSDContext->pTX->PcmBuf_64k;
}

uint32_t AudioBTCVSDControlInterface::Audio_IIRHPF_GetBufferSize(int a)
{
	return 1024;
}

void * AudioBTCVSDControlInterface::Audio_IIRHPF_Init(int8_t *pBuf, const int32_t *btsco_FilterCoeff_8K, int a)
{
	return NULL;
}

void AudioBTCVSDControlInterface::Audio_IIRHPF_Process(void *handle, uint16_t *inbuf, uint16_t *InSample, uint16_t *outbuf, uint16_t *OutSample)
{
	
}

#if 0 //replace .so temporarily
uint32_t AudioBTCVSDControlInterface::CVSD_ENC_GetBufferSize(void)
{
	return 1024;
}

uint32_t AudioBTCVSDControlInterface::CVSD_DEC_GetBufferSize(void)
{
	return 1024;
}
void * AudioBTCVSDControlInterface::CVSD_ENC_Init(int8_t *pBuf)
{
	return NULL;
}

void * AudioBTCVSDControlInterface::CVSD_DEC_Init(int8_t *pBuf)
{
	return NULL;
}

void AudioBTCVSDControlInterface::CVSD_DEC_Process(
   void   *pHandle,  //handle
   char   *pInBuf,   //input CVSD packet
   int    *pInLen,   //input length (Byte)
   short  *pOutBuf,  //output Sample
   int    *pOutLen   //output length (Word)
)
{

}


void AudioBTCVSDControlInterface::CVSD_ENC_Process(
   void   *pHandle,  //handle
   short  *pInBuf,   //input Samples
   int    *pInLen,   //input length (word)
   char   *pOutBuf,  //CVSD packet
   int    *pOutLen,  //output Length (byte)
   short  iGain      //Digital Gain 0~0x7FFF
)
{

}

uint32_t AudioBTCVSDControlInterface::g711plc_GetMemorySize(void)
{
	return 1024;
}

void AudioBTCVSDControlInterface::g711plc_construct(void *pBuf)
{

}

void AudioBTCVSDControlInterface::g711plc_addtohistory(void *lc, int16_t *s)
{

}
void AudioBTCVSDControlInterface::g711plc_dofe(void *lc,int16_t *out)
{

}
#endif

#endif
}
