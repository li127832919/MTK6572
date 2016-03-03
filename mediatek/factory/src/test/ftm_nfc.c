/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <signal.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ctype.h>
#include <dirent.h>
#include <pthread.h>
#include <sys/un.h>
#include <errno.h>   /* Error number definitions */

#include "common.h"
#include "miniui.h"
#include "ftm.h"

#if 1//def FEATURE_FTM_NFC

#include "./../external/mtknfc/inc/mtk_nfc_sys_type_ext.h"
#include "./../../packages/apps/Nfc/mtk-nfc/jni-dload/mtk_nfc_dynamic_load.h"
#include "mtk_nfc_ext_msg.h"


#define TAG             "[NFC]   "
#define INFO_SIZE       (1024)
#define SOCKET_NFC_PORT (7500)
//#define MTKNFC_COMM_SOCK    "/data/nfc_socket/mtknfc_server"
#define MTKNFC_COMM_SOCK    "/data/mtknfc_server"

//#define MTKNFC_COMM_SOCK    "/data/data/com.mediatek.engineermode/mtknfc_server"

#define USING_LOCAL_SOCKET

//extern int NativeDynamicLoad_queryVersion(void);


/*----------------------------------------------------*/
#define MTK_NFC_MAX_ILM_BUFFER_SIZE  (1)

typedef enum {
    MSG_ID_NFC_TEST_REQ = 4,
    MSG_ID_NFC_TEST_RSP,

}MTK_NFC_MSG_TYPE;

typedef enum {
    MOD_NFC,
    MOD_NFC_APP
}MTK_NFC_MOD_TYOE;

typedef enum
{
    NFC_SAP,
    NFC_APP_SAP,
} MTK_NFC_SAP_TYPE;

typedef struct {
    unsigned char used;
    MTK_NFC_MSG_TYPE msg_id;
    MTK_NFC_MOD_TYOE src_mod_id;
    MTK_NFC_MOD_TYOE dest_mod_id;
    MTK_NFC_SAP_TYPE sap_id;
    unsigned char local_para_ptr[MTK_NFC_MAX_ILM_BUFFER_SIZE];
} ilm_struct;

/*----------------------------------------------------*/


int i4TestItem;
int i4TestItem_Wait;

enum {
    ITEM_1,
    ITEM_2,
    ITEM_3,
    ITEM_4,
    ITEM_5,
    ITEM_PASS,
    ITEM_FAIL
};

#if 0
static item_t nfc_items_mt6605[] = {
    item(ITEM_1,    "NFC_SWP_Test"),
    //item(ITEM_2,    "NFC_Tag_UID"),
    item(ITEM_3,    "NFC_Tag_DEP"),
    item(ITEM_4,    "NFC_Card_Mode"),
    item(ITEM_PASS,  "NFC_Test_Pass"),
    item(ITEM_FAIL,  "NFC_Test_Fail"),
    item(-1, NULL),
};
#endif
static item_t nfc_items_mt6605[] = {
    item(ITEM_1,    uistr_info_nfc_swp_test),
    item(ITEM_3,    uistr_info_nfc_tag_dep),
    item(ITEM_4,    uistr_info_nfc_card_mode),
    item(ITEM_5,    uistr_info_nfc_vcard_mode),
    item(ITEM_PASS,  uistr_info_test_pass),
    item(ITEM_FAIL,  uistr_info_test_fail),
    item(-1, NULL),
};


///=== msr3110
static item_t nfc_items_msr3110[] = {
    item(ITEM_1,    "NFC_SWP_Test"),
    //item(ITEM_2,    "NFC_ANT_sf_test"),
    item(ITEM_3,    "NFC_Tag_UID"),
    item(ITEM_4,    "NFC_Card_Mode"),
    item(ITEM_PASS,  "NFC_Test Pass"),
    item(ITEM_FAIL,  "NFC_Test Fail"),
    item(-1, NULL),
};
///===

struct nfc_desc {
    char         info[INFO_SIZE];
    char        *mntpnt;
    bool         exit_thd;
    text_t title;
    text_t text;    
    pthread_t update_thd;
    struct ftm_module *mod;
    struct itemview *iv;
};

#define mod_to_nfc(p)  (struct nfc_desc*)((char*)(p) + sizeof(struct ftm_module))

#define C_INVALID_PID  (-1)   /*invalid process id*/
#define C_INVALID_TID  (-1)   /*invalid thread id*/
#define C_INVALID_FD   (-1)   /*invalid file handle*/
#define C_INVALID_SOCKET (-1) /*invalid socket id*/


int nfc_sockfd = C_INVALID_SOCKET;

///=== msr3110 
pid_t server_pid = C_INVALID_PID;
pid_t server_inside_pid = C_INVALID_PID;
///===

static int nfc_close_mt6605(void)
{  
   int ret=0;
   s_mtk_nfc_main_msg *pnfc_msg;
    
   //Set NEC MEssage
   pnfc_msg = malloc(sizeof(s_mtk_nfc_main_msg));                
   pnfc_msg->msg_type = MTK_NFC_FM_STOP_CMD;
   pnfc_msg->msg_length = 0x00;
   
   ret = write(nfc_sockfd, (const char*)pnfc_msg, sizeof(s_mtk_nfc_main_msg) ); 
    
   return 0;	
}

static int nfc_open_mt6605(void)
{
	  pid_t pid;
    struct sockaddr_in serv_addr;
    struct hostent *server;	
    
    struct sockaddr_un address;
    //int nfc_sockfd = C_INVALID_SOCKET;
    int len;
    
    
    // run gps driver (libmnlp)
    if ((pid = fork()) < 0) 
    {
        LOGD(TAG"nfc_open: fork fails: %d (%s)\n", errno, strerror(errno));
        return (-2);
    } 
    else if (pid == 0)  /*child process*/
    {
        int err;

        LOGD(TAG"nfc_open: execute: %s\n", "/system/xbin/nfcstackp");
        err = execl("/system/xbin/nfcstackp", "nfcstackp", "NFC_TEST_MODE", NULL);
        if (err == -1)
        {
            LOGD(TAG"nfc_open: execl error: %s\n", strerror(errno));
            return (-3);
        }
        return 0;
    } 
    else  /*parent process*/
    {
        //mnl_pid = pid;
        LOGD(TAG"nfc_open: mnl_pid = %d\n", pid);
    }

	  // create socket connection to gps driver
    //portno = SOCKET_NFC_PORT;
    /* Create a socket point */
    #ifdef USING_LOCAL_SOCKET
    //sleep(5);  // sleep 5sec for libmnlp to finish initialization   
    //printf("nfc_open: SELF TEST COD"); 
    //nfc_sockfd = socket(AF_UNIX, SOCK_STREAM, 0); 
    nfc_sockfd = socket(AF_LOCAL, SOCK_STREAM, 0); 
    if (nfc_sockfd < 0)     
    {        
       printf("nfc_open: ERROR opening socket");        
       return (-4);    
    } 

    address.sun_family = AF_LOCAL;//AF_UNIX;
    strcpy (address.sun_path, MTKNFC_COMM_SOCK);
    len = sizeof (address);
            
    sleep(2);  // sleep 5sec for libmnlp to finish initialization        
      
    printf("connecting(%s)...\r\n",address.sun_path);   
            
    /* Now connect to the server */    
    if (connect(nfc_sockfd, (struct sockaddr *)&address, sizeof(address)) < 0)     
    {         
       printf("NFC_Open: ERROR connecting\r\n");         
       return (-6);    
    }    
    #else            
    nfc_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (nfc_sockfd < 0) 
    {
        LOGD(TAG"nfc_open: ERROR opening socket");
        return (-4);
    }
    
    
    bzero((char *) &serv_addr, sizeof(serv_addr));    
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
   
    serv_addr.sin_port = htons(7500);    
    sleep(2);   

    
    /* Now connect to the server */
    if (connect(nfc_sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    {
         LOGD(TAG"GPS_Open: ERROR connecting");
         return (-6);
    }	
    #endif
    if (1) // config Socket read function to non-blocking type
    {
       int x;
       x=fcntl(nfc_sockfd,F_GETFL,0);
       fcntl(nfc_sockfd,F_SETFL,x | O_NONBLOCK);
    }
   
    LOGD(TAG"nfc_open: success\n"); 
    return 0;  
	
}

void vFM_TestItem_1(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int ret, rec_bytes = 0;
    char tmpbuf[1024], *ptr;            	  
    s_mtk_nfc_main_msg *pnfc_msg;
    s_mtk_nfc_fm_swp_test_req pnfc_swp;
    
    //Set NEC MEssage
    pnfc_msg = malloc(sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_fm_swp_test_req));                
    pnfc_msg->msg_type = MTK_NFC_FM_SWP_TEST_REQ;
    pnfc_msg->msg_length = sizeof(s_mtk_nfc_fm_swp_test_req);
    
    pnfc_swp.action = 1;
    pnfc_swp.SEmap = 0;
    
    #ifdef MTK_NFC_SE_SIM1
    pnfc_swp.SEmap += EM_ALS_CARD_M_SW_NUM_SWIO1;
    #endif
    #ifdef MTK_NFC_SE_SIM2
    pnfc_swp.SEmap += EM_ALS_CARD_M_SW_NUM_SWIO2;
    #endif
    #ifdef MTK_NFC_SE_SD
    pnfc_swp.SEmap += EM_ALS_CARD_M_SW_NUM_SWIOSE;
    #endif
    
    LOGD(TAG"SEmap = %d\n",pnfc_swp.SEmap);
    
    memcpy( ((char*)pnfc_msg + sizeof(s_mtk_nfc_main_msg)), &pnfc_swp, sizeof(s_mtk_nfc_fm_swp_test_req) );                
         	  
	  
    memset(nfc->info, '\n', INFO_SIZE);                
    ptr = nfc->info;
    ptr += sprintf(ptr, "%s\n",uistr_info_nfc_testing);
    iv->redraw(iv);     

    ret = write(nfc_sockfd, (const char*)pnfc_msg, (sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_fm_swp_test_req))); 
    
    while(1)
    {
        rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));             
        if (rec_bytes > 0)
        {
        	  char *p;
            s_mtk_nfc_main_msg *pnfc_msg_ptr;
            s_mtk_nfc_fm_swp_test_rsp *presult;
        	  
        	  pnfc_msg_ptr = (s_mtk_nfc_main_msg*) tmpbuf;
        	  
        	  if (pnfc_msg_ptr->msg_type == MTK_NFC_FM_SWP_TEST_RSP)
        	  {
               presult = (s_mtk_nfc_fm_swp_test_rsp *)((char *)pnfc_msg_ptr + sizeof(s_mtk_nfc_main_msg));        	  
               //SHOW RESULT !!
               if(presult->result == 0)
               {
                  ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_pass);	 
               }	
               else
               {
                  ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_fail);	 	
               }
        	  	
               iv->redraw(iv);
               i4TestItem = -1;
               break;
            }
        }
        else
        {
            usleep(100000); // wake up every 0.1sec		
        }
        
        if (nfc->exit_thd)
        {
           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);           
           break;
        }
    }
 
    return;
}


#if 0
void vFM_TestItem_2(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int ret, rec_bytes = 0;
    char tmpbuf[1024], *ptr;            	  
    s_mtk_nfc_main_msg *pnfc_msg;
    s_mtk_nfc_em_als_readerm_req pnfc_taguid;
    
    //Set NEC MEssage
    pnfc_msg = malloc(sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_als_readerm_req));                
    pnfc_msg->msg_type = MTK_NFC_FM_READ_UID_TEST_REQ;
    pnfc_msg->msg_length = sizeof(s_mtk_nfc_em_als_readerm_req);
    
    memset (&pnfc_taguid, 0, sizeof(s_mtk_nfc_em_als_readerm_req));
    
    pnfc_taguid.action = 0x00;
    pnfc_taguid.supporttype = EM_ALS_READER_M_TYPE_A;
    pnfc_taguid.typeA_datarate = EM_ALS_READER_M_SPDRATE_106 + EM_ALS_READER_M_SPDRATE_212 + EM_ALS_READER_M_SPDRATE_424 + EM_ALS_READER_M_SPDRATE_848;
    
    memcpy( ((char*)pnfc_msg + sizeof(s_mtk_nfc_main_msg)), &pnfc_taguid, sizeof(s_mtk_nfc_em_als_readerm_req) );                
         	  
	  
    memset(nfc->info, '\n', INFO_SIZE);                
    ptr = nfc->info;
    ptr += sprintf(ptr, "TEST ...\n");
    iv->redraw(iv);     

    ret = write(nfc_sockfd, (const char*)pnfc_msg, (sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_als_readerm_req))); 
    
    while(1)
    {
        rec_bytes = read(nfc_sockfd, &tmpbuf[0], 1024);  
        
        
        
        LOGD(TAG "%s, rec_bytes = %d \n", __FUNCTION__,rec_bytes);	    
               
        if (rec_bytes > 0)
        {
            s_mtk_nfc_main_msg *pnfc_msg_ptr;
            s_mtk_nfc_em_als_readerm_ntf *presult;
        	  
        	  pnfc_msg_ptr = (s_mtk_nfc_main_msg*) tmpbuf;
        	  
        	  presult = (s_mtk_nfc_em_als_readerm_ntf *) ((char *)pnfc_msg_ptr + sizeof(s_mtk_nfc_main_msg));    
            LOGD(TAG "%s, msg_type =%d, msg_length = %d \n", __FUNCTION__, pnfc_msg_ptr->msg_type, pnfc_msg_ptr->msg_length);	    	  
        	  
        	  if (pnfc_msg_ptr->msg_type == MTK_NFC_FM_READ_UID_TEST_RSP)
        	  {
        	     //SHOW RESULT !!
        	     if(presult->result == 0)
        	     {
        	        int i =0;
        	  	 
        	  	    ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_pass);	
        	        //ptr += sprintf(ptr,"TEST RESULT: PASS \n");	
        	        LOGD(TAG"%d,%d,%d,%d,%d",presult->isNDEF,presult->UidLen,presult->Uid[0],presult->Uid[1],presult->Uid[2]);	
        	     
        	        #if 1
        	        if (presult->isNDEF != 0)
        	        { 
        	           for (i =0;i < presult->UidLen; i++)
        	           {
        	              LOGD(TAG "[%02x]",presult->Uid[i]);
        	           }
        	           LOGD(TAG "\n");
        	        }
        	        #endif
        	     }	
        	     else
        	     {
        	     	  ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_fail);	
        	        //ptr += sprintf(ptr,"TEST RESULT: FAIL \n");	 	
        	     }
        	  	
            iv->redraw(iv);
            i4TestItem = -1;
            break;
           }
        }
        else
        {
            usleep(100000); // wake up every 0.1sec		
        }
        
        if (nfc->exit_thd)
        {
           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);           
           break;
        }
    }
 
    return;
}
#endif


void vFM_TestItem_3(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int ret, rec_bytes = 0;
    char tmpbuf[1024], *ptr;            	  
    s_mtk_nfc_main_msg *pnfc_msg;
    s_mtk_nfc_em_als_readerm_req pnfc_taguid;
    
    //Set NEC MEssage
    pnfc_msg = malloc(sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_als_readerm_req));                
    pnfc_msg->msg_type = MTK_NFC_FM_READ_DEP_TEST_REQ;
    pnfc_msg->msg_length = sizeof(s_mtk_nfc_em_als_readerm_req);
    
    memset (&pnfc_taguid, 0, sizeof(s_mtk_nfc_em_als_readerm_req));
    
    pnfc_taguid.action = 0x00;
    pnfc_taguid.supporttype = EM_ALS_READER_M_TYPE_A;
    pnfc_taguid.typeA_datarate = EM_ALS_READER_M_SPDRATE_106 + EM_ALS_READER_M_SPDRATE_212 + EM_ALS_READER_M_SPDRATE_424 + EM_ALS_READER_M_SPDRATE_848;
    
    memcpy( ((char*)pnfc_msg + sizeof(s_mtk_nfc_main_msg)), &pnfc_taguid, sizeof(s_mtk_nfc_em_als_readerm_req) );                
         	  
	  
    memset(nfc->info, '\n', INFO_SIZE);                
    ptr = nfc->info;
    ptr += sprintf(ptr, "%s\n",uistr_info_nfc_testing);
    iv->redraw(iv);     

    ret = write(nfc_sockfd, (const char*)pnfc_msg, (sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_als_readerm_req))); 
    
    while(1)
    {
        rec_bytes = read(nfc_sockfd, &tmpbuf[0], 1024);  
        
        LOGD(TAG "%s, rec_bytes = %d \n", __FUNCTION__,rec_bytes);	    
               
        if (rec_bytes > 0)
        {
            s_mtk_nfc_main_msg *pnfc_msg_ptr;
            s_mtk_nfc_em_als_readerm_opt_rsp *presult;
        	  
        	  pnfc_msg_ptr = (s_mtk_nfc_main_msg*) tmpbuf;
        	  
        	  presult = (s_mtk_nfc_em_als_readerm_opt_rsp *) ((char *)pnfc_msg_ptr + sizeof(s_mtk_nfc_main_msg));  
        	  
        	  
            LOGD(TAG "%s, msg_type =%d, msg_length = %d \n", __FUNCTION__, pnfc_msg_ptr->msg_type, pnfc_msg_ptr->msg_length);	      	  
        	  
        	  if (pnfc_msg_ptr->msg_type == MTK_NFC_FM_READ_DEP_TEST_RSP)
        	  {
               //SHOW RESULT !!
               if(presult->result == 0)
               {
                  int i =0;        	  	 
                  ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_pass);	
                  //ptr += sprintf(ptr,"TEST RESULT: PASS \n");	
                  //ptr += sprintf(ptr,"%d,%d,%d,%d,%d",presult->isNDEF,presult->UidLen,presult->Uid[0],presult->Uid[1],presult->Uid[2]);	  	     
               }	
               else
               {
               	  ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_fail);	
                  //ptr += sprintf(ptr,"TEST RESULT: FAIL \n");	 	
               }
        	  	
               iv->redraw(iv);
               i4TestItem = -1;
               break;
            }
        }
        else
        {
            usleep(100000); // wake up every 0.1sec		
        }
        
        if (nfc->exit_thd)
        {
           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);           
           break;
        }
    }
 
    return;
}


void vFM_TestItem_4(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int ret, rec_bytes = 0;
    char tmpbuf[1024], *ptr;  
    s_mtk_nfc_main_msg *pnfc_msg;
    s_mtk_nfc_em_als_cardm_req pnfc_cardm;
    
    //Set NEC MEssage
    pnfc_msg = malloc(sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_als_cardm_req));                
    pnfc_msg->msg_type = MTK_NFC_FM_CARD_MODE_TEST_REQ;
    pnfc_msg->msg_length = sizeof(s_mtk_nfc_em_als_cardm_req);
    
    memset (&pnfc_cardm, 0, sizeof(s_mtk_nfc_em_als_cardm_req));
    
    pnfc_cardm.action = 0x00;
    pnfc_cardm.SWNum = 1;
    pnfc_cardm.supporttype = 0x01; //type A/B/F/B'
    pnfc_cardm.fgvirtualcard = 0;
    
    memcpy( ((char*)pnfc_msg + sizeof(s_mtk_nfc_main_msg)), &pnfc_cardm, sizeof(s_mtk_nfc_em_als_cardm_req) );                
         	  
	  
    memset(nfc->info, '\n', INFO_SIZE);                
    ptr = nfc->info;
    ptr += sprintf(ptr, "%s\n",uistr_info_nfc_testing);
    iv->redraw(iv);     

    ret = write(nfc_sockfd, (const char*)pnfc_msg, (sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_als_cardm_req))); 
    
    while(1)
    {
        rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));             
        if (rec_bytes > 0)
        {
        	  char *p;
            s_mtk_nfc_main_msg *pnfc_msg_ptr;
            s_mtk_nfc_em_als_cardm_rsp *presult;
        	  
        	  pnfc_msg_ptr = (s_mtk_nfc_main_msg*) tmpbuf;
        	  
        	  presult = (s_mtk_nfc_em_als_cardm_rsp *)((char *)pnfc_msg_ptr + sizeof(s_mtk_nfc_main_msg));
        	  
        	  //SHOW RESULT !!
        	  if(presult->result == 0)
        	  {
        	     //ptr += sprintf(ptr,"TEST RESULT: CLOSE TO READER \n");	 
        	     ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_nfc_put_dut2reader_cm);	
        	  }	
        	  else
        	  {
        	  	 //ptr += sprintf(ptr,"TEST RESULT: FAIL \n");
        	     ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_fail);		 	
        	  }
        	  	
            iv->redraw(iv);
            i4TestItem = -1;
            break;
        }
        else
        {
            usleep(100000); // wake up every 0.1sec		
        }
        
        if (nfc->exit_thd)
        {
           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);           
           break;
        }
    }
 
    return;
}




void vFM_TestItem_5(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int ret, rec_bytes = 0;
    char tmpbuf[1024], *ptr;  
    s_mtk_nfc_main_msg *pnfc_msg;
    s_mtk_nfc_em_virtual_card_req vCardM;
    
    
    //Set NEC MEssage
    pnfc_msg = malloc(sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_virtual_card_req));                
    pnfc_msg->msg_type = MTK_NFC_FM_VIRTUAL_CARD_REQ;
    pnfc_msg->msg_length = sizeof(s_mtk_nfc_em_virtual_card_req);
    
    memset (&vCardM, 0, sizeof(s_mtk_nfc_em_virtual_card_req));
    
    vCardM.supporttype = EM_ALS_READER_M_TYPE_A + EM_ALS_READER_M_TYPE_B + EM_ALS_READER_M_TYPE_F;
    vCardM.typeF_datarate = EM_ALS_CARD_M_TYPE_F212 + EM_ALS_READER_M_SPDRATE_424;
    
    memcpy( ((char*)pnfc_msg + sizeof(s_mtk_nfc_main_msg)), &vCardM, sizeof(s_mtk_nfc_em_virtual_card_req) );                
         	  
	  
    memset(nfc->info, '\n', INFO_SIZE);                
    ptr = nfc->info;
    ptr += sprintf(ptr, "%s\n",uistr_info_nfc_testing);
    iv->redraw(iv);     

    ret = write(nfc_sockfd, (const char*)pnfc_msg, (sizeof(s_mtk_nfc_main_msg) + sizeof(s_mtk_nfc_em_virtual_card_req))); 
    
    while(1)
    {
        rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));             
        if (rec_bytes > 0)
        {
        	  char *p;
            s_mtk_nfc_main_msg *pnfc_msg_ptr;
            //s_mtk_nfc_em_als_cardm_rsp *presult;
            s_mtk_nfc_em_virtual_card_rsp *presult;
        	  
        	  pnfc_msg_ptr = (s_mtk_nfc_main_msg*) tmpbuf;
        	  
        	  presult = (s_mtk_nfc_em_virtual_card_rsp *)((char *)pnfc_msg_ptr + sizeof(s_mtk_nfc_main_msg));
        	  
        	  //SHOW RESULT !!
        	  if(presult->result == 0)
        	  {
        	     //ptr += sprintf(ptr,"TEST RESULT: CLOSE TO READER \n");	 
        	     ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_nfc_put_dut2reader_vcm);	
        	  }	
        	  else if (presult->result == 0xE3)
        	  {
        	  	 ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_nfc_vcard_removedSIM); 
        	  }
        	  else
        	  {
        	  	 //ptr += sprintf(ptr,"TEST RESULT: FAIL \n");
        	     ptr += sprintf(ptr,"%s:[%s]\n",uistr_info_sensor_alsps_result,uistr_info_fail);		 	
        	  }
        	  	
            iv->redraw(iv);
            i4TestItem = -1;
            break;
        }
        else
        {
            usleep(100000); // wake up every 0.1sec		
        }
        
        if (nfc->exit_thd)
        {
           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);           
           break;
        }
    }
 
    return;
}

static void *nfc_update_iv_thread_mt6605(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int count = 1, chkcnt = 10;
    int init_status;

    LOGD(TAG "%s: Start\n", __FUNCTION__);
    //init NFC driver
    memset(nfc->info, '\n', INFO_SIZE);
    sprintf(nfc->info, "%s\n",uistr_info_nfc_init);
    iv->redraw(iv);
    sleep(1);
    init_status = nfc_open_mt6605();
    
    if(init_status != 0)    // nfc init fail
    {
        memset(nfc->info, '\n', INFO_SIZE);
        //sprintf(nfc->info, "NFC failed! (%d)\n", init_status);
        sprintf(nfc->info, "%s,%s\n", uistr_nfc,uistr_info_test_fail);
        iv->redraw(iv);
    }
    else
    {
    	  int exitThread = 0;
        memset(nfc->info, '\n', INFO_SIZE);
        iv->redraw(iv);
        
        while (!exitThread) 
        {
        	  // SWP TEST
            if(i4TestItem == ITEM_1)
            {
               vFM_TestItem_1(priv);
            }
            
        #if 0
            // TAG UID TEST            
            if(i4TestItem == ITEM_2)
            {
               vFM_TestItem_2(priv);
            }
            #endif 
            // TAG DEP TEST            
            if(i4TestItem == ITEM_3)
            {
               vFM_TestItem_3(priv);
            }
              
            // CARD MODE TEST            
            if(i4TestItem == ITEM_4)
            {
               vFM_TestItem_4(priv);
            }          
            
            // VIRTUAL CARD MODE TEST            
            if(i4TestItem == ITEM_5)
            {
               vFM_TestItem_5(priv);
            }          
            
            
            
            if (nfc->exit_thd)
            {
                LOGD(TAG "%s, nfc->exit_thd = true,exitThread,%d\n", __FUNCTION__,exitThread);
                exitThread = 1;
                break;
            }

            usleep(100000); // wake up every 0.1sec		
            iv->redraw(iv);
        }
    }
    //close NFC driver
    nfc_close_mt6605();
    //close NFC driver done
    LOGD(TAG "%s: Exit\n", __FUNCTION__);
    pthread_exit(NULL);

	return NULL;
}



int nfc_entry_mt6605(struct ftm_param *param, void *priv)
{
    char *ptr;
    int chosen;
    bool exit = false;
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv;

    LOGD(TAG "%s\n", __FUNCTION__);

    init_text(&nfc->title, param->name, COLOR_YELLOW);
    init_text(&nfc->text, &nfc->info[0], COLOR_YELLOW);
    
    nfc->exit_thd = false;
    
      if (!nfc->iv) {
        iv = ui_new_itemview();
        if (!iv) {
            LOGD(TAG "No memory");
            return -1;
        }
        nfc->iv = iv;
    }  
 
 
     iv = nfc->iv;
    iv->set_title(iv, &nfc->title);
    iv->set_items(iv, nfc_items_mt6605, 0);
    iv->set_text(iv, &nfc->text);
    
    pthread_create(&nfc->update_thd, NULL, nfc_update_iv_thread_mt6605, priv);
    
    i4TestItem = -1;
    do {
        chosen = iv->run(iv, &exit);
        LOGD(TAG "%s, chosen = %d\n", __FUNCTION__, chosen);
        switch (chosen) {
        case ITEM_1:
        	   i4TestItem = ITEM_1;
             ;
            break;
        #if 0
        case ITEM_2:
        	   i4TestItem = ITEM_2;
             ;
            break;
        #endif
        case ITEM_3:
        	   i4TestItem = ITEM_3;
             ;
            break;
        case ITEM_4:
        	   i4TestItem = ITEM_4;
             ;
            break;
        case ITEM_5:
        	   i4TestItem = ITEM_5;
             ;
            break;
        case ITEM_PASS:
        case ITEM_FAIL:
            if (chosen == ITEM_PASS) {
                nfc->mod->test_result = FTM_TEST_PASS;
            } else if (chosen == ITEM_FAIL) {
                nfc->mod->test_result = FTM_TEST_FAIL;
            }           
            exit = true;
            break;
        }
        
        if (exit) {
            nfc->exit_thd = true;
            LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
            break;
        }        
        
        usleep(100000); // wake up every 0.1sec		
    } while (1);
    pthread_join(nfc->update_thd, NULL);
       
    
    return 0;
}


/// ========== MSR3110 START =========


void nfc_setop_test(void)
{
    int ret,rec_bytes = 0;
    char tmpbuf[(1024*2)], *ptr, *P1;
    ilm_struct *nfc_ilm_ptr_loc;
    nfc_msg_struct *nfc_msg_struct_loc;
    nfc_script_request nfc_script_request_loc;
    
    //memset(nfc->info, '\n', INFO_SIZE);                
    //ptr = nfc->info;
    //ptr += sprintf(ptr, "Start NFC_SWP_sf_test ...\n");
    //iv->redraw(iv);     
               
    //Set Test message
    nfc_script_request_loc.type = 1;
    nfc_script_request_loc.action = 1;                
    //Set NEC MEssage
    nfc_msg_struct_loc = malloc(sizeof(nfc_msg_struct) + sizeof(nfc_script_request));                
    nfc_msg_struct_loc->msg_type = MSG_ID_NFC_STOP_TEST_REQ;
    nfc_msg_struct_loc->msg_length = sizeof(nfc_script_request);
    P1 = nfc_msg_struct_loc;
    
    P1 += sizeof(nfc_msg_struct_loc);                
    memcpy(P1, &nfc_script_request_loc, sizeof(nfc_script_request) );                
        
    //TRY to SEND MESSAGE to NFC driver!! 
    nfc_ilm_ptr_loc = malloc(sizeof(ilm_struct) + sizeof(nfc_msg_struct) + sizeof(nfc_script_request));
    nfc_ilm_ptr_loc->msg_id = MSG_ID_NFC_TEST_REQ;  
    nfc_ilm_ptr_loc->src_mod_id = 0;
    memcpy((nfc_ilm_ptr_loc->local_para_ptr), nfc_msg_struct_loc, sizeof(nfc_msg_struct) );           
   
    ret = write(nfc_sockfd, (const char*)nfc_ilm_ptr_loc, sizeof(ilm_struct)); 
                    
    //Free memory 
    free(nfc_ilm_ptr_loc);
    nfc_ilm_ptr_loc= NULL;                
    free(nfc_msg_struct_loc);
    nfc_msg_struct_loc= NULL;	
}


static int nfc_open_msr3110(void)
{
	//pid_t pid1,pid2;
    //int portno;
    struct sockaddr_in serv_addr;
    struct hostent *server;	
        
    #ifdef MTK_NFC_INSIDE
    // Start the Open NFC Server
    if ((server_inside_pid = fork()) < 0) 
    {
        LOGD(TAG"nfc_open_msr3110: fork fails: %d (%s)\n", errno, strerror(errno));
        return (-2);
    } 
    else if (server_inside_pid == 0)  /*child process*/
    {
        int err;

        LOGD(TAG"nfc_open_msr3110: execute: %s\n", "/system/bin/server_open_nfc");
        err = execl("/system/bin/server_open_nfc", "server_open_nfc", NULL);
        if (err == -1)
        {
            LOGD(TAG"nfc_open_msr3110: execl server_open_nfc error: %s\n", strerror(errno));
            return (-3);
        }
        return 0;
    } 
    else  /*parent process*/
    {
        //mnl_pid = pid;
        
        //server_inside_pid = pid1;
        LOGD(TAG"nfc_open_msr3110: mnl_pid = %d\n", server_inside_pid);
    }
    #endif

    // run gps driver (libmnlp)
    if ((server_pid = fork()) < 0) 
    {
        LOGD(TAG"nfc_open_msr3110: fork fails: %d (%s)\n", errno, strerror(errno));
        return (-2);
    } 
    else if (server_pid == 0)  /*child process*/
    {
        int err;

        LOGD(TAG"nfc_open_msr3110: execute: %s\n", "/system/xbin/nfcservice");
        err = execl("/system/xbin/nfcservice", "nfcservice", NULL);
        //err = execl("/system/xbin/libmnlp", "libmnlp", NULL);
        if (err == -1)
        {
            LOGD(TAG"nfc_open_msr3110: execl error: %s\n", strerror(errno));
            return (-33);
        }
        return 0;
    } 
    else  /*parent process*/
    {
        //mnl_pid = pid;
        //server_pid = pid2;
        LOGD(TAG"nfc_open_msr3110: mnl_pid = %d\n", server_pid);
    }

	
	  // create socket connection to gps driver
    //portno = SOCKET_NFC_PORT;
    /* Create a socket point */
    nfc_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (nfc_sockfd < 0) 
    {
        LOGD(TAG"nfc_open_msr3110: ERROR opening socket");
        return (-4);
    }
    server = gethostbyname("127.0.0.1");
    if (server == NULL) {
        LOGD(TAG"nfc_open_msr3110: ERROR, no such host\n");
        return (-5);
    }


    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(SOCKET_NFC_PORT);

    sleep(2);  // sleep 5sec for libmnlp to finish initialization
    
    /* Now connect to the server */
    if (connect(nfc_sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    {
         LOGD(TAG"nfc_open_msr3110: ERROR connecting");
         return (-6);
    }	

    {
    int x;
    x=fcntl(nfc_sockfd,F_GETFL,0);
    fcntl(nfc_sockfd,F_SETFL,x | O_NONBLOCK);
    }
    
   
    LOGD(TAG"nfc_open_msr3110: step1\n"); 
    return 0;  
	
}


static void *nfc_update_iv_thread_msr3110(void *priv)
{
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv = nfc->iv;
    int count = 1, chkcnt = 10;
    int init_status;

    LOGD(TAG "%s: Start\n", __FUNCTION__);
    //init GPS driver
    memset(nfc->info, '\n', INFO_SIZE);
    sprintf(nfc->info, "nfc initializing...\n");
    iv->redraw(iv);
    sleep(1);
    init_status = nfc_open_msr3110();
    //init_status = 1;
    
    if(init_status != 0)    // nfc init fail
    {
        memset(nfc->info, '\n', INFO_SIZE);
        sprintf(nfc->info, "NFC failed! (%d)\n", init_status);
        iv->redraw(iv);
    }
    else
    {
        //init driver done
        memset(nfc->info, '\n', INFO_SIZE);
        iv->redraw(iv);
        
        while (1) {
            usleep(100000); // wake up every 0.1sec
            //chkcnt--;

            if(i4TestItem == ITEM_1)
            {
            	  int ret,rec_bytes = 0;
            	  char tmpbuf[(1024*2)], *ptr, *P1;
            	  ilm_struct *nfc_ilm_ptr_loc;
            	  nfc_msg_struct *nfc_msg_struct_loc;
            	  nfc_script_request nfc_script_request_loc;
            	  
                memset(nfc->info, '\n', INFO_SIZE);                
                ptr = nfc->info;
                ptr += sprintf(ptr, "Start NFC_SWP_sf_test ...\n");
                iv->redraw(iv);                
                //Set Test message
                nfc_script_request_loc.type = 1;
                nfc_script_request_loc.action = 1;                
                //Set NEC MEssage
                nfc_msg_struct_loc = malloc(sizeof(nfc_msg_struct) + sizeof(nfc_script_request));                
                nfc_msg_struct_loc->msg_type = MSG_ID_NFC_SWP_SELF_TEST_REQ;
                nfc_msg_struct_loc->msg_length = sizeof(nfc_script_request);
                P1 = nfc_msg_struct_loc;
                
                P1 += sizeof(nfc_msg_struct_loc);                
                memcpy(P1, &nfc_script_request_loc, sizeof(nfc_script_request) );                
                    
                //TRY to SEND MESSAGE to NFC driver!! 
                nfc_ilm_ptr_loc = malloc(sizeof(ilm_struct) + sizeof(nfc_msg_struct) + sizeof(nfc_script_request));
                nfc_ilm_ptr_loc->msg_id = MSG_ID_NFC_TEST_REQ;  
                nfc_ilm_ptr_loc->src_mod_id = 0;
                memcpy((nfc_ilm_ptr_loc->local_para_ptr), nfc_msg_struct_loc, sizeof(nfc_msg_struct) );           

                ret = write(nfc_sockfd, (const char*)nfc_ilm_ptr_loc, sizeof(ilm_struct)); 
                                
                //Free memory 
                free(nfc_ilm_ptr_loc);
                nfc_ilm_ptr_loc= NULL;                
                free(nfc_msg_struct_loc);
                nfc_msg_struct_loc= NULL;
                
                   
                while(1)
                {
                
                    rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));             
                    

                    if (rec_bytes > 0)
                    {
                    	  char *p;
            	          ilm_struct *nfc_ilm_ptr;
                    	  nfc_script_response *nfc_script_response_ptr;
                    	  
                    	  nfc_ilm_ptr = (ilm_struct*) tmpbuf;
                    	  
                    	  p = nfc_ilm_ptr->local_para_ptr;
                    	  p += sizeof(nfc_msg_struct);
                    	  
                    	  nfc_script_response_ptr = (nfc_script_response*) p;
                    	  
                    	  
                    	  if(nfc_script_response_ptr->result == 0)
                    	  {
                    	     ptr += sprintf(ptr,"TEST RESULT: PASS \n");	 
                    	  }	
                    	  else
                    	  {
                    	  	 ptr += sprintf(ptr,"TEST RESULT: FAIL [0x%02x]\n", nfc_script_response_ptr->result);	 	
                    	  }
                    	  	
                        iv->redraw(iv);
                        i4TestItem = -1;
                        break;
                    }
                    else
                    {
                        
                        if (nfc->exit_thd)
                        {
                           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
                           nfc_setop_test();     
                           usleep(10000); // wake up every 10msec  
                           break;
                        }
                    }
                	
                }
               
             }
         
            if(i4TestItem == ITEM_2)
            {
                int ret,rec_bytes = 0;
                char tmpbuf[(1024*2)], *ptr, *P1;
                ilm_struct *nfc_ilm_ptr_loc;
                nfc_msg_struct *nfc_msg_struct_loc;
                nfc_script_request nfc_script_request_loc;
            	  
                memset(nfc->info, '\n', INFO_SIZE);                
                ptr = nfc->info;
                ptr += sprintf(ptr, "Start NFC_ANT_sf_test ...\n");
                iv->redraw(iv);
                
                //Set Test message
                nfc_script_request_loc.type = 1;
                nfc_script_request_loc.action = 1;                
                //Set NEC MEssage
                nfc_msg_struct_loc = malloc(sizeof(nfc_msg_struct) + sizeof(nfc_script_request));                
                nfc_msg_struct_loc->msg_type = MSG_ID_NFC_ANTENNA_SELF_TEST_REQ;
                nfc_msg_struct_loc->msg_length = sizeof(nfc_script_request);
                P1 = nfc_msg_struct_loc;                
                P1 += sizeof(nfc_msg_struct);         
                      
                memcpy(P1, &nfc_script_request_loc, sizeof(nfc_script_request) );
                                    
                //TRY to SEND MESSAGE to NFC driver!! 
                nfc_ilm_ptr_loc = malloc(sizeof(ilm_struct) + sizeof(nfc_msg_struct) + sizeof(nfc_script_request));
                nfc_ilm_ptr_loc->msg_id = MSG_ID_NFC_TEST_REQ; 
                nfc_ilm_ptr_loc->src_mod_id = 0;
                memcpy((nfc_ilm_ptr_loc->local_para_ptr), nfc_msg_struct_loc, sizeof(nfc_msg_struct) );           

                ret = write(nfc_sockfd, (const char*)nfc_ilm_ptr_loc, sizeof(ilm_struct)); 
                
                //Free memory 
                free(nfc_ilm_ptr_loc);
                nfc_ilm_ptr_loc= NULL;
                
                free(nfc_msg_struct_loc);
                nfc_msg_struct_loc= NULL;
                
                while(1)
                {
                    rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));          
                    if (rec_bytes > 0)
                    {
                    	  char *p;
            	          ilm_struct *nfc_ilm_ptr;
                    	  nfc_script_response *nfc_script_response_ptr;
                    	  
                    	  nfc_ilm_ptr = (ilm_struct*) tmpbuf;
                    	  
                    	  p = nfc_ilm_ptr->local_para_ptr;
                    	  p += sizeof(nfc_msg_struct);
                    	  
                    	  nfc_script_response_ptr = (nfc_script_response*) p;
                    	  
                    	  
                    	  if(nfc_script_response_ptr->result == 0)
                    	  {
                    	     ptr += sprintf(ptr,"TEST RESULT: PASS \n");	 
                    	  }	
                    	  else
                    	  {
                    	  	 ptr += sprintf(ptr,"TEST RESULT: FAIL \n");	 	
                    	  }
                    	  	
                        iv->redraw(iv);
                        i4TestItem = -1;
                        break;
                    }
                    else
                    {
                                    
                        
                        if (nfc->exit_thd)
                        {
                           LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
                           nfc_setop_test();     
                           usleep(10000); // wake up every 10msec  
                           break;
                        }
                    }
                	
                }
               
             }     
         
             
       
            if(i4TestItem == ITEM_3)
            {
            	  int ret,rec_bytes = 0;
            	  char tmpbuf[(1024*2)], *ptr, *P1;
            	  ilm_struct *nfc_ilm_ptr_loc;
            	  nfc_msg_struct *nfc_msg_struct_loc;
            	  nfc_script_uid_request nfc_script_uid_request_loc;
            	  
                memset(nfc->info, '\n', INFO_SIZE);                
                ptr = nfc->info;
                ptr += sprintf(ptr, "Start NFC_Tag_Uid ...\n");
                iv->redraw(iv);
                                
                //Set Test message
                nfc_script_uid_request_loc.type = 1;
                nfc_script_uid_request_loc.action = 1;
                
                //Set NEC MEssage
                nfc_msg_struct_loc = malloc(sizeof(nfc_msg_struct) + sizeof(nfc_script_uid_request));
                
                nfc_msg_struct_loc->msg_type = MSG_ID_NFC_TAG_UID_RW_REQ;
                nfc_msg_struct_loc->msg_length = sizeof(nfc_script_uid_request);
                //P1 = nfc_msg_struct_loc;
                
                //P1 += sizeof(nfc_msg_struct);
                
                //memcpy(P1, &nfc_msg_struct_loc, sizeof(nfc_script_uid_request) );
                
                                    
                //TRY to SEND MESSAGE to NFC driver!! 
                nfc_ilm_ptr_loc = malloc(sizeof(ilm_struct) + sizeof(nfc_msg_struct) + sizeof(nfc_script_uid_request));
                nfc_ilm_ptr_loc->msg_id = MSG_ID_NFC_TEST_REQ; 
                nfc_ilm_ptr_loc->src_mod_id = 0;
                memcpy((nfc_ilm_ptr_loc->local_para_ptr), nfc_msg_struct_loc, sizeof(nfc_script_uid_request) );           

                ret = write(nfc_sockfd, (const char*)nfc_ilm_ptr_loc, sizeof(ilm_struct)); 
                
                //Free memory 
                free(nfc_ilm_ptr_loc);
                nfc_ilm_ptr_loc= NULL;
                
                free(nfc_msg_struct_loc);
                nfc_msg_struct_loc= NULL;
                
                
                ptr += sprintf(ptr,"\n ****Please Put a TAG close to RF**** \n");
                iv->redraw(iv);
                while(1)
                {
                    rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));           
                    if (rec_bytes > 0)
                    {
             	          char *p;
            	          ilm_struct *nfc_ilm_ptr;
                    	  nfc_script_uid_response *nfc_script_uid_response_ptr;
                    	  
                    	  nfc_ilm_ptr = (ilm_struct*) tmpbuf;
                    	  
                    	  p = nfc_ilm_ptr->local_para_ptr;
                    	  p += sizeof(nfc_msg_struct);
                    	  
                    	  nfc_script_uid_response_ptr = (nfc_script_uid_response*) p;
                    	  
                    	  
                    	  if(nfc_script_uid_response_ptr->result == 0)
                    	  {
                    	   
                    	#if 1
                    	    int i, length;
                      	    ptr += sprintf(ptr,"TEST RESULT: PASS \nUid,");
                             
                            //if (nfc_script_uid_response_ptr->uid_type == 1) {
                            //    length = 4;
                            //} else {
                            //    length = 7;
                            //}
                            
                    	    for(i=0;i< nfc_script_uid_response_ptr->uid_type;i++){
                            //for(i=0; i< length; i++) {
                    	       ptr += sprintf(ptr,"[%02x],", nfc_script_uid_response_ptr->data[i]);
                    	    }
                        #else
                    	     ptr += sprintf(ptr,"TEST RESULT: PASS \n");
                    	     ptr += sprintf(ptr,"Uid,[%01x],[%01x],[%01x],[%01x],[%01x],[%01x],[%01x], \n",
                    	                       nfc_script_uid_response_ptr->data[0],
                    	                       nfc_script_uid_response_ptr->data[1],
                    	                       nfc_script_uid_response_ptr->data[2],
                    	                       nfc_script_uid_response_ptr->data[3],
                    	                       nfc_script_uid_response_ptr->data[4],
                    	                       nfc_script_uid_response_ptr->data[5],
                    	                       nfc_script_uid_response_ptr->data[6]);
                    	#endif
                    	  }	
                    	  else
                    	  {
                    	  	 ptr += sprintf(ptr,"TEST RESULT: FAIL \n");	 	
                    	  }
                    	  	
                        iv->redraw(iv);
                        i4TestItem = -1;
                        break;
                    }
                    else
                    {
                	
                        if (nfc->exit_thd)
                        {
                            LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
                            nfc_setop_test();     
                            usleep(10000); // wake up every 10msec  
                            break;
                        }
                    }
                }
               
             }     
               
             
             if(i4TestItem == ITEM_4)
            {
            	  int ret,rec_bytes = 0;
            	  char tmpbuf[(1024*2)], *ptr, *P1;
            	  ilm_struct *nfc_ilm_ptr_loc;
            	  nfc_msg_struct *nfc_msg_struct_loc;
                nfc_script_request nfc_script_request_loc;
            	  
                memset(nfc->info, '\n', INFO_SIZE);                
                ptr = nfc->info;
                ptr += sprintf(ptr, "Start NFC_Card_Mode_test ...\n");
                iv->redraw(iv);
                
                //Set Test message
                nfc_script_request_loc.type = 1;
                nfc_script_request_loc.action = 1;                
                //Set NEC MEssage
                nfc_msg_struct_loc = malloc(sizeof(nfc_msg_struct) + sizeof(nfc_script_request));                
                nfc_msg_struct_loc->msg_type = MSG_ID_NFC_CARD_EMULATION_MODE_TEST_REQ;
                nfc_msg_struct_loc->msg_length = sizeof(nfc_script_request);
                P1 = nfc_msg_struct_loc;                
                P1 += sizeof(nfc_msg_struct);
                
                memcpy(P1, &nfc_script_request_loc, sizeof(nfc_script_request) );
                    
                //TRY to SEND MESSAGE to NFC driver!! 
                nfc_ilm_ptr_loc = malloc(sizeof(ilm_struct) + sizeof(nfc_msg_struct) + sizeof(nfc_script_request));
                nfc_ilm_ptr_loc->msg_id = MSG_ID_NFC_TEST_REQ; 
                nfc_ilm_ptr_loc->src_mod_id = 0;
                memcpy((nfc_ilm_ptr_loc->local_para_ptr), nfc_msg_struct_loc, sizeof(nfc_msg_struct) );           

                ret = write(nfc_sockfd, (const char*)nfc_ilm_ptr_loc, sizeof(ilm_struct)); 
                
                //Free memory 
                free(nfc_ilm_ptr_loc);
                nfc_ilm_ptr_loc= NULL;
                
                free(nfc_msg_struct_loc);
                nfc_msg_struct_loc= NULL;
                
                while(1)
                {
                    rec_bytes = read(nfc_sockfd, &tmpbuf[0], sizeof(tmpbuf));  
                    if (rec_bytes > 0)
                    {
                    	char *p;
            	        ilm_struct *nfc_ilm_ptr;
                    	nfc_script_response *nfc_script_response_ptr;
                    	  
                    	  nfc_ilm_ptr = (ilm_struct*) tmpbuf;
                    	  
                    	  p = nfc_ilm_ptr->local_para_ptr;
                    	  p += sizeof(nfc_msg_struct);
                    	  
                    	nfc_script_response_ptr = (nfc_script_response*) p;
                    	  
                    	  
                    	if(nfc_script_response_ptr->result == 0)
                    	{
                    	    ptr += sprintf(ptr,"TEST RESULT: PASS \n");	 
                    	}	
                    	else
                    	{
                    	    ptr += sprintf(ptr,"TEST RESULT: FAIL \n");	 	
                    	}
                    	                      	  
                        iv->redraw(iv);
                        i4TestItem = -1;
                        break;
                    }
                    else
                    {
                        if (nfc->exit_thd)
                        {
                            LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
                            nfc_setop_test();     
                            usleep(10000); // wake up every 10msec  
                            break;
                        }
                    }
                	
                }
               
             }
             
             
            if (nfc->exit_thd)
             {
                LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
                break;
            }

            iv->redraw(iv);
        }
    }
                
    if (nfc_sockfd != C_INVALID_SOCKET)  
    {   
        close(nfc_sockfd);    
        nfc_sockfd = C_INVALID_SOCKET;
    }
                
               
    if(server_inside_pid != C_INVALID_PID) 
    {
        int s;
        if(waitpid(server_inside_pid, &s, WNOHANG) > 0) 
        {   
            kill(server_inside_pid, SIGKILL);   
        }
        //kill(server_inside_pid, SIGKILL); 
        usleep(500000); //500ms 
    }
    
    if(server_pid != C_INVALID_PID) 
    { 
    	  int s;
    	  if(waitpid(server_pid, &s, WNOHANG) > 0)
    	  {
    	     kill(server_pid, SIGKILL);   
        }
        //kill(server_pid, SIGKILL); 
        usleep(500000); //500ms 
    }
    
    LOGD(TAG "%s: Exit\n", __FUNCTION__);
    pthread_exit(NULL);

	return NULL;
}



int nfc_entry_msr3110(struct ftm_param *param, void *priv)
{
    char *ptr;
    int chosen;
    bool exit = false;
    struct nfc_desc *nfc = (struct nfc_desc *)priv;
    struct itemview *iv;

    LOGD(TAG "%s\n", __FUNCTION__);

    init_text(&nfc->title, param->name, COLOR_YELLOW);
    init_text(&nfc->text, &nfc->info[0], COLOR_YELLOW);
    
    nfc->exit_thd = false;
    
    if (!nfc->iv) {
        iv = ui_new_itemview();
        if (!iv) {
            LOGD(TAG "No memory");
            return -1;
        }
        nfc->iv = iv;
    }  
 
 
    iv = nfc->iv;
    iv->set_title(iv, &nfc->title);
    iv->set_items(iv, nfc_items_msr3110, 0);
    iv->set_text(iv, &nfc->text);
    
    pthread_create(&nfc->update_thd, NULL, nfc_update_iv_thread_msr3110, priv);
    
    i4TestItem = -1;
    do {
        chosen = iv->run(iv, &exit);
        LOGD(TAG "%s, chosen = %d\n", __FUNCTION__, chosen);
        switch (chosen) {
        case ITEM_1:
        	   i4TestItem = ITEM_1;
             ;
            break;
        case ITEM_2:
        	   i4TestItem = ITEM_2;
             ;
            break;
        case ITEM_3:
        	   i4TestItem = ITEM_3;
             ;
            break;
        
        case ITEM_4:
        	   i4TestItem = ITEM_4;
             ;
            break;
        #if 0    
        case ITEM_20:
        	   i4TestItem = ITEM_20;
             ;
            break;
        #endif
        case ITEM_PASS:
        case ITEM_FAIL:
            if (chosen == ITEM_PASS) {
                nfc->mod->test_result = FTM_TEST_PASS;
            } else if (chosen == ITEM_FAIL) {
                nfc->mod->test_result = FTM_TEST_FAIL;
            }           
            exit = true;
            break;
        }
        
        if (exit) {
            nfc->exit_thd = true;
            LOGD(TAG "%s, nfc->exit_thd = true\n", __FUNCTION__);
            break;
        }        
    } while (1);
    
    pthread_join(nfc->update_thd, NULL);
           
    return 0;
}





/// ========== MSR3110 END =========


int nfc_init(void)
{
    int ret = 0,handle = 0,version = 0x01;
    struct ftm_module *mod;
    struct nfc_desc *nfc;

    LOGD(TAG "%s\n", __FUNCTION__);
    
    #if 0    
    handle = open("/dev/mt6605", O_RDWR | O_NOCTTY);
    //query_verison
    // Return Value 
    //   0x01: 3110 
    //   0x02: 6605
    //   0xFF: Error
    if(handle != 0x00)
    {
       version = ioctl(handle,0xFEFE, 0x00);    
       close(handle); 
    }
    #endif
    
    version = NativeDynamicLoad_queryVersion();
    
    LOGD(TAG "%s,version,%d\n", __FUNCTION__, version);
    
    mod = ftm_alloc(ITEM_NFC, sizeof(struct nfc_desc));
    nfc  = mod_to_nfc(mod);

    nfc->mod      = mod;
    
    i4TestItem = 0;
    i4TestItem_Wait=0;

    if (!mod)
    {
        return -ENOMEM;
    }

    if (version == 0x01)
    {
        //Doing MSR3110 init
        ret = ftm_register(mod, nfc_entry_msr3110, (void*)nfc);
    }
    else
    {
        ret = ftm_register(mod, nfc_entry_mt6605, (void*)nfc);   		
    }

    return ret;
}
#endif

