/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

//#include <string.h>
#include "can.h"

//FDCAN_TxHeaderTypeDef TxHeader;
//FDCAN_RxHeaderTypeDef RxHeader;
//uint8_t RxTxData[64];


void can_init(FDCAN_HandleTypeDef *hcan, FDCAN_GlobalTypeDef *instance)
{
    hcan->Instance = instance;
    hcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hcan->Init.Mode = FDCAN_MODE_NORMAL;
    hcan->Init.AutoRetransmission = DISABLE;
    hcan->Init.TransmitPause = DISABLE;
    hcan->Init.ProtocolException = DISABLE;
    hcan->Init.NominalPrescaler = 6;
    hcan->Init.NominalSyncJumpWidth = 1;
    hcan->Init.NominalTimeSeg1 = 13;
    hcan->Init.NominalTimeSeg2 = 2;
    hcan->Init.DataPrescaler = 6;
    hcan->Init.DataSyncJumpWidth = 1;
    hcan->Init.DataTimeSeg1 = 13;
    hcan->Init.DataTimeSeg2 = 2;
    hcan->Init.StdFiltersNbr = 0;
    hcan->Init.ExtFiltersNbr = 0;
    hcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

}

bool can_set_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
    if ((brp > 0) && (brp <= 1024)
      && (phase_seg1 > 0) && (phase_seg1 <= 16)
      && (phase_seg2 > 0) && (phase_seg2 <= 8)
      && (sjw > 0) && (sjw <= 4)) 
    {
    
        hcan->Init.NominalPrescaler = brp;
        hcan->Init.NominalTimeSeg1 = phase_seg1;
        hcan->Init.NominalTimeSeg2 = phase_seg2;
        hcan->Init.NominalSyncJumpWidth = sjw;
        hcan->Init.DataPrescaler = brp;
        hcan->Init.DataTimeSeg1 = phase_seg1;
        hcan->Init.DataTimeSeg2 = phase_seg2;
        hcan->Init.DataSyncJumpWidth = sjw;   
    }

}

void can_enable(FDCAN_HandleTypeDef *hcan, bool loop_back, bool listen_only, bool one_shot)
{
    FDCAN_FilterTypeDef sFilterConfig;
        
    hcan->Init.AutoRetransmission = ENABLE;
    if (one_shot)
    {
        hcan->Init.AutoRetransmission = DISABLE;    
    }    
    
    hcan->Init.Mode = FDCAN_MODE_NORMAL;
    
    if (loop_back && listen_only)
    {
        hcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;    
    }
    else if (loop_back)
    {
        hcan->Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
    }
    else if (listen_only)
    {
        hcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
    }
    else
    {
        //normal is good
    }
    
    hcan->Init.AutoRetransmission = DISABLE;
    
    HAL_FDCAN_Init(hcan);
        
    /* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF;
    
    HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);
    
    /* Configure global filter on both FDCAN instances:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
           
    // Start CAN using HAL
    HAL_FDCAN_Start(hcan);

}

void can_disable(FDCAN_HandleTypeDef *hcan)
{
    //Stop can using HAL
    HAL_FDCAN_Stop(hcan);
}

bool can_is_enabled(FDCAN_HandleTypeDef *hcan)
{
    if(hcan->State == HAL_FDCAN_STATE_BUSY)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool can_receive(FDCAN_HandleTypeDef *hcan, struct gs_host_frame *rx_frame)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    
    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return false;
    }
    
    rx_frame->can_id = RxHeader.Identifier;
    
    if (RxHeader.IdType == FDCAN_EXTENDED_ID)
    {
        rx_frame->can_id |= CAN_EFF_FLAG;       
         
    }
        
    if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
    {
        rx_frame->can_id |= CAN_RTR_FLAG;
    }
    
    rx_frame->can_dlc = (RxHeader.DataLength & 0x000F0000) >> 16;

    //TODO: change to memcpy to support FDCAN
    //memcpy(rx_frame->data, RxTxData, rx_frame->can_dlc);
    rx_frame->data[0] = RxData[0];
    rx_frame->data[1] = RxData[1];
    rx_frame->data[2] = RxData[2];
    rx_frame->data[3] = RxData[3];
    rx_frame->data[4] = RxData[4];
    rx_frame->data[5] = RxData[5];
    rx_frame->data[6] = RxData[6];
    rx_frame->data[7] = RxData[7];
            
    return true;
}

bool can_send(FDCAN_HandleTypeDef *hcan, struct gs_host_frame *frame)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[16];
    
    if (frame->can_id & CAN_RTR_FLAG)
    {
        TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
    }
    else
    {
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;    
    }

    TxHeader.DataLength = frame->can_dlc << 16;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    if (frame->can_id & CAN_EFF_FLAG) //extended ID
    {
        TxHeader.IdType = FDCAN_EXTENDED_ID;
        TxHeader.Identifier = frame->can_id & 0x1FFFFFFF;
    }
    else
    {
        TxHeader.IdType = FDCAN_STANDARD_ID;
        TxHeader.Identifier = frame->can_id & 0x7FF;
    }
    
    //TODO: change to memcpy to support FDCAN
    //memcpy(RxTxData, frame->data, frame->can_dlc);
    TxData[0] = frame->data[0];
    TxData[1] = frame->data[1];
    TxData[2] = frame->data[2];
    TxData[3] = frame->data[3];
    TxData[4] = frame->data[4];
    TxData[5] = frame->data[5];
    TxData[6] = frame->data[6];
    TxData[7] = frame->data[7];
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
    {
        return false;        
    }
    else
    {
        return true;
    }
}

bool can_get_error_status(FDCAN_HandleTypeDef *hcan, FDCAN_ProtocolStatusTypeDef *status)
{
    if (HAL_FDCAN_GetProtocolStatus(hcan, status) != HAL_OK)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool can_parse_error_status(FDCAN_ProtocolStatusTypeDef *status, struct gs_host_frame *frame)
{
	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->can_dlc = CAN_ERR_DLC;
	frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->data[5] = 0;
	frame->data[6] = 0;
	frame->data[7] = 0;

    if (status->BusOff == 1) 
    {
		frame->can_id |= CAN_ERR_BUSOFF;
	}

	/*
	uint8_t tx_error_cnt = (err>>16) & 0xFF;
	uint8_t rx_error_cnt = (err>>24) & 0xFF;
	*/

    if (status->ErrorPassive == 1) {
		frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
    } else if (status->Warning) {
		frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
	}

    uint8_t lec = status->LastErrorCode;
	if (lec!=0) { /* protocol error */
		switch (lec) {
			case 0x01: /* stuff error */
				frame->can_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_STUFF;
				break;
			case 0x02: /* form error */
				frame->can_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_FORM;
				break;
			case 0x03: /* ack error */
				frame->can_id |= CAN_ERR_ACK;
				break;
			case 0x04: /* bit recessive error */
				frame->can_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_BIT1;
				break;
			case 0x05: /* bit dominant error */
				frame->can_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_BIT0;
				break;
			case 0x06: /* CRC error */
				frame->can_id |= CAN_ERR_PROT;
				frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
				break;
			default:
				break;
		}
	}

	return true;
}
