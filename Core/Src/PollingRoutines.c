#include "PollingRoutines.h"
#include "main.h"
#include "UsbAndCanConvert.h"
#include "usbd_conf.h" // path to USBD_CUSTOMHID_OUTREPORT_BUF_SIZE define

#include "CAN_Buffer.h"
#include "protocolCommands.h"
#include "USB_Buffer.h"
#include "GPIO_Ports.h"


// these variables are defined in main.c
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t usbRxBuffer[USB_MAX_RING_BUFF][USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
extern RING_BUFF_INFO usbRxRingBuffPtr;

uint8_t USB_TX_Buffer[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE]; // To send usb data to PC
uint8_t canBusActive = 0;

const char* Version = "v1.0.1";
const char* Hardware = "BT-CRUXv2.0";
const char* Frequency = "APB1_36mHz";// this is the APB1 clock frequency


/*
 *  Description: The main entry point. Typically in all my projects I call this routine from main all do all my polling here, if i am not using Tasks.
 *
 */
void PollingRoutine(void){
	// turn on led to indicate device is running
	PortC_On(LED_Red_Pin);

	ParseUsbRec();
	SendUsbMessage();


	// CAN1
	SendCanTxMessage1(&hcan1);
	//CAN2
	SendCanTxMessage2(&hcan2);

	ParseCanRec();

	BlinkkLed();
}

/*
 * function: blink blue led when there is CAN bus activity
 *
 */
void BlinkkLed(void) {
	static uint32_t ledBlinkMode = 0, ledDelayCount;
	switch(ledBlinkMode)  {
	case 0:
		if(canBusActive) {
			canBusActive = 0;
			PortC_On(LED_Blue_Pin);
			ledBlinkMode++;
		}
		break;
	case 1:
		if(++ledDelayCount > 2000) {
			PortC_Off(LED_Blue_Pin);
			ledBlinkMode++;
			ledDelayCount = 0;
		}
		break;
	case 2:
		if(++ledDelayCount > 50000) {
			ledBlinkMode = 0;
			ledDelayCount = 0;
		}
		break;
	}
}

/*
 * function: Parse the USB data in the buffer.
 * input: none
 * output: none
 *
 */
void ParseUsbRec(void) {
	uint8_t usbData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
	if(UsbDataAvailable(usbData)) {
		uint8_t node = GetNode(usbData);
		switch(usbData[0])
		{
		case COMMAND_MESSAGE:
			switch(node) {
			case CAN1_NODE:
				SendUsbDataToCanBus(CAN1_NODE, usbData);
				break;
			case CAN2_NODE:
				SendUsbDataToCanBus(CAN2_NODE, usbData);
				break;
			}
			break;
		case COMMAND_BAUD:
			switch(usbData[5]) { // index 5 is node
			case CAN1_NODE:
				CanSnifferCanInit(&hcan1, usbData);
				break;
			case CAN2_NODE:
				CanSnifferCanInit(&hcan2, usbData);
				break;
			}
			break;
		case COMMAND_INFO:
			SendHardwareInfo();
			SendVersionInfo();
			SendFrequency();
			switch(node) {
			case CAN1_NODE:
				Send_CAN_BTR(&hcan1);
				break;
			case CAN2_NODE:
				Send_CAN_BTR(&hcan2);
				break;
			}
			break;
		}
	}
}

void SendHardwareInfo(void) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint8_t i = 0;
	data[0] = COMMAND_HARDWARE;
	while( Hardware[i] != '\0') {
		data[i + 1] = (uint8_t) Hardware[i];
		i++;
	}
	AddUsbTxBuffer(data);
}

void SendVersionInfo(void) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint8_t i = 0;
	data[0] = COMMAND_VERSION;
	while( Version[i] != '\0') {
		data[i + 1] = (uint8_t) Version[i];
		i++;
	}
	AddUsbTxBuffer(data);
}

void SendFrequency(void) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint8_t i = 0;
	data[0] = COMMAND_FREQUENCY;
	while( Frequency[i] != '\0') {
		data[i + 1] = (uint8_t) Frequency[i];
		i++;
	}
	AddUsbTxBuffer(data);
}

void Send_CAN_BTR(CAN_HandleTypeDef *hcan) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint32_t btrValue = READ_REG(hcan->Instance->BTR);

	data[0] = COMMAND_CAN_BTR;
	data[1] = btrValue >> 24 & 0xFF;
	data[2] = btrValue >> 16 & 0xFF;
	data[3] = btrValue >> 8 & 0xFF;
	data[4] = btrValue & 0xFF;
	if(hcan->Instance == hcan1.Instance){
		data[5] = CAN1_NODE;
	} else if(hcan->Instance == hcan2.Instance) {
		data[5] = CAN2_NODE;
	}

	AddUsbTxBuffer(data);
}

/*
 * function: Parse the CAN data in the buffer.
 * input: none
 * output: none
 *
 */
void ParseCanRec(void) {
	uint8_t canMsgAvailableFlag = 0;
	CanRxMsgTypeDef canRxMsg;
	uint8_t usbData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];

	// CAN1
	memset(&usbData, 0, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

	canMsgAvailableFlag = Can1DataAvailable(&canRxMsg); // check ring buffer for new message
	if(canMsgAvailableFlag) {
		if(canRxMsg.CAN_RxHeaderTypeDef.IDE == CAN_EXT_ID) { // EXT ID
			SendCanDataToUsb(&canRxMsg, CAN1_NODE);
		} else { // STD ID
			SendCanDataToUsb(&canRxMsg, CAN1_NODE);
		}
	}

	// CAN2
	memset(&usbData, 0, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

	canMsgAvailableFlag = Can2DataAvailable(&canRxMsg); // check ring buffer for new message
	if(canMsgAvailableFlag) {
		if(canRxMsg.CAN_RxHeaderTypeDef.IDE == CAN_EXT_ID) { // EXT ID
			SendCanDataToUsb(&canRxMsg, CAN2_NODE);
		} else { // STD ID
			SendCanDataToUsb(&canRxMsg, CAN2_NODE);
		}
	}
}


/*
 * function: This is copied from the CAN_Buffer.c file. You can use this to toggle LED to indicate CAN bus activity
 * input: On or Off state of LED
 * output: none
 */
void CanBusActivityStatus(uint8_t status) {
	canBusActive = status;
}

uint8_t GetCanBusActive(void) {
	return canBusActive;
}

/*
 * Description: Changes the CAN handle baud rate received from the PC. Use the calculator from "bittiming.can-wiki.info" to get the CAN_BTR value
 * Input: the CAN Handle and the CAN_BTR value
 * Output: none
 */
void CanSnifferCanInit(CAN_HandleTypeDef *hcan, uint8_t *data) {

	uint32_t btrValue = 0;
	uint8_t usbData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};

	btrValue = data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4]; // parse the BTR data

	// some of these snippets were copied from HAL_CAN_Init()
	HAL_CAN_DeInit(hcan);

	if (hcan->State == HAL_CAN_STATE_RESET)
	{
		/* Init the low level hardware: CLOCK, NVIC */
		HAL_CAN_MspInit(hcan);
	}

	/* Set the bit timing register */
	WRITE_REG(hcan->Instance->BTR, (uint32_t)(btrValue));

	/* Initialize the error code */
	hcan->ErrorCode = HAL_CAN_ERROR_NONE;

	/* Initialize the CAN state */
	hcan->State = HAL_CAN_STATE_READY;

	if(HAL_CAN_Start(hcan) != HAL_OK) { // start the CAN module
		usbData[0] = COMMAND_NAK; // NAK PC
		AddUsbTxBuffer(usbData);
		return;
	}

	usbData[0] = COMMAND_ACK; // ACK PC back
	AddUsbTxBuffer(usbData);
}


