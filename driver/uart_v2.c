/*****************************************************************************
 *   uart.c:  UART API file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.12.07  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_UART==2
#include "uart.h"

#define TXBUFF_LEN	128

volatile uint32_t UARTStatus;
volatile uint8_t  UARTTxEmpty = 1;
volatile uint32_t UARTCount = 0;

volatile uint32_t TxIdx = 0;
volatile uint32_t TxWIdx = 0;
uint8_t  *TxBuffer;
static SemaphoreHandle_t xTxSem = NULL;
static QueueHandle_t xTxQueue = NULL;
volatile uint32_t DataSend = 0;


#define UART_TX_PARAMETER			( 0x5566UL )
#define	UART_TX_PRIORITY		( tskIDLE_PRIORITY + 1 )
static void UartTxTask( void *pvParameters );

#if CONFIG_UART_DEFAULT_UART_IRQHANDLER==1
/*****************************************************************************
** Function name:		UART_IRQHandler
**
** Descriptions:		UART interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART_IRQHandler(void)
{
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	IIRValue = LPC_UART->IIR;
		
	IIRValue >>= 1;			/* skip pending bit in IIR */
	IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS)		/* Receive Line Status */
	{
#if 0
		LSRValue = LPC_UART->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
		{
		  /* There are errors or break interrupt */
		  /* Read LSR will clear the interrupt */
		  UARTStatus = LSRValue;
		  Dummy = LPC_UART->RBR;	/* Dummy read on RX to clear 
						interrupt, then bail out */
		  return;
		}
		if (LSRValue & LSR_RDR)	/* Receive Data Ready */			
		{
		  /* If no error on RLS, normal ready, save into the data buffer. */
		  /* Note: read RBR will clear the interrupt */
		  UARTBuffer[UARTCount++] = LPC_UART->RBR;
		  if (UARTCount == BUFSIZE)
		  {
		    UARTCount = 0;		/* buffer overflow */
		  }	
		}
#endif
	}
	else if (IIRValue == IIR_RDA)	/* Receive Data Available */
	{
#if 0
		/* Receive Data Available */
		UARTBuffer[UARTCount++] = LPC_UART->RBR;
		if (UARTCount == BUFSIZE)
		{
		  UARTCount = 0;		/* buffer overflow */
		}
#endif
	}
	else if (IIRValue == IIR_CTI)	/* Character timeout indicator */
	{
		/* Character Time-out indicator */
		UARTStatus |= 0x100;		/* Bit 9 as the CTI error */
	}
	else if (IIRValue == IIR_THRE)	/* THRE, transmit holding register empty */
	{
		/* THRE interrupt */
		LSRValue = LPC_UART->LSR;		/* Check status in the LSR to see if
						valid data in U0THR or not */
		if (LSRValue & LSR_THRE)
		{
		  UARTTxEmpty = 1;
		}
		else
		{
		  UARTTxEmpty = 0;
		}
		xQueueSendFromISR( xTxQueue, NULL, &xHigherPriorityTaskWoken );
	}


	/* Now the buffer is empty we can switch context if necessary. */
	if( xHigherPriorityTaskWoken )
	{
		/* Actual macro used here is port specific. */
		taskYIELD ();
	}
	return;
}
#endif


static void UartTxTask( void *pvParameters )
{

	unsigned long ulReceivedValue;
	/* Check the task parameter is as expected. */
	configASSERT( ( ( unsigned long ) pvParameters ) == UART_TX_PARAMETER );

	for( ;; )
	{
#if 0
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, toggle the LED. */
		if( ulReceivedValue == 100UL )
		{
			vMainToggleLED();
			ulReceivedValue = 0U;
		}
#endif
		if( DataSend == 1 && UARTTxEmpty == 1){
			if(xSemaphoreTake(xTxSem, 0)  == pdTRUE){

				LPC_UART->THR = TxBuffer[TxWIdx];
				if(TxWIdx == (TxIdx -1))
					DataSend = 0;
				TxWIdx++;
				UARTTxEmpty = 0;	/* not empty in the THR until it shifts out */

				xSemaphoreGive(xTxSem);
			}

		}else{

		}


	}
}

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART0 port, setup pin select,
**				clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
** 
*****************************************************************************/
void UARTInit(uint32_t baudrate)
{
	uint32_t Fdiv;
	uint32_t regVal;

	UARTTxEmpty = 1;
	UARTCount = 0;
	
	NVIC_DisableIRQ(UART_IRQn);

	LPC_IOCON->PIO1_6 &= ~0x07;    /*  UART I/O config */
	LPC_IOCON->PIO1_6 |= 0x01;     /* UART RXD */
	LPC_IOCON->PIO1_7 &= ~0x07;	
	LPC_IOCON->PIO1_7 |= 0x01;     /* UART TXD */
	/* Enable UART clock */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
	LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

	LPC_UART->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
	regVal = LPC_SYSCON->UARTCLKDIV;

	Fdiv = (((SystemCoreClock*LPC_SYSCON->SYSAHBCLKDIV)/regVal)/16)/baudrate ;	/*baud rate */

	LPC_UART->DLM = Fdiv / 256;							
	LPC_UART->DLL = Fdiv % 256;
	LPC_UART->LCR = 0x03;		/* DLAB = 0 */
	LPC_UART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

	/* Read to clear the line status. */
	regVal = LPC_UART->LSR;

	/* Ensure a clean start, no data in either TX or RX FIFO. */
// CodeRed - added parentheses around comparison in operand of &
	while (( LPC_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
	while ( LPC_UART->LSR & LSR_RDR )
	{
	regVal = LPC_UART->RBR;	/* Dump data from RX FIFO */
	}
	xTxSem = xSemaphoreCreateMutex();
	if(xTxSem == NULL){
		return ;
	}

	TxBuffer = pvPortMalloc(TXBUFF_LEN);
	if(TxBuffer == NULL){
		return ;
	}

	xTxQueue = xQueueCreate( 2, 0 );
	if(xTxQueue == NULL){
		return ;
	}

	xTaskCreate( UartTxTask, "UartTX", configMINIMAL_STACK_SIZE, ( void * ) UART_TX_PARAMETER, UART_TX_PRIORITY, NULL );



#if CONFIG_UART_ENABLE_INTERRUPT==1
	/* Enable the UART Interrupt */
	NVIC_EnableIRQ(UART_IRQn);
#if CONFIG_UART_ENABLE_TX_INTERRUPT==1
	LPC_UART->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART interrupt */
#else
	LPC_UART->IER = IER_RBR | IER_RLS;	/* Enable UART interrupt */
#endif
#endif
	return;
}

/*****************************************************************************
** Function name:		UARTSend
**
** Descriptions:		Send a block of data to the UART 0 port based
**				on the data length
**
** parameters:		buffer pointer, and data length
** Returned value:	None
** 
*****************************************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
	uint32_t wlen = 0;
	if( xTxSem == NULL ){
		return;
	}
	if(xSemaphoreTake(xTxSem, 0)  == pdTRUE){
		while(Length > 0){
			if(Length + TxIdx <= TXBUFF_LEN){
				wlen = Length;
				memcpy(TxBuffer+TxIdx, BufferPtr, wlen);
				Length -= wlen;				
				TxIdx += wlen;
			}else{
				wlen = TXBUFF_LEN - TxIdx;
				memcpy(TxBuffer+TxIdx, BufferPtr, wlen); 
				Length -= wlen;				
				TxIdx = 0;
			} 
		}
		DataSend = 1;
		xSemaphoreGive(xTxSem);
	}
	xQueueSend( xTxQueue, NULL, 0U );
	return;
}
#if 0
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
  uint32_t len = Length;
  uint8_t *Ptr = BufferPtr;
  while ( Length != 0 )
  {
	  /* THRE status, contain valid data */
#if CONFIG_UART_ENABLE_TX_INTERRUPT==1
	  /* Below flag is set inside the interrupt handler when THRE occurs. */
      while ( !(UARTTxEmpty & 0x01) );
	  LPC_UART->THR = *BufferPtr;
      UARTTxEmpty = 0;	/* not empty in the THR until it shifts out */
#else
	  while ( !(LPC_UART->LSR & LSR_THRE) );
	  LPC_UART->THR = *BufferPtr;
#endif
      BufferPtr++;
      Length--;
  }
  UARTSend2(Ptr, len);
  return;
}
#endif

#endif

/******************************************************************************
**                            End Of File
******************************************************************************/
