/*
 * Copyright (c) 2015 - 2016 , Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * main-blinky.c is included when the "Blinky" build configuration is used.
 * main-full.c is included when the "Full" build configuration is used.
 *
 * main-blinky.c (this file) defines a very simple demo that creates two tasks,
 * one queue, and one timer.  It also demonstrates how Cortex-M4 interrupts can
 * interact with FreeRTOS tasks/timers.
 *
 * This simple demo project runs 'stand alone' (without the rest of the tower
 * system) on the Freedom Board or Validation Board, which is populated with a
 * S32K144 Cortex-M4 microcontroller.
 *
 * The idle hook function:
 * The idle hook function demonstrates how to query the amount of FreeRTOS heap
 * space that is remaining (see vApplicationIdleHook() defined in this file).
 *
 * The main() Function:
 * main() creates one software timer, one queue, and two tasks.  It then starts
 * the scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 200 milliseconds, before sending the value 100 to the queue that
 * was created within main().  Once the value is sent, the task loops back
 * around to block for another 200 milliseconds.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop that causes it to
 * repeatedly attempt to read data from the queue that was created within
 * main().  When data is received, the task checks the value of the data, and
 * if the value equals the expected 100, toggles the green LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the task
 * should be held in the Blocked state indefinitely to wait for data to be
 * available on the queue.  The queue receive task will only leave the Blocked
 * state when the queue send task writes to the queue.  As the queue send task
 * writes to the queue every 200 milliseconds, the queue receive task leaves the
 * Blocked state every 200 milliseconds, and therefore toggles the blue LED
 * every 200 milliseconds.
 *
 * The LED Software Timer and the Button Interrupt:
 * The user button BTN1 is configured to generate an interrupt each time it is
 * pressed.  The interrupt service routine switches the red LED on, and
 * resets the LED software timer.  The LED timer has a 5000 millisecond (5
 * second) period, and uses a callback function that is defined to just turn the
 * LED off again.  Therefore, pressing the user button will turn the LED on, and
 * the LED will remain on until a full five seconds pass without the button
 * being pressed.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* SDK includes. */
#include "interrupt_manager.h"
#include "sdk_project_config.h"
#include "freemaster.h"

#include "BoardDefines.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

#define mainUART_TASK_PRIORITY              ( tskIDLE_PRIORITY + 1 )
#define mainUART_TASK_STACK_SIZE            ( configMINIMAL_STACK_SIZE + 200 )

#define mainFMSTR_TASK_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define mainFMSTR_TASK_STACK_SIZE           ( configMINIMAL_STACK_SIZE + 200 )

#define mainADC_TASK_PRIORITY               ( tskIDLE_PRIORITY + 1 )
#define mainADC_TASK_STACK_SIZE             ( configMINIMAL_STACK_SIZE + 200 )

#define mainCAN_RX_TASK_PRIORITY            ( tskIDLE_PRIORITY + 2 )
#define mainCAN_RX_TASK_STACK_SIZE          ( configMINIMAL_STACK_SIZE + 200 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 200 / portTICK_PERIOD_MS )

/* The LED will remain on until the button has not been pushed for a full
5000ms. */
#define mainBUTTON_LED_TIMER_PERIOD_MS		( 5000UL / portTICK_PERIOD_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/* The LED toggle by the queue receive task (blue). */
#define mainTASK_CONTROLLED_LED				( 1UL << 0UL )

/* The LED turned on by the button interrupt, and turned off by the LED timer
(green). */
#define mainTIMER_CONTROLLED_LED			( 1UL << 1UL )

/* The vector used by the GPIO port C.  Button SW7 is configured to generate
an interrupt on this port. */
#define mainGPIO_C_VECTOR					( 61 )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK						( 0UL )

/* Definition of power modes indexes, as configured in Power Manager Component
 *  Refer to the Reference Manual for details about the power modes.
 */
#define HSRUN   (0u)
#define RUN     (1u)
#define VLPR    (2u)
#define STOP1   (3u)
#define STOP2   (4u)
#define VLPS    (5u)

/* Global variables used by FreeMASTER */

#define ADC_INSTANCE        0UL
#define ADC_CHN             12U
#define ADC_NR_SAMPLES      72
#define ADC_CHANNEL_INDEX   0UL
#define LPIT_CHANNEL        0UL

volatile bool       g_adcEvent = false;
volatile uint16_t   g_conversionResult = 0U;

volatile uint16_t   adcRawValue = 0U;

/*-----------------------------------------------------------*/
/* CAN definitions                                           */
/*-----------------------------------------------------------*/

/* Use same role mapping as can_pal example */
#define MASTER
/* #define SLAVE */

#if defined(MASTER)
    #define TX_MAILBOX  (1UL)
    #define TX_MSG_ID   (1UL)
    #define RX_MAILBOX  (0UL)
    #define RX_MSG_ID   (2UL)
#elif defined(SLAVE)
    #define TX_MAILBOX  (0UL)
    #define TX_MSG_ID   (2UL)
    #define RX_MAILBOX  (1UL)
    #define RX_MSG_ID   (1UL)
#endif

#define CAN_TX_TRIGGER_PIN   (12UL)

typedef enum
{
    CAN_TOGGLE_REQUEST = 0x00U
} can_commands_list;

static uint8_t g_canCommand = (uint8_t)CAN_TOGGLE_REQUEST;

static volatile bool g_canTxRequest = false;

/*-----------------------------------------------------------*/

/*
 * Setup the NVIC, LED outputs, and button inputs.
 */
static void prvSetupHardware( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );
static void prvUartTask( void *pvParameters );
static void prvFreeMasterTask( void *pvParameters );
static void prvCanRxTask( void *pvParameters );
static void prvCanTxTask( void *pvParameters );

/*
 * The LED timer callback function.  This does nothing but switch off the
 * LED defined by the mainTIMER_CONTROLLED_LED constant.
 */
static void prvButtonLEDTimerCallback( TimerHandle_t xTimer );
static void prvUartPrint( const char *str );
static uint8_t prvUartReadChar( void );

static void prvHandlePowerMode( uint8_t ch );
static void prvPrintCoreClock( void );

static void prvLedInit( void );
static void prvLedApply( bool redOn, bool greenOn, bool blueOn );
static void prvLedBluePulseTrigger( void );
static void prvLedSetBaseModeRun( void );
static void prvLedSetBaseModeSleep( void );
static void prvLedService( void );

/*
 * ADC low power hardware trigger chain
 */
static void prvInitAdcForFreeMaster( void );

/*
 * CAN helpers
 */
static void prvCanInit( void );
static void prvCanSendToggleRequest( void );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/* The LED software timer.  This uses prvButtonLEDTimerCallback() as its callback function. */
static TimerHandle_t xButtonLEDTimer = NULL;

/*-----------------------------------------------------------*/
/* LED Manager                                               */
/*-----------------------------------------------------------*/

#define BLUE_PULSE_MS         (100UL / portTICK_PERIOD_MS)
#define BASE_BLINK_PERIOD_MS  (200UL / portTICK_PERIOD_MS)

typedef enum
{
    LED_BASE_MODE_RUN = 0,
    LED_BASE_MODE_SLEEP
} led_base_mode_t;

typedef struct
{
    led_base_mode_t baseMode;
    bool baseBlinkOn;
    TickType_t baseBlinkNextTick;

    bool bluePulseActive;
    TickType_t bluePulseOffTick;
} led_state_t;

static led_state_t g_ledState =
{
    LED_BASE_MODE_RUN,
    false,
    0U,
    false,
    0U
};

/*-----------------------------------------------------------*/

void rtos_start( void )
{
	/* Configure the NVIC, LED outputs and button inputs. */
	prvSetupHardware();

	/* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

	if( xQueue != NULL )
	{
		/* Start the two tasks as described in the comments at the top of this
		file. */

		xTaskCreate( prvQueueReceiveTask, "RX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );
		xTaskCreate( prvUartTask, "UART", mainUART_TASK_STACK_SIZE, NULL, mainUART_TASK_PRIORITY, NULL );
		xTaskCreate( prvFreeMasterTask, "FMSTR", mainFMSTR_TASK_STACK_SIZE, NULL, mainFMSTR_TASK_PRIORITY, NULL );
		xTaskCreate( prvCanRxTask, "CANRX", mainCAN_RX_TASK_STACK_SIZE, NULL, mainCAN_RX_TASK_PRIORITY, NULL );
		xTaskCreate( prvCanTxTask, "CANTX", mainCAN_RX_TASK_STACK_SIZE, NULL, mainCAN_RX_TASK_PRIORITY, NULL );

		/* Create the software timer that is responsible for turning off the LED
		if the button is not pushed within 5000ms, as described at the top of
		this file. */
		xButtonLEDTimer = xTimerCreate( "ButtonLEDTimer", 			/* A text name, purely to help debugging. */
									mainBUTTON_LED_TIMER_PERIOD_MS,	/* The timer period, in this case 5000ms (5s). */
									pdFALSE,						/* This is a one shot timer, so xAutoReload is set to pdFALSE. */
									( void * ) 0,					/* The ID is not used, so can be set to anything. */
									prvButtonLEDTimerCallback		/* The callback function that switches the LED off. */
								);

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
}

static void prvLedInit( void )
{
    PINS_DRV_SetPinsDirection(LED_GPIO, (1U << LED_RED) | (1U << LED_GREEN) | (1U << LED_BLUE));

    g_ledState.baseMode = LED_BASE_MODE_RUN;
    g_ledState.baseBlinkOn = false;
    g_ledState.baseBlinkNextTick = xTaskGetTickCount() + BASE_BLINK_PERIOD_MS;
    g_ledState.bluePulseActive = false;
    g_ledState.bluePulseOffTick = 0U;

    prvLedApply(false, false, false);
}

static void prvLedApply( bool redOn, bool greenOn, bool blueOn )
{
    uint32_t pinsToClear = 0U;
    uint32_t pinsToSet   = 0U;

    if (redOn)
    {
        pinsToClear |= (1U << LED_RED);
    }
    else
    {
        pinsToSet |= (1U << LED_RED);
    }

    if (greenOn)
    {
        pinsToClear |= (1U << LED_GREEN);
    }
    else
    {
        pinsToSet |= (1U << LED_GREEN);
    }

    if (blueOn)
    {
        pinsToClear |= (1U << LED_BLUE);
    }
    else
    {
        pinsToSet |= (1U << LED_BLUE);
    }

    if (pinsToClear != 0U)
    {
        PINS_DRV_ClearPins(LED_GPIO, pinsToClear);
    }

    if (pinsToSet != 0U)
    {
        PINS_DRV_SetPins(LED_GPIO, pinsToSet);
    }
}

static void prvLedBluePulseTrigger( void )
{
    g_ledState.bluePulseActive = true;
    g_ledState.bluePulseOffTick = xTaskGetTickCount() + BLUE_PULSE_MS;
}

static void prvLedSetBaseModeRun( void )
{
    g_ledState.baseMode = LED_BASE_MODE_RUN;
    g_ledState.baseBlinkOn = false;
    g_ledState.baseBlinkNextTick = xTaskGetTickCount() + BASE_BLINK_PERIOD_MS;
}

static void prvLedSetBaseModeSleep( void )
{
    g_ledState.baseMode = LED_BASE_MODE_SLEEP;
    g_ledState.baseBlinkOn = false;
    g_ledState.baseBlinkNextTick = xTaskGetTickCount() + BASE_BLINK_PERIOD_MS;
}

static void prvLedService( void )
{
    TickType_t now = xTaskGetTickCount();
    bool redOn = false;
    bool greenOn = false;
    bool blueOn = false;

    if (now >= g_ledState.baseBlinkNextTick)
    {
        g_ledState.baseBlinkOn = !g_ledState.baseBlinkOn;
        g_ledState.baseBlinkNextTick = now + BASE_BLINK_PERIOD_MS;
    }

    if ((g_ledState.bluePulseActive == true) &&
        (now >= g_ledState.bluePulseOffTick))
    {
        g_ledState.bluePulseActive = false;
    }

    if (g_ledState.bluePulseActive == true)
    {
        blueOn = true;
    }
    else
    {
        if (g_ledState.baseMode == LED_BASE_MODE_RUN)
        {
            greenOn = g_ledState.baseBlinkOn;
        }
        else
        {
            redOn = g_ledState.baseBlinkOn;
        }
    }

    prvLedApply(redOn, greenOn, blueOn);
}

/*-----------------------------------------------------------*/

static void prvButtonLEDTimerCallback( TimerHandle_t xTimer )
{
	/* Casting xTimer to void because it is unused */
	(void)xTimer;
}
/*-----------------------------------------------------------*/

/* The ISR executed when the user button is pushed. */
void vPort_C_ISRHandler( void )
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    uint32_t portFlags = PINS_DRV_GetPortIntFlag(BTN_PORT);

    if ((portFlags & (1UL << BTN_PIN)) != 0UL)
    {
		/* This interrupt safe FreeRTOS function can be called from this interrupt
		because the interrupt priority is below the
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY setting in FreeRTOSConfig.h. */
		if (xButtonLEDTimer != NULL)
		{
			xTimerResetFromISR( xButtonLEDTimer, &xHigherPriorityTaskWoken );
		}

		prvLedSetBaseModeRun();
		POWER_SYS_SetMode(RUN, POWER_MANAGER_POLICY_AGREEMENT);

		/* Clear the interrupt before leaving. */
		PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, BTN_PIN);
    }

    if ((portFlags & (1UL << CAN_TX_TRIGGER_PIN)) != 0UL)
    {
        /* CAN TX request flag set only */
        g_canTxRequest = true;

        PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, CAN_TX_TRIGGER_PIN);
    }

	/* If calling xTimerResetFromISR() caused a task (in this case the timer
	service/daemon task) to unblock, and the unblocked task has a priority
	higher than or equal to the task that was interrupted, then
	xHigherPriorityTaskWoken will now be set to pdTRUE, and calling
	portEND_SWITCHING_ISR() will ensure the unblocked task runs next. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------*/
/* UART helpers                                              */
/*-----------------------------------------------------------*/

static void prvUartPrint( const char *str )
{
    uint32_t bytesRemaining;

    LPUART_DRV_SendData(INST_LPUART_2, (uint8_t *)str, strlen(str));
    while (LPUART_DRV_GetTransmitStatus(INST_LPUART_2, &bytesRemaining) != STATUS_SUCCESS)
    {
    	vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static uint8_t prvUartReadChar( void )
{
    uint32_t bytesRemaining;
    uint8_t ch = 0U;

    LPUART_DRV_ReceiveData(INST_LPUART_2, &ch, 1U);
    while (LPUART_DRV_GetReceiveStatus(INST_LPUART_2, &bytesRemaining) != STATUS_SUCCESS)
    {
    	vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ch;
}

static void prvPrintCoreClock( void )
{
    uint32_t frequency;
    char buffer[32];

    (void)CLOCK_SYS_GetFreq(CORE_CLOCK, &frequency);
    snprintf(buffer, sizeof(buffer), "%lu", (unsigned long)frequency);

    prvUartPrint(buffer);
    prvUartPrint("[Hz] \r\n");
}

/*-----------------------------------------------------------*/
/* Power mode handler                                        */
/*-----------------------------------------------------------*/

static void prvHandlePowerMode( uint8_t ch )
{
    status_t retV = STATUS_SUCCESS;

    switch (ch)
    {
        case '1':
            retV = POWER_SYS_SetMode(HSRUN, POWER_MANAGER_POLICY_AGREEMENT);
            if (retV == STATUS_SUCCESS)
            {
            	prvLedSetBaseModeRun();

            	prvUartPrint("************************ CPU is in HSRUN mode.\r\n");
            	prvUartPrint("************************ Core frequency: ");
            	prvPrintCoreClock();
            }
            else
            {
                prvUartPrint("[PM] Switch HSRUN mode unsuccessfully\r\n");
            }
            break;

        case '2':
            retV = POWER_SYS_SetMode(RUN, POWER_MANAGER_POLICY_AGREEMENT);
            if (retV == STATUS_SUCCESS)
            {
            	prvLedSetBaseModeRun();

            	prvUartPrint("************************ CPU is in RUN mode.\r\n");
            	prvUartPrint("************************ Core frequency: ");
            	prvPrintCoreClock();
            }
            else
            {
                prvUartPrint("[PM] Switch RUN mode unsuccessfully\r\n");
            }
            break;

        case '3':
            retV = POWER_SYS_SetMode(VLPR, POWER_MANAGER_POLICY_AGREEMENT);
            if (retV == STATUS_SUCCESS)
            {
            	prvLedSetBaseModeRun();

            	prvUartPrint("************************ CPU is in VLPR mode.\r\n");
            	prvUartPrint("************************ Core frequency: ");
            	prvPrintCoreClock();
            }
            else
            {
                prvUartPrint("[PM] Switch VLPR mode unsuccessfully\r\n");
            }
            break;

		case '4':
			prvUartPrint("******** CPU is going in STOP1 mode...\r\n");

			prvLedSetBaseModeSleep();

			retV = POWER_SYS_SetMode(STOP1, POWER_MANAGER_POLICY_AGREEMENT);
			if (retV == STATUS_SUCCESS)
			{
				prvUartPrint("CPU was entered STOP1 mode successfully and then woke up to exit STOP1 mode.\r\n");
				prvUartPrint("Current mode is RUN because STOP mode can only be switched from this mode.\r\n");
			}
			else
			{
				prvUartPrint("Switch STOP1 mode unsuccessfully\r\n");
			}
			break;

		case '5':
			prvUartPrint("******** CPU is going in STOP2 mode...\r\n");

			prvLedSetBaseModeSleep();

			retV = POWER_SYS_SetMode(STOP2, POWER_MANAGER_POLICY_AGREEMENT);
			if (retV == STATUS_SUCCESS)
			{
				prvUartPrint("CPU was entered STOP2 mode successfully and then woke up to exit STOP2 mode.\r\n");
				prvUartPrint("Current mode is RUN because STOP mode can only be switched from this mode.\r\n");
			}
			else
			{
				prvUartPrint("Switch STOP2 mode unsuccessfully\r\n");
			}
			break;

		case '6':
			prvUartPrint("******** CPU is going in VLPS mode...\r\n");

			prvLedSetBaseModeSleep();

			retV = POWER_SYS_SetMode(VLPS, POWER_MANAGER_POLICY_AGREEMENT);
			if (retV == STATUS_SUCCESS)
			{
				prvUartPrint("CPU was entered VLPS mode successfully and then woke up to exit VLPS mode.\r\n");

				if (POWER_SYS_GetCurrentMode() == POWER_MANAGER_RUN)
				{
				    prvUartPrint("Current mode is RUN mode.\r\n");
				    prvUartPrint("Clock source is remained in SIRC (8 MHz) before MCU switches from RUN to VLP mode.\r\n");
				    prvUartPrint("In order to set to default clock, press option RUN mode or re-initialize clock configuration.\r\n");

				    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

				    prvUartPrint("************************ Core frequency after re-initialized clock: ");
				    prvPrintCoreClock();
				}
				else
				{
					prvUartPrint("Current mode is VLPR mode.\r\n");
					prvUartPrint("************************ Core frequency: ");
					prvPrintCoreClock();
				}
			}
			else
			{
				prvUartPrint("Switch VLPS mode unsuccessfully\r\n");
			}
			break;

        default:
            break;
    }
    prvUartPrint("----------------------------------------------------------------------------\r\n");
}

/*-----------------------------------------------------------*/
/* ADC IRQ                                                   */
/*-----------------------------------------------------------*/

void ADC_IRQHandler(void)
{
    uint16_t adcValue = 0U;

    ADC_DRV_GetChanResult(ADC_INSTANCE, ADC_CHANNEL_INDEX, &adcValue);

    g_conversionResult = adcValue;
    adcRawValue = adcValue;
    g_adcEvent = true;
}

/*-----------------------------------------------------------*/
/* UART Task                                                 */
/*-----------------------------------------------------------*/

static void prvUartTask( void *pvParameters )
{
    uint8_t ch;
    char msg[64];

    (void)pvParameters;

    prvUartPrint("\r\n================ UART MENU ================\r\n");
    prvUartPrint("Press 1~6\r\n");
    prvUartPrint("1) HSRUN\r\n");
    prvUartPrint("2) RUN\r\n");
    prvUartPrint("3) VLPR\r\n");
    prvUartPrint("4) STOP1\r\n");
    prvUartPrint("5) STOP2\r\n");
    prvUartPrint("6) VLPS\r\n");
    prvUartPrint("===========================================\r\n");

    for (;;)
    {
        prvUartPrint("Input: ");
        ch = prvUartReadChar();

        LPUART_DRV_SendData(INST_LPUART_2, &ch, 1U);
        {
            uint32_t bytesRemaining;
            while (LPUART_DRV_GetTransmitStatus(INST_LPUART_2, &bytesRemaining) != STATUS_SUCCESS)
            {
            	vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        prvUartPrint("\r\n");

        switch (ch)
        {
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
                prvHandlePowerMode(ch);
                break;

            default:
                snprintf(msg, sizeof(msg), "[UART] invalid input: %c\r\n", ch);
                prvUartPrint(msg);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void prvQueueSendTask( void *pvParameters )
{
TickType_t xNextWakeTime;
const unsigned long ulValueToSend = 100UL;

	/* Casting pvParameters to void because it is unused */
	(void)pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, the constant used converts ticks
		to ms.  While in the Blocked state this task will not consume any CPU
		time. */
		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

		/* Send to the queue - causing the queue receive task to unblock and
		toggle an LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		xQueueSend( xQueue, &ulValueToSend, mainDONT_BLOCK );
	}
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
unsigned long ulReceivedValue;

	/* Casting pvParameters to void because it is unused */
	(void)pvParameters;

	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, toggle the LED. */
		if( ulReceivedValue == 100UL )
		{
			/* heartbeat LED disabled during CAN debug */
		}
	}
}

/*-----------------------------------------------------------*/
/* FreeMASTER Task                                           */
/*-----------------------------------------------------------*/

static void prvFreeMasterTask( void *pvParameters )
{
    (void)pvParameters;

    FMSTR_Init();

    for( ;; )
    {
        FMSTR_Poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*-----------------------------------------------------------*/
/* CAN RX Task                                               */
/*-----------------------------------------------------------*/

static void prvCanRxTask( void *pvParameters )
{
    can_message_t recvMsg;

    (void)pvParameters;

    for (;;)
    {
        /* Start receiving data in RX mailbox */
        CAN_Receive(&can_pal_1_instance, RX_MAILBOX, &recvMsg);

        /* Wait until receive completed */
        while (CAN_GetTransferStatus(&can_pal_1_instance, RX_MAILBOX) == STATUS_BUSY)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if ((recvMsg.data[0] == CAN_TOGGLE_REQUEST) && (recvMsg.id == RX_MSG_ID))
        {
            prvLedBluePulseTrigger();
        }
    }
}

static void prvCanTxTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (g_canTxRequest == true)
        {
            g_canTxRequest = false;
            prvCanSendToggleRequest();
        }

        prvLedService();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*-----------------------------------------------------------*/
/* CAN Helpers                                               */
/*-----------------------------------------------------------*/

static void prvCanInit( void )
{
    can_buff_config_t buffCfg =
    {
        .enableFD = false,
        .enableBRS = false,
        .fdPadding = 0U,
        .idType = CAN_MSG_ID_STD,
        .isRemote = false
    };

    CAN_Init(&can_pal_1_instance, &can_pal_1_config);

    /* Configure RX buffer */
    CAN_ConfigRxBuff(&can_pal_1_instance, RX_MAILBOX, &buffCfg, RX_MSG_ID);

    /* Configure TX buffer */
    CAN_ConfigTxBuff(&can_pal_1_instance, TX_MAILBOX, &buffCfg);
}

static void prvCanSendToggleRequest( void )
{
    status_t status;
    can_message_t message =
    {
        .cs = 0U,
        .id = TX_MSG_ID,
        .data[0] = g_canCommand,
        .length = 1U
    };

    status = CAN_Send(&can_pal_1_instance, TX_MAILBOX, &message);

    if (status != STATUS_SUCCESS)
    {
        return;
    }

    while (CAN_GetTransferStatus(&can_pal_1_instance, TX_MAILBOX) == STATUS_BUSY)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    /* Initialize pin mux:
	 * - ADC0_SE12 analog input
	 * - LPUART1 for FreeMASTER
	 * - LPUART2 for TeraTerm menu
	 * - LED / Button pins
	 */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);

    boardSetup();

    /* Power manager is already prepared in this RTOS project.
	 * We will keep using it for RUN/VLPR/STOP/VLPS transition testing.
	 */
    POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT, &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);

    /*
     * UART for TeraTerm menu
     */
    LPUART_DRV_Init(INST_LPUART_2, &lpuart_2_State, &lpuart_2_InitConfig0);

    /*
     * UART for FreeMASTER
     */
	LPUART_DRV_Init(INST_LPUART_1, &lpuart_1_State, &lpuart_1_InitConfig0);
	INT_SYS_InstallHandler(LPUART1_RxTx_IRQn, FMSTR_Isr, NULL);
	INT_SYS_EnableIRQ(LPUART1_RxTx_IRQn);

	/*
	 * ADC low power hardware trigger chain
	 * LPIT CH0 -> TRGMUX -> ADC0 hardware trigger line
	 */
	TRGMUX_DRV_Init(INST_TRGMUX, &trgmux1_InitConfig0);

	LPIT_DRV_Init(INST_LPIT_CONFIG_1, &lpit1_InitConfig);
	LPIT_DRV_InitChannel(INST_LPIT_CONFIG_1, LPIT_CHANNEL, &lpit1_ChnConfig0);

	prvInitAdcForFreeMaster();

	/*
	 * Start LPIT channel after ADC / IRQ setup is completed
	 */
	LPIT_DRV_StartTimerChannels(INST_LPIT_CONFIG_1, (1UL << LPIT_CHANNEL));

	/* LED init is centralized in LED Manager. */
	prvLedInit();

	PINS_DRV_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_RISING_EDGE);
	PINS_DRV_SetPinIntSel(BTN_PORT, CAN_TX_TRIGGER_PIN, PORT_INT_RISING_EDGE);

	/* Install Button interrupt handler */
    INT_SYS_InstallHandler(BTN_PORT_IRQn, vPort_C_ISRHandler, (isr_t *)NULL);
    /* Enable Button interrupt handler */
    INT_SYS_EnableIRQ(BTN_PORT_IRQn);

    /* The interrupt calls an interrupt safe API function - so its priority must
    be equal to or lower than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
    INT_SYS_SetPriority( BTN_PORT_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );

    /* CAN init */
	prvCanInit();
}

static void prvInitAdcForFreeMaster( void )
{
    IRQn_Type adcIRQ;

    /* Configure ADC converter to use hardware trigger.
	 * ConvConfig0 must be the hardware-trigger configuration
	 * copied from adc_low_power project.
	 */
    ADC_DRV_ConfigConverter(ADC_INSTANCE, &adc_config_1_ConvConfig0);

    /* Auto calibration improves measurement stability before starting
	 * periodic LPIT-triggered conversions.
	 */
    ADC_DRV_AutoCalibration(ADC_INSTANCE);

    /* Hardware compare:
	 * only generate valid conversion event when ADC result is above threshold.
	 * In the low power example, threshold is 2048 for 12-bit ADC.
	 */
    ADC_DRV_ConfigHwCompare(ADC_INSTANCE, &adc_config_1_HwCompConfig0);

    /* Channel configuration:
	 * - external input channel 12 (ADC0_SE12)
	 * - interrupt enabled
	 */
    ADC_DRV_ConfigChan(ADC_INSTANCE, 0U, &adc_config_1_ChnConfig0);

    switch (ADC_INSTANCE)
    {
        case 0UL:
            adcIRQ = ADC0_IRQn;
            break;
        case 1UL:
            adcIRQ = ADC1_IRQn;
            break;
        default:
            adcIRQ = ADC0_IRQn;
            break;
    }

    INT_SYS_InstallHandler(adcIRQ, ADC_IRQHandler, NULL);
    INT_SYS_EnableIRQ(adcIRQ);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	if( xFreeHeapSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}

}
/*-----------------------------------------------------------*/

/* The Blinky build configuration does not include run time stats gathering,
however, the Full and Blinky build configurations share a FreeRTOSConfig.h
file.  Therefore, dummy run time stats functions need to be defined to keep the
linker happy. */
void vMainConfigureTimerForRunTimeStats( void ) {}
unsigned long ulMainGetRunTimeCounterValue( void ) { return 0UL; }

/* A tick hook is used by the "Full" build configuration.  The Full and blinky
build configurations share a FreeRTOSConfig.h header file, so this simple build
configuration also has to define a tick hook - even though it does not actually
use it for anything. */
void vApplicationTickHook( void ) {}

/*-----------------------------------------------------------*/
