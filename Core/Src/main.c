/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "fifo.h"
#include "flash_eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EEPROM_WHITELIST_SIGNATURE 0xBAADF00D
#define MOTOR_RTC_SIGNATURE 0xABCD

#define MASTER_NUMBER "+91xxxxxxxxxx"
#define MAX_NUMBERS     5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//APP RELATED COMMANDS
#define MOTOR_ON_CMD_APP "11"
#define MOTOR_OFF_CMD_APP "10"

//PROTOCOL 2
//[CMD]:[ON/OFF]:[MOTORS]:[ON AFTER X SECONDS]:[ON FOR Y SECONDS]

//CMD LIST
#define MOTOR_ON_OFF_CMD   "MOT"

//MOT:ON:1110:30:60 - //MOTOR1,2,3 ON AFTER 30 SECONDS, ON DURATION 60 SECONDS 
//MOT:OFF:1110        //MEANS IMMEDIATELY OFF 1,3,4


#define RELAY_1_OFF      GPIOA->ODR |= 1<<3;
#define RELAY_1_ON     GPIOA->ODR &= ~(1<<3);

#define RELAY_2_OFF      GPIOA->ODR |= 1<<4;
#define RELAY_2_ON     GPIOA->ODR &= ~(1<<4);

#define RELAY_3_OFF      GPIOA->ODR |= 1<<5;
#define RELAY_3_ON     GPIOA->ODR &= ~(1<<5);

#define RELAY_4_OFF      GPIOA->ODR |= 1<<6;
#define RELAY_4_ON     GPIOA->ODR &= ~(1<<6);
#define MOTOR_STATE_OFF                      0
#define MOTOR_STATE_SCHEDULED_TO_ON          1
#define MOTOR_STATE_SCHEDULED_TO_OFF         2
#define RTC_ADDRESS 0x68
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
fifo_t rx_fp;
volatile uint32_t loop_count;
char line_buff[1000];
char msg[200];
char number[15];
//this structure will be backedup as it in into eeprom_buff and then the eeprom buff into flash...
struct whitelist_s {
  uint32_t signature; //validating DEADBEEF 
  char master_number[15];
  char list[MAX_NUMBERS][15];
}whitelist;

struct motor_t {
  uint8_t state;
  uint64_t scheduled_on_time; 
  uint64_t scheduled_off_time; 
} motor[4];

struct motor_rtc {
  uint8_t state;
  uint16_t pending_duration_seconds;
};

struct motor_rtc_mem_t { //10 bytes out of 56 
  uint16_t signature;
  struct motor_rtc motor_backup[4];
}motor_rtc_mem;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sim800_init(void);
int backup_motor_state_to_rtc_ram(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//////////////////////////////////////////////////////////////////


void uart3_send_string(char *p) {
  while (*p) {
    LL_USART_TransmitData8(USART3, *p++);
     while (!LL_USART_IsActiveFlag_TXE(USART3));
  }
}

void UART3_PutChar(uint8_t data) {
  LL_USART_TransmitData8(USART3, data);
  /* Wait for TXE flag to be raised */
  while (!LL_USART_IsActiveFlag_TXE(USART3));
}

void UART3_PutStr(char *string) {
  while (*string != '\0') {
    LL_USART_TransmitData8(USART3, *string++);
      while (!LL_USART_IsActiveFlag_TXE(USART3));
  }
   //UART3_PutChar(*string++);
}

/////////////////////////////////////////////////////////////////////


void USART1_IRQHandler(void) {
  /* USER CODE BEGIN USART1_IRQn 0 */
  
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  if (LL_USART_IsActiveFlag_RXNE(USART1)
      && LL_USART_IsEnabledIT_RXNE(USART1)) {
        uint8_t bufbyte = LL_USART_ReceiveData8(USART1);

       
        //LL_USART_TransmitData8(USART3, bufbyte);
        //  while (!LL_USART_IsActiveFlag_TXE(USART3));
        
         fifo_push(&rx_fp, bufbyte);
      };
  
  /* USER CODE END USART1_IRQn 1 */
}
void USART3_IRQHandler(void) {
  /* USER CODE BEGIN USART1_IRQn 0 */
  
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  if (LL_USART_IsActiveFlag_RXNE(USART3)
      && LL_USART_IsEnabledIT_RXNE(USART3)) {
        uint8_t bufbyte = LL_USART_ReceiveData8(USART3);
           
        
        LL_USART_TransmitData8(USART1, bufbyte);
            while (!LL_USART_IsActiveFlag_TXE(USART1));

        //fifo_push(&rx_fp, bufbyte);
      };
  
  /* USER CODE END USART1_IRQn 1 */
}

void uprintf(uint8_t Value) {
  //	LL_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value);
  // LL_USART_ReceiveData8(USART_TypeDef *USARTx)
  
  LL_USART_TransmitData8(USART1, Value);
   while (!LL_USART_IsActiveFlag_TXE(USART1))
    ;
}



void uart_send_string(char *p) {
  while (*p) {
    LL_USART_TransmitData8(USART1, *p++);
     while (!LL_USART_IsActiveFlag_TXE(USART3));
  }
}

void UART1_PutChar(uint8_t data) {
  LL_USART_TransmitData8(USART1, data);
  /* Wait for TXE flag to be raised */
  while (!LL_USART_IsActiveFlag_TXE(USART1));
}

void UART1_PutStr(char *string) {
  while (*string != '\0')
    UART1_PutChar(*string++);
}

int get_line(char *p)
{
  unsigned long timeout = HAL_GetTick() + 2000;
  //*p = '\0';
  
  while( 1 ){ 
    
    //OK\r\n  => OK\0
    //OK\n    => OK\0
    //\r\n      => \0
    //\n      => \0
    
    if(fifo_pop(&rx_fp, p) == 0){
      if(*p == '\n'){
        *p = '\0';
        return 0;
      } 
      
      if(*p != '\r'){
        p++;
      }
    }
    
    if( HAL_GetTick() > timeout) return -1;
    //see the p+++ code 
  }
}

void backup_whitelist_to_flash(void)
{
  
  //skip if no change in buffer and whitelist. 
  if( memcmp( eeprom_get_buff_address(),  &whitelist, sizeof(whitelist) ) == 0)
    return; //PREVENT an unwanted flash erase and write cycle.. Flash erase-write cycles are having its own life, say 10K OR LESS...
  //it is called flash endurance, we are not supposed to write flash too many times because it can damage the memory.   
  
  memcpy(eeprom_get_buff_address(), &whitelist, sizeof(whitelist));
  
  eeprom_write_to_flash();  
}

void add_whitelist(char *number)
{
  
  //fill it in first empty space in the list of 5 numbers
  for(int i = 0; i < MAX_NUMBERS; i++) {
    if(whitelist.list[i][0] == '\0') {
      strcpy(whitelist.list[i], number);
      break;
    }
  }
  backup_whitelist_to_flash();
}

void remove_whitelist(char *number)
{
  for( int i = 0; i < MAX_NUMBERS; i++ ) {
    if( strcmp( whitelist.list[i], number) == 0 ) {
      memset( whitelist.list[i], 0, strlen(number));
    }
  }
  backup_whitelist_to_flash(); 
}

void remove_whitelist_all(void)
{
  for( int i = 0; i < MAX_NUMBERS; i++ ) {
      memset( whitelist.list[i], 0, sizeof(whitelist.list[0]));   
  }
  backup_whitelist_to_flash(); 
}


void sim800_send(char *cmd)
{
  UART1_PutStr( cmd );
  UART1_PutStr("\r\n");
}

int sim800_get(char *cmd, char *response)
{
  int ret;
  fifo_flush(&rx_fp);
  sim800_send(cmd);
  //UART1_PutStr(cmd);
  
  ////// COMMAND SENDING AND RECEIVING SAME COMMAND AS ECHO 
  ret = get_line(response);
  if(ret) goto end;
  else{
    if(strcmp(cmd, response) != 0){
      ret = -2;
      goto end;
    } 
  }
  
  ret = get_line(response);
  
end:
  return ret;
  
}


int send_sms(char *num, char* msg)
{
  //AT+CMGS=<number><CR><message><CTRL-Z>
  //https://www.diafaan.com/sms-tutorials/gsm-modem-tutorial/at-cmgs-text-mode
  char cmd[200];
  char *p = cmd;
  while( sim800_get("AT+CMGF=1", line_buff) ){
    HAL_Delay(1000);
  }
  HAL_Delay(100);
  while( sim800_get("AT+CMGF=1", line_buff) ){
    HAL_Delay(1000);
  }
  HAL_Delay(100);
  memset(cmd, 0, sizeof(cmd));
  strcat(cmd, "AT+CMGS=\""); 
  strcat(cmd, num);
  strcat(cmd, "\"");
  
  //add <cr> to same command 
  p += strlen(cmd);
  *p++ = 13; 
  
  strcat(p, msg); //append message after <CR>
  
  //seek p to end of p + 1 
  p += strlen(p);
  
  //add controlZ ascii
  *p = 26;
  
  int len = (int)(p-cmd) + 1;
  for(int i = 0; i < len; i++) {
    UART1_PutChar(cmd[i]);
    if(cmd[i] == '\r'){
      HAL_Delay(2000); //trick to wait for prompt by just delay
      UART3_PutStr("waiting for >\n");
    }
   }
  HAL_Delay(100);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  fifo_create(&rx_fp, 1000);
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
  GPIOA->ODR &= ~(1<<7);
  HAL_Delay(1000);
  GPIOA->ODR |= (1<<7);
  
  RELAY_1_OFF;
  RELAY_2_OFF;
  RELAY_3_OFF; 
  RELAY_4_OFF;
  ////////////////////////////////////////////////////////////////////////////////////
  
  
  //HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, RTC_ADDRESS<<1, 5, 1000);
  
  //memset(buff, 0, sizeof(buff));
 // sprintf(buff, "TESTING RAM INSIDER RTC IC");

  //write buff to RTC NV RAM
  //ret = HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS<<1, 0x08, 1, buff, 56, 1000);

  //CLEAR buff data
 // memset(buff, 0, sizeof(buff));
  
  //read the remaining time duration for each ON motors and restore those in motor structure.
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, RTC_ADDRESS<<1, 0x08, 1, (uint8_t *)&motor_rtc_mem, sizeof(motor_rtc_mem), 1000);
  if(ret == HAL_OK) {
    if(motor_rtc_mem.signature == 0xABCD) {
      //if valid signature insider rtc memory, that means a valid data also available. 
      //4 motors 
      for(int i = 0; i <4; i++) {
        motor[i].state = motor_rtc_mem.motor_backup[i].state;
        motor[i].scheduled_on_time = 0;
        //restore future turn off time in milli seconds from current tick time of mcu.
        motor[i].scheduled_off_time = HAL_GetTick() + (uint64_t)motor_rtc_mem.motor_backup[i].pending_duration_seconds * 1000;
        //technically it(HAL_GetTick) will be near to zero or few millis thats all coz mcu is just starting.
         //HAL_GetTick is the time in millis since the MCU got power and starts running..
      }
      
      //if motor ON at time of Power cut
      //then turn on those motors and the main while loop will turn it 
      //off when the motor[i].scheduled_off_time matches the HAL_GetTick 
     
      if(motor[0].state == MOTOR_STATE_SCHEDULED_TO_OFF)RELAY_1_ON;
      if(motor[1].state == MOTOR_STATE_SCHEDULED_TO_OFF)RELAY_2_ON;
      if(motor[2].state == MOTOR_STATE_SCHEDULED_TO_OFF)RELAY_3_ON;
      if(motor[3].state == MOTOR_STATE_SCHEDULED_TO_OFF)RELAY_4_ON;
      
      
      
    }
  }
  
  
  //read last page 1024 bytes from flash to eeprom_buff emulated eeprom buffer..
  eeprom_read_from_flash();
  
  //copy whitelist structure data from eeprom_buff starting address to whitlist
  memcpy(&whitelist, eeprom_get_buff_address(), sizeof(whitelist));
 // this eeprom_get_buff_address() return address of 1kb buffer in ram   
  
  //verify if the structure is valid or not. On firt time it might not be valid.
  if( whitelist.signature != EEPROM_WHITELIST_SIGNATURE ) {
    //format the eeprom
    
    //0xBAADF00D is a hexadecimal token used widely for integety and checking 
    //memset(&whitelist, 0, sizeof(whitelist));
    //set signature so that on next boot this if condition will get skipped.
    //make all to 0x00 in whitelist so that the whitelist data gets 0 initialized
    memset(&whitelist, 0, sizeof(whitelist));
    
    //write signature to prevent next time bootup eeprom formating
    whitelist.signature = EEPROM_WHITELIST_SIGNATURE;
    
 //      add_whitelist(MASTER_NUMBER);
    
    
      //copy complete whitelist structure as it is to eeprom_buff
    memcpy(eeprom_get_buff_address(), &whitelist, sizeof(whitelist));
    
    //finally write the 1KB eeprom_buff to flash 
    eeprom_write_to_flash();
    
  }
  
  strcpy(whitelist.master_number, MASTER_NUMBER);
  
  
  
 
  
  //  uint8_t *p = eeprom_get_buff_address();
  //  strcpy(p, "eeprom testing, hello world, 123456789\n");
  //  eeprom_write_to_flash();
  //////////////////////////////////////////////////////////////////////////////////////////////
  
  //LL_USART_EnableIT_RXNE(USART1);
  // UART1_PutStr("gsm\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  
  sim800_init();
  HAL_Delay(1000);
  //send_sms("+91xxxxxxxx", "DEVICE POWERED ON"); 
  
  ///   printf("%s\n", line_buff);
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    loop_count++;
    int ret = get_line(line_buff);
    if(ret == 0) {
      UART3_PutStr(line_buff);
      UART3_PutStr("\n");
      
      //+CMT: "+91xxxxxxxxxx","","20/12/22,16:02:07+22"
      //Motor on
      
      if(strncmp(line_buff, "+CMT:", 5) ==  0 ) {
        memset(number, 0, sizeof(number));
        memset(msg, 0, sizeof(msg));
        
        
        char* num_start  = strstr(line_buff, "\"+");
        if(num_start){
          num_start += 1; //skip "
          char *num_end = strstr(num_start, "\"");
          if(num_end) {
            int num_len = (int)(num_end - num_start);
            strncpy(number, num_start, num_len);
            
            
            int attempts = 0;
            while(get_line(line_buff)) {
              attempts++;
              if(attempts > 5) break;              
            }
            
            if(attempts >5)continue;
            
            strcpy(msg, line_buff);              
            
            //validate if number is in white list
            int matching = 0;
            for(int i = 0; i < MAX_NUMBERS; i++) {
              if(matching)break;
              
              
              if(strcmp(number, whitelist.list[i]) == 0  ||   strcmp(number, whitelist.master_number) == 0 ) {
                //HAL_Delay(100);
                matching++;
                
                //this is the response message buffer,we will fill it based oncommands 
                char response_buff[160]; //one sms max 160 characters
                memset(response_buff, 0, sizeof(response_buff));
               
                if(strcmp(msg, MOTOR_ON_CMD_APP) == 0) {
                  UART3_PutStr("all motor on\n");
                  sprintf(response_buff, "ACK: ALL MOTORS ON");
                  for(int j = 0; j < 4; j++) {
                    motor[j].state = MOTOR_STATE_SCHEDULED_TO_ON;
                    motor[j].scheduled_on_time = HAL_GetTick();
                    motor[j].scheduled_off_time = -1;
                  }
                  //RELAY_1_ON;RELAY_2_ON;RELAY_3_ON;RELAY_4_ON;
                }
                
                
                else if(strcmp(msg, MOTOR_OFF_CMD_APP) == 0) {
                  for(int j = 0; j < 4; j++) {
                    motor[j].state = MOTOR_STATE_SCHEDULED_TO_OFF;
                    motor[j].scheduled_on_time = HAL_GetTick();
                    motor[j].scheduled_off_time = HAL_GetTick();
                  }
                  //RELAY_1_OFF;RELAY_2_OFF;RELAY_3_OFF;RELAY_4_OFF;                  
                  UART3_PutStr("all motor off\n");
                  sprintf(response_buff, "ACK: ALL MOTORS OFF");
                }
                
                //MOT:ON:1110:30:60 - //MOTOR1,2,3 ON AFTER 30 SECONDS, ON DURATION 60 SECONDS 
                //MOT:OFF:1110        //MEANS IMMEDIATELY OFF 1,3,4
                
                else if(strncmp(msg, "MOT:", 4) == 0) {
                  //protocol 2 matching
                  int mot = 0, on_time = 0, on_duration = 0;
                  int ret = sscanf(msg, "MOT:ON:%x:%d:%d", &mot, &on_time, &on_duration);
                  if(ret == 3) {
                    sprintf(response_buff, "ACK: %s", msg);
                    for(int i = 0; i < 4; i++) {
                      if(mot & 1) {
                        motor[3-i].state = MOTOR_STATE_SCHEDULED_TO_ON;
                        motor[3-i].scheduled_on_time = on_time*60*1000 + HAL_GetTick(); //even though sms is in minutes we converted to milli seconds
                        motor[3-i].scheduled_off_time = on_time*60*1000 + HAL_GetTick() + on_duration*60*1000; //all in milli seconds since we are using gettick                       
                      }
                      mot >>= 4;
                      
                    }                    
                  } else {
                    int ret = sscanf(msg, "MOT:OFF:%x", &mot );
                    if(ret == 1) {                      
                      sprintf(response_buff, "ACK: %s", msg);
                      for(int i = 0; i < 4; i++) {
                        if(mot & 1) {
                          motor[3-i].state = MOTOR_STATE_SCHEDULED_TO_OFF;
                          motor[3-i].scheduled_off_time = 0;////make it zero so that the motor control code will detect curret time is > than 0 and will turrn off it immediatley                       
                        }
                        mot >>= 4;
                      }
                      
                    }
                    
                  }
                  
                }
                
                
                /////////////////////////////////////  Flash save / delete number ////////////////////////////////////////   
                
                else if(strncmp(msg, "NUM:", 4) == 0) {     
                  
                  char num[15];
                  
                  
                  if(strncmp(msg, "NUM:ADD", 7) == 0) {     
                    int ret = sscanf(msg, "NUM:ADD:%s", num);
                    if(ret == 1) {
                      add_whitelist(num);
                      sprintf(response_buff, "ACK: ADDED %s TO WHITELIST", msg);
                    }
                  }
                  
                  if(strncmp(msg, "NUM:DEL", 7) == 0) {
                    int ret = sscanf(msg, "NUM:DEL:%s", num);
                    if(ret == 1) {        
                      remove_whitelist(num);
                      sprintf(response_buff, "ACK: REMOVED %s FROM WHITELIST", msg);
                    }  
                  }
                  if(strncmp(msg, "NUM:DEL:ALL", strlen("NUM:DEL:ALL")) == 0) {                      
                      remove_whitelist_all();                  
                      sprintf(response_buff, "ACK: REMOVED ALL NUMBERS FROM WHITELIST", msg);                      
                  }                                    
                }
                
                
                UART3_PutStr("MSG = ");
                UART3_PutStr(msg);
                UART3_PutStr("\n");                
               
                
                
                if(strlen(response_buff) == 0) {
                  //response buff not filled above because the message doesn't match any command format.
                  sprintf(response_buff, "INVALID COMMAND");
                }
                
                send_sms(number, response_buff);
              }
              
            }
            //HAL_Delay(1000);
            
            if(matching == 0) UART3_PutStr("UNKNOWN NUMBER SMS!!\n"); 
            
          }
        }
      }    
    }
    //turn on the motor after scheduled on time
    for(int i = 0; i < 4; i++) {
      if( motor[i].state == MOTOR_STATE_SCHEDULED_TO_ON ) {
        if(HAL_GetTick() >  motor[i].scheduled_on_time) {          
          char deb[20];            
          sprintf(deb, "motor %d on\n", i);
          UART3_PutStr(deb);        
          
          if(i == 0)RELAY_1_ON;
          if(i == 1)RELAY_2_ON;
          if(i == 2)RELAY_3_ON;
          if(i == 3)RELAY_4_ON;
          
          motor[i].state = MOTOR_STATE_SCHEDULED_TO_OFF;

          //every time any event of on / off happens, update the motor rtc backup.
          backup_motor_state_to_rtc_ram();
          
        }
      }
      
      //turn off the motor after scheduled off time
      else if( motor[i].state == MOTOR_STATE_SCHEDULED_TO_OFF ) {
        if(HAL_GetTick() >  motor[i].scheduled_off_time) {            
          
          char deb[20];            
          sprintf(deb, "motor %d off\n", i);
          UART3_PutStr(deb);        
          
          
          if(i == 0)RELAY_1_OFF;
          if(i == 1)RELAY_2_OFF;
          if(i == 2)RELAY_3_OFF;
          if(i == 3)RELAY_4_OFF;
          
          motor[i].state = MOTOR_STATE_OFF;
          //every time any event of on / off happens, update the motor rtc backup.
          backup_motor_state_to_rtc_ram();
          
          
        }
      }
      
      
    }
    //update the rtc backup ram for motor status and remaining time because 
    //remaining timestamp is required for abrupt power off power on time.
    backup_motor_state_to_rtc_ram();
    
    
    
    
  }//while
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL =RCC_PLL_MUL4;// RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int backup_motor_state_to_rtc_ram(void)
{
  
//struct motor_rtc {
//  uint8_t state;
//  uint16_t pending_duration_seconds;
//};
//
//struct motor_rtc_mem { //10 bytes out of 56 
//  uint16_t signature;
//  struct motor_rtc motor_backup[4];
//}motor_rtc_mem;


  motor_rtc_mem.signature = MOTOR_RTC_SIGNATURE;

  for(int i = 0; i < 4; i++) {  
    motor_rtc_mem.motor_backup[i].state = motor[i].state;
    
    if(motor[i].state == MOTOR_STATE_SCHEDULED_TO_OFF) {
      uint64_t rem_millis = motor[i].scheduled_off_time - HAL_GetTick(); //we are using hall tick in milliseconds as time reference and calculations for motor on off
      motor_rtc_mem.motor_backup[i].pending_duration_seconds = rem_millis/1000;       
    } else {
      motor_rtc_mem.motor_backup[i].pending_duration_seconds = 0;
    }      
  }
  
  //backup the motor_rtc_mem structure as it is  to rtc memory for retrieving when MCU turns on. POWER OFF ON
  //RTC is always powered using button cell which can retain the 56 bytes of internal NV RAM. 
  //We are using a small portion of that ram to accomodate motor_rtc_mem
  //structure insider it and we update it in the main while loop almost every seconds. 
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS<<1, 0x08, 1, (uint8_t *)&motor_rtc_mem, sizeof(motor_rtc_mem), 1000);

  return ret;
  
}


void sim800_init(void)
{  
  while( sim800_get("AT", line_buff) ){
    HAL_Delay(1000);               //if ablove line returns error then wait 1s before reatempt
  }
  UART3_PutStr( line_buff);
  UART3_PutStr("\n"); 
  // HAL_Delay(1000);
  while( sim800_get("AT+IPR=115200", line_buff) ){
    HAL_Delay(1000);               //if ablove line returns error then wait 1s before reatempt
  }
  UART3_PutStr( "AT+IPR=115200 SENT\n");
  UART3_PutStr( line_buff);
  UART3_PutStr("\n"); 
  // HAL_Delay(1000);
  ////////////////////////////////////////////////////////////////
  // WAIT FOR NETWORK TO REGISTER
  while(1) {
    while( sim800_get("AT+CREG?", line_buff) ){
      HAL_Delay(1000);
    }
    UART3_PutStr(line_buff);
    if(strcmp(line_buff, "+CREG: 0,1") == 0) {
      UART3_PutStr("NETWORK REGISTERED!\n");  
      break;
    } else {
      UART3_PutStr("NETWORK NOT REGISTERED\n");
      HAL_Delay(1000);
    }
  }
   HAL_Delay(100);
  ////////////////////////////////////////////////////////////////
  while( sim800_get("AT+CMGF=1", line_buff) ){
    HAL_Delay(1000);
  }
  UART3_PutStr("AT+CMGF=1 SENT\n"); 
  UART3_PutStr(line_buff);
  UART3_PutStr("\n");                

  
   
  ////////////////////////////////////////////////////////////////
  while( sim800_get("AT+CNMI=2,2,0,0,0", line_buff) ){
    HAL_Delay(1000);
  } 
   UART3_PutStr("AT+CNMI=2,2,0,0,0 SENT\n");

  HAL_Delay(100);
  
  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
