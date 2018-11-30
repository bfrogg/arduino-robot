#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOSVariant.h>
//#include <heap_3.c>
#include <task.h>
#include <portmacro.h>
#include <queue.h>
#include <semphr.h>
#include <PS2X_lib.h>

#define PS2_DAT        12
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        13

#define LEFT_MOTOR_STRAIGHT  3         
#define LEFT_MOTOR_REVERSE   2         
#define RIGHT_MOTOR_STRAIGHT 5         
#define RIGHT_MOTOR_REVERSE  4         

#define RIGHT_SHOULDER   4
#define RIGHT_ELBOW      6
#define RIGHT_HAND       8
#define LEFT_SHOULDER    14
#define LEFT_ELBOW       12
#define LEFT_HAND        10
#define HEAD_X           18
#define HEAD_Y           16

#define LOW_SPEED        2300
#define HIGH_SPEED       100
#define LOW_DELAY        500

#define QUEUE_SIZE 50

//#define DEBUG

enum Commands {
  Neutral,
  Handshake,
  Hands_up_straight,
  Hands_up_angle,
  Hands_low,
  Hands_forward_straight,
  Hands_forward_angle,
  Handshake_2_hands,
  Head_up,
  Head_down,
  Head_right,
  Head_left,
  Head_straight,
  Right_hand_open,
  Right_hand_closed,
  Left_hand_open,
  Left_hand_closed,
  Handshake_left_hand
};

QueueHandle_t q;
SemaphoreHandle_t xGamepadSemaphore;

bool canMove = false;
bool rightHand = true;
bool leftHand = false;

PS2X ps2x;

void TaskGamepadPolling( void *pvParameters );
void TaskServoCommunication( void *pvParameters );
void TaskWheelsContol( void *pvParameters );

// Функция ожидания ответа контроллера сервоприводов
void wait_serial_return_ok();

// Перегруженные функции для формирования команд сервоприводам
inline void MakeServoCommand(byte ch, int pos, int t, int d);
inline void MakeServoCommand(byte ch1, int pos1, byte ch2, int pos2, int t, int d);
inline void MakeServoCommand(byte ch1, int pos1, byte ch2, int pos2, byte ch3, int pos3, byte ch4, int pos4, int t, int d);

void setup() {

  Serial.begin(9600);

  //Serial.println("Init...");

  while (ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false) != 0) {
    //Serial.println("Error configuring gamepad");
    delay(200);
  }

  //Serial.println("Gamepad OK");

  pinMode(LEFT_MOTOR_STRAIGHT, OUTPUT);
  pinMode(RIGHT_MOTOR_STRAIGHT, OUTPUT);
  pinMode(LEFT_MOTOR_REVERSE, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE, OUTPUT);

  q = xQueueCreate( QUEUE_SIZE, sizeof( unsigned char ) );

  if (q == NULL) {
    //Serial.println("Error creating the queue");
    return;
  }

  if ( xGamepadSemaphore == NULL )
  {
    xGamepadSemaphore = xSemaphoreCreateMutex();
    if ( ( xGamepadSemaphore ) != NULL ) {
      xSemaphoreGive( ( xGamepadSemaphore ) );
      //Serial.println("Semaphore OK");
    }
    else {
      //Serial.println("Error creating the semaphore");
      return;
    }
  }

  xTaskCreate(
    TaskGamepadPolling
    ,  (const portCHAR *)"Опрос кнопок"
    ,  64
    ,  NULL
    ,  2
    ,  NULL );


  xTaskCreate(
    TaskServoCommunication
    ,  (const portCHAR *) "Управление сервоприводами"
    ,  128
    ,  NULL
    ,  1
    ,  NULL );

  xTaskCreate(
    TaskWheelsContol
    ,  (const portCHAR *) "Управление колесами"
    ,  128
    ,  NULL
    ,  3
    ,  NULL );

  //Serial.println("Init OK");

}

void loop()
{

}

// Функция блокирует поток до тех пор, пока не будет получен ответ от платы управления сервоприводами
inline void wait_serial_return_ok()
{
  int num = 0;
  char c[16];
  while (1)
  {
    while (Serial.available() > 0)
    {
      c[num] = Serial.read();
      num++;
      if (num >= 15)
        num = 0;
    }
    if (c[num - 2] == 'O' && c[num - 1] == 'K') {
#ifdef DEBUG
      Serial.println(c);
#endif
      break;
    }
  }
}

void MakeServoCommand(byte ch, int pos, int t, int d) {
  Serial.print('#');
  Serial.print(ch);
  Serial.print('P');
  Serial.print(pos);
  Serial.print('T');
  Serial.print(t);
  Serial.print('D');
  Serial.print(d);
  Serial.print('\r');
  Serial.print('\n');
}

void MakeServoCommand(byte ch1, int pos1, byte ch2, int pos2, int t, int d) {
  Serial.print('#');
  Serial.print(ch1);
  Serial.print('P');
  Serial.print(pos1);
  Serial.print('#');
  Serial.print(ch2);
  Serial.print('P');
  Serial.print(pos2);
  Serial.print('T');
  Serial.print(t);
  Serial.print('D');
  Serial.print(d);
  Serial.print('\r');
  Serial.print('\n');
}

void MakeServoCommand(byte ch1, int pos1, byte ch2, int pos2, byte ch3, int pos3, byte ch4, int pos4, int t, int d) {
  Serial.print('#');
  Serial.print(ch1);
  Serial.print('P');
  Serial.print(pos1);
  Serial.print('#');
  Serial.print(ch2);
  Serial.print('P');
  Serial.print(pos2);
  Serial.print('#');
  Serial.print(ch3);
  Serial.print('P');
  Serial.print(pos3);
  Serial.print('#');
  Serial.print(ch4);
  Serial.print('P');
  Serial.print(pos4);
  Serial.print('T');
  Serial.print(t);
  Serial.print('D');
  Serial.print(d);
  Serial.print('\r');
  Serial.print('\n');
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskGamepadPolling(void *pvParameters)
{
  (void) pvParameters;

  int cmd;

  //Serial.println("Polling");
  vTaskDelay( 1000 / portTICK_PERIOD_MS  );

  // Опрос геймпада раз в 50 мс
  for (;;)
  {
    //Serial.println("Polling loop");
    if ( xSemaphoreTake( xGamepadSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      ps2x.read_gamepad(false, false);
      xSemaphoreGive( xGamepadSemaphore );
    }

    if (ps2x.ButtonPressed(PSB_START)) {
      cmd = Neutral;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("START");
#endif
    }
    if (ps2x.ButtonPressed(PSB_SELECT)) {
      cmd = PSB_SELECT;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("SELECT");
#endif
    }
    if (ps2x.ButtonPressed(PSB_PAD_UP)) {
      cmd = Hands_up_straight;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("PAD_UP");
#endif
    }
    if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
      cmd = Hands_forward_straight;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("PAD_RIGHT");
#endif
    }
    if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
      cmd = Hands_forward_angle;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("PAD_LEFT");
#endif
    }
    if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
      cmd = Hands_up_angle;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("PAD_DOWN");
#endif
    }
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
      cmd = Handshake_2_hands;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("TRIANGLE");
#endif
    }
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      cmd = Hands_low;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("CIRCLE");
#endif
    }
    if (ps2x.ButtonPressed(PSB_CROSS)) {
      cmd = Handshake;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("CROSS");
#endif
    }
    if (ps2x.ButtonPressed(PSB_SQUARE)) {
      cmd = Handshake_left_hand;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("SQUARE");
#endif
    }
    if (ps2x.ButtonPressed(PSB_L1)) {
      if (leftHand)
        cmd = Left_hand_closed;
      else cmd = Left_hand_open;
      leftHand ^= 1;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("L1");
#endif
    }
    if (ps2x.Button(PSB_L2)) {
      canMove = true;
    } else canMove = false;
    if (ps2x.ButtonPressed(PSB_R1)) {
      if (rightHand)
        cmd = Right_hand_closed;
      else cmd = Right_hand_open;
      rightHand ^= 1;
      xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
      Serial.println("R1");
#endif
    }
    if (ps2x.Button(PSB_R2)) {
      // Зона нечувствительности добавлена намеренно
      if (ps2x.Analog(PSS_RY) > 138) {
        cmd = Head_down;
        xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
        Serial.println("RY_DOWN");
#endif
      }
      if (ps2x.Analog(PSS_RY) < 118) {
        cmd = Head_up;
        xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
        Serial.println("RY_UP");
#endif
      }
      if (ps2x.Analog(PSS_RX) > 138) {
        cmd = Head_right;
        xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
        Serial.println("RX_RIGHT");
#endif
      }
      if (ps2x.Analog(PSS_RX) < 118) {
        cmd = Head_left;
        xQueueSend(q, &cmd, portMAX_DELAY);
#ifdef DEBUG
        Serial.println("RX_LEFT");
#endif
      }
      if (ps2x.ButtonPressed(PSB_R3)) {
        cmd = Head_straight;
        xQueueSend(q, &cmd, portMAX_DELAY);
      }
    }

    vTaskDelay( 50 / portTICK_PERIOD_MS  );
  }
}

void TaskServoCommunication(void *pvParameters)
{
  (void) pvParameters;

  int cmd = Neutral;

  vTaskDelay( 1000 / portTICK_PERIOD_MS  );

  //Serial.println("Servo");

  for (;;)
  {
    //Serial.println("Servo loop");
    xQueueReceive(q, &cmd, portMAX_DELAY);
    switch (cmd) {
      case Neutral: {
#ifdef DEBUG
          Serial.println("Neutral");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          1500,
                                          RIGHT_ELBOW,
                                          1500,
                                          LEFT_SHOULDER,
                                          1500,
                                          LEFT_ELBOW,
                                          1500,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Handshake: {
#ifdef DEBUG
          Serial.println("Handshake");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          1300,
                                          RIGHT_ELBOW,
                                          1000,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Handshake_left_hand: {
          MakeServoCommand(LEFT_SHOULDER,
                                          1300,
                                          LEFT_ELBOW,
                                          1000,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Left_hand_open: {
          MakeServoCommand(LEFT_HAND,
                                          1500,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Left_hand_closed: {
          MakeServoCommand(LEFT_HAND,
                                          1360,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Right_hand_closed: {
          MakeServoCommand(RIGHT_HAND,
                                          1670,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Right_hand_open: {
          MakeServoCommand(RIGHT_HAND,
                                          1500,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Handshake_2_hands: {
#ifdef DEBUG
          Serial.println("Handshake_2_hands");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          1300,
                                          RIGHT_ELBOW,
                                          1000,
                                          LEFT_SHOULDER,
                                          1300,
                                          LEFT_ELBOW,
                                          1000,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Hands_up_straight: {
#ifdef DEBUG
          Serial.println("Hands_up_straight");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          500,
                                          RIGHT_ELBOW,
                                          1500,
                                          LEFT_SHOULDER,
                                          500,
                                          LEFT_ELBOW,
                                          1500,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Hands_low: {
          MakeServoCommand(LEFT_SHOULDER,
                                          500,
                                          LEFT_ELBOW,
                                          1500,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Hands_forward_straight: {
#ifdef DEBUG
          Serial.println("Hands_forward_straight");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          900,
                                          RIGHT_ELBOW,
                                          1000,
                                          LEFT_SHOULDER,
                                          900,
                                          LEFT_ELBOW,
                                          1000,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Hands_forward_angle: {
#ifdef DEBUG
          Serial.println("Hands_forward_straight");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          900,
                                          RIGHT_ELBOW,
                                          1500,
                                          LEFT_SHOULDER,
                                          900,
                                          LEFT_ELBOW,
                                          1500,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Hands_up_angle: {
#ifdef DEBUG
          Serial.println("Hands_up_angle");
#endif
          MakeServoCommand(RIGHT_SHOULDER,
                                          500,
                                          RIGHT_ELBOW,
                                          700,
                                          LEFT_SHOULDER,
                                          500,
                                          LEFT_ELBOW,
                                          700,
                                          LOW_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Head_up: {
#ifdef DEBUG
          Serial.println("Head_up");
#endif
          MakeServoCommand(HEAD_Y,
                                          1100,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Head_down: {
#ifdef DEBUG
          Serial.println("Head_down");
#endif
          MakeServoCommand(HEAD_Y,
                                          1500,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Head_left: {
#ifdef DEBUG
          Serial.println("Head_left");
#endif
          MakeServoCommand(HEAD_X,
                                          1900,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Head_right: {
#ifdef DEBUG
          Serial.println("Head_right");
#endif
          MakeServoCommand(HEAD_X,
                                          1100,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      case Head_straight: {
#ifdef DEBUG
          Serial.println("Head_straight");
#endif
          MakeServoCommand(HEAD_X,
                                          1500,
                                          HEAD_Y,
                                          1350,
                                          HIGH_SPEED,
                                          LOW_DELAY);
          wait_serial_return_ok();
          break;
        }
      default:
#ifdef DEBUG
        Serial.println("No command");
#endif
        break;
    }
  }
  vTaskDelay( 100 / portTICK_PERIOD_MS  );
}

void TaskWheelsContol(void *pvParameters)
{
  (void) pvParameters;

  int controllerX = 0, controllerY = 0;
  int leftMotorSpeed = 0, rightMotorSpeed = 0;
  const float Xcoeff = 1.5;
  bool leftMotorDirection, rightMotorDirection;

  digitalWrite(LEFT_MOTOR_STRAIGHT, true);
  digitalWrite(RIGHT_MOTOR_STRAIGHT, true);
  digitalWrite(LEFT_MOTOR_REVERSE, true);
  digitalWrite(RIGHT_MOTOR_REVERSE, true);

  //Serial.println("Wheel");
  vTaskDelay( 500 / portTICK_PERIOD_MS  );

  for (;;)
  {   
    if (canMove) {
      // Семафор для синхронизации между потоками
      if ( xSemaphoreTake( xGamepadSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        controllerY = ps2x.Analog(PSS_LY);
        controllerX = ps2x.Analog(PSS_LX);
        xSemaphoreGive( xGamepadSemaphore );
      };

      if (controllerX > 253 && (controllerY < 130 && controllerY > 126)) {
        digitalWrite(LEFT_MOTOR_STRAIGHT, true);
        digitalWrite(LEFT_MOTOR_REVERSE, false);
        digitalWrite(RIGHT_MOTOR_STRAIGHT, false);
        digitalWrite(RIGHT_MOTOR_REVERSE, true);
      } else if (controllerX < 2 && (controllerY < 130 && controllerY > 126)) {
        digitalWrite(LEFT_MOTOR_STRAIGHT, false);
        digitalWrite(LEFT_MOTOR_REVERSE, true);
        digitalWrite(RIGHT_MOTOR_STRAIGHT, true);
        digitalWrite(RIGHT_MOTOR_REVERSE, false);
      } else if (controllerY > 135) {
        digitalWrite(LEFT_MOTOR_STRAIGHT, false);
        digitalWrite(LEFT_MOTOR_REVERSE, true);
        digitalWrite(RIGHT_MOTOR_STRAIGHT, false);
        digitalWrite(RIGHT_MOTOR_REVERSE, true);
      } else if (controllerY < 120) {
        digitalWrite(LEFT_MOTOR_STRAIGHT, true);
        digitalWrite(LEFT_MOTOR_REVERSE, false);
        digitalWrite(RIGHT_MOTOR_STRAIGHT, true);
        digitalWrite(RIGHT_MOTOR_REVERSE, false);
      } else {
        digitalWrite(LEFT_MOTOR_STRAIGHT, false);
        digitalWrite(RIGHT_MOTOR_STRAIGHT, false);
        digitalWrite(LEFT_MOTOR_REVERSE, false);
        digitalWrite(RIGHT_MOTOR_REVERSE, false);
      }
    } else {
      digitalWrite(LEFT_MOTOR_STRAIGHT, false);
      digitalWrite(RIGHT_MOTOR_STRAIGHT, false);
      digitalWrite(LEFT_MOTOR_REVERSE, false);
      digitalWrite(RIGHT_MOTOR_REVERSE, false);
    }

#ifdef DEBUG
    vTaskDelay( 100 / portTICK_PERIOD_MS  );
#endif

    vTaskDelay( 50 / portTICK_PERIOD_MS  );
  }
}


