#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10

#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>

Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
sensors_event_t event;

volatile int cnt = 0;
volatile int mode = 0;
volatile int8_t select = 0;
volatile double x[9] = {0};
volatile double y[9] = {0};
volatile double z[9] = {0};
volatile double init_x[9] = {0};          // 최초 센서 값 x 
volatile double init_y[9] = {0};          // 최초 센서 값 y
volatile double init_z[9] = {0};          // 최초 센서 값 z
volatile double current_x[9] = {0};
volatile double current_y[9] = {0};
volatile double current_z[9] = {0};
volatile double abs_x[9] = {0};
volatile double abs_y[9] = {0};
volatile double abs_z[9] = {0};
volatile double max_x = 0;
volatile double max_y = 0;
volatile double max_z = 0;
volatile double current_i = 0;
volatile int num = 7;             // 테스트할 때 쓰는 변수. 특정 센서 값을 출력하려면 해당 변수를 사용할 것.
volatile double checksum = 0;        // Checksum 값을 저장하는 변수.
volatile char packet[9] = {'0'};    // 패킷의 알맹이 값을 임시 저장하는 버퍼

volatile int high_num_int = 0;          // 센서값 앞 정수형 네자리 
volatile int low_num_int = 0;           // 센서값 뒤 정수형 네자리
volatile double high_num_real = 0;      // 센서값 앞 실수형 네자리
volatile double low_num_real = 0;       // 센서값 뒤 실수형 네자리
volatile int iii = 0;                   // 전체 패킷에 대한 인덱스 변수
char whole_packet[292] = {0, };     // 최종 포장된 패킷 배열 
char whole_packet_sub[32] = {0, };   // 패킷을 32개씩 쪼개서 저장할 2차원 배열 선언
                                            // 마지막 10번쩨 배열은 5개만 저장
volatile int row_num = 0;
volatile int column_num = 0;

volatile double background_b[9][3] = {0,};     // 배경에 깔려있는 자기장 값을 저장하는 배열 변수
volatile int background_flag = 0;              // 배경에 깔려있는 자기장 값을 한 번만 받게 하기 위한 flag변수



/////////////////
/// 함수 선언 ///
////////////////
void high_low_seperating(double num_real);
void prints(int a, int b);	


//////////////////////
// 버튼 인터럽트 설정 //
/////////////////////
ISR(INT0_vect)
{
  //Serial.println("Mode 1");
}
ISR(INT1_vect)
{
  //Serial.println("Mode 2");
}





////////////////////////////////////////////
// 타이머카운터 인터럽트를 이용한 제어주기 설정 //
////////////////////////////////////////////
ISR(TIMER1_OVF_vect) // 제어주기: 0.01s
{
  TCNT1 = 64911;
  cnt++;
  if(cnt == 1)  // 1s
  { 
    if(mode == 0 || mode == 1) // 처음 시작할 때: 날 값 그대로 출력
    {
      iii = 0;        // 전체 패킷 인덱스 0부터 시작

      whole_packet[iii++] = 'Z';     // 0번째 위치
      whole_packet[iii++] = 'Z';     // 1번째 위치
      whole_packet[iii++] = 'Z';     // 2번째 위치


      // n번째 센서 값 받아오기
      for(select = 0; select < 9 ; select++) 
      {                          
         PORTD = PORTD & 0b00001111;
         PORTD = PORTD | (select<<4);
         sensors_event_t event;
         lis2mdl.getEvent(&event);
         init_x[select] = (event.magnetic.x * 100);
         init_y[select] = (event.magnetic.y * 100);
         init_z[select] = (event.magnetic.z * 100);   
                            // 패킷 통신을 비교적 쉽게 받기 위해 소수점을 제거한 상태로 저장


         whole_packet[iii++] = select + 65;

         high_low_seperating(init_x[select]);   // n번째 센서의 x축 값 포장
         whole_packet[iii++] = 44;    // 콤마
         high_low_seperating(init_y[select]);   // n번째 센서의 y축 값 포장
         whole_packet[iii++] = 44;    // 콤마
         high_low_seperating(init_z[select]);   // n번째 센서의 z축 값 포장
         
         whole_packet[iii++] = select + 97; // n번째 센서값 끝을 알리는 플래그변수
      }

      // Checksum 계산
      for(select = 0; select < 9 ; select++)
        checksum += (init_x[select]) + (init_y[select]) + (init_z[select]);

      high_low_seperating(checksum);            // checksum 값도 부호를 포함한 9자리로 재포장 후 출력    
      whole_packet[iii++] = 'Y';
    
      if(mode == 0){

        iii = 0;
        for(row_num = 0; row_num < 9 ; row_num++){
          for(column_num = 0; column_num < 32; column_num++)
            whole_packet_sub[column_num] = whole_packet[iii++];
          Serial.print(whole_packet_sub);
        }
        // 마지막 5개 문자 출력 
        for(column_num = 0; column_num < 4; column_num++)
          whole_packet_sub[column_num] = whole_packet[iii++];

        for(column_num = 4; column_num < 32; column_num++)
          whole_packet_sub[column_num] = 'Y';
          Serial.print(whole_packet_sub);
      }

      checksum = 0;     // checksum은 누적되서 계산하므로 한 주기 끝날 때마다 0으로 초기화 

    }

    cnt = 0;
  }


}







///////////
// 메인문 //
///////////
void setup() 
{
  //Interrupt Setting
  DDRD = (1<<DDD7)|(1<<DDD6)|(1<<DDD5)|(1<<DDD4);   // demultiplexer와 버튼 등 핀 설정 = 0b 0000 1111
  EICRA = (1<<ISC11)|(1<<ISC01);                    // 인터럽트 여부 설정
  EIMSK = (1<<INT1)|(1<<INT0);                      // 인터럽트를 활성화 여부 설정
  
  Serial.begin(115200);                 // Baud Rate 설정
  // Serial.begin(250000);
  // Serial.println("X, Y, Z");          // 범례 설정(아두이노의 시리얼 플로터 관련 설정(확인 다 끝나면 주석처리 하셈))
  
  //CS
  PORTD = PORTD & 0b00001111;
  select = 0;
  PORTD = PORTD | (select<<4);
  lis2mdl.enableAutoRange(true);
  
  //LIS2MDL connect
  for(select = 0; select < 9 ; select++) 
  {
    PORTD = PORTD & 0b00001111;
    PORTD = PORTD | (select<<4);
    if (! lis2mdl.begin_SPI(LIS2MDL_CS, LIS2MDL_CLK, LIS2MDL_MISO, LIS2MDL_MOSI)) // soft SPI 
     { 
       //Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
        while(1) 
       {
          if (! lis2mdl.begin_SPI(LIS2MDL_CS, LIS2MDL_CLK, LIS2MDL_MISO, LIS2MDL_MOSI)) 
         {
           //Serial.println(" is now ok");
            break;
         }
       }
     }

  // LIS2MDL 자체 레지스터 접근 및 설정
  lis2mdl.CFG_REG_A(); // Continuous mode로 설정 & ODR(Output Data Rate)=100[Hz] 설정
  lis2mdl.CFG_REG_B(); // 출력되는 data에 대한 설정
  
  }
  
  
  
  //16bit Timer 10ms
  TCCR1A = 0x00;//Nomal mode
  TCCR1B = (1<<CS12);//256분주비
  TIMSK1 = (1<<TOIE1);//O.I.E.
  TCNT1 = 64911;
  
  sei();
}



void loop() {}



void high_low_seperating(double num_real) {


	if (num_real < 0) whole_packet[iii++] = '1';	// 음수면 1
	else whole_packet[iii++] = '0';				// 양수면 0

  num_real = abs(num_real);

	// 앞 네 자리 분리
	high_num_int = num_real / 10000;
  

	// 뒤 네 자리 마저 분리
	low_num_int = (num_real - (double)(high_num_int * 10000));

  //Serial.print(high_num_int);
  //Serial.print(" ");
  //Serial.print(low_num_int);


	prints(high_num_int, low_num_int);    // 부호 상태까지 포함한 9자리 배열로 재포장

}


void prints(int a, int b) {

  /*
	// 앞 네 자리 패킷에 순차 저장
	packet[1] = (a / 1000) + 48;
	packet[2] = ((a % 1000) / 100) + 48;
	packet[3] = (((a % 1000) % 100) / 10) + 48;
	packet[4] = (((a % 1000) % 100) % 10) + 48;

	// 뒤 네 자리도 패킷에 순차 저장
	packet[5] = (b / 1000) + 48;
	packet[6] = ((b % 1000) / 100) + 48;
	packet[7] = (((b % 1000) % 100) / 10) + 48;
	packet[8] = (((b % 1000) % 100) % 10) + 48;
  */
  //packet[9] = '\0';

  // 앞 네 자리 패킷에 순차 저장
	whole_packet[iii++] = (a / 1000) + 48;
	whole_packet[iii++] = ((a % 1000) / 100) + 48;
	whole_packet[iii++] = (((a % 1000) % 100) / 10) + 48;
	whole_packet[iii++] = (((a % 1000) % 100) % 10) + 48;

	// 뒤 네 자리도 패킷에 순차 저장
	whole_packet[iii++] = (b / 1000) + 48;
	whole_packet[iii++] = ((b % 1000) / 100) + 48;
	whole_packet[iii++] = (((b % 1000) % 100) / 10) + 48;
	whole_packet[iii++] = (((b % 1000) % 100) % 10) + 48;

  // 버퍼 패킷 출력
  //for(iii = 0; iii < 9; iii++) Serial.print(packet[iii]);

}











































