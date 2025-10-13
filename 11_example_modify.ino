#include <Servo.h>

// =================================================================
// 1. 핀 할당 (PIN ASSIGNMENT)
// =================================================================
#define PIN_LED      9    // LED 핀 번호 (Active-low: HIGH일 때 꺼짐)
#define PIN_TRIG     12   // 초음파 센서 TRIGGER 핀
#define PIN_ECHO     13   // 초음파 센서 ECHO 핀
#define PIN_SERVO    10   // 서보 모터 핀

// =================================================================
// 2. 설정 값 및 상수 정의 (CONFIGURATION & CONSTANTS)
// =================================================================
#define SND_VEL 346.0      // 음속 (섭씨 24도 기준, 단위: m/sec)
#define INTERVAL 25        // 측정 간격 (단위: msec)
#define PULSE_DURATION 10  // 초음파 펄스 지속시간 (단위: usec)

// 측정 유효 범위 (단위: mm)
#define _DIST_MIN 180.0    // 측정할 최소 거리 -> 0도
#define _DIST_MAX 360.0    // 측정할 최대 거리 -> 180도

// EMA(지수이동평균) 필터 가중치 (0 ~ 1.0)
#define _EMA_ALPHA 0.3

// 타이밍 및 거리 변환 상수
#define TIMEOUT ((INTERVAL / 2) * 1000.0) // 최대 초음파 대기 시간 (단위: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // pulseIn 값을 거리(mm)로 변환하기 위한 계수

// 서보 모터 펄스 폭 설정 (myservo.writeMicroseconds() 사용 시)
#define _DUTY_MIN 400   // 0도 위치 펄스 폭
#define _DUTY_NEU 1480  // 90도 (중립) 위치 펄스 폭
#define _DUTY_MAX 2600  // 180도 위치 펄스 폭

// =================================================================
// 3. 전역 변수 (GLOBAL VARIABLES)
// =================================================================
Servo myservo;                       // 서보 객체 생성

float  dist_ema;                     // EMA 필터가 적용된 최종 거리 값
float  dist_prev;                    // 이전에 측정된 유효한 거리 값 (범위 필터링용)
unsigned long last_sampling_time;    // 마지막 측정 시간 (단위: ms)

// =================================================================
// 4. 초기화 함수 (setup)
// =================================================================
void setup() {
  // GPIO 핀 모드 초기화
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW); // TRIGGER 핀 LOW로 설정
  
  // 서보 모터 초기화 및 중앙 위치 설정
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU); // 시작 시 서보를 중앙(90도)에 위치시킴
  
  // 거리 관련 변수들을 중간값으로 초기화 (시작 시 서보가 중앙에서 대기)
  dist_prev = (_DIST_MIN + _DIST_MAX) / 2.0;
  dist_ema = dist_prev;
  
  // 시리얼 통신 시작
  Serial.begin(57600);
}

// =================================================================
// 5. 메인 루프 함수 (loop)
// =================================================================
void loop() {
  float  dist_raw, dist_filtered;
  
  // 설정된 측정 간격(INTERVAL)이 지났는지 확인 (논-블로킹)
  if (millis() < last_sampling_time + INTERVAL)
    return;
  
  // 초음파 센서로 거리 측정
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  
  // 범위 필터: 측정된 값이 유효한지 확인
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
      // 유효 범위를 벗어나면, 이전에 저장된 유효 값(dist_prev)을 사용
      dist_filtered = dist_prev; 
      digitalWrite(PIN_LED, HIGH);   // LED를 끔 (Active-low 방식)
  } else {    // 유효 범위 안일 경우
      // 현재 측정값을 사용하고, 다음 측정을 위해 '이전 값'으로 저장
      dist_filtered = dist_raw;    
      dist_prev = dist_raw;      
      digitalWrite(PIN_LED, LOW);    // LED를 켬 (Active-low 방식)
  }
  
  // EMA(지수이동평균) 필터를 적용하여 측정값의 급격한 변화를 줄임
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;
  
  // map() 함수를 사용해 필터링된 거리(dist_ema)를 서보 각도(0~180)로 변환합니다.
  // map 함수는 long 타입을 받으므로, dist_ema를 명시적으로 long으로 변환합니다.
  int angle = map((long)dist_ema, (long)_DIST_MIN, (long)_DIST_MAX, 0, 180);
  
  // 변환된 각도로 서보 모터를 움직입니다.
  myservo.write(angle);
  
  // 현재 상태를 시리얼 모니터에 출력 (디버깅 정보)
  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",dist:");   Serial.print(dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",angle:");  Serial.print(angle);
  Serial.print(",Servo:");  Serial.print(myservo.read());
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");
  
  // 마지막 측정 시간 업데이트
  last_sampling_time += INTERVAL;
}

// =================================================================
// 6. 초음파 측정 함수 (USS_measure)
// =================================================================
// 초음파 센서로 거리를 측정하는 함수 (단위: mm)
float USS_measure(int TRIG, int ECHO)
{
  // TRIGGER 핀으로 10us 길이의 초음파 펄스 발생
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  // ECHO 핀이 HIGH가 되는 시간(왕복 시간)을 측정하여 거리로 변환 후 반환
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
