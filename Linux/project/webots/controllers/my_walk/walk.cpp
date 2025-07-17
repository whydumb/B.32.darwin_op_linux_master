// 필요한 헤더 파일들
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include "Walk.hpp"
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

// 로봇 상태 전역변수
int gain = 128;
bool isWalking = false;
double xAmplitude = 0.0;
double yAmplitude = 0.0;
double aAmplitude = 0.0;
pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL",
  "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL",
  "PelvR", "PelvL", "LegUpperR", "LegUpperL",
  "LegLowerR", "LegLowerL", "AnkleR", "AnkleL",
  "FootR", "FootL", "Neck", "Head"
};

char* load_html() {
    FILE* f = fopen("walk.html", "r");
    if (!f) {
        static const char html[] = 
            "HTTP/1.1 200 OK\r\n\r\n"
            "<html><body>"
            "<h1>Robot Control</h1>"
            "<button onclick=\"fetch('/?walk_start')\">Start Walking</button>"
            "<button onclick=\"fetch('/?walk_stop')\">Stop Walking</button><br><br>"
            "<button onclick=\"fetch('/?move_forward')\">Forward</button>"
            "<button onclick=\"fetch('/?move_backward')\">Backward</button><br>"
            "<button onclick=\"fetch('/?turn_left')\">Turn Left</button>"
            "<button onclick=\"fetch('/?turn_right')\">Turn Right</button>"
            "</body></html>";
        
        // 동적으로 메모리 할당하여 반환
        char* result = (char*)malloc(strlen(html) + 1);
        strcpy(result, html);
        return result;
    }
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);
    char* html = (char*)malloc(size + 1);
    fread(html, 1, size, f);
    html[size] = '\0';
    fclose(f);
    return html;
}

void* server(void* arg) {
    (void)arg; // unused parameter warning 제거
    
    printf("Server thread starting...\n");  // 디버그 로그 추가
    
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8081);  // 8080 → 8081로 변경
    addr.sin_addr.s_addr = INADDR_ANY;  // 모든 IP에서 접속 허용
    
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("Bind failed!\n");
        return NULL;
    }
    listen(s, 1);
    
    printf("Server: http://0.0.0.0:8081 (accessible from external)\n");
    
    while (1) {
        printf("Waiting for connection...\n");  // 디버그 로그 추가
        int client = accept(s, NULL, NULL);
        printf("Client connected!\n");  // 디버그 로그 추가
        
        char buf[512];
        ssize_t bytes_read = read(client, buf, sizeof(buf) - 1);
        buf[bytes_read] = '\0';
        
        printf("Received: %s\n", buf);  // 디버그 로그 추가
        
        // 요청 파싱 (GET 라인만 추출)
        char *get_line = strtok(buf, "\r\n");
        printf("GET line: %s\n", get_line);
        
        pthread_mutex_lock(&stateMutex);
        
        if (strstr(get_line, "walk_start")) {
            if (!isWalking) {
                isWalking = true;
                // mGaitManager->start()는 run() 루프에서 한 번만 호출됨
                printf("Walking started\n");
            }
        }
        else if (strstr(get_line, "walk_stop")) {
            if (isWalking) {
                isWalking = false;
                xAmplitude = yAmplitude = aAmplitude = 0.0;
                // mGaitManager->stop()는 run() 루프에서 한 번만 호출됨
                printf("Walking stopped\n");
            }
        }
        else if (strstr(get_line, "move_forward")) {
            if (xAmplitude == 1.0) {
                xAmplitude = 0.0;  // 이미 전진이면 정지
                printf("Forward stopped\n");
            } else {
                xAmplitude = 1.0;  // 아니면 전진
                printf("Moving forward\n");
            }
        }
        else if (strstr(get_line, "move_backward")) {
            if (xAmplitude == -1.0) {
                xAmplitude = 0.0;  // 이미 후진이면 정지
                printf("Backward stopped\n");
            } else {
                xAmplitude = -1.0;  // 아니면 후진
                printf("Moving backward\n");
            }
        }
        else if (strstr(get_line, "turn_left")) {
            if (aAmplitude == 0.5) {
                aAmplitude = 0.0;  // 이미 좌회전이면 정지
                printf("Left turn stopped\n");
            } else {
                aAmplitude = 0.5;  // 아니면 좌회전
                printf("Turning left\n");
            }
        }
        else if (strstr(get_line, "turn_right")) {
            if (aAmplitude == -0.5) {
                aAmplitude = 0.0;  // 이미 우회전이면 정지
                printf("Right turn stopped\n");
            } else {
                aAmplitude = -0.5;  // 아니면 우회전
                printf("Turning right\n");
            }
        }
        
        pthread_mutex_unlock(&stateMutex);
        
        char* html = load_html();
        ssize_t write_result = write(client, html, strlen(html));
        (void)write_result; // unused variable warning 제거
        free(html);
        
        close(client);
    }
    return NULL;
}

Walk::Walk(): Robot() {
  mTimeStep = getBasicTimeStep();
  
  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  
  getGyro("Gyro")->enable(mTimeStep);
  
  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
  }
  
  keyboardEnable(mTimeStep);
  
  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager = new DARwInOPGaitManager(this, "config.ini");
}

Walk::~Walk() {
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void Walk::run() {
  cout << "-------Walk example of DARwIn-OP-------" << endl;
  cout << "Web control enabled on port 8080" << endl;
  
  // 웹 서버 스레드 시작
  pthread_t serverThread;
  pthread_create(&serverThread, NULL, server, NULL);
  
  myStep();
  mMotionManager->playPage(9); // init position
  wait(200);
  
  while (true) {
    checkIfFallen();
    
    // 원본처럼 매 루프마다 초기화
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setYAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    
    pthread_mutex_lock(&stateMutex);
    
    // 웹에서 받은 값으로 로봇 제어 (원본 키보드 방식과 동일)
    mGaitManager->setXAmplitude(xAmplitude);
    mGaitManager->setYAmplitude(yAmplitude);
    mGaitManager->setAAmplitude(aAmplitude);
    
    pthread_mutex_unlock(&stateMutex);
    
    mGaitManager->step(mTimeStep);
    myStep(); 
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;
  
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;
  
  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;
  
  if (fup > acc_step) {
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9); // init position    
    fup = 0;
  }
  else if (fdown > acc_step) {
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9); // init position
    fdown = 0;
  }
}
