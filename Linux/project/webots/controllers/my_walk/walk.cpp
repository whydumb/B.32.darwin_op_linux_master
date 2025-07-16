#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>

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
        static char html[] = 
            "HTTP/1.1 200 OK\r\n\r\n"
            "<html><body>"
            "<h1>Robot Control</h1>"
            "<button onclick=\"fetch('/?gain_plus')\">Gain +</button>"
            "<button onclick=\"fetch('/?gain_minus')\">Gain -</button><br><br>"
            "<button onclick=\"fetch('/?walk_start')\">Start Walking</button>"
            "<button onclick=\"fetch('/?walk_stop')\">Stop Walking</button><br><br>"
            "<button onclick=\"fetch('/?move_forward')\">Forward</button>"
            "<button onclick=\"fetch('/?move_backward')\">Backward</button><br>"
            "<button onclick=\"fetch('/?turn_left')\">Turn Left</button>"
            "<button onclick=\"fetch('/?turn_right')\">Turn Right</button>"
            "</body></html>";
        return strdup(html);
    }
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);
    char* html = malloc(size + 1);
    fread(html, 1, size, f);
    html[size] = 0;
    fclose(f);
    return html;
}

void* server(void*) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {AF_INET, htons(8080), {INADDR_ANY}};
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(s, (struct sockaddr*)&addr, sizeof(addr));
    listen(s, 1);
    
    printf("Server: http://localhost:8080\n");
    
    while (1) {
        int client = accept(s, NULL, NULL);
        char buf[512];
        read(client, buf, sizeof(buf));
        
        pthread_mutex_lock(&stateMutex);
        
        if (strstr(buf, "gain_plus")) {
            gain += 10;
            printf("Gain: %d\n", gain);
        }
        else if (strstr(buf, "gain_minus")) {
            gain -= 10;
            printf("Gain: %d\n", gain);
        }
        else if (strstr(buf, "walk_start")) {
            isWalking = true;
            printf("Walking started\n");
        }
        else if (strstr(buf, "walk_stop")) {
            isWalking = false;
            xAmplitude = yAmplitude = aAmplitude = 0.0;
            printf("Walking stopped\n");
        }
        else if (strstr(buf, "move_forward")) {
            xAmplitude = 1.0;
            printf("Moving forward\n");
        }
        else if (strstr(buf, "move_backward")) {
            xAmplitude = -1.0;
            printf("Moving backward\n");
        }
        else if (strstr(buf, "turn_left")) {
            aAmplitude = 0.5;
            printf("Turning left\n");
        }
        else if (strstr(buf, "turn_right")) {
            aAmplitude = -0.5;
            printf("Turning right\n");
        }
        
        pthread_mutex_unlock(&stateMutex);
        
        char* html = load_html();
        write(client, html, strlen(html));
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
    
    pthread_mutex_lock(&stateMutex);
    
    // 웹에서 받은 값으로 로봇 제어
    if (isWalking) {
      mGaitManager->setXAmplitude(xAmplitude);
      mGaitManager->setYAmplitude(yAmplitude);
      mGaitManager->setAAmplitude(aAmplitude);
      
      if (!mGaitManager->isRunning()) {
        mGaitManager->start();
      }
    } else {
      if (mGaitManager->isRunning()) {
        mGaitManager->stop();
      }
      mGaitManager->setXAmplitude(0.0);
      mGaitManager->setYAmplitude(0.0);
      mGaitManager->setAAmplitude(0.0);
    }
    
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
