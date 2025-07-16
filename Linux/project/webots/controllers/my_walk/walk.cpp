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
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

// 로봇 제어 상태 변수들
struct RobotState {
    bool isWalking;
    double xAmplitude;
    double yAmplitude;
    double aAmplitude;
    int gain;
    bool balanceEnable;
    
    RobotState() : isWalking(false), xAmplitude(0.0), yAmplitude(0.0), 
                   aAmplitude(0.0), gain(128), balanceEnable(true) {}
} robotState;

pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

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

// HTML 파일 로드 함수 (기존 walk.html과 호환)
char* loadHTML() {
    FILE* f = fopen("walk.html", "r");
    if (!f) {
        // 기존 walk.html과 동일한 기본 HTML
        static char defaultHTML[] = 
            "<!DOCTYPE html>\n"
            "<html>\n"
            "<head>\n"
            "    <title>Robot Control</title>\n"
            "</head>\n"
            "<body>\n"
            "    <h1>Robot Control</h1>\n"
            "    <button onclick=\"fetch('/?gain_plus')\">Gain +</button>\n"
            "    <button onclick=\"fetch('/?gain_minus')\">Gain -</button>\n"
            "    <br><br>\n"
            "    <!-- 추가 로봇 제어 버튼들 -->\n"
            "    <button onclick=\"fetch('/?walk_start')\">Start Walking</button>\n"
            "    <button onclick=\"fetch('/?walk_stop')\">Stop Walking</button>\n"
            "    <br><br>\n"
            "    <button onclick=\"fetch('/?move_forward')\">Forward</button>\n"
            "    <button onclick=\"fetch('/?move_backward')\">Backward</button>\n"
            "    <button onclick=\"fetch('/?turn_left')\">Turn Left</button>\n"
            "    <button onclick=\"fetch('/?turn_right')\">Turn Right</button>\n"
            "</body>\n"
            "</html>";
        return strdup(defaultHTML);
    }
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);
    char* html = (char*)malloc(size + 1);
    fread(html, 1, size, f);
    html[size] = 0;
    fclose(f);
    return html;
}

// 명령어 처리 함수 (기존 gain_plus/gain_minus 명령어 호환)
void processCommand(const char* command, char* response) {
    pthread_mutex_lock(&stateMutex);
    
    // 기존 gain 명령어들 (원래 walk.cpp와 호환)
    if (strstr(command, "gain_plus")) {
        robotState.gain += 10;
        if (robotState.gain > 255) robotState.gain = 255;
        sprintf(response, "%d", robotState.gain);
        printf("Gain increased to: %d\n", robotState.gain);
    }
    else if (strstr(command, "gain_minus")) {
        robotState.gain -= 10;
        if (robotState.gain < 0) robotState.gain = 0;
        sprintf(response, "%d", robotState.gain);
        printf("Gain decreased to: %d\n", robotState.gain);
    }
    // 새로운 로봇 제어 명령어들
    else if (strstr(command, "walk_start")) {
        robotState.isWalking = true;
        strcpy(response, "Walking started");
        printf("Walking started\n");
    }
    else if (strstr(command, "walk_stop")) {
        robotState.isWalking = false;
        robotState.xAmplitude = 0.0;
        robotState.yAmplitude = 0.0;
        robotState.aAmplitude = 0.0;
        strcpy(response, "Walking stopped");
        printf("Walking stopped\n");
    }
    else if (strstr(command, "move_forward")) {
        robotState.xAmplitude = 1.0;
        strcpy(response, "Moving forward");
        printf("Moving forward\n");
    }
    else if (strstr(command, "move_backward")) {
        robotState.xAmplitude = -1.0;
        strcpy(response, "Moving backward");
        printf("Moving backward\n");
    }
    else if (strstr(command, "turn_left")) {
        robotState.aAmplitude = 0.5;
        strcpy(response, "Turning left");
        printf("Turning left\n");
    }
    else if (strstr(command, "turn_right")) {
        robotState.aAmplitude = -0.5;
        strcpy(response, "Turning right");
        printf("Turning right\n");
    }
    else if (strstr(command, "get_status")) {
        sprintf(response, "%d", robotState.gain);
    }
    else {
        strcpy(response, "Unknown command");
        printf("Unknown command: %s\n", command);
    }
    
    pthread_mutex_unlock(&stateMutex);
}

// 웹 서버 스레드
void* webServer(void* walkInstance) {
    Walk* walk = (Walk*)walkInstance;
    
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {AF_INET, htons(8080), {INADDR_ANY}};
    int opt = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(server_socket, (struct sockaddr*)&addr, sizeof(addr));
    listen(server_socket, 1);
    
    printf("Web server started at http://localhost:8080\n");
    
    while (1) {
        int client = accept(server_socket, NULL, NULL);
        char buf[1024];
        int bytes_read = read(client, buf, sizeof(buf) - 1);
        buf[bytes_read] = '\0';
        
        char response[2048];
        
        if (strstr(buf, "GET /?")) {
            // URL에서 명령어 추출 (기존 방식 유지)
            char* command_start = NULL;
            
            // 기존 gain_plus/gain_minus 방식 지원
            if (strstr(buf, "gain_plus")) {
                command_start = "gain_plus";
            } else if (strstr(buf, "gain_minus")) {
                command_start = "gain_minus";
            } 
            // 새로운 명령어들
            else if (strstr(buf, "walk_start")) {
                command_start = "walk_start";
            } else if (strstr(buf, "walk_stop")) {
                command_start = "walk_stop";
            } else if (strstr(buf, "move_forward")) {
                command_start = "move_forward";
            } else if (strstr(buf, "move_backward")) {
                command_start = "move_backward";
            } else if (strstr(buf, "turn_left")) {
                command_start = "turn_left";
            } else if (strstr(buf, "turn_right")) {
                command_start = "turn_right";
            }
            
            if (command_start) {
                char command_response[256];
                processCommand(command_start, command_response);
                
                sprintf(response, 
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "\r\n%s", command_response);
            } else {
                sprintf(response, 
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "\r\nUnknown command");
            }
        }
        else {
            // HTML 페이지 반환
            char* html = loadHTML();
            sprintf(response, 
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                "\r\n%s", html);
            free(html);
        }
        
        write(client, response, strlen(response));
        close(client);
    }
    
    close(server_socket);
    return NULL;
}

// 모션 제어 함수
void Walk::executeMotion(const char* command) {
    if (strstr(command, "motion_hello")) {
        mMotionManager->playPage(4); // Hello motion
        wait(2000);
    }
    else if (strstr(command, "motion_init")) {
        mMotionManager->playPage(9); // Init position
        wait(2000);
    }
}

// 메인 실행 함수
void Walk::run() {
    cout << "-------DARwIn-OP Web Control-------" << endl;
    cout << "Starting web server on port 8080..." << endl;
    cout << "Open http://localhost:8080 to control the robot" << endl;

    // 웹 서버 스레드 시작
    pthread_t webThread;
    pthread_create(&webThread, NULL, webServer, this);
    pthread_detach(webThread);

    // 첫 번째 스텝
    myStep();

    // 초기 위치로 이동
    mMotionManager->playPage(9); // init position
    wait(200);
    
    // 메인 루프
    int key = 0;
    
    while (true) {
        checkIfFallen();
        
        pthread_mutex_lock(&stateMutex);
        
        // 걷기 상태 업데이트
        if (robotState.isWalking) {
            if (!mGaitManager) {
                // 걷기 시작
                mGaitManager = new DARwInOPGaitManager(this, "config.ini");
                mGaitManager->start();
                wait(200);
            }
            
            // 걷기 파라미터 설정
            mGaitManager->setXAmplitude(robotState.xAmplitude);
            mGaitManager->setYAmplitude(robotState.yAmplitude);
            mGaitManager->setAAmplitude(robotState.aAmplitude);
            mGaitManager->setBalanceEnable(robotState.balanceEnable);
        } else {
            if (mGaitManager) {
                // 걷기 중지
                mGaitManager->stop();
                delete mGaitManager;
                mGaitManager = nullptr;
                wait(200);
            }
        }
        
        pthread_mutex_unlock(&stateMutex);
        
        // 키보드 입력 처리 (기존 기능 유지)
        while((key = keyboardGetKey()) != 0) {
            switch(key) {
                case ' ': // Space bar 
                    pthread_mutex_lock(&stateMutex);
                    robotState.isWalking = !robotState.isWalking;
                    if (!robotState.isWalking) {
                        robotState.xAmplitude = 0.0;
                        robotState.yAmplitude = 0.0;
                        robotState.aAmplitude = 0.0;
                    }
                    pthread_mutex_unlock(&stateMutex);
                    break;
                case KEYBOARD_UP:
                    pthread_mutex_lock(&stateMutex);
                    robotState.xAmplitude = 1.0;
                    pthread_mutex_unlock(&stateMutex);
                    break;
                case KEYBOARD_DOWN:
                    pthread_mutex_lock(&stateMutex);
                    robotState.xAmplitude = -1.0;
                    pthread_mutex_unlock(&stateMutex);
                    break;
                case KEYBOARD_RIGHT:
                    pthread_mutex_lock(&stateMutex);
                    robotState.aAmplitude = -0.5;
                    pthread_mutex_unlock(&stateMutex);
                    break;
                case KEYBOARD_LEFT:
                    pthread_mutex_lock(&stateMutex);
                    robotState.aAmplitude = 0.5;
                    pthread_mutex_unlock(&stateMutex);
                    break;
            }
        }

        if (mGaitManager) {
            mGaitManager->step(mTimeStep);
        }
        
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
    
    // 로봇이 앞으로 넘어짐
    if (fup > acc_step) {
        pthread_mutex_lock(&stateMutex);
        robotState.isWalking = false;
        pthread_mutex_unlock(&stateMutex);
        
        mMotionManager->playPage(10); // f_up
        mMotionManager->playPage(9);  // init position    
        fup = 0;
    }
    // 로봇이 뒤로 넘어짐
    else if (fdown > acc_step) {
        pthread_mutex_lock(&stateMutex);
        robotState.isWalking = false;
        pthread_mutex_unlock(&stateMutex);
        
        mMotionManager->playPage(11); // b_up
        mMotionManager->playPage(9);  // init position
        fdown = 0;
    }
}
