// 기존 walk.cpp 구조를 유지하면서 DARwIn-OP 기능 추가
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>

// DARwIn-OP 관련 헤더 (실제 로봇용)
#ifdef REAL_ROBOT
#include "LinuxDARwIn.h"
#include "Action.h"
#include "Walking.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "MotionStatus.h"
#include "JointData.h"
#include "MX28.h"
#endif

// 로봇 상태 구조체
struct RobotState {
    int gain;
    bool isWalking;
    double xAmplitude;
    double yAmplitude;
    double aAmplitude;
    bool isInitialized;
    
    RobotState() : gain(128), isWalking(false), xAmplitude(0.0), 
                   yAmplitude(0.0), aAmplitude(0.0), isInitialized(false) {}
};

RobotState robotState;
pthread_mutex_t stateMutex = PTHREAD_MUTEX_INITIALIZER;

#ifdef REAL_ROBOT
// 실제 로봇 제어용 전역 변수
Robot::LinuxCM730* linux_cm730 = nullptr;
Robot::CM730* cm730 = nullptr;
Robot::LinuxMotionTimer* motion_timer = nullptr;
#endif

// HTML 파일 로드 (기존과 동일)
char* load_html() {
    FILE* f = fopen("walk.html", "r");
    if (!f) {
        // 기존 walk.html 구조 + 추가 기능
        static char html[] = 
            "<!DOCTYPE html>\n"
            "<html>\n"
            "<head>\n"
            "    <title>Robot Control</title>\n"
            "    <style>\n"
            "        body { font-family: Arial; margin: 20px; }\n"
            "        button { padding: 10px 20px; margin: 5px; font-size: 16px; }\n"
            "        .gain-btn { background-color: #28a745; color: white; border: none; }\n"
            "        .walk-btn { background-color: #007bff; color: white; border: none; }\n"
            "        .status { margin: 20px 0; padding: 10px; background-color: #f8f9fa; }\n"
            "    </style>\n"
            "</head>\n"
            "<body>\n"
            "    <h1>Robot Control</h1>\n"
            "    \n"
            "    <!-- 기존 gain 제어 (원래 walk.html과 동일) -->\n"
            "    <button class=\"gain-btn\" onclick=\"fetch('/?gain_plus')\">Gain +</button>\n"
            "    <button class=\"gain-btn\" onclick=\"fetch('/?gain_minus')\">Gain -</button>\n"
            "    \n"
            "    <br><br>\n"
            "    \n"
            "    <!-- 추가 로봇 제어 기능 -->\n"
            "    <h2>Walking Control</h2>\n"
            "    <button class=\"walk-btn\" onclick=\"fetch('/?walk_start')\">Start Walking</button>\n"
            "    <button class=\"walk-btn\" onclick=\"fetch('/?walk_stop')\">Stop Walking</button>\n"
            "    \n"
            "    <br><br>\n"
            "    \n"
            "    <h2>Movement</h2>\n"
            "    <button onclick=\"fetch('/?move_forward')\">Forward</button>\n"
            "    <button onclick=\"fetch('/?move_backward')\">Backward</button>\n"
            "    <br>\n"
            "    <button onclick=\"fetch('/?turn_left')\">Turn Left</button>\n"
            "    <button onclick=\"fetch('/?turn_right')\">Turn Right</button>\n"
            "    \n"
            "    <div class=\"status\">\n"
            "        <p>Current Gain: <span id=\"gain\">128</span></p>\n"
            "        <p>Walking: <span id=\"walking\">Stopped</span></p>\n"
            "    </div>\n"
            "    \n"
            "    <script>\n"
            "        // 상태 업데이트\n"
            "        setInterval(function() {\n"
            "            fetch('/?get_status')\n"
            "                .then(response => response.text())\n"
            "                .then(data => {\n"
            "                    try {\n"
            "                        const status = JSON.parse(data);\n"
            "                        document.getElementById('gain').textContent = status.gain;\n"
            "                        document.getElementById('walking').textContent = status.walking ? 'Walking' : 'Stopped';\n"
            "                    } catch(e) {\n"
            "                        document.getElementById('gain').textContent = data;\n"
            "                    }\n"
            "                });\n"
            "        }, 1000);\n"
            "    </script>\n"
            "</body>\n"
            "</html>";
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

// 로봇 초기화 함수
bool initializeRobot() {
#ifdef REAL_ROBOT
    printf("Initializing DARwIn-OP robot...\n");
    
    // CM730 초기화
    linux_cm730 = new Robot::LinuxCM730("/dev/ttyUSB0");
    cm730 = new Robot::CM730(linux_cm730);
    
    if (cm730->Connect() == false) {
        printf("Failed to connect to CM730\n");
        return false;
    }
    
    // MotionManager 초기화
    if (Robot::MotionManager::GetInstance()->Initialize(cm730) == false) {
        printf("Failed to initialize MotionManager\n");
        return false;
    }
    
    // 모션 타이머 시작
    motion_timer = new Robot::LinuxMotionTimer(Robot::MotionManager::GetInstance());
    motion_timer->Start();
    
    // Walking 모듈 추가
    Robot::MotionManager::GetInstance()->AddModule((Robot::MotionModule*)Robot::Walking::GetInstance());
    Robot::MotionManager::GetInstance()->AddModule((Robot::MotionModule*)Robot::Action::GetInstance());
    
    // LED를 초록색으로 설정 (초기화 완료)
    cm730->WriteWord(Robot::CM730::ID_CM, Robot::CM730::P_LED_HEAD_L, 1984, 0);
    
    printf("Robot initialized successfully!\n");
    return true;
#else
    printf("Robot simulation mode - no real robot connected\n");
    return true;
#endif
}

// 로봇 제어 함수
void controlRobot() {
#ifdef REAL_ROBOT
    if (!robotState.isInitialized) return;
    
    pthread_mutex_lock(&stateMutex);
    
    if (robotState.isWalking) {
        // 걷기 시작
        if (!Robot::Walking::GetInstance()->IsRunning()) {
            Robot::Walking::GetInstance()->Start();
        }
        
        // 걷기 파라미터 설정
        Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE = robotState.xAmplitude * 20.0;
        Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE = robotState.yAmplitude * 20.0;
        Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE = robotState.aAmplitude * 30.0;
    } else {
        // 걷기 중지
        if (Robot::Walking::GetInstance()->IsRunning()) {
            Robot::Walking::GetInstance()->Stop();
            // 걷기 파라미터 리셋
            Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
            Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
            Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
        }
    }
    
    pthread_mutex_unlock(&stateMutex);
#endif
}

// 명령어 처리 (기존 방식 유지)
void processCommand(const char* url, char* response) {
    pthread_mutex_lock(&stateMutex);
    
    printf("Processing command: %s\n", url);
    
    if (strstr(url, "gain_plus")) {
        robotState.gain += 10;
        if (robotState.gain > 255) robotState.gain = 255;
        sprintf(response, "%d", robotState.gain);
        printf("Gain increased to: %d\n", robotState.gain);
    }
    else if (strstr(url, "gain_minus")) {
        robotState.gain -= 10;
        if (robotState.gain < 0) robotState.gain = 0;
        sprintf(response, "%d", robotState.gain);
        printf("Gain decreased to: %d\n", robotState.gain);
    }
    else if (strstr(url, "walk_start")) {
        robotState.isWalking = true;
        strcpy(response, "Walking started");
        printf("Walking started\n");
    }
    else if (strstr(url, "walk_stop")) {
        robotState.isWalking = false;
        robotState.xAmplitude = 0.0;
        robotState.yAmplitude = 0.0;
        robotState.aAmplitude = 0.0;
        strcpy(response, "Walking stopped");
        printf("Walking stopped\n");
    }
    else if (strstr(url, "move_forward")) {
        robotState.xAmplitude = 1.0;
        strcpy(response, "Moving forward");
        printf("Moving forward\n");
    }
    else if (strstr(url, "move_backward")) {
        robotState.xAmplitude = -1.0;
        strcpy(response, "Moving backward");
        printf("Moving backward\n");
    }
    else if (strstr(url, "turn_left")) {
        robotState.aAmplitude = 0.5;
        strcpy(response, "Turning left");
        printf("Turning left\n");
    }
    else if (strstr(url, "turn_right")) {
        robotState.aAmplitude = -0.5;
        strcpy(response, "Turning right");
        printf("Turning right\n");
    }
    else if (strstr(url, "get_status")) {
        // JSON 형태로 상태 반환
        sprintf(response, "{\"gain\":%d,\"walking\":%s}", 
                robotState.gain, 
                robotState.isWalking ? "true" : "false");
    }
    else {
        strcpy(response, "Unknown command");
        printf("Unknown command: %s\n", url);
    }
    
    pthread_mutex_unlock(&stateMutex);
}

// 웹 서버 (기존 구조 유지)
void* server(void* arg) {
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
        int bytes_read = read(client, buf, sizeof(buf) - 1);
        buf[bytes_read] = '\0';
        
        char response[2048];
        
        // GET 요청 파싱 (기존 방식 유지)
        if (strstr(buf, "GET /?") && !strstr(buf, "GET / ")) {
            // 명령어 처리
            char command_response[256];
            processCommand(buf, command_response);
            
            sprintf(response, 
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "\r\n%s", command_response);
        }
        else {
            // HTML 페이지 반환
            char* html = load_html();
            sprintf(response, 
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                "\r\n%s", html);
            free(html);
        }
        
        write(client, response, strlen(response));
        close(client);
    }
    
    close(s);
    return NULL;
}

// 로봇 제어 루프 스레드
void* robotControlLoop(void* arg) {
    while (1) {
        controlRobot();
        usleep(50000); // 50ms 주기
    }
    return NULL;
}

// 메인 함수 (기존 구조 유지)
int main() {
    printf("Starting Robot Web Control System...\n");
    
    // 로봇 초기화
    robotState.isInitialized = initializeRobot();
    
    // 웹 서버 스레드 시작 (기존과 동일)
    pthread_t serverThread;
    pthread_create(&serverThread, NULL, server, NULL);
    
    // 로봇 제어 스레드 시작
    pthread_t robotThread;
    pthread_create(&robotThread, NULL, robotControlLoop, NULL);
    
    // 메인 루프 (기존과 동일)
    while (1) {
        printf("Current gain: %d, Walking: %s\n", 
               robotState.gain, 
               robotState.isWalking ? "Yes" : "No");
        sleep(3);
    }
    
    return 0;
}
