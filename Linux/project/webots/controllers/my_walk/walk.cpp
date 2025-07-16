#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>

int gain = 128;

char* load_html() {
    FILE* f = fopen("index.html", "r");
    if (!f) return NULL;
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
        
        if (strstr(buf, "gain_plus")) {
            gain += 10;
            printf("Gain: %d\n", gain);
        }
        else if (strstr(buf, "gain_minus")) {
            gain -= 10;
            printf("Gain: %d\n", gain);
        }
        
        char* html = load_html();
        if (html) {
            char response[2048];
            sprintf(response, "HTTP/1.1 200 OK\r\n\r\n%s", html);
            write(client, response, strlen(response));
            free(html);
        } else {
            char fallback[] = 
                "HTTP/1.1 200 OK\r\n\r\n"
                "<html><body>"
                "<h1>Robot Control</h1>"
                "<button onclick=\"fetch('/?gain_plus')\">Gain +</button>"
                "<button onclick=\"fetch('/?gain_minus')\">Gain -</button>"
                "</body></html>";
            write(client, fallback, strlen(fallback));
        }
        
        close(client);
    }
    return NULL;
}

int main() {
    pthread_t t;
    pthread_create(&t, NULL, server, NULL);
    
    while (1) {
        printf("Current gain: %d\n", gain);
        sleep(3);
    }
}
