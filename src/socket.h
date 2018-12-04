#include <fstream>
#include <sstream>
const int MAXRECV = 10240;
#define BACKLOG 10 /* 最大同时连接请求数 */
using namespace std;


struct sockaddr_in my_addr; /* 本机地址信息 */
struct sockaddr_in remote_addr; /* 客户端地址信息 */


int initializeDataEngine(unsigned short port_open){
    int sockfd;
   // socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)//建立 socket
    {
        perror("socket");
        exit(1);
    } 

    my_addr.sin_family=AF_INET;
    my_addr.sin_port=htons(port_open);
    my_addr.sin_addr.s_addr = INADDR_ANY; //表示监听任何地址
    bzero(&(my_addr.sin_zero),8);
    //printf("ok\n");
    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) //将本机地址与建立的套接字号进行绑定
    {
        perror("bind");
        exit(1);
    }
    //printf("ok\n");
    if (listen(sockfd, BACKLOG) == -1) //开始监听
    {
        perror("listen");
        exit(1);
    }
    return sockfd;
}

//向指定客户端发送数据
bool sendData(const int client_fd, const char *ch, const int len)
{
    int status = send(client_fd, ch, len, 0);
    if ( status == -1 )
    {
        return false;
    }
    else
    {
        return true;
    }
}
//接收数据
int recvData(const int client_fd, char buf[], int len)
{
    memset (buf, 0, len);

    int i=0;
    while(i<len){
        char buf_tem[MAXRECV];
        memset (buf_tem, 0, MAXRECV);
        int status = recv(client_fd, buf_tem, MAXRECV, 0);
        memcpy(buf+i, buf_tem, status);
        i = i+status;

        //printf("i:%d\n", i);
        //printf("len:%d\n", len);
        //printf("status:%d\n", status);

        if ( status == -1 )
        {
            printf("status == -1 errno == %s in Socket::recv\n",errno);
            return 0;
        }
        else if( status == 0 )
        {
            //stop(client_fd);
            //return 0;
            break;
        }
        else if(len<=MAXRECV+1)
        {
            break;
        }
        
    }
}
