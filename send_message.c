#include<stdio.h>
#include<string.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr

//The purpose of this script to test whether the communication from the bbb to the cyclone board is working as intended

//This message is a CarControlMessage/CControlMessage that will be sent to the cyclone board from the bbb.
char arr[21] = {'C', 'A', 'R', 'P', 0x00, 0x01, 0x00, 0x0c, '0', 0x0c, 0x00, 0x00, 0x00, '2', 0x00, '2', 0x00, '2', 0x00, '2', '\0'};
 
int main(int argc , char *argv[])
{
    int socket_desc;
    struct sockaddr_in server;
    char *message;
     
    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
         
    server.sin_addr.s_addr = inet_addr("192.168.0.200"); //cyclone board ip
    server.sin_family = AF_INET;
    server.sin_port = htons( 23 );
 
    //Connect to remote server
    if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        puts("connect error");
        return 1;
    }
     
    puts("Connected\n");
     
    //Send some data
    //there is a gotcha in the send statement, if we use strlen on arr, it will only give len of 4 because of the 0s.
 
    if(send(socket_desc, arr , 20, 0) < 0)
    {
        puts("Send failed");
        return 1;
    }
    puts("Data Send\n");
     
    return 0;
}
