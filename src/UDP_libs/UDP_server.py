import socket
import time


class UDP_Server():
    def __init__(self, IP_adress, port):
        self.IP_ADRESS = IP_adress
        self.PORT = port
        
        self.sock = socket.socket(socket.AF_INET, type=socket.SOCK_DGRAM)
        self.sock.bind((self.IP_ADRESS, self.PORT))
        
        self.MESSAGE_SIZE = 1024
        
        
    def get_message(self):
                 
        message, addr = self.sock.recvfrom(self.MESSAGE_SIZE)
        message = message.decode(encoding='utf-8')
        
        return message, addr
    
    
    def send_message(self, message, addr):
        self.sock.sendto(message.encode(encoding='utf-8'), addr)
        
    def end(self):
        self.sock.close()
         


def main_test():
    
    IP_ADRESS = "127.0.0.1"
    PORT = 8890
    udp_server = UDP_Server(IP_ADRESS, PORT)
    
    while True:
        
        message_client, addr = udp_server.get_message()
        print(message_client)
        
        message_server = "Prepared, OK"
        udp_server.send_message(message_server, addr)
        
    udp_server.end()     

if __name__ == "__main__":
    main_test()
            
        
"""
M_SIZE = 1024

# 
host = '127.0.0.1'
port = 8890

locaddr = (host, port)

# ①ソケットを作成する
sock = socket.socket(socket.AF_INET, type=socket.SOCK_DGRAM)
print('create socket')

# ②自ホストで使用するIPアドレスとポート番号を指定
sock.bind(locaddr)

while True:
    try :
        # ③Clientからのmessageの受付開始
        print('Waiting message')
        message, cli_addr = sock.recvfrom(M_SIZE)
        message = message.decode(encoding='utf-8')
        print(f'Received message is [{message}]')

        # Clientが受信待ちになるまで待つため
        time.sleep(1)

        # ④Clientへ受信完了messageを送信
        print('Send response to Client')
        sock.sendto('Success to receive message'.encode(encoding='utf-8'), cli_addr)

    except KeyboardInterrupt:
        print ('\n . . .\n')
        sock.close()
        break
"""
