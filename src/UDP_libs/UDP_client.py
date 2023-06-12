import socket


class UDP_Client():
    def __init__(self, server_IP_adress, server_port):
        self.SERVER_IP_ADRESS = server_IP_adress
        self.SERVER_PORT = server_port
        
        self.sock = socket.socket(socket.AF_INET, type=socket.SOCK_DGRAM)
        
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
    
    SERVER_IP_ADRESS = "127.0.0.1"
    SERVER_PORT = 8890
    udp_client = UDP_Client(SERVER_IP_ADRESS, SERVER_PORT)
    
    server_addr = (SERVER_IP_ADRESS, SERVER_PORT)
    
    while True:
        
        message_server = "OK, I will start."
        udp_client.send_message(message_server, server_addr)
        
        message_server, server_addr = udp_client.get_message()
        print(message_server)
        
    udp_client.end()     

if __name__ == "__main__":
    main_test()
            


"""
M_SIZE = 1024

# Serverのアドレスを用意。Serverのアドレスは確認しておく必要がある。
serv_address = ('127.0.0.1', 8890)

# ①ソケットを作成する
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    try:
        # ②messageを送信する
        print('Input any messages, Type [end] to exit')
        message = input()
        if message != 'end':
            send_len = sock.sendto(message.encode('utf-8'), serv_address)
            # ※sendtoメソッドはkeyword arguments(address=serv_addressのような形式)を受け付けないので注意

            # ③Serverからのmessageを受付開始
            print('Waiting response from Server')
            rx_meesage, addr = sock.recvfrom(M_SIZE)
            print(f"[Server]: {rx_meesage.decode(encoding='utf-8')}")

        else:
            print('closing socket')
            sock.close()
            print('done')
            break

    except KeyboardInterrupt:
        print('closing socket')
        sock.close()
        print('done')
        break
"""