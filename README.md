# xBTSAT - An TTL to Bluetooth transceiver, with AT commands and OTA 

How to use OTA:
    IP: 192.168.1.1
    Username: admin
    Password: admin


VERY VERY VERY IMPORTANT, Always clean the project and full build before trusting the build bin file, for some reason its not really updated (esp idf 4.4.4)


Como funciona:
o Conversor xBTSAT cria uma conexão serial bluetooth com o nome xBTSAT_0000000, onde os 0000000 são um codigo em hexadecimal unico de cada chip.
A conexão serve como uma ponte entre o bluetooth e a saida serial
O Módulo suporta configuração via comandos AT:
    "AT+? - Exibe ajuda\r\n"
    "AT+HELP - Exibe ajuda\r\n"
    "AT+BAUD - Responde a velocidade atual\r\n"
    "AT+BAUD=9600 - Define a nova velocidade\r\n"
    "AT+VER - Responde a versao de firmware\r\n"
    "AT+RESET - Restaura todas as configuracoes\r\n"
    "AT+OTA - Ativa o Wifi para atualizacao OTA\r\n";

Como usar o OTA
1 - Conecte no Bluetooth e envie AT+OTA, voce deve receber uma mensagem confirmando que o OTA foi ativado.
2 - Acesse em uma pagina o IP 192.168.1.1 ou digite simova.local
3 - Usuario simova, senha simova
4 - Escolha o novo firmware
5 - Clique no botão UPLOAD
6 - Conecte no bluetooth e envie AT+VER para conferir se o firmware foi atualizado