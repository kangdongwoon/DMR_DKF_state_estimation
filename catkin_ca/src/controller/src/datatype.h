
#ifndef DATATYPE_H
#define DATATYPE_H

typedef struct _packet_data{
    unsigned char header[4];
    unsigned char size, id;
    unsigned char mode, check;
    float z_x; //posx
    float z_y; //posy
    float d_x; //orix
    float d_y; //oriy
    float yaw; //oriz
    float gainK;
}Packet_data_t;

typedef union _packet {
    Packet_data_t stData;
    unsigned char buffer[sizeof(Packet_data_t)];
}Packet_t;

#endif // DATATYPE_H
