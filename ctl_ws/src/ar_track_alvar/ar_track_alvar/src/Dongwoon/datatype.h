#ifndef DATATYPE_H
#define DATATYPE_H

typedef struct _packet_data{
    unsigned char header[4];
    unsigned char size, id;
    unsigned char mode, check;
    float z_x; // xi     = z_x
    float z_y; // lambda = z_y
    float d_x; // zeta   = d_x
    float d_y; // mu     = d_y
    float yaw;
    float gainK;
}Packet_data_t;

typedef union _packet {
    Packet_data_t stData;
    unsigned char buffer[sizeof(Packet_data_t)];
}Packet_t;

#endif // DATATYPE_H
