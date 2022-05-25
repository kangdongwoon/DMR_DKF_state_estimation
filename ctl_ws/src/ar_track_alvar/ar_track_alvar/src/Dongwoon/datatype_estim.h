#ifndef DATATYPE_H
#define DATATYPE_H

typedef struct _packet_data{
    unsigned char header[4];
    unsigned char size, id;
    unsigned char mode, check;
    float xi[4]    ;
    float lambda[4];
    float zeta[10] ;
    float mu[10]   ;
    float y_i;
    float reserved;
}Packet_data_t;

typedef union _packet {
    Packet_data_t stData;
    unsigned char buffer[sizeof(Packet_data_t)];
}Packet_t;

#endif // DATATYPE_H
