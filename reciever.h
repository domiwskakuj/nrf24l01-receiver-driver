#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define BUFFER_SIZE 32

int read_payload(char *buffer, int size){
    if (size != 32)
        return 1;
    size_t el_size = 1;
    size_t max_el = 32;
    FILE* f = fopen("/dev/nrf24", "r");
    fread(buffer, el_size, max_el, f);
    if (ferror(f) !=0 )
    {
	printf("Blad odczytu\n");
    }
    return 0;
}
