#include <mega328p.h>
#include <spi.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <RC522CVEBRS.h>


void main(void)
{
int i;
int status;
int count = 0;
unsigned char sector         = 1;
unsigned char blockAddr      = 4;
unsigned char dataBlock[]    = {
    0xFF, 0x12, 0x03, 0x04, //  1,  2,   3,  4,
    0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
    0x09, 0x0a, 0xff, 0x0b, //  9, 10, 255, 11,
    0x0c, 0x0d, 0x0e, 0x0f  // 12, 13, 14, 15
};            
unsigned char trailerBlock   = 7;
unsigned char buffer[18];
unsigned char size = sizeof(buffer);
MIFARE_Key key;


DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (0<<DDB4) | (1<<DDB3) | (1<<DDB2) | (0<<DDB1) | (0<<DDB0);
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// SPI initialization
// SPI Type: Master
// SPI Clock Rate: 2*125.000 kHz
// SPI Clock Phase: Cycle Start
// SPI Clock Polarity: Low
// SPI Data Order: MSB First
SPCR=(0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1<<SPR1) | (1<<SPR0);
SPSR=(1<<SPI2X);

UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x67;

printf("by Mr.Null\n");
pcd_initialization();
for(i=0;i<6;i++)
{
key.keyByte[i] = 0xFF;
}   

while (1)
          {
                if (picc_is_new_card_present())    
                {              
                picc_read_card_serial();
                   
                 printf("Card UID:\n");
                 dump_byte_array(uid.uidByte,uid.size);
                 printf("\n");
                 printf("PICC type:\n");           
                 piccType = picc_get_type(uid.sak);
                 picc_get_type_name(piccType);

                // Check for compatibility
                if (piccType != PICC_TYPE_MIFARE_MINI &&  piccType != PICC_TYPE_MIFARE_1K &&  piccType != PICC_TYPE_MIFARE_4K) {
                    printf("This sample only works with MIFARE Classic cards.\n");
                    return;
                }          
                
                //StatusCode status;
                
                // Authenticate using key A
                printf("Authenticating using key A...\n");
                status = (status_code) pcd_authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &uid);
                if (status != STATUS_OK) {
                       printf("PCD_Authenticate() failed:\n");
                    return;
                }

                // Show the whole sector as it currently is
                printf("Current data in sector:\n");
                picc_dump_mifare_classic_sector_to_serial(&uid, &key, sector);   

                // Read data from the block
                printf("Reading data from block "); printf("%d\n",blockAddr);
                status = (status_code) mifare_read(blockAddr, buffer,&size );     
                
                if (status != STATUS_OK) 
                {
                    printf("MIFARE_Read() failed:\n");
                }          
                
                printf("Data in block ");  printf("%d\n",blockAddr); printf(":\n");
                dump_byte_array(buffer, 16); printf("\n");

                // Authenticate using key B
                printf("Authenticating again using key B...\n");
                status = (status_code) pcd_authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, &uid);
                 if (status != STATUS_OK) {
                       printf("PCD_Authenticate() failed:\n");
                    return;
                }
                       //////////////////////////UPG////////////////////////
                // Write data to the block
                printf("Writing data into block ");  printf("%d\n",blockAddr);
                dump_byte_array(dataBlock, 16);
                status = (status_code)mifare_write(blockAddr, dataBlock, 16);
                if (status != STATUS_OK) 
                {
                    printf("MIFARE_Write() failed:\n");
                } 

                // Read data from the block (again, should now be what we have written)
                printf("\nReading data from block ");
                status = (status_code) mifare_read(blockAddr, buffer, &size);
                if (status != STATUS_OK) 
                {
                    printf("MIFARE_Read() failed:\n");
                } 
                printf("Data in block "); printf("%d\n",blockAddr);
                dump_byte_array(buffer, 16); 

                // Check that data in block is what we have written
                // by counting the number of bytes that are equal
                printf("\nChecking result...\n");
                for ( i = 0; i < 16; i++) {
                    // Compare buffer (= what we've read) with dataBlock (= what we've written)
                    if (buffer[i] == dataBlock[i])
                        count++;
                }
                if (count == 16) {
                    printf("Success :-)\n");
                } 
                else 
                {
                    printf("Fail\n");
                }

                // Dump the sector data
                printf("Current data in sector:\n");
                picc_dump_mifare_classic_sector_to_serial(&uid, &key, sector);

                // Halt PICC
                picc_halt_a();
                // Stop encryption on PCD
                pcd_stop_cryptol();
               //
              } 
          }
}
                                                                                        