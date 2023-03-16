/*

* Assembler1.S

*

* Created: 9/5/2022 4:08:43 PM

*  Author: ISRAEL MACIAS

*   This program sends a byte in the NZR format starting with the MSB using the PORTD.PIND0

*/

 #define __SFR_OFFSET 0          // Use 0 for the I/O register offset

#include <avr/io.h> 

 

.global  RGB_BYTE

 

 

 

.section .text                  ; DefineS a code section

 

RGB_BYTE:

      CLI                        ;GLOBAL INTERRUPT DISABLE

      MOV R18,R24

      ANDI R18,0X80

 

       CLR R31      

  L0:  CPI R31,8

       BRLO L1

       RET

  

  L1:  CPI R18,0X80

       BRLO RGB_0

 

RGB_1:   //RGB 1 CODE

//T1H

     IN R30,PORTD      ;SET PORTD.0 

     ORI R30,1

     OUT PORTD, R30

         

     NOP

     NOP

     NOP

     NOP

     NOP

     INC R31

     LSL R24

     MOV R18,R24

     ANDI R18,0X80

 

     IN R30,PORTD     ;CLEAR PORTD.0 

     ANDI R30,0XFE

     OUT PORTD, R30

 

     CPI R31,8

     BREQ END

     CPI R18,0x80

     BREQ RGB_1


 

   RGB_0:   //RGB 1 CODE   //T1H

 

     IN R30,PORTD      ;SET PORTD.0 

     ORI R30,1

     OUT PORTD, R30

         

     NOP

     NOP

     NOP

 

     IN R30,PORTD     ;CLEAR PORTD.0 

     ANDI R30,0XFE

     OUT PORTD, R30


     INC R31

     CPI R31,8

     BREQ END

     LSL R24

     MOV R18,R24

     ANDI R18,0x80

     CPI R18,0x80

     BREQ RGB_1

     JMP RGB_0

 

END: SEI         ;GLOBAL INTERRUPT ENABLE

     RET

 

//       LDI R31,0        ;LOOP

//L0:  INC R31

//     CPI R31,1

//     BRLO L0