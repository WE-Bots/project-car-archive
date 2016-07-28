;==========================================================================
;  PIC33FJ06GS101A Standard Assembly Include File
; 
;  (c) Copyright 2014 Microchip Technology, All rights reserved
;  Part support version v1.24.A(02-Jan-2015)
; 
 ; This software is developed by Microchip Technology Inc. and its
 ; subsidiaries ("Microchip").
 ; 
 ; Redistribution and use in source and binary forms, with or without
 ; modification, are permitted provided that the following conditions are 
 ; met:
 ; 
 ; 1.      Redistributions of source code must retain the above copyright
 ;         notice, this list of conditions and the following disclaimer.
 ; 2.      Redistributions in binary form must reproduce the above 
 ;         copyright notice, this list of conditions and the following 
 ;         disclaimer in the documentation and/or other materials provided 
 ;         with the distribution.
 ; 3.      Microchip's name may not be used to endorse or promote products
 ;         derived from this software without specific prior written 
 ;         permission.
 ; 
 ; THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS OR IMPLIED
 ; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ; MERCHANTABILITY AND FITNESS FOR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 ; SHALL MICROCHIP BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 ; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT LIMITED TO
 ; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA OR PROFITS;
 ; OR BUSINESS INTERRUPTION) HOWSOEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 ; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 ; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ; ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ; 
 ;

.NOLIST

;==========================================================================
;  This header file defines configurations, registers, and other useful
;  bits of information for the PIC33FJ06GS101A microcontroller.  These names
;  are taken to match the data sheets as closely as possible.
;
;  Note that the processor must be selected before this file is included.
;  The processor may be selected the following ways:
;
;       1. Command line switch:
;               C:\> pic30-as file.s -p33FJ06GS101A
;       2. Placing a ".equ" directive before the ".include":
;               .equ __33FJ06GS101A, 1
;               .include "p33FJ06GS101A.inc"
;       3. Setting the processor in the MPLAB IDE Project Dialog
;==========================================================================

;==========================================================================
;
;       Verify Processor
;
;==========================================================================

.IFNDEF __33FJ06GS101A
   .ERROR "Include file does not match processor setting."
.ENDIF


;==========================================================================
;
;       Register Definitions
;
;==========================================================================

;----- Register Equates (Low Byte, High Byte) -----------------------------

        .extern WREG0
        .equiv  WREG0L, _WREG0
        .equiv  WREG0H, _WREG0+1

        .extern WREG1
        .equiv  WREG1L, _WREG1
        .equiv  WREG1H, _WREG1+1

        .extern WREG2
        .equiv  WREG2L, _WREG2
        .equiv  WREG2H, _WREG2+1

        .extern WREG3
        .equiv  WREG3L, _WREG3
        .equiv  WREG3H, _WREG3+1

        .extern WREG4
        .equiv  WREG4L, _WREG4
        .equiv  WREG4H, _WREG4+1

        .extern WREG5
        .equiv  WREG5L, _WREG5
        .equiv  WREG5H, _WREG5+1

        .extern WREG6
        .equiv  WREG6L, _WREG6
        .equiv  WREG6H, _WREG6+1

        .extern WREG7
        .equiv  WREG7L, _WREG7
        .equiv  WREG7H, _WREG7+1

        .extern WREG8
        .equiv  WREG8L, _WREG8
        .equiv  WREG8H, _WREG8+1

        .extern WREG9
        .equiv  WREG9L, _WREG9
        .equiv  WREG9H, _WREG9+1

        .extern WREG10
        .equiv  WREG10L, _WREG10
        .equiv  WREG10H, _WREG10+1

        .extern WREG11
        .equiv  WREG11L, _WREG11
        .equiv  WREG11H, _WREG11+1

        .extern WREG12
        .equiv  WREG12L, _WREG12
        .equiv  WREG12H, _WREG12+1

        .extern WREG13
        .equiv  WREG13L, _WREG13
        .equiv  WREG13H, _WREG13+1

        .extern WREG14
        .equiv  WREG14L, _WREG14
        .equiv  WREG14H, _WREG14+1

        .extern WREG15
        .equiv  WREG15L, _WREG15
        .equiv  WREG15H, _WREG15+1

        .extern SPLIM
        .equiv  SPLIML, _SPLIM
        .equiv  SPLIMH, _SPLIM+1

        .extern ACCAL
        .equiv  ACCALL, _ACCAL
        .equiv  ACCALH, _ACCAL+1

        .extern ACCAH
        .equiv  ACCAHL, _ACCAH
        .equiv  ACCAHH, _ACCAH+1

        .extern ACCAU
        .equiv  ACCAUL, _ACCAU
        .equiv  ACCAUH, _ACCAU+1

        .extern ACCBL
        .equiv  ACCBLL, _ACCBL
        .equiv  ACCBLH, _ACCBL+1

        .extern ACCBH
        .equiv  ACCBHL, _ACCBH
        .equiv  ACCBHH, _ACCBH+1

        .extern ACCBU
        .equiv  ACCBUL, _ACCBU
        .equiv  ACCBUH, _ACCBU+1

        .extern PCL
        .equiv  PCLL, _PCL
        .equiv  PCLH, _PCL+1

        .extern PCH
        .equiv  PCHL, _PCH
        .equiv  PCHH, _PCH+1

        .extern TBLPAG
        .equiv  TBLPAGL, _TBLPAG
        .equiv  TBLPAGH, _TBLPAG+1

        .extern PSVPAG
        .equiv  PSVPAGL, _PSVPAG
        .equiv  PSVPAGH, _PSVPAG+1

        .extern RCOUNT
        .equiv  RCOUNTL, _RCOUNT
        .equiv  RCOUNTH, _RCOUNT+1

        .extern DCOUNT
        .equiv  DCOUNTL, _DCOUNT
        .equiv  DCOUNTH, _DCOUNT+1

        .extern DOSTARTL
        .equiv  DOSTARTLL, _DOSTARTL
        .equiv  DOSTARTLH, _DOSTARTL+1

        .extern DOSTARTH
        .equiv  DOSTARTHL, _DOSTARTH
        .equiv  DOSTARTHH, _DOSTARTH+1

        .extern DOENDL
        .equiv  DOENDLL, _DOENDL
        .equiv  DOENDLH, _DOENDL+1

        .extern DOENDH
        .equiv  DOENDHL, _DOENDH
        .equiv  DOENDHH, _DOENDH+1

        .extern SR
        .equiv  SRL, _SR
        .equiv  SRH, _SR+1

        .extern CORCON
        .equiv  CORCONL, _CORCON
        .equiv  CORCONH, _CORCON+1

        .extern MODCON
        .equiv  MODCONL, _MODCON
        .equiv  MODCONH, _MODCON+1

        .extern XMODSRT
        .equiv  XMODSRTL, _XMODSRT
        .equiv  XMODSRTH, _XMODSRT+1

        .extern XMODEND
        .equiv  XMODENDL, _XMODEND
        .equiv  XMODENDH, _XMODEND+1

        .extern YMODSRT
        .equiv  YMODSRTL, _YMODSRT
        .equiv  YMODSRTH, _YMODSRT+1

        .extern YMODEND
        .equiv  YMODENDL, _YMODEND
        .equiv  YMODENDH, _YMODEND+1

        .extern XBREV
        .equiv  XBREVL, _XBREV
        .equiv  XBREVH, _XBREV+1

        .extern DISICNT
        .equiv  DISICNTL, _DISICNT
        .equiv  DISICNTH, _DISICNT+1

        .extern CNEN1
        .equiv  CNEN1L, _CNEN1
        .equiv  CNEN1H, _CNEN1+1

        .extern CNPU1
        .equiv  CNPU1L, _CNPU1
        .equiv  CNPU1H, _CNPU1+1

        .extern INTCON1
        .equiv  INTCON1L, _INTCON1
        .equiv  INTCON1H, _INTCON1+1

        .extern INTCON2
        .equiv  INTCON2L, _INTCON2
        .equiv  INTCON2H, _INTCON2+1

        .extern IFS0
        .equiv  IFS0L, _IFS0
        .equiv  IFS0H, _IFS0+1

        .extern IFS1
        .equiv  IFS1L, _IFS1
        .equiv  IFS1H, _IFS1+1

        .extern IFS3
        .equiv  IFS3L, _IFS3
        .equiv  IFS3H, _IFS3+1

        .extern IFS4
        .equiv  IFS4L, _IFS4
        .equiv  IFS4H, _IFS4+1

        .extern IFS5
        .equiv  IFS5L, _IFS5
        .equiv  IFS5H, _IFS5+1

        .extern IFS6
        .equiv  IFS6L, _IFS6
        .equiv  IFS6H, _IFS6+1

        .extern IFS7
        .equiv  IFS7L, _IFS7
        .equiv  IFS7H, _IFS7+1

        .extern IEC0
        .equiv  IEC0L, _IEC0
        .equiv  IEC0H, _IEC0+1

        .extern IEC1
        .equiv  IEC1L, _IEC1
        .equiv  IEC1H, _IEC1+1

        .extern IEC3
        .equiv  IEC3L, _IEC3
        .equiv  IEC3H, _IEC3+1

        .extern IEC4
        .equiv  IEC4L, _IEC4
        .equiv  IEC4H, _IEC4+1

        .extern IEC5
        .equiv  IEC5L, _IEC5
        .equiv  IEC5H, _IEC5+1

        .extern IEC6
        .equiv  IEC6L, _IEC6
        .equiv  IEC6H, _IEC6+1

        .extern IEC7
        .equiv  IEC7L, _IEC7
        .equiv  IEC7H, _IEC7+1

        .extern IPC0
        .equiv  IPC0L, _IPC0
        .equiv  IPC0H, _IPC0+1

        .extern IPC1
        .equiv  IPC1L, _IPC1
        .equiv  IPC1H, _IPC1+1

        .extern IPC2
        .equiv  IPC2L, _IPC2
        .equiv  IPC2H, _IPC2+1

        .extern IPC3
        .equiv  IPC3L, _IPC3
        .equiv  IPC3H, _IPC3+1

        .extern IPC4
        .equiv  IPC4L, _IPC4
        .equiv  IPC4H, _IPC4+1

        .extern IPC5
        .equiv  IPC5L, _IPC5
        .equiv  IPC5H, _IPC5+1

        .extern IPC7
        .equiv  IPC7L, _IPC7
        .equiv  IPC7H, _IPC7+1

        .extern IPC14
        .equiv  IPC14L, _IPC14
        .equiv  IPC14H, _IPC14+1

        .extern IPC16
        .equiv  IPC16L, _IPC16
        .equiv  IPC16H, _IPC16+1

        .extern IPC20
        .equiv  IPC20L, _IPC20
        .equiv  IPC20H, _IPC20+1

        .extern IPC23
        .equiv  IPC23L, _IPC23
        .equiv  IPC23H, _IPC23+1

        .extern IPC24
        .equiv  IPC24L, _IPC24
        .equiv  IPC24H, _IPC24+1

        .extern IPC27
        .equiv  IPC27L, _IPC27
        .equiv  IPC27H, _IPC27+1

        .extern IPC28
        .equiv  IPC28L, _IPC28
        .equiv  IPC28H, _IPC28+1

        .extern INTTREG
        .equiv  INTTREGL, _INTTREG
        .equiv  INTTREGH, _INTTREG+1

        .extern TMR1
        .equiv  TMR1L, _TMR1
        .equiv  TMR1H, _TMR1+1

        .extern PR1
        .equiv  PR1L, _PR1
        .equiv  PR1H, _PR1+1

        .extern T1CON
        .equiv  T1CONL, _T1CON
        .equiv  T1CONH, _T1CON+1

        .extern TMR2
        .equiv  TMR2L, _TMR2
        .equiv  TMR2H, _TMR2+1

        .extern PR2
        .equiv  PR2L, _PR2
        .equiv  PR2H, _PR2+1

        .extern T2CON
        .equiv  T2CONL, _T2CON
        .equiv  T2CONH, _T2CON+1

        .extern OC1RS
        .equiv  OC1RSL, _OC1RS
        .equiv  OC1RSH, _OC1RS+1

        .extern OC1R
        .equiv  OC1RL, _OC1R
        .equiv  OC1RH, _OC1R+1

        .extern OC1CON
        .equiv  OC1CONL, _OC1CON
        .equiv  OC1CONH, _OC1CON+1

        .extern I2C1RCV
        .equiv  I2C1RCVL, _I2C1RCV
        .equiv  I2C1RCVH, _I2C1RCV+1

        .extern I2CRCV
        .equiv  I2CRCVL, _I2CRCV
        .equiv  I2CRCVH, _I2CRCV+1

        .extern I2C1TRN
        .equiv  I2C1TRNL, _I2C1TRN
        .equiv  I2C1TRNH, _I2C1TRN+1

        .extern I2CTRN
        .equiv  I2CTRNL, _I2CTRN
        .equiv  I2CTRNH, _I2CTRN+1

        .extern I2C1BRG
        .equiv  I2C1BRGL, _I2C1BRG
        .equiv  I2C1BRGH, _I2C1BRG+1

        .extern I2C1CON
        .equiv  I2C1CONL, _I2C1CON
        .equiv  I2C1CONH, _I2C1CON+1

        .extern I2CCON
        .equiv  I2CCONL, _I2CCON
        .equiv  I2CCONH, _I2CCON+1

        .extern I2C1STAT
        .equiv  I2C1STATL, _I2C1STAT
        .equiv  I2C1STATH, _I2C1STAT+1

        .extern I2CSTAT
        .equiv  I2CSTATL, _I2CSTAT
        .equiv  I2CSTATH, _I2CSTAT+1

        .extern I2C1ADD
        .equiv  I2C1ADDL, _I2C1ADD
        .equiv  I2C1ADDH, _I2C1ADD+1

        .extern I2CADD
        .equiv  I2CADDL, _I2CADD
        .equiv  I2CADDH, _I2CADD+1

        .extern I2C1MSK
        .equiv  I2C1MSKL, _I2C1MSK
        .equiv  I2C1MSKH, _I2C1MSK+1

        .extern U1MODE
        .equiv  U1MODEL, _U1MODE
        .equiv  U1MODEH, _U1MODE+1

        .extern U1STA
        .equiv  U1STAL, _U1STA
        .equiv  U1STAH, _U1STA+1

        .extern U1TXREG
        .equiv  U1TXREGL, _U1TXREG
        .equiv  U1TXREGH, _U1TXREG+1

        .extern U1RXREG
        .equiv  U1RXREGL, _U1RXREG
        .equiv  U1RXREGH, _U1RXREG+1

        .extern U1BRG
        .equiv  U1BRGL, _U1BRG
        .equiv  U1BRGH, _U1BRG+1

        .extern SPI1STAT
        .equiv  SPI1STATL, _SPI1STAT
        .equiv  SPI1STATH, _SPI1STAT+1

        .extern SPI1CON1
        .equiv  SPI1CON1L, _SPI1CON1
        .equiv  SPI1CON1H, _SPI1CON1+1

        .extern SPI1CON2
        .equiv  SPI1CON2L, _SPI1CON2
        .equiv  SPI1CON2H, _SPI1CON2+1

        .extern SPI1BUF
        .equiv  SPI1BUFL, _SPI1BUF
        .equiv  SPI1BUFH, _SPI1BUF+1

        .extern TRISA
        .equiv  TRISAL, _TRISA
        .equiv  TRISAH, _TRISA+1

        .extern PORTA
        .equiv  PORTAL, _PORTA
        .equiv  PORTAH, _PORTA+1

        .extern LATA
        .equiv  LATAL, _LATA
        .equiv  LATAH, _LATA+1

        .extern ODCA
        .equiv  ODCAL, _ODCA
        .equiv  ODCAH, _ODCA+1

        .extern TRISB
        .equiv  TRISBL, _TRISB
        .equiv  TRISBH, _TRISB+1

        .extern PORTB
        .equiv  PORTBL, _PORTB
        .equiv  PORTBH, _PORTB+1

        .extern LATB
        .equiv  LATBL, _LATB
        .equiv  LATBH, _LATB+1

        .extern ODCB
        .equiv  ODCBL, _ODCB
        .equiv  ODCBH, _ODCB+1

        .extern ADCON
        .equiv  ADCONL, _ADCON
        .equiv  ADCONH, _ADCON+1

        .extern ADPCFG
        .equiv  ADPCFGL, _ADPCFG
        .equiv  ADPCFGH, _ADPCFG+1

        .extern ADSTAT
        .equiv  ADSTATL, _ADSTAT
        .equiv  ADSTATH, _ADSTAT+1

        .extern ADBASE
        .equiv  ADBASEL, _ADBASE
        .equiv  ADBASEH, _ADBASE+1

        .extern ADCPC0
        .equiv  ADCPC0L, _ADCPC0
        .equiv  ADCPC0H, _ADCPC0+1

        .extern ADCPC1
        .equiv  ADCPC1L, _ADCPC1
        .equiv  ADCPC1H, _ADCPC1+1

        .extern ADCPC3
        .equiv  ADCPC3L, _ADCPC3
        .equiv  ADCPC3H, _ADCPC3+1

        .extern ADCBUF0
        .equiv  ADCBUF0L, _ADCBUF0
        .equiv  ADCBUF0H, _ADCBUF0+1

        .extern ADCBUF1
        .equiv  ADCBUF1L, _ADCBUF1
        .equiv  ADCBUF1H, _ADCBUF1+1

        .extern ADCBUF2
        .equiv  ADCBUF2L, _ADCBUF2
        .equiv  ADCBUF2H, _ADCBUF2+1

        .extern ADCBUF3
        .equiv  ADCBUF3L, _ADCBUF3
        .equiv  ADCBUF3H, _ADCBUF3+1

        .extern ADCBUF6
        .equiv  ADCBUF6L, _ADCBUF6
        .equiv  ADCBUF6H, _ADCBUF6+1

        .extern ADCBUF7
        .equiv  ADCBUF7L, _ADCBUF7
        .equiv  ADCBUF7H, _ADCBUF7+1

        .extern ADCBUF13
        .equiv  ADCBUF13L, _ADCBUF13
        .equiv  ADCBUF13H, _ADCBUF13+1

        .extern PTCON
        .equiv  PTCONL, _PTCON
        .equiv  PTCONH, _PTCON+1

        .extern PTCON2
        .equiv  PTCON2L, _PTCON2
        .equiv  PTCON2H, _PTCON2+1

        .extern PTPER
        .equiv  PTPERL, _PTPER
        .equiv  PTPERH, _PTPER+1

        .extern SEVTCMP
        .equiv  SEVTCMPL, _SEVTCMP
        .equiv  SEVTCMPH, _SEVTCMP+1

        .extern MDC
        .equiv  MDCL, _MDC
        .equiv  MDCH, _MDC+1

        .extern CHOP
        .equiv  CHOPL, _CHOP
        .equiv  CHOPH, _CHOP+1

        .extern PWMCON1
        .equiv  PWMCON1L, _PWMCON1
        .equiv  PWMCON1H, _PWMCON1+1

        .extern IOCON1
        .equiv  IOCON1L, _IOCON1
        .equiv  IOCON1H, _IOCON1+1

        .extern FCLCON1
        .equiv  FCLCON1L, _FCLCON1
        .equiv  FCLCON1H, _FCLCON1+1

        .extern PDC1
        .equiv  PDC1L, _PDC1
        .equiv  PDC1H, _PDC1+1

        .extern PHASE1
        .equiv  PHASE1L, _PHASE1
        .equiv  PHASE1H, _PHASE1+1

        .extern DTR1
        .equiv  DTR1L, _DTR1
        .equiv  DTR1H, _DTR1+1

        .extern ALTDTR1
        .equiv  ALTDTR1L, _ALTDTR1
        .equiv  ALTDTR1H, _ALTDTR1+1

        .extern SDC1
        .equiv  SDC1L, _SDC1
        .equiv  SDC1H, _SDC1+1

        .extern SPHASE1
        .equiv  SPHASE1L, _SPHASE1
        .equiv  SPHASE1H, _SPHASE1+1

        .extern TRIG1
        .equiv  TRIG1L, _TRIG1
        .equiv  TRIG1H, _TRIG1+1

        .extern TRGCON1
        .equiv  TRGCON1L, _TRGCON1
        .equiv  TRGCON1H, _TRGCON1+1

        .extern STRIG1
        .equiv  STRIG1L, _STRIG1
        .equiv  STRIG1H, _STRIG1+1

        .extern PWMCAP1
        .equiv  PWMCAP1L, _PWMCAP1
        .equiv  PWMCAP1H, _PWMCAP1+1

        .extern LEBCON1
        .equiv  LEBCON1L, _LEBCON1
        .equiv  LEBCON1H, _LEBCON1+1

        .extern AUXCON1
        .equiv  AUXCON1L, _AUXCON1
        .equiv  AUXCON1H, _AUXCON1+1

        .extern PWMCON4
        .equiv  PWMCON4L, _PWMCON4
        .equiv  PWMCON4H, _PWMCON4+1

        .extern IOCON4
        .equiv  IOCON4L, _IOCON4
        .equiv  IOCON4H, _IOCON4+1

        .extern FCLCON4
        .equiv  FCLCON4L, _FCLCON4
        .equiv  FCLCON4H, _FCLCON4+1

        .extern PDC4
        .equiv  PDC4L, _PDC4
        .equiv  PDC4H, _PDC4+1

        .extern PHASE4
        .equiv  PHASE4L, _PHASE4
        .equiv  PHASE4H, _PHASE4+1

        .extern DTR4
        .equiv  DTR4L, _DTR4
        .equiv  DTR4H, _DTR4+1

        .extern ALTDTR4
        .equiv  ALTDTR4L, _ALTDTR4
        .equiv  ALTDTR4H, _ALTDTR4+1

        .extern SDC4
        .equiv  SDC4L, _SDC4
        .equiv  SDC4H, _SDC4+1

        .extern SPHASE4
        .equiv  SPHASE4L, _SPHASE4
        .equiv  SPHASE4H, _SPHASE4+1

        .extern TRIG4
        .equiv  TRIG4L, _TRIG4
        .equiv  TRIG4H, _TRIG4+1

        .extern TRGCON4
        .equiv  TRGCON4L, _TRGCON4
        .equiv  TRGCON4H, _TRGCON4+1

        .extern STRIG4
        .equiv  STRIG4L, _STRIG4
        .equiv  STRIG4H, _STRIG4+1

        .extern PWMCAP4
        .equiv  PWMCAP4L, _PWMCAP4
        .equiv  PWMCAP4H, _PWMCAP4+1

        .extern LEBCON4
        .equiv  LEBCON4L, _LEBCON4
        .equiv  LEBCON4H, _LEBCON4+1

        .extern AUXCON4
        .equiv  AUXCON4L, _AUXCON4
        .equiv  AUXCON4H, _AUXCON4+1

        .extern RPINR0
        .equiv  RPINR0L, _RPINR0
        .equiv  RPINR0H, _RPINR0+1

        .extern RPINR1
        .equiv  RPINR1L, _RPINR1
        .equiv  RPINR1H, _RPINR1+1

        .extern RPINR2
        .equiv  RPINR2L, _RPINR2
        .equiv  RPINR2H, _RPINR2+1

        .extern RPINR3
        .equiv  RPINR3L, _RPINR3
        .equiv  RPINR3H, _RPINR3+1

        .extern RPINR11
        .equiv  RPINR11L, _RPINR11
        .equiv  RPINR11H, _RPINR11+1

        .extern RPINR18
        .equiv  RPINR18L, _RPINR18
        .equiv  RPINR18H, _RPINR18+1

        .extern RPINR20
        .equiv  RPINR20L, _RPINR20
        .equiv  RPINR20H, _RPINR20+1

        .extern RPINR21
        .equiv  RPINR21L, _RPINR21
        .equiv  RPINR21H, _RPINR21+1

        .extern RPINR29
        .equiv  RPINR29L, _RPINR29
        .equiv  RPINR29H, _RPINR29+1

        .extern RPINR30
        .equiv  RPINR30L, _RPINR30
        .equiv  RPINR30H, _RPINR30+1

        .extern RPINR31
        .equiv  RPINR31L, _RPINR31
        .equiv  RPINR31H, _RPINR31+1

        .extern RPINR32
        .equiv  RPINR32L, _RPINR32
        .equiv  RPINR32H, _RPINR32+1

        .extern RPINR33
        .equiv  RPINR33L, _RPINR33
        .equiv  RPINR33H, _RPINR33+1

        .extern RPINR34
        .equiv  RPINR34L, _RPINR34
        .equiv  RPINR34H, _RPINR34+1

        .extern RPOR0
        .equiv  RPOR0L, _RPOR0
        .equiv  RPOR0H, _RPOR0+1

        .extern RPOR1
        .equiv  RPOR1L, _RPOR1
        .equiv  RPOR1H, _RPOR1+1

        .extern RPOR2
        .equiv  RPOR2L, _RPOR2
        .equiv  RPOR2H, _RPOR2+1

        .extern RPOR3
        .equiv  RPOR3L, _RPOR3
        .equiv  RPOR3H, _RPOR3+1

        .extern RPOR16
        .equiv  RPOR16L, _RPOR16
        .equiv  RPOR16H, _RPOR16+1

        .extern RPOR17
        .equiv  RPOR17L, _RPOR17
        .equiv  RPOR17H, _RPOR17+1

        .extern RCON
        .equiv  RCONL, _RCON
        .equiv  RCONH, _RCON+1

        .extern OSCCON

        .extern OSCCONL
        .equiv  OSCCONLL, _OSCCONL
        .equiv  OSCCONLH, _OSCCONL+1

        .extern OSCCONH
        .equiv  OSCCONHL, _OSCCONH
        .equiv  OSCCONHH, _OSCCONH+1

        .extern CLKDIV
        .equiv  CLKDIVL, _CLKDIV
        .equiv  CLKDIVH, _CLKDIV+1

        .extern PLLFBD
        .equiv  PLLFBDL, _PLLFBD
        .equiv  PLLFBDH, _PLLFBD+1

        .extern OSCTUN
        .equiv  OSCTUNL, _OSCTUN
        .equiv  OSCTUNH, _OSCTUN+1

        .extern LFSR
        .equiv  LFSRL, _LFSR
        .equiv  LFSRH, _LFSR+1

        .extern REFOCON
        .equiv  REFOCONL, _REFOCON
        .equiv  REFOCONH, _REFOCON+1

        .extern ACLKCON
        .equiv  ACLKCONL, _ACLKCON
        .equiv  ACLKCONH, _ACLKCON+1

        .extern NVMCON
        .equiv  NVMCONL, _NVMCON
        .equiv  NVMCONH, _NVMCON+1

        .extern NVMKEY
        .equiv  NVMKEYL, _NVMKEY
        .equiv  NVMKEYH, _NVMKEY+1

        .extern PMD1
        .equiv  PMD1L, _PMD1
        .equiv  PMD1H, _PMD1+1

        .extern PMD2
        .equiv  PMD2L, _PMD2
        .equiv  PMD2H, _PMD2+1

        .extern PMD4
        .equiv  PMD4L, _PMD4
        .equiv  PMD4H, _PMD4+1

        .extern PMD6
        .equiv  PMD6L, _PMD6
        .equiv  PMD6H, _PMD6+1

;----- SR Bits -----------------------------------------------------
        .equiv C,       0x0000
        .equiv Z,       0x0001
        .equiv OV,      0x0002
        .equiv N,       0x0003
        .equiv RA,      0x0004
        .equiv DC,      0x0008
        .equiv DA,      0x0009
        .equiv SAB,     0x000A
        .equiv OAB,     0x000B
        .equiv SB,      0x000C
        .equiv SA,      0x000D
        .equiv OB,      0x000E
        .equiv OA,      0x000F

        .equiv IPL0,    0x0005
        .equiv IPL1,    0x0006
        .equiv IPL2,    0x0007


;----- CORCON Bits -----------------------------------------------------
        .equiv IF,      0x0000
        .equiv RND,     0x0001
        .equiv PSV,     0x0002
        .equiv IPL3,    0x0003
        .equiv ACCSAT,  0x0004
        .equiv SATDW,   0x0005
        .equiv SATB,    0x0006
        .equiv SATA,    0x0007
        .equiv EDT,     0x000B
        .equiv US,      0x000C

        .equiv DL0,     0x0008
        .equiv DL1,     0x0009
        .equiv DL2,     0x000A


;----- MODCON Bits -----------------------------------------------------
        .equiv YMODEN,  0x000E
        .equiv XMODEN,  0x000F

        .equiv XWM0,    0x0000
        .equiv XWM1,    0x0001
        .equiv XWM2,    0x0002
        .equiv XWM3,    0x0003
        .equiv YWM0,    0x0004
        .equiv YWM1,    0x0005
        .equiv YWM2,    0x0006
        .equiv YWM3,    0x0007
        .equiv BWM0,    0x0008
        .equiv BWM1,    0x0009
        .equiv BWM2,    0x000A
        .equiv BWM3,    0x000B


;----- XBREV Bits -----------------------------------------------------
        .equiv BREN,    0x000F

        .equiv XB0,     0x0000
        .equiv XB1,     0x0001
        .equiv XB2,     0x0002
        .equiv XB3,     0x0003
        .equiv XB4,     0x0004
        .equiv XB5,     0x0005
        .equiv XB6,     0x0006
        .equiv XB7,     0x0007
        .equiv XB8,     0x0008
        .equiv XB9,     0x0009
        .equiv XB10,    0x000A
        .equiv XB11,    0x000B
        .equiv XB12,    0x000C
        .equiv XB13,    0x000D
        .equiv XB14,    0x000E


;----- CNEN1 Bits -----------------------------------------------------
        .equiv CN0IE,   0x0000
        .equiv CN1IE,   0x0001
        .equiv CN2IE,   0x0002
        .equiv CN3IE,   0x0003
        .equiv CN4IE,   0x0004
        .equiv CN5IE,   0x0005
        .equiv CN6IE,   0x0006
        .equiv CN7IE,   0x0007


;----- CNPU1 Bits -----------------------------------------------------
        .equiv CN0PUE,  0x0000
        .equiv CN1PUE,  0x0001
        .equiv CN2PUE,  0x0002
        .equiv CN3PUE,  0x0003
        .equiv CN4PUE,  0x0004
        .equiv CN5PUE,  0x0005
        .equiv CN6PUE,  0x0006
        .equiv CN7PUE,  0x0007


;----- INTCON1 Bits -----------------------------------------------------
        .equiv OSCFAIL, 0x0001
        .equiv STKERR,  0x0002
        .equiv ADDRERR, 0x0003
        .equiv MATHERR, 0x0004
        .equiv DIV0ERR, 0x0006
        .equiv SFTACERR, 0x0007
        .equiv COVTE,   0x0008
        .equiv OVBTE,   0x0009
        .equiv OVATE,   0x000A
        .equiv COVBERR, 0x000B
        .equiv COVAERR, 0x000C
        .equiv OVBERR,  0x000D
        .equiv OVAERR,  0x000E
        .equiv NSTDIS,  0x000F


;----- INTCON2 Bits -----------------------------------------------------
        .equiv INT0EP,  0x0000
        .equiv INT1EP,  0x0001
        .equiv INT2EP,  0x0002
        .equiv DISI,    0x000E
        .equiv ALTIVT,  0x000F


;----- IFS0 Bits -----------------------------------------------------
        .equiv INT0IF,  0x0000
        .equiv OC1IF,   0x0002
        .equiv T1IF,    0x0003
        .equiv T2IF,    0x0007
        .equiv SPI1EIF, 0x0009
        .equiv SPI1IF,  0x000A
        .equiv U1RXIF,  0x000B
        .equiv U1TXIF,  0x000C
        .equiv ADIF,    0x000D


;----- IFS1 Bits -----------------------------------------------------
        .equiv SI2C1IF, 0x0000
        .equiv MI2C1IF, 0x0001
        .equiv CNIF,    0x0003
        .equiv INT1IF,  0x0004
        .equiv INT2IF,  0x000D


;----- IFS3 Bits -----------------------------------------------------
        .equiv PSEMIF,  0x0009


;----- IFS4 Bits -----------------------------------------------------
        .equiv U1EIF,   0x0001


;----- IFS5 Bits -----------------------------------------------------
        .equiv JTAGIF,  0x0000
        .equiv PWM1IF,  0x000E


;----- IFS6 Bits -----------------------------------------------------
        .equiv PWM4IF,  0x0001
        .equiv ADCP0IF, 0x000E
        .equiv ADCP1IF, 0x000F


;----- IFS7 Bits -----------------------------------------------------
        .equiv ADCP3IF, 0x0001


;----- IEC0 Bits -----------------------------------------------------
        .equiv INT0IE,  0x0000
        .equiv OC1IE,   0x0002
        .equiv T1IE,    0x0003
        .equiv T2IE,    0x0007
        .equiv SPI1EIE, 0x0009
        .equiv SPI1IE,  0x000A
        .equiv U1RXIE,  0x000B
        .equiv U1TXIE,  0x000C
        .equiv ADIE,    0x000D


;----- IEC1 Bits -----------------------------------------------------
        .equiv SI2C1IE, 0x0000
        .equiv MI2C1IE, 0x0001
        .equiv CNIE,    0x0003
        .equiv INT1IE,  0x0004
        .equiv INT2IE,  0x000D


;----- IEC3 Bits -----------------------------------------------------
        .equiv PSEMIE,  0x0009


;----- IEC4 Bits -----------------------------------------------------
        .equiv U1EIE,   0x0001


;----- IEC5 Bits -----------------------------------------------------
        .equiv JTAGIE,  0x0000
        .equiv PWM1IE,  0x000E


;----- IEC6 Bits -----------------------------------------------------
        .equiv PWM4IE,  0x0001
        .equiv ADCP0IE, 0x000E
        .equiv ADCP1IE, 0x000F


;----- IEC7 Bits -----------------------------------------------------
        .equiv ADCP3IE, 0x0001


;----- IPC0 Bits -----------------------------------------------------
        .equiv INT0IP0, 0x0000
        .equiv INT0IP1, 0x0001
        .equiv INT0IP2, 0x0002
        .equiv OC1IP0,  0x0008
        .equiv OC1IP1,  0x0009
        .equiv OC1IP2,  0x000A
        .equiv T1IP0,   0x000C
        .equiv T1IP1,   0x000D
        .equiv T1IP2,   0x000E


;----- IPC1 Bits -----------------------------------------------------
        .equiv T2IP0,   0x000C
        .equiv T2IP1,   0x000D
        .equiv T2IP2,   0x000E


;----- IPC2 Bits -----------------------------------------------------
        .equiv SPI1EIP0, 0x0004
        .equiv SPI1EIP1, 0x0005
        .equiv SPI1EIP2, 0x0006
        .equiv SPI1IP0, 0x0008
        .equiv SPI1IP1, 0x0009
        .equiv SPI1IP2, 0x000A
        .equiv U1RXIP0, 0x000C
        .equiv U1RXIP1, 0x000D
        .equiv U1RXIP2, 0x000E


;----- IPC3 Bits -----------------------------------------------------
        .equiv U1TXIP0, 0x0000
        .equiv U1TXIP1, 0x0001
        .equiv U1TXIP2, 0x0002
        .equiv ADIP0,   0x0004
        .equiv ADIP1,   0x0005
        .equiv ADIP2,   0x0006


;----- IPC4 Bits -----------------------------------------------------
        .equiv SI2C1IP0, 0x0000
        .equiv SI2C1IP1, 0x0001
        .equiv SI2C1IP2, 0x0002
        .equiv MI2C1IP0, 0x0004
        .equiv MI2C1IP1, 0x0005
        .equiv MI2C1IP2, 0x0006
        .equiv CNIP0,   0x000C
        .equiv CNIP1,   0x000D
        .equiv CNIP2,   0x000E


;----- IPC5 Bits -----------------------------------------------------
        .equiv INT1IP0, 0x0000
        .equiv INT1IP1, 0x0001
        .equiv INT1IP2, 0x0002


;----- IPC7 Bits -----------------------------------------------------
        .equiv INT2IP0, 0x0004
        .equiv INT2IP1, 0x0005
        .equiv INT2IP2, 0x0006


;----- IPC14 Bits -----------------------------------------------------
        .equiv PSEMIP0, 0x0004
        .equiv PSEMIP1, 0x0005
        .equiv PSEMIP2, 0x0006


;----- IPC16 Bits -----------------------------------------------------
        .equiv U1EIP0,  0x0004
        .equiv U1EIP1,  0x0005
        .equiv U1EIP2,  0x0006


;----- IPC20 Bits -----------------------------------------------------
        .equiv JTAGIP0, 0x0000
        .equiv JTAGIP1, 0x0001
        .equiv JTAGIP2, 0x0002


;----- IPC23 Bits -----------------------------------------------------
        .equiv PWM1IP0, 0x0008
        .equiv PWM1IP1, 0x0009
        .equiv PWM1IP2, 0x000A


;----- IPC24 Bits -----------------------------------------------------
        .equiv PWM4IP0, 0x0004
        .equiv PWM4IP1, 0x0005
        .equiv PWM4IP2, 0x0006


;----- IPC27 Bits -----------------------------------------------------
        .equiv ADCP0IP0, 0x0008
        .equiv ADCP0IP1, 0x0009
        .equiv ADCP0IP2, 0x000A
        .equiv ADCP1IP0, 0x000C
        .equiv ADCP1IP1, 0x000D
        .equiv ADCP1IP2, 0x000E


;----- IPC28 Bits -----------------------------------------------------
        .equiv ADCP3IP0, 0x0004
        .equiv ADCP3IP1, 0x0005
        .equiv ADCP3IP2, 0x0006


;----- INTTREG Bits -----------------------------------------------------
        .equiv VECNUM0, 0x0000
        .equiv VECNUM1, 0x0001
        .equiv VECNUM2, 0x0002
        .equiv VECNUM3, 0x0003
        .equiv VECNUM4, 0x0004
        .equiv VECNUM5, 0x0005
        .equiv VECNUM6, 0x0006
        .equiv ILR0,    0x0008
        .equiv ILR1,    0x0009
        .equiv ILR2,    0x000A
        .equiv ILR3,    0x000B


;----- T1CON Bits -----------------------------------------------------
        .equiv TCS,     0x0001
        .equiv TSYNC,   0x0002
        .equiv TGATE,   0x0006
        .equiv TSIDL,   0x000D
        .equiv TON,     0x000F

        .equiv TCKPS0,  0x0004
        .equiv TCKPS1,  0x0005


;----- T2CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005


;----- OC1CON Bits -----------------------------------------------------
        .equiv OCFLT,   0x0004
        .equiv OCSIDL,  0x000D

        .equiv OCM0,    0x0000
        .equiv OCM1,    0x0001
        .equiv OCM2,    0x0002


;----- I2C1CON Bits -----------------------------------------------------
        .equiv SEN,     0x0000
        .equiv RSEN,    0x0001
        .equiv PEN,     0x0002
        .equiv RCEN,    0x0003
        .equiv ACKEN,   0x0004
        .equiv ACKDT,   0x0005
        .equiv STREN,   0x0006
        .equiv GCEN,    0x0007
        .equiv SMEN,    0x0008
        .equiv DISSLW,  0x0009
        .equiv A10M,    0x000A
        .equiv IPMIEN,  0x000B
        .equiv SCLREL,  0x000C
        .equiv I2CSIDL, 0x000D
        .equiv I2CEN,   0x000F


;----- I2CCON Bits -----------------------------------------------------
       ;.equiv SEN,     0x0000
       ;.equiv RSEN,    0x0001
       ;.equiv PEN,     0x0002
       ;.equiv RCEN,    0x0003
       ;.equiv ACKEN,   0x0004
       ;.equiv ACKDT,   0x0005
       ;.equiv STREN,   0x0006
       ;.equiv GCEN,    0x0007
       ;.equiv SMEN,    0x0008
       ;.equiv DISSLW,  0x0009
       ;.equiv A10M,    0x000A
       ;.equiv IPMIEN,  0x000B
       ;.equiv SCLREL,  0x000C
       ;.equiv I2CSIDL, 0x000D
       ;.equiv I2CEN,   0x000F


;----- I2C1STAT Bits -----------------------------------------------------
        .equiv TBF,     0x0000
        .equiv RBF,     0x0001
        .equiv R_W,     0x0002
        .equiv S,       0x0003
        .equiv P,       0x0004
        .equiv D_A,     0x0005
        .equiv I2COV,   0x0006
        .equiv IWCOL,   0x0007
        .equiv ADD10,   0x0008
        .equiv GCSTAT,  0x0009
        .equiv BCL,     0x000A
        .equiv TRSTAT,  0x000E
        .equiv ACKSTAT, 0x000F


;----- I2CSTAT Bits -----------------------------------------------------
       ;.equiv TBF,     0x0000
       ;.equiv RBF,     0x0001
       ;.equiv R_W,     0x0002
       ;.equiv S,       0x0003
       ;.equiv P,       0x0004
       ;.equiv D_A,     0x0005
       ;.equiv I2COV,   0x0006
       ;.equiv IWCOL,   0x0007
       ;.equiv ADD10,   0x0008
       ;.equiv GCSTAT,  0x0009
       ;.equiv BCL,     0x000A
       ;.equiv TRSTAT,  0x000E
       ;.equiv ACKSTAT, 0x000F


;----- U1MODE Bits -----------------------------------------------------
        .equiv STSEL,   0x0000
        .equiv BRGH,    0x0003
        .equiv URXINV,  0x0004
        .equiv ABAUD,   0x0005
        .equiv LPBACK,  0x0006
        .equiv WAKE,    0x0007
        .equiv RTSMD,   0x000B
        .equiv IREN,    0x000C
        .equiv USIDL,   0x000D
        .equiv UARTEN,  0x000F

        .equiv PDSEL0,  0x0001
        .equiv PDSEL1,  0x0002
        .equiv UEN0,    0x0008
        .equiv UEN1,    0x0009


;----- U1STA Bits -----------------------------------------------------
        .equiv URXDA,   0x0000
        .equiv OERR,    0x0001
        .equiv FERR,    0x0002
        .equiv PERR,    0x0003
        .equiv RIDLE,   0x0004
        .equiv ADDEN,   0x0005
        .equiv TRMT,    0x0008
        .equiv UTXBF,   0x0009
        .equiv UTXEN,   0x000A
        .equiv UTXBRK,  0x000B
        .equiv UTXISEL0, 0x000D
        .equiv UTXINV,  0x000E
        .equiv UTXISEL1, 0x000F

        .equiv URXISEL0, 0x0006
        .equiv URXISEL1, 0x0007


;----- U1TXREG Bits -----------------------------------------------------
        .equiv UTXREG0, 0x0000
        .equiv UTXREG1, 0x0001
        .equiv UTXREG2, 0x0002
        .equiv UTXREG3, 0x0003
        .equiv UTXREG4, 0x0004
        .equiv UTXREG5, 0x0005
        .equiv UTXREG6, 0x0006
        .equiv UTXREG7, 0x0007
        .equiv UTXREG8, 0x0008


;----- U1RXREG Bits -----------------------------------------------------
        .equiv URXREG0, 0x0000
        .equiv URXREG1, 0x0001
        .equiv URXREG2, 0x0002
        .equiv URXREG3, 0x0003
        .equiv URXREG4, 0x0004
        .equiv URXREG5, 0x0005
        .equiv URXREG6, 0x0006
        .equiv URXREG7, 0x0007
        .equiv URXREG8, 0x0008


;----- SPI1STAT Bits -----------------------------------------------------
        .equiv SPIRBF,  0x0000
        .equiv SPITBF,  0x0001
        .equiv SPIROV,  0x0006
        .equiv SPISIDL, 0x000D
        .equiv SPIEN,   0x000F


;----- SPI1CON1 Bits -----------------------------------------------------
        .equiv MSTEN,   0x0005
        .equiv CKP,     0x0006
        .equiv SSEN,    0x0007
        .equiv CKE,     0x0008
        .equiv SMP,     0x0009
        .equiv MODE16,  0x000A
        .equiv DISSDO,  0x000B
        .equiv DISSCK,  0x000C

        .equiv PPRE0,   0x0000
        .equiv PPRE1,   0x0001
        .equiv SPRE0,   0x0002
        .equiv SPRE1,   0x0003
        .equiv SPRE2,   0x0004


;----- SPI1CON2 Bits -----------------------------------------------------
        .equiv FRMDLY,  0x0001
        .equiv FRMPOL,  0x000D
        .equiv SPIFSD,  0x000E
        .equiv FRMEN,   0x000F


;----- TRISA Bits -----------------------------------------------------
        .equiv TRISA0,  0x0000
        .equiv TRISA1,  0x0001
        .equiv TRISA2,  0x0002
        .equiv TRISA3,  0x0003
        .equiv TRISA4,  0x0004


;----- PORTA Bits -----------------------------------------------------
        .equiv RA0,     0x0000
        .equiv RA1,     0x0001
        .equiv RA2,     0x0002
        .equiv RA3,     0x0003
        .equiv RA4,     0x0004


;----- LATA Bits -----------------------------------------------------
        .equiv LATA0,   0x0000
        .equiv LATA1,   0x0001
        .equiv LATA2,   0x0002
        .equiv LATA3,   0x0003
        .equiv LATA4,   0x0004


;----- ODCA Bits -----------------------------------------------------
        .equiv ODCA3,   0x0003
        .equiv ODCA4,   0x0004


;----- TRISB Bits -----------------------------------------------------
        .equiv TRISB0,  0x0000
        .equiv TRISB1,  0x0001
        .equiv TRISB2,  0x0002
        .equiv TRISB3,  0x0003
        .equiv TRISB4,  0x0004
        .equiv TRISB5,  0x0005
        .equiv TRISB6,  0x0006
        .equiv TRISB7,  0x0007


;----- PORTB Bits -----------------------------------------------------
        .equiv RB0,     0x0000
        .equiv RB1,     0x0001
        .equiv RB2,     0x0002
        .equiv RB3,     0x0003
        .equiv RB4,     0x0004
        .equiv RB5,     0x0005
        .equiv RB6,     0x0006
        .equiv RB7,     0x0007


;----- LATB Bits -----------------------------------------------------
        .equiv LATB0,   0x0000
        .equiv LATB1,   0x0001
        .equiv LATB2,   0x0002
        .equiv LATB3,   0x0003
        .equiv LATB4,   0x0004
        .equiv LATB5,   0x0005
        .equiv LATB6,   0x0006
        .equiv LATB7,   0x0007


;----- ODCB Bits -----------------------------------------------------
        .equiv ODCB6,   0x0006
        .equiv ODCB7,   0x0007


;----- ADCON Bits -----------------------------------------------------
        .equiv ASYNCSAMP, 0x0004
        .equiv SEQSAMP, 0x0005
        .equiv ORDER,   0x0006
        .equiv EIE,     0x0007
        .equiv FORM,    0x0008
        .equiv GSWTRG,  0x000A
        .equiv SLOWCLK, 0x000C
        .equiv ADSIDL,  0x000D
        .equiv ADON,    0x000F

        .equiv ADCS0,   0x0000
        .equiv ADCS1,   0x0001
        .equiv ADCS2,   0x0002


;----- ADPCFG Bits -----------------------------------------------------
        .equiv PCFG0,   0x0000
        .equiv PCFG1,   0x0001
        .equiv PCFG2,   0x0002
        .equiv PCFG3,   0x0003
        .equiv PCFG6,   0x0006
        .equiv PCFG7,   0x0007


;----- ADSTAT Bits -----------------------------------------------------
        .equiv P0RDY,   0x0000
        .equiv P1RDY,   0x0001
        .equiv P3RDY,   0x0003
        .equiv P6RDY,   0x0006


;----- ADCPC0 Bits -----------------------------------------------------
        .equiv SWTRG0,  0x0005
        .equiv PEND0,   0x0006
        .equiv IRQEN0,  0x0007
        .equiv SWTRG1,  0x000D
        .equiv PEND1,   0x000E
        .equiv IRQEN1,  0x000F

        .equiv TRGSRC00, 0x0000
        .equiv TRGSRC01, 0x0001
        .equiv TRGSRC02, 0x0002
        .equiv TRGSRC03, 0x0003
        .equiv TRGSRC04, 0x0004
        .equiv TRGSRC10, 0x0008
        .equiv TRGSRC11, 0x0009
        .equiv TRGSRC12, 0x000A
        .equiv TRGSRC13, 0x000B
        .equiv TRGSRC14, 0x000C


;----- ADCPC1 Bits -----------------------------------------------------
        .equiv SWTRG3,  0x000D
        .equiv PEND3,   0x000E
        .equiv IRQEN3,  0x000F

        .equiv TRGSRC30, 0x0008
        .equiv TRGSRC31, 0x0009
        .equiv TRGSRC32, 0x000A
        .equiv TRGSRC33, 0x000B
        .equiv TRGSRC34, 0x000C


;----- ADCPC3 Bits -----------------------------------------------------
        .equiv SWTRG6,  0x0005
        .equiv PEND6,   0x0006
        .equiv IRQEN6,  0x0007

        .equiv TRGSRC60, 0x0000
        .equiv TRGSRC61, 0x0001
        .equiv TRGSRC62, 0x0002
        .equiv TRGSRC63, 0x0003
        .equiv TRGSRC64, 0x0004


;----- PTCON Bits -----------------------------------------------------
        .equiv SYNCEN,  0x0007
        .equiv SYNCOEN, 0x0008
        .equiv SYNCPOL, 0x0009
        .equiv EIPU,    0x000A
        .equiv SEIEN,   0x000B
        .equiv SESTAT,  0x000C
        .equiv PTSIDL,  0x000D
        .equiv PTEN,    0x000F

        .equiv SEVTPS0, 0x0000
        .equiv SEVTPS1, 0x0001
        .equiv SEVTPS2, 0x0002
        .equiv SEVTPS3, 0x0003
        .equiv SYNCSRC0, 0x0004
        .equiv SYNCSRC1, 0x0005


;----- PTCON2 Bits -----------------------------------------------------
        .equiv PCLKDIV0, 0x0000
        .equiv PCLKDIV1, 0x0001
        .equiv PCLKDIV2, 0x0002


;----- CHOP Bits -----------------------------------------------------
        .equiv CHPCLKEN, 0x000F

        .equiv CHOPCLK0, 0x0003
        .equiv CHOPCLK1, 0x0004
        .equiv CHOPCLK2, 0x0005
        .equiv CHOPCLK3, 0x0006
        .equiv CHOPCLK4, 0x0007
        .equiv CHOPCLK5, 0x0008
        .equiv CHOPCLK6, 0x0009


;----- PWMCON1 Bits -----------------------------------------------------
        .equiv IUE,     0x0000
        .equiv XPRES,   0x0001
        .equiv CAM,     0x0002
        .equiv MDCS,    0x0008
        .equiv ITB,     0x0009
        .equiv TRGIEN,  0x000A
        .equiv CLIEN,   0x000B
        .equiv FLTIEN,  0x000C
        .equiv TRGSTAT, 0x000D
        .equiv CLSTAT,  0x000E
        .equiv FLTSTAT, 0x000F

        .equiv DTC0,    0x0006
        .equiv DTC1,    0x0007


;----- IOCON1 Bits -----------------------------------------------------
        .equiv OSYNC,   0x0000
        .equiv SWAP,    0x0001
        .equiv OVRENL,  0x0008
        .equiv OVRENH,  0x0009
        .equiv POLL,    0x000C
        .equiv POLH,    0x000D
        .equiv PENL,    0x000E
        .equiv PENH,    0x000F

        .equiv CLDAT0,  0x0002
        .equiv CLDAT1,  0x0003
        .equiv FLTDAT0, 0x0004
        .equiv FLTDAT1, 0x0005
        .equiv OVRDAT0, 0x0006
        .equiv OVRDAT1, 0x0007
        .equiv PMOD0,   0x000A
        .equiv PMOD1,   0x000B


;----- FCLCON1 Bits -----------------------------------------------------
        .equiv FLTPOL,  0x0002
        .equiv CLMOD,   0x0008
        .equiv CLPOL,   0x0009
        .equiv IFLTMOD, 0x000F

        .equiv FLTMOD0, 0x0000
        .equiv FLTMOD1, 0x0001
        .equiv FLTSRC0, 0x0003
        .equiv FLTSRC1, 0x0004
        .equiv FLTSRC2, 0x0005
        .equiv FLTSRC3, 0x0006
        .equiv FLTSRC4, 0x0007
        .equiv CLSRC0,  0x000A
        .equiv CLSRC1,  0x000B
        .equiv CLSRC2,  0x000C
        .equiv CLSRC3,  0x000D
        .equiv CLSRC4,  0x000E


;----- TRIG1 Bits -----------------------------------------------------
        .equiv TRGCMP0, 0x0003
        .equiv TRGCMP1, 0x0004
        .equiv TRGCMP2, 0x0005
        .equiv TRGCMP3, 0x0006
        .equiv TRGCMP4, 0x0007
        .equiv TRGCMP5, 0x0008
        .equiv TRGCMP6, 0x0009
        .equiv TRGCMP7, 0x000A
        .equiv TRGCMP8, 0x000B
        .equiv TRGCMP9, 0x000C
        .equiv TRGCMP10, 0x000D
        .equiv TRGCMP11, 0x000E
        .equiv TRGCMP12, 0x000F


;----- TRGCON1 Bits -----------------------------------------------------
        .equiv DTM,     0x0007

        .equiv TRGSTRT0, 0x0000
        .equiv TRGSTRT1, 0x0001
        .equiv TRGSTRT2, 0x0002
        .equiv TRGSTRT3, 0x0003
        .equiv TRGSTRT4, 0x0004
        .equiv TRGSTRT5, 0x0005
        .equiv TRGDIV0, 0x000C
        .equiv TRGDIV1, 0x000D
        .equiv TRGDIV2, 0x000E
        .equiv TRGDIV3, 0x000F


;----- STRIG1 Bits -----------------------------------------------------
        .equiv STRGCMP0, 0x0003
        .equiv STRGCMP1, 0x0004
        .equiv STRGCMP2, 0x0005
        .equiv STRGCMP3, 0x0006
        .equiv STRGCMP4, 0x0007
        .equiv STRGCMP5, 0x0008
        .equiv STRGCMP6, 0x0009
        .equiv STRGCMP7, 0x000A
        .equiv STRGCMP8, 0x000B
        .equiv STRGCMP9, 0x000C
        .equiv STRGCMP10, 0x000D
        .equiv STRGCMP11, 0x000E
        .equiv STRGCMP12, 0x000F


;----- PWMCAP1 Bits -----------------------------------------------------
        .equiv PWMCAP10, 0x0003
        .equiv PWMCAP11, 0x0004
        .equiv PWMCAP12, 0x0005
        .equiv PWMCAP13, 0x0006
        .equiv PWMCAP14, 0x0007
        .equiv PWMCAP15, 0x0008
        .equiv PWMCAP16, 0x0009
        .equiv PWMCAP17, 0x000A
        .equiv PWMCAP18, 0x000B
        .equiv PWMCAP19, 0x000C
        .equiv PWMCAP110, 0x000D
        .equiv PWMCAP111, 0x000E
        .equiv PWMCAP112, 0x000F


;----- LEBCON1 Bits -----------------------------------------------------
        .equiv CLLEBEN, 0x000A
        .equiv FLTLEBEN, 0x000B
        .equiv PLF,     0x000C
        .equiv PLR,     0x000D
        .equiv PHF,     0x000E
        .equiv PHR,     0x000F

        .equiv LEB0,    0x0003
        .equiv LEB1,    0x0004
        .equiv LEB2,    0x0005
        .equiv LEB3,    0x0006
        .equiv LEB4,    0x0007
        .equiv LEB5,    0x0008
        .equiv LEB6,    0x0009


;----- AUXCON1 Bits -----------------------------------------------------
        .equiv CHOPLEN, 0x0000
        .equiv CHOPHEN, 0x0001
        .equiv HRDDIS,  0x000E
        .equiv HRPDIS,  0x000F


;----- PWMCON4 Bits -----------------------------------------------------
       ;.equiv IUE,     0x0000
       ;.equiv XPRES,   0x0001
       ;.equiv CAM,     0x0002
       ;.equiv MDCS,    0x0008
       ;.equiv ITB,     0x0009
       ;.equiv TRGIEN,  0x000A
       ;.equiv CLIEN,   0x000B
       ;.equiv FLTIEN,  0x000C
       ;.equiv TRGSTAT, 0x000D
       ;.equiv CLSTAT,  0x000E
       ;.equiv FLTSTAT, 0x000F

       ;.equiv DTC0,    0x0006
       ;.equiv DTC1,    0x0007


;----- IOCON4 Bits -----------------------------------------------------
       ;.equiv OSYNC,   0x0000
       ;.equiv SWAP,    0x0001
       ;.equiv OVRENL,  0x0008
       ;.equiv OVRENH,  0x0009
       ;.equiv POLL,    0x000C
       ;.equiv POLH,    0x000D
       ;.equiv PENL,    0x000E
       ;.equiv PENH,    0x000F

       ;.equiv CLDAT0,  0x0002
       ;.equiv CLDAT1,  0x0003
       ;.equiv FLTDAT0, 0x0004
       ;.equiv FLTDAT1, 0x0005
       ;.equiv OVRDAT0, 0x0006
       ;.equiv OVRDAT1, 0x0007
       ;.equiv PMOD0,   0x000A
       ;.equiv PMOD1,   0x000B


;----- FCLCON4 Bits -----------------------------------------------------
       ;.equiv FLTPOL,  0x0002
       ;.equiv CLMOD,   0x0008
       ;.equiv CLPOL,   0x0009
       ;.equiv IFLTMOD, 0x000F

       ;.equiv FLTMOD0, 0x0000
       ;.equiv FLTMOD1, 0x0001
       ;.equiv FLTSRC0, 0x0003
       ;.equiv FLTSRC1, 0x0004
       ;.equiv FLTSRC2, 0x0005
       ;.equiv FLTSRC3, 0x0006
       ;.equiv FLTSRC4, 0x0007
       ;.equiv CLSRC0,  0x000A
       ;.equiv CLSRC1,  0x000B
       ;.equiv CLSRC2,  0x000C
       ;.equiv CLSRC3,  0x000D
       ;.equiv CLSRC4,  0x000E


;----- TRIG4 Bits -----------------------------------------------------
       ;.equiv TRGCMP0, 0x0003
       ;.equiv TRGCMP1, 0x0004
       ;.equiv TRGCMP2, 0x0005
       ;.equiv TRGCMP3, 0x0006
       ;.equiv TRGCMP4, 0x0007
       ;.equiv TRGCMP5, 0x0008
       ;.equiv TRGCMP6, 0x0009
       ;.equiv TRGCMP7, 0x000A
       ;.equiv TRGCMP8, 0x000B
       ;.equiv TRGCMP9, 0x000C
       ;.equiv TRGCMP10, 0x000D
       ;.equiv TRGCMP11, 0x000E
       ;.equiv TRGCMP12, 0x000F


;----- TRGCON4 Bits -----------------------------------------------------
       ;.equiv DTM,     0x0007

       ;.equiv TRGSTRT0, 0x0000
       ;.equiv TRGSTRT1, 0x0001
       ;.equiv TRGSTRT2, 0x0002
       ;.equiv TRGSTRT3, 0x0003
       ;.equiv TRGSTRT4, 0x0004
       ;.equiv TRGSTRT5, 0x0005
       ;.equiv TRGDIV0, 0x000C
       ;.equiv TRGDIV1, 0x000D
       ;.equiv TRGDIV2, 0x000E
       ;.equiv TRGDIV3, 0x000F


;----- STRIG4 Bits -----------------------------------------------------
       ;.equiv STRGCMP0, 0x0003
       ;.equiv STRGCMP1, 0x0004
       ;.equiv STRGCMP2, 0x0005
       ;.equiv STRGCMP3, 0x0006
       ;.equiv STRGCMP4, 0x0007
       ;.equiv STRGCMP5, 0x0008
       ;.equiv STRGCMP6, 0x0009
       ;.equiv STRGCMP7, 0x000A
       ;.equiv STRGCMP8, 0x000B
       ;.equiv STRGCMP9, 0x000C
       ;.equiv STRGCMP10, 0x000D
       ;.equiv STRGCMP11, 0x000E
       ;.equiv STRGCMP12, 0x000F


;----- LEBCON4 Bits -----------------------------------------------------
       ;.equiv CLLEBEN, 0x000A
       ;.equiv FLTLEBEN, 0x000B
       ;.equiv PLF,     0x000C
       ;.equiv PLR,     0x000D
       ;.equiv PHF,     0x000E
       ;.equiv PHR,     0x000F

       ;.equiv LEB0,    0x0003
       ;.equiv LEB1,    0x0004
       ;.equiv LEB2,    0x0005
       ;.equiv LEB3,    0x0006
       ;.equiv LEB4,    0x0007
       ;.equiv LEB5,    0x0008
       ;.equiv LEB6,    0x0009


;----- AUXCON4 Bits -----------------------------------------------------
       ;.equiv CHOPLEN, 0x0000
       ;.equiv CHOPHEN, 0x0001
       ;.equiv HRDDIS,  0x000E
       ;.equiv HRPDIS,  0x000F


;----- RPINR0 Bits -----------------------------------------------------
        .equiv INT1R0,  0x0008
        .equiv INT1R1,  0x0009
        .equiv INT1R2,  0x000A
        .equiv INT1R3,  0x000B
        .equiv INT1R4,  0x000C
        .equiv INT1R5,  0x000D


;----- RPINR1 Bits -----------------------------------------------------
        .equiv INT2R0,  0x0000
        .equiv INT2R1,  0x0001
        .equiv INT2R2,  0x0002
        .equiv INT2R3,  0x0003
        .equiv INT2R4,  0x0004
        .equiv INT2R5,  0x0005


;----- RPINR2 Bits -----------------------------------------------------
        .equiv T1CKR0,  0x0008
        .equiv T1CKR1,  0x0009
        .equiv T1CKR2,  0x000A
        .equiv T1CKR3,  0x000B
        .equiv T1CKR4,  0x000C
        .equiv T1CKR5,  0x000D


;----- RPINR3 Bits -----------------------------------------------------
        .equiv T2CKR0,  0x0000
        .equiv T2CKR1,  0x0001
        .equiv T2CKR2,  0x0002
        .equiv T2CKR3,  0x0003
        .equiv T2CKR4,  0x0004
        .equiv T2CKR5,  0x0005


;----- RPINR11 Bits -----------------------------------------------------
        .equiv OCFAR0,  0x0000
        .equiv OCFAR1,  0x0001
        .equiv OCFAR2,  0x0002
        .equiv OCFAR3,  0x0003
        .equiv OCFAR4,  0x0004
        .equiv OCFAR5,  0x0005


;----- RPINR18 Bits -----------------------------------------------------
        .equiv U1RXR0,  0x0000
        .equiv U1RXR1,  0x0001
        .equiv U1RXR2,  0x0002
        .equiv U1RXR3,  0x0003
        .equiv U1RXR4,  0x0004
        .equiv U1RXR5,  0x0005
        .equiv U1CTSR0, 0x0008
        .equiv U1CTSR1, 0x0009
        .equiv U1CTSR2, 0x000A
        .equiv U1CTSR3, 0x000B
        .equiv U1CTSR4, 0x000C
        .equiv U1CTSR5, 0x000D


;----- RPINR20 Bits -----------------------------------------------------
        .equiv SDI1R0,  0x0000
        .equiv SDI1R1,  0x0001
        .equiv SDI1R2,  0x0002
        .equiv SDI1R3,  0x0003
        .equiv SDI1R4,  0x0004
        .equiv SDI1R5,  0x0005
        .equiv SCK1R0,  0x0008
        .equiv SCK1R1,  0x0009
        .equiv SCK1R2,  0x000A
        .equiv SCK1R3,  0x000B
        .equiv SCK1R4,  0x000C
        .equiv SCK1R5,  0x000D


;----- RPINR21 Bits -----------------------------------------------------
        .equiv SS1R0,   0x0000
        .equiv SS1R1,   0x0001
        .equiv SS1R2,   0x0002
        .equiv SS1R3,   0x0003
        .equiv SS1R4,   0x0004
        .equiv SS1R5,   0x0005


;----- RPINR29 Bits -----------------------------------------------------
        .equiv FLT1R0,  0x0008
        .equiv FLT1R1,  0x0009
        .equiv FLT1R2,  0x000A
        .equiv FLT1R3,  0x000B
        .equiv FLT1R4,  0x000C
        .equiv FLT1R5,  0x000D


;----- RPINR30 Bits -----------------------------------------------------
        .equiv FLT2R0,  0x0000
        .equiv FLT2R1,  0x0001
        .equiv FLT2R2,  0x0002
        .equiv FLT2R3,  0x0003
        .equiv FLT2R4,  0x0004
        .equiv FLT2R5,  0x0005
        .equiv FLT3R0,  0x0008
        .equiv FLT3R1,  0x0009
        .equiv FLT3R2,  0x000A
        .equiv FLT3R3,  0x000B
        .equiv FLT3R4,  0x000C
        .equiv FLT3R5,  0x000D


;----- RPINR31 Bits -----------------------------------------------------
        .equiv FLT4R0,  0x0000
        .equiv FLT4R1,  0x0001
        .equiv FLT4R2,  0x0002
        .equiv FLT4R3,  0x0003
        .equiv FLT4R4,  0x0004
        .equiv FLT4R5,  0x0005
        .equiv FLT5R0,  0x0008
        .equiv FLT5R1,  0x0009
        .equiv FLT5R2,  0x000A
        .equiv FLT5R3,  0x000B
        .equiv FLT5R4,  0x000C
        .equiv FLT5R5,  0x000D


;----- RPINR32 Bits -----------------------------------------------------
        .equiv FLT6R0,  0x0000
        .equiv FLT6R1,  0x0001
        .equiv FLT6R2,  0x0002
        .equiv FLT6R3,  0x0003
        .equiv FLT6R4,  0x0004
        .equiv FLT6R5,  0x0005
        .equiv FLT7R0,  0x0008
        .equiv FLT7R1,  0x0009
        .equiv FLT7R2,  0x000A
        .equiv FLT7R3,  0x000B
        .equiv FLT7R4,  0x000C
        .equiv FLT7R5,  0x000D


;----- RPINR33 Bits -----------------------------------------------------
        .equiv FLT8R0,  0x0000
        .equiv FLT8R1,  0x0001
        .equiv FLT8R2,  0x0002
        .equiv FLT8R3,  0x0003
        .equiv FLT8R4,  0x0004
        .equiv FLT8R5,  0x0005
        .equiv SYNCI1R0, 0x0008
        .equiv SYNCI1R1, 0x0009
        .equiv SYNCI1R2, 0x000A
        .equiv SYNCI1R3, 0x000B
        .equiv SYNCI1R4, 0x000C
        .equiv SYNCI1R5, 0x000D


;----- RPINR34 Bits -----------------------------------------------------
        .equiv SYNCI2R0, 0x0000
        .equiv SYNCI2R1, 0x0001
        .equiv SYNCI2R2, 0x0002
        .equiv SYNCI2R3, 0x0003
        .equiv SYNCI2R4, 0x0004
        .equiv SYNCI2R5, 0x0005


;----- RPOR0 Bits -----------------------------------------------------
        .equiv RP0R0,   0x0000
        .equiv RP0R1,   0x0001
        .equiv RP0R2,   0x0002
        .equiv RP0R3,   0x0003
        .equiv RP0R4,   0x0004
        .equiv RP0R5,   0x0005
        .equiv RP1R0,   0x0008
        .equiv RP1R1,   0x0009
        .equiv RP1R2,   0x000A
        .equiv RP1R3,   0x000B
        .equiv RP1R4,   0x000C
        .equiv RP1R5,   0x000D


;----- RPOR1 Bits -----------------------------------------------------
        .equiv RP2R0,   0x0000
        .equiv RP2R1,   0x0001
        .equiv RP2R2,   0x0002
        .equiv RP2R3,   0x0003
        .equiv RP2R4,   0x0004
        .equiv RP2R5,   0x0005
        .equiv RP3R0,   0x0008
        .equiv RP3R1,   0x0009
        .equiv RP3R2,   0x000A
        .equiv RP3R3,   0x000B
        .equiv RP3R4,   0x000C
        .equiv RP3R5,   0x000D


;----- RPOR2 Bits -----------------------------------------------------
        .equiv RP4R0,   0x0000
        .equiv RP4R1,   0x0001
        .equiv RP4R2,   0x0002
        .equiv RP4R3,   0x0003
        .equiv RP4R4,   0x0004
        .equiv RP4R5,   0x0005
        .equiv RP5R0,   0x0008
        .equiv RP5R1,   0x0009
        .equiv RP5R2,   0x000A
        .equiv RP5R3,   0x000B
        .equiv RP5R4,   0x000C
        .equiv RP5R5,   0x000D


;----- RPOR3 Bits -----------------------------------------------------
        .equiv RP6R0,   0x0000
        .equiv RP6R1,   0x0001
        .equiv RP6R2,   0x0002
        .equiv RP6R3,   0x0003
        .equiv RP6R4,   0x0004
        .equiv RP6R5,   0x0005
        .equiv RP7R0,   0x0008
        .equiv RP7R1,   0x0009
        .equiv RP7R2,   0x000A
        .equiv RP7R3,   0x000B
        .equiv RP7R4,   0x000C
        .equiv RP7R5,   0x000D


;----- RPOR16 Bits -----------------------------------------------------
        .equiv RP32R0,  0x0000
        .equiv RP32R1,  0x0001
        .equiv RP32R2,  0x0002
        .equiv RP32R3,  0x0003
        .equiv RP32R4,  0x0004
        .equiv RP32R5,  0x0005
        .equiv RP33R0,  0x0008
        .equiv RP33R1,  0x0009
        .equiv RP33R2,  0x000A
        .equiv RP33R3,  0x000B
        .equiv RP33R4,  0x000C
        .equiv RP33R5,  0x000D


;----- RPOR17 Bits -----------------------------------------------------
        .equiv RP34R0,  0x0000
        .equiv RP34R1,  0x0001
        .equiv RP34R2,  0x0002
        .equiv RP34R3,  0x0003
        .equiv RP34R4,  0x0004
        .equiv RP34R5,  0x0005
        .equiv RP35R0,  0x0008
        .equiv RP35R1,  0x0009
        .equiv RP35R2,  0x000A
        .equiv RP35R3,  0x000B
        .equiv RP35R4,  0x000C
        .equiv RP35R5,  0x000D


;----- RCON Bits -----------------------------------------------------
        .equiv POR,     0x0000
        .equiv BOR,     0x0001
        .equiv IDLE,    0x0002
        .equiv SLEEP,   0x0003
        .equiv WDTO,    0x0004
        .equiv SWDTEN,  0x0005
        .equiv SWR,     0x0006
        .equiv EXTR,    0x0007
        .equiv VREGS,   0x0008
        .equiv CM,      0x0009
        .equiv IOPUWR,  0x000E
        .equiv TRAPR,   0x000F


;----- OSCCON Bits -----------------------------------------------------
        .equiv OSWEN,   0x0000
        .equiv CF,      0x0003
        .equiv LOCK,    0x0005
        .equiv IOLOCK,  0x0006
        .equiv CLKLOCK, 0x0007

        .equiv NOSC0,   0x0008
        .equiv NOSC1,   0x0009
        .equiv NOSC2,   0x000A
        .equiv COSC0,   0x000C
        .equiv COSC1,   0x000D
        .equiv COSC2,   0x000E


;----- CLKDIV Bits -----------------------------------------------------
        .equiv DOZEN,   0x000B
        .equiv ROI,     0x000F

        .equiv PLLPRE0, 0x0000
        .equiv PLLPRE1, 0x0001
        .equiv PLLPRE2, 0x0002
        .equiv PLLPRE3, 0x0003
        .equiv PLLPRE4, 0x0004
        .equiv PLLPOST0, 0x0006
        .equiv PLLPOST1, 0x0007
        .equiv FRCDIV0, 0x0008
        .equiv FRCDIV1, 0x0009
        .equiv FRCDIV2, 0x000A
        .equiv DOZE0,   0x000C
        .equiv DOZE1,   0x000D
        .equiv DOZE2,   0x000E


;----- PLLFBD Bits -----------------------------------------------------
        .equiv PLLDIV0, 0x0000
        .equiv PLLDIV1, 0x0001
        .equiv PLLDIV2, 0x0002
        .equiv PLLDIV3, 0x0003
        .equiv PLLDIV4, 0x0004
        .equiv PLLDIV5, 0x0005
        .equiv PLLDIV6, 0x0006
        .equiv PLLDIV7, 0x0007
        .equiv PLLDIV8, 0x0008


;----- OSCTUN Bits -----------------------------------------------------
        .equiv TUN0,    0x0000
        .equiv TUN1,    0x0001
        .equiv TUN2,    0x0002
        .equiv TUN3,    0x0003
        .equiv TUN4,    0x0004
        .equiv TUN5,    0x0005


;----- REFOCON Bits -----------------------------------------------------
        .equiv ROSEL,   0x000C
        .equiv ROSIDL,  0x000D
        .equiv ROON,    0x000F

        .equiv RODIV0,  0x0008
        .equiv RODIV1,  0x0009
        .equiv RODIV2,  0x000A
        .equiv RODIV3,  0x000B
        .equiv ROSSLP,  0x000D


;----- ACLKCON Bits -----------------------------------------------------
        .equiv FRCSEL,  0x0006
        .equiv ASRCSEL, 0x0007
        .equiv SELACLK, 0x000D
        .equiv APLLCK,  0x000E
        .equiv ENAPLL,  0x000F

        .equiv APSTSCLR0, 0x0008
        .equiv APSTSCLR1, 0x0009
        .equiv APSTSCLR2, 0x000A


;----- NVMCON Bits -----------------------------------------------------
        .equiv ERASE,   0x0006
        .equiv WRERR,   0x000D
        .equiv WREN,    0x000E
        .equiv WR,      0x000F

        .equiv NVMOP0,  0x0000
        .equiv NVMOP1,  0x0001
        .equiv NVMOP2,  0x0002
        .equiv NVMOP3,  0x0003


;----- PMD1 Bits -----------------------------------------------------
        .equiv ADCMD,   0x0000
        .equiv SPI1MD,  0x0003
        .equiv U1MD,    0x0005
        .equiv I2C1MD,  0x0007
        .equiv PWMMD,   0x0009
        .equiv T1MD,    0x000B
        .equiv T2MD,    0x000C


;----- PMD2 Bits -----------------------------------------------------
        .equiv OC1MD,   0x0000


;----- PMD4 Bits -----------------------------------------------------
        .equiv REFOMD,  0x0003


;----- PMD6 Bits -----------------------------------------------------
        .equiv PWM1MD,  0x0008
        .equiv PWM4MD,  0x000B




;  Some useful macros for PWRSAV instructions

   .equiv SLEEP_MODE, 0x0
   .equiv IDLE_MODE, 0x1


;==========================================================================
;
;   Configuration Words
;
;   Configuration words exist in Program Space and their locations are
;   defined in the device linker script. They can be set in source code
;   or in the MPLAB IDE. Each configuration word should be specified
;   only once (multiple settings may be combined.)
;
;==========================================================================
;
;  Setting configuration words using macros:
;
;  The following macro named 'config' can be used to set configuration words:
;
        .macro config REG, VALUE
        .pushsection \REG.sec, code
        .global \REG
\REG:   .pword \VALUE
        .popsection
        .endm
;
;  For example, to set CONFIG_REG using the macro above, the following line
;  can be pasted only at the beginning of the assembly source code,
;  immediately below the '.include' directive.
;
;        config __CONFIG_REG, SETTING_A & SETTING_B
;
;  Note that the 'config' macro takes two arguments: the first is the name
;  of a configuration word (in this case, __CONFIG_REG), the second argument
;  is a boolean expression that may include multiple settings.
;  The example above would enable SETTING_A and also SETTING_B.
;
;  A description of all possible settings for each configuration word
;  appears below.
;
;==========================================================================
;
;  Setting configuration words without using macros:
;
;  To set configuration words without using the 'config' macro,
;  use the following format:
;
;           .section __CONFIG_REG.sec, code
;           .global __CONFIG_REG
;__CONFIG_REG: .pword SETTING_A & SETTING_B
;
;  This source code is equivalent to the expanded macro from the previous
;  example.
;
;==========================================================================


;----- FICD (0xff0) --------------------------------------------------
;
;  The following settings are available for FICD:
;
;   ICD Communication Channel Select bits:
;     ICS_NONE             Reserved, do not use
;     ICS_PGD3             Communicate on PGEC3 and PGED3
;     ICS_PGD2             Communicate on PGEC2 and PGED2
;     ICS_PGD1             Communicate on PGEC1 and PGED1
;
;   JTAG Enable bit:
;     JTAGEN_OFF           JTAG is disabled
;     JTAGEN_ON            JTAG is enabled
;
        .equiv ICS_NONE,        0xFFFC ; Reserved, do not use
        .equiv ICS_PGD3,        0xFFFD ; Communicate on PGEC3 and PGED3
        .equiv ICS_PGD2,        0xFFFE ; Communicate on PGEC2 and PGED2
        .equiv ICS_PGD1,        0xFFFF ; Communicate on PGEC1 and PGED1

        .equiv JTAGEN_OFF,      0xFFDF ; JTAG is disabled
        .equiv JTAGEN_ON,       0xFFFF ; JTAG is enabled


;----- FWDT (0xff4) --------------------------------------------------
;
;  The following settings are available for FWDT:
;
;   Watchdog Timer Postscaler bits:
;     WDTPOST_PS1          1:1
;     WDTPOST_PS2          1:2
;     WDTPOST_PS4          1:4
;     WDTPOST_PS8          1:8
;     WDTPOST_PS16         1:16
;     WDTPOST_PS32         1:32
;     WDTPOST_PS64         1:64
;     WDTPOST_PS128        1:128
;     WDTPOST_PS256        1:256
;     WDTPOST_PS512        1:512
;     WDTPOST_PS1024       1:1,024
;     WDTPOST_PS2048       1:2,048
;     WDTPOST_PS4096       1:4,096
;     WDTPOST_PS8192       1:8,192
;     WDTPOST_PS16384      1:16,384
;     WDTPOST_PS32768      1:32,768
;
;   Watchdog Timer Prescaler bit:
;     WDTPRE_PR32          1:32
;     WDTPRE_PR128         1:128
;
;   PLL Lock Enable bit:
;     PLLKEN_OFF           Clock switch will not wait for the PLL lock signal.
;     PLLKEN_ON            Clock switch to PLL source will wait until the PLL lock signal is valid.
;
;   Watchdog Timer Enable bit:
;     FWDTEN_OFF           Watchdog timer enabled/disabled by user software
;     FWDTEN_ON            Watchdog timer always enabled
;
        .equiv WDTPOST_PS1,     0xFFF0 ; 1:1
        .equiv WDTPOST_PS2,     0xFFF1 ; 1:2
        .equiv WDTPOST_PS4,     0xFFF2 ; 1:4
        .equiv WDTPOST_PS8,     0xFFF3 ; 1:8
        .equiv WDTPOST_PS16,    0xFFF4 ; 1:16
        .equiv WDTPOST_PS32,    0xFFF5 ; 1:32
        .equiv WDTPOST_PS64,    0xFFF6 ; 1:64
        .equiv WDTPOST_PS128,   0xFFF7 ; 1:128
        .equiv WDTPOST_PS256,   0xFFF8 ; 1:256
        .equiv WDTPOST_PS512,   0xFFF9 ; 1:512
        .equiv WDTPOST_PS1024,  0xFFFA ; 1:1,024
        .equiv WDTPOST_PS2048,  0xFFFB ; 1:2,048
        .equiv WDTPOST_PS4096,  0xFFFC ; 1:4,096
        .equiv WDTPOST_PS8192,  0xFFFD ; 1:8,192
        .equiv WDTPOST_PS16384, 0xFFFE ; 1:16,384
        .equiv WDTPOST_PS32768, 0xFFFF ; 1:32,768

        .equiv WDTPRE_PR32,     0xFFEF ; 1:32
        .equiv WDTPRE_PR128,    0xFFFF ; 1:128

        .equiv PLLKEN_OFF,      0xFFDF ; Clock switch will not wait for the PLL lock signal.
        .equiv PLLKEN_ON,       0xFFFF ; Clock switch to PLL source will wait until the PLL lock signal is valid.

        .equiv FWDTEN_OFF,      0xFF7F ; Watchdog timer enabled/disabled by user software
        .equiv FWDTEN_ON,       0xFFFF ; Watchdog timer always enabled


;----- FOSC (0xff6) --------------------------------------------------
;
;  The following settings are available for FOSC:
;
;   Primary Oscillator Mode Select bits:
;     POSCMD_EC            EC (External Clock) Mode
;     POSCMD_XT            XT Crystal Oscillator Mode
;     POSCMD_HS            HS Crystal Oscillator Mode
;     POSCMD_NONE          Primary Oscillator disabled
;
;   OSC2 Pin Function bit:
;     OSCIOFNC_ON          OSC2 is general purpose digital I/O pin
;     OSCIOFNC_OFF         OSC2 is clock output
;
;   Peripheral pin select configuration:
;     IOL1WAY_OFF          Allow multiple reconfigurations
;     IOL1WAY_ON           Allow only one reconfiguration
;
;   Clock Switching Mode bits:
;     FCKSM_CSECME         Both Clock switching and Fail-safe Clock Monitor are enabled
;     FCKSM_CSECMD         Clock switching is enabled,Fail-safe Clock Monitor is disabled
;     FCKSM_CSDCMD         Both Clock switching and Fail-safe Clock Monitor are disabled
;
        .equiv POSCMD_EC,       0xFFFC ; EC (External Clock) Mode
        .equiv POSCMD_XT,       0xFFFD ; XT Crystal Oscillator Mode
        .equiv POSCMD_HS,       0xFFFE ; HS Crystal Oscillator Mode
        .equiv POSCMD_NONE,     0xFFFF ; Primary Oscillator disabled

        .equiv OSCIOFNC_ON,     0xFFFB ; OSC2 is general purpose digital I/O pin
        .equiv OSCIOFNC_OFF,    0xFFFF ; OSC2 is clock output

        .equiv IOL1WAY_OFF,     0xFFDF ; Allow multiple reconfigurations
        .equiv IOL1WAY_ON,      0xFFFF ; Allow only one reconfiguration

        .equiv FCKSM_CSECME,    0xFF3F ; Both Clock switching and Fail-safe Clock Monitor are enabled
        .equiv FCKSM_CSECMD,    0xFF7F ; Clock switching is enabled,Fail-safe Clock Monitor is disabled
        .equiv FCKSM_CSDCMD,    0xFFBF ; Both Clock switching and Fail-safe Clock Monitor are disabled


;----- FOSCSEL (0xff8) --------------------------------------------------
;
;  The following settings are available for FOSCSEL:
;
;   Oscillator Source Selection:
;     FNOSC_FRC            Internal Fast RC (FRC)
;     FNOSC_FRCPLL         Fast RC Oscillator with divide-by-N with PLL module (FRCPLL)
;     FNOSC_PRI            Primary Oscillator (XT, HS, EC)
;     FNOSC_PRIPLL         Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
;     FNOSC_LPRC           Low-Power RC Oscillator (LPRC)
;     FNOSC_FRCDIVN        Internal Fast RC (FRC) Oscillator with postscaler
;
;   Two-speed Oscillator Start-up Enable bit:
;     IESO_OFF             Start up with user-selected oscillator source
;     IESO_ON              Start up device with FRC, then switch to user-selected oscillator source
;
        .equiv FNOSC_FRC,       0xFFF8 ; Internal Fast RC (FRC)
        .equiv FNOSC_FRCPLL,    0xFFF9 ; Fast RC Oscillator with divide-by-N with PLL module (FRCPLL)
        .equiv FNOSC_PRI,       0xFFFA ; Primary Oscillator (XT, HS, EC)
        .equiv FNOSC_PRIPLL,    0xFFFB ; Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
        .equiv FNOSC_LPRC,      0xFFFD ; Low-Power RC Oscillator (LPRC)
        .equiv FNOSC_FRCDIVN,   0xFFFF ; Internal Fast RC (FRC) Oscillator with postscaler

        .equiv IESO_OFF,        0xFF7F ; Start up with user-selected oscillator source
        .equiv IESO_ON,         0xFFFF ; Start up device with FRC, then switch to user-selected oscillator source


;----- FGS (0xffa) --------------------------------------------------
;
;  The following settings are available for FGS:
;
;   General Segment Write-Protect bit:
;     GWRP_ON              General Segment is write protected
;     GWRP_OFF             General Segment may be written
;
;   General Segment Code-Protect bit:
;     GCP_ON               General Segment Code protection Enabled
;     GCP_OFF              General Segment Code protect is Disabled
;
        .equiv GWRP_ON,         0xFFFE ; General Segment is write protected
        .equiv GWRP_OFF,        0xFFFF ; General Segment may be written

        .equiv GCP_ON,          0xFFFD ; General Segment Code protection Enabled
        .equiv GCP_OFF,         0xFFFF ; General Segment Code protect is Disabled


.LIST
