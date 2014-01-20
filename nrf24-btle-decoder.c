/*
The MIT License (MIT)

Copyright (c) 2014 Omri Iluz (omri@il.uz / http://cyberexplorer.me)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

CREDITS:
Dmitry Grinberg, CRC and Whiten code for BTLE - http://goo.gl/G9m8Ud
Open Source Mobile Communication, RTL-SDR information - http://sdr.osmocom.org/trac/wiki/rtl-sdr
Steve Markgraf, RTL-SDR Library - https://github.com/steve-m/librtlsdr


*/

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>  
#include <math.h>
#include <stdbool.h>
#include <inttypes.h>
#if defined(WIN32)
	#include <io.h>
	#include <fcntl.h>
#endif /* defined(WIN32) */
#include <stdlib.h> // For exit function

uint8_t inline SwapBits(uint8_t a){

	uint8_t v = 0;
	
	if(a & 0x80) v |= 0x01;
	if(a & 0x40) v |= 0x02;
	if(a & 0x20) v |= 0x04;
	if(a & 0x10) v |= 0x08;
	if(a & 0x08) v |= 0x10;
	if(a & 0x04) v |= 0x20;
	if(a & 0x02) v |= 0x40;
	if(a & 0x01) v |= 0x80;

	return v;
}


uint32_t NRFCrc(const uint8_t* data, size_t data_len)
{
    uint8_t i;
    bool bit;
    uint8_t c;
	uint_fast16_t crc=0x3C18;
    while (data_len--) {
        c = *data++;
        for (i = 0x80; i > 0; i >>= 1) {
            bit = crc & 0x8000;
            if (c & i) {
                bit = !bit;
            }
            crc <<= 1;
            if (bit) {
                crc ^= 0x1021;
            }
        }
        crc &= 0xffff;
    }
    return (uint16_t)(crc & 0xffff);
}

uint32_t BTLECrc(const uint8_t* data, uint8_t len, uint8_t* dst){

	uint8_t v, t, d;
	uint32_t crc=0;
	while(len--){
	
		d = SwapBits(*data++);
		for(v = 0; v < 8; v++, d >>= 1){
		
			t = dst[0] >> 7;
			
			dst[0] <<= 1;
			if(dst[1] & 0x80) dst[0] |= 1;
			dst[1] <<= 1;
			if(dst[2] & 0x80) dst[1] |= 1;
			dst[2] <<= 1;
			
		
			if(t != (d & 1)){
			
				dst[2] ^= 0x5B;
				dst[1] ^= 0x06;
			}
		}	
	}
	for (v=0;v<3;v++) crc=(crc<<8)|dst[v];
	return crc;
}


void BTLEWhiten(uint8_t* data, uint8_t len, uint8_t chan){

	uint8_t  i;	
	uint8_t lfsr = SwapBits(chan) | 2;
	while(len--){
		for(i = 0x80; i; i >>= 1){
		
			if(lfsr & 0x80){
				
				lfsr ^= 0x11;
				(*data) ^= i;
			}
			lfsr <<= 1;
		}
		data++;
	}
}

/* Ring Buffer */
#define RB_SIZE 1000
int rb_head=-1;
int16_t *rb_buf;
/* Init Ring Buffer */
void RB_init(void){
	rb_buf = (int16_t *)malloc(RB_SIZE*2);	
}
/* increment Ring Buffer Head */
void RB_inc(void){
	rb_head++;
	rb_head=(rb_head)%RB_SIZE;
}
/* Access Ring Buffer location l */
#define RB(l) rb_buf[(rb_head+(l))%RB_SIZE]
/* end Ring Buffer */

/* Global variables */
int32_t g_threshold; // Quantization threshold
int g_srate; // sample rate downconvert ratio

/* helper functions */
/* Quantize sample at location l by checking whether it's over the threshold */
/* Important - takes into account the sample rate downconversion ratio */
inline bool Quantize(l){
	return RB(l*g_srate) > g_threshold;
}
#define Q(l) Quantize(l) 

/* Extract quantization threshold from preamble sequence */
int32_t ExtractThreshold(void){		
	int32_t threshold=0;
	int c;
	for (c=0;c<8*g_srate;c++){
		threshold+=(int32_t)RB(c);			
	}	
	return (int32_t)threshold/(8*g_srate);
}	

/* Identify preamble sequence */
bool DetectPreamble(void){
	int transitions=0;
	int c;

	/* preamble sequence is based on the 9th symbol (either 0x55555555 or 0xAAAAAAAA) */
	if (Q(9)){
		for (c=0;c<8;c++){
			transitions += Q(c)>Q(c+1);
		}
	} else {
		for (c=0;c<8;c++){
			transitions += Q(c)<Q(c+1);
		}
	}
	return transitions==4 && abs(g_threshold)<15500;
}

/* Extract byte from ring buffer starting location l */
uint8_t inline ExtractByte(int l){
	uint8_t byte=0;
	int c;
	for (c=0;c<8;c++) byte |= Q(l+c)<<(7-c);
	return byte;
}

/* Extract count bytes from ring buffer starting location l into buffer*/
void inline ExtractBytes(int l, uint8_t* buffer, int count){
	int t;
	for (t=0;t<count;t++){
		buffer[t]=ExtractByte(l+t*8);
	}
}

/* Prepare packed bit stream for CRC calculation */
void PackPacket(uint64_t packet_addr_l, uint16_t pcf, uint8_t* packet_data, int packet_length, uint8_t* packet_packed){
	int c;
	uint64_t packet_header=packet_addr_l;
	packet_header<<=9;
	packet_header|=pcf;
	for (c=0;c<7;c++){
		packet_packed[c]=(packet_header>>((6-c)*8))&0xFF;
	}
	
	for (c=0;c<packet_length;c++){
		packet_packed[c+7]=packet_data[c];
	}
}

bool DecodeBTLEPacket(int32_t sample, int srate){
	int c;
	struct timeval tv;
	uint8_t packet_data[500];
	int packet_length;
	uint32_t packet_crc;
	uint32_t calced_crc;
	uint64_t packet_addr_l;
	uint8_t crc[3];
	uint8_t packet_header_arr[2];

	g_srate=srate;

	/* extract address */
	packet_addr_l=0;
	for (c=0;c<4;c++) packet_addr_l|=((uint64_t)SwapBits(ExtractByte((c+1)*8)))<<(8*c);
	
	/* extract pdu header */
	ExtractBytes(5*8, packet_header_arr, 2);
	
	/* whiten header only so we can extract pdu length */
	BTLEWhiten(packet_header_arr, 2, 38);
	
	if (packet_addr_l==0x8E89BED6){  // Advertisement packet
		packet_length=SwapBits(packet_header_arr[1])&0x3F;
	} else {
		packet_length=0;			// TODO: data packets unsupported		

	}

	/* extract and whiten pdu+crc */
	ExtractBytes(5*8, packet_data, packet_length+2+3);			
	BTLEWhiten(packet_data, packet_length+2+3, 38);
	
	if (packet_addr_l==0x8E89BED6){  // Advertisement packet
		crc[0]=crc[1]=crc[2]=0x55;
	} else {
		crc[0]=crc[1]=crc[2]=0;		// TODO: data packets unsupported			
	}

	/* calculate packet crc */
	calced_crc=BTLECrc(packet_data, packet_length+2, crc);					
	packet_crc=0;
	for (c=0;c<3;c++) packet_crc=(packet_crc<<8)|packet_data[packet_length+2+c];
	
	/* BTLE packet found, dump information */
	if (packet_crc==calced_crc){
		gettimeofday(&tv, NULL);
		printf("%ld.%06ld ", (long)tv.tv_sec, tv.tv_usec);
		printf("BTLE Packet start sample %"PRId32", Threshold:%"PRId32", Address: 0x%08"PRIX64", CRC:0x%06X ",sample,g_threshold,packet_addr_l, packet_crc);
		printf("length:%d data:",packet_length);
		for (c=0;c<packet_length+2;c++) printf("%02X ",SwapBits(packet_data[c]));														
		printf("\n");
		return true;
	} else return false;
}

bool DecodeNRFPacket(int32_t sample, int srate){
	int c,t;
	struct timeval tv;
	uint8_t tmp_buf[10];
	uint8_t packet_data[500];
	uint8_t packet_packed[50];
	int packet_length;
	uint16_t pcf;
	uint32_t packet_crc;
	uint32_t calced_crc;
	uint64_t packet_addr_l;

	g_srate=srate;

/* extract address */
	packet_addr_l=0;
	ExtractBytes(1*8, tmp_buf, 5);
	for (t=0;t<5;t++) packet_addr_l|=((uint64_t)tmp_buf[t])<<(4-t)*8;
	
	/* extract pcf */
	ExtractBytes(6*8, tmp_buf, 2);
	pcf = tmp_buf[0]<<8 | tmp_buf[1];
	pcf >>=7;

	/* extract packet length, avoid excessive length packets */
	packet_length=(int)pcf>>3;
	if (packet_length>32)packet_length=0;

	/* extract data */
	ExtractBytes(6*8+9, packet_data, packet_length);

	/* Prepare packed bit stream for CRC calculation */
	PackPacket(packet_addr_l, pcf, packet_data, packet_length, packet_packed);
	
	/* calculate packet crc */
	calced_crc=NRFCrc(packet_packed, 7+packet_length);

	/* extract crc */
	ExtractBytes((6+packet_length)*8+9, tmp_buf, 2);
	packet_crc = tmp_buf[0]<<8 | tmp_buf[1];

	/* NRF24L01+ packet found, dump information */
	if (packet_crc==calced_crc){
		gettimeofday(&tv, NULL);
		printf("%ld.%06ld ", (long)tv.tv_sec, tv.tv_usec);		
		printf("NRF24 Packet start sample %"PRId32", Threshold:%"PRId32", Address: 0x%08"PRIX64" ",sample,g_threshold,packet_addr_l);
		printf("length:%d, pid:%d, no_ack:%d, CRC:0x%04X data:",packet_length,(pcf&0b110)>>1,pcf&0b1,packet_crc);
		for (c=0;c<packet_length;c++) printf("%02X ",packet_data[c]);
		printf("\n");
		return true;
	} else return false;
}


bool DecodePacket(int32_t sample, int srate){
	bool packet_detected=false;
	g_srate=srate;
	g_threshold = ExtractThreshold();

	if (DetectPreamble()){
		// btle
		if (1){
			packet_detected |= DecodeBTLEPacket(sample, srate);
		}
		//NRF24
		if (1){
			packet_detected |= DecodeNRFPacket(sample, srate);
		}
	}
	return packet_detected;
}

void usage(void)
{
	fprintf(stderr,
		"nrf24-btle-decoder, decode NRF24L01+ and Bluetooth Low Energy packets using RTL-SDR\n\n"
		"Use:\tnrf24-btle-decoder -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t    use multiple -f for scanning (requires squelch)\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-d device_index (default: 0)]\n"

	exit(1);
}

int main (int argc, char**argv){	
	int16_t cursamp;
	int32_t samples=0;
	int skipSamples;

	#if defined(WIN32)
		_setmode(_fileno(stdin), _O_BINARY);
	#endif /* defined(WIN32) */

	printf("NRF24L01+ and Bluetooth Low Energy decoder v0.4\n\n");
while ((opt = getopt(argc, argv, "d:f:g:s:b:l:o:t:r:p:E:F:A:M:h")) != -1) {
		switch (opt) {
		case 'd':
			//dongle.dev_index = verbose_device_search(optarg);
			//dev_given = 1;
			break;
		default:
			usage();
			break;
		}
	}
	RB_init();
	g_threshold = 0;

	skipSamples=1000;
	time_t start_time = time(NULL);
	
	while(!feof(stdin) ) {
		cursamp  = (int16_t) ( fgetc(stdin) | fgetc(stdin)<<8);
		RB_inc();
		RB(0)=(int)cursamp;
		if (--skipSamples<1)if (DecodePacket(++samples,2)) skipSamples=20;
	}

	printf("%"PRId32" samples received in %d seconds \n",samples,(int)(time(NULL)-start_time));
	return 0;
}
