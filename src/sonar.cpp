#include "sonar.h"
//g++ sonar.cpp logger.cpp -I ../include -std=c++11
/*
Copyright (c) <2015>, <University of Pennsylvania:GRASP Lab>                                                             
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the university of pennsylvania nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL UNIVERSITY OF PENNSYLVANIA  BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

Sonar::Sonar(std::string PATH2SONAR)
{
	//(this->data_size) = DATASIZE; //including first and last checkbytes
	open_port(PATH2SONAR, 1);
}
int Sonar::open_port(std::string PATH2SONAR, int bytes_to_block)
{
	printf("Opening an sonar USB port...   ");
	struct termios newtio;
	int port;
	port = open(PATH2SONAR.c_str(), O_RDWR | O_NOCTTY);

	tcgetattr(port, &newtio);
	cfsetospeed(&newtio, BAUDRATE_SONAR);
	cfsetispeed(&newtio, BAUDRATE_SONAR);

	//set the number of data bits.
	newtio.c_cflag &= ~CSIZE;  // Mask the character size bits
	newtio.c_cflag |= CS8;

	//set the number of stop bits to 1
	newtio.c_cflag &= ~CSTOPB;

	//Set parity to None
	newtio.c_cflag &=~PARENB;

	//set for non-canonical (raw processing, no echo, etc.)
	newtio.c_iflag = IGNPAR; // ignore parity check close_port
	newtio.c_oflag = 0; // raw output
	newtio.c_lflag = 0; // raw input

	//Time-Outs -- won't work with NDELAY option in the call to open
	newtio.c_cc[VMIN]  = bytes_to_block;   // block reading until RX x characers. If x = 0, it is non-blocking.
	newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

	//Set local mode and enable the receiver
	newtio.c_cflag |= (CLOCAL | CREAD);

	//tcflush(port, TCIFLUSH);
	//Set the new options for the port...
	int status=tcsetattr(port, TCSANOW, &newtio);
	if (status != 0) 
	{
		printf("Configuring SONAR comport failed\n");
		
	}
	
	(this->port) = port;
	(this->PATH2SONAR) = PATH2SONAR;
	clean();

	if (port < 0) 
	{
		printf("\n Error opening USB port: %i !!", port);
		//open_port("/dev/ttyUSB1", 1);
        }
	 else   printf("Done!\n");
	printf("Port Name: %i \n", port);

	return port;
}
int Sonar::get_sonar_data(void)
{
//the sensor outputs 6 bytes at a time. The first byte is 82 in decimal, the next 4 bytes are the the constituent parts of a 32 bit int form Most Significant Byte to Least Significant Byte, the 6th byte is 13 in decimal combine four unsigned chars into one 32 bit int
	
//select: clear read_ds(set of file_descriptors to watch for reading), add port to this set, set time to 0 to returnimmediately,
    // select returns the number of fd's ready
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;
   

    return check_first_read();
}
int Sonar::get_sonar_data(SonarTest& s)
{
    // select returns the number of fd's ready
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;
   
    return check_first_read_test(s);
}

SonarTest Sonar::get_stats(void)
{
SonarTest s = {0};
//printf("Not Ready to Read Count:  %ld, Ready to Read Count: %ld \n\n",  (this->num_fds_0), (this->num_fds_1));

s.dt   		 = this->calc_dt;
s.index 	 = this->index;
s.distance       = this->returnLastDistance();
s.foundFirstByte = this->foundFirstByte;
s.num_fds_n1	 = this->num_fds_n1;
s.num_fds_0	 = this->num_fds_0;
s.num_fds_1	 = this->num_fds_1;
s.num_fds_p	 = this->num_fds_p;
s.lastByte	 = this->lastByte;
return s;
}
int Sonar::check_first_read_test(SonarTest& s)
{
//Once we find start byte, we begin filling in array byte-by-byte until we get 4 bytes. We then confirm that the next byte is the last byte, if it is we then cast the array into a 32 bit integer and return. If the last byte is incorrect, we dicard array, reset index, and continue searching for start byte again.
    int num_fds = select(port+1, &read_fds, NULL, NULL, &no_timeout); 
   	int returnVal= -10;
   s.num_fds = num_fds;
   if(num_fds == -1)      (this->num_fds_n1)++;
   else if (num_fds == 0) (this->num_fds_0)++;
   else if (num_fds == 1) (this->num_fds_1)++; 
   else if (num_fds > 1)  (this->num_fds_p)++;
  //No data ready to read
    if(num_fds == 0)  
    {
	s.succ_read = -1;
	returnVal= -1;
	return -1;
    }
    //select returned an error
    else if(num_fds ==-1)
    {
	s.succ_read=  -2;
	returnVal= -2;

	return -2;
    }

    //A file descriptor is ready to read: check which one
    //else if(FD_ISSET((this->port) , &read_fds))
      else if(num_fds ==1)   
	{
                char sensor_byte[1] = {255}; 

                lseek(port, -(this->data_size), SEEK_END);
		
     		int result = read(port, &sensor_byte[0], 1);
		char a = sensor_byte[0];
		(this->lastByte) = a;
				if(result == 0) 
		{
			printf("read 0 bytes \n");
			clean();
			returnVal = -3;
		}
		//finished, found all bytes
		else if( (foundFirstByte == true) && (index == 4) )
		{
			//printf("		number of bytes read: %i, index: %i, byte: %d \n", result, index, a);


			if(sensor_byte[0] == 13) 
			{	
				uint32_t range_char = 0;
				sscanf(&dist[0], "%d", &range_char);
				//printf("read last byte 13 with index = 4 \n");
				//printf("					Distance: %i \n", range_char);
				clean();
				calcDt();
				(this->last_distance) = range_char;
				returnVal = range_char;
		
			}
			else
			{
				//printf("read last byte NOT 13 with index = 4\n");
				tcflush(port, TCIFLUSH);
				clean();
				returnVal = -4;
			}		
		}
		
		//still collecting bytes, add to array, increment index 
		else if(foundFirstByte == true)
		{
			//printf("		adding to array & index++\n");
			dist[index] = sensor_byte[0];
			index++;
			returnVal = -5; 
		}
		//havent found first or last byte, check each byte to see if this is first
		else if(sensor_byte[0] == 82)
		{
			//printf("found first byte\n");
			foundFirstByte = true;
			index = 0;
			returnVal = -6;
		}
		else 
		{
			returnVal = -7;
			clean();
		}

		s.succ_read = returnVal;
		s.lastByte = sensor_byte[0];
		s.distance = returnVal;
		s.index = index;
		s.foundFirstByte = foundFirstByte;
		s.foundLastByte  = foundLastByte;

		
		
		return returnVal;
	}
		clean();
	
		return -8;
}
int Sonar::check_first_read(void)
{
//Once we find start byte, we begin filling in array byte-by-byte until we get 4 bytes. We then confirm that the next byte is the last byte, if it is we then cast the array into a 32 bit integer and return. If the last byte is incorrect, we dicard array, reset index, and continue searching for start byte again.
    int num_fds = select(port+1, &read_fds, NULL, NULL, &no_timeout); 
    
    //No data ready to read
    if(num_fds == 0)  return -2;

    //select returned an error
    else if(num_fds ==-1)  return  -3;

    //A file descriptor is ready to read: check which one
    else if(FD_ISSET((this->port) , &read_fds))
         {

		int returnVal;
	
	//	cout << "in check_first_read succ!!!! \n";
                char sensor_byte[1] = {255}; 

                lseek(port, -(this->data_size), SEEK_END);
		
                //read in 1 byte of data from port to the address in memory &sensor_bytes2 result indicates success of reading
		//read is guarunteed not to block! but can still return 0 bytes!!!!
     		int result = read(port, &sensor_byte[0], 1);
		//printf("%x \n",sensor_byte[0]);
		char a = sensor_byte[0];
		std::cout << "result: " << result << ",  "<< hex <<  a << std::endl;
		//printf("FirstByteFound %i, index %i, New byte is %d result of read: %i\n", foundFirstByte, index, sensor_byte[0], result);
		if(result == 0) 
		{
			cout << "NO BYTES \n";
			printf("Failed read: read returns no (0) bytes! Might be disconnected\n\n");
			clean();
			returnVal = -1;
		}
		//cout << "Index: " << index << "First Byte Flag: " << foundFirstByte << endl;
		//finished, found all bytes
		if( (foundFirstByte == true) && (index == 4) )
		{
			cout << "FB true and index 4 \n";
			
			//check to make sure last byte correct then return distance value
			//ensure to clean up array after
			if(sensor_byte[0] == 13) 
			{
				cout << "LAST BYTE CORRECT!!!!!!!! \n";
				uint32_t range_char = 0;
				sscanf(&dist[0], "%d", &range_char);
				printf("Range: %d \n\n", range_char);
				cout << range_char << endl;
				//print_raw_bytes(dist, 4);
				//uint32_t range_char_copy = range_char;
				clean();
				//printf("1\n");
				calcDt();
				(this->last_distance) = (int) range_char;
				//printf("frequency of succ read: %f \n", 1/calc_dt); 
				returnVal = range_char;
			}
			else
			{
				cout << "last byte incorrect! \n";
				printf("Last byte wrong: byte is %d should be 13. index is %i \n", sensor_byte[0], index);
				tcflush(port, TCIFLUSH);
				clean();
				//printf("2\n"); 
				returnVal = -4;
			}		
		}
		
		//still collecting bytes, add to array, increment index 
		else if(foundFirstByte == true)
		{
			dist[index] = sensor_byte[0];
			index++;
			//printf("3\n"); 
			returnVal = -1; 
		}
		//havent found first or last byte, check each byte to see if this is first
		else if(sensor_byte[0] == 82)
		{
			foundFirstByte = true;
			index = 0;
			//printf("4\n"); 
			returnVal = -1;
		}
		//cout << "returnVal: " << returnVal << endl;
		return returnVal;
	}
		//printf("5\n"); 
		clean();
		//open_port("/dev/ttyUSB0", 1);
	
		return -1;
}


void Sonar::clean(void)
{
	index = 0;

	foundFirstByte = false;
	foundLastByte = false;
}
void Sonar::print_raw_bytes(const char arr[], int arr_length)
{
        printf("\nBytes: \n");
        for(int i = 0; i < arr_length; i++) 
	{
		printf("%i : %d ,", i, arr[i]); //NOTE: PRINTING HEX REPRES OF CHAR, NEED TO PRINT DECIMAL
        	//if(arr[i] == 82) printf("FOUND 82 in %i th position \n",i);
	}
	printf("Location of first byte: %i", indexOfStartByte(arr, arr_length));

	printf("\n\n\n");
}
void Sonar::calcDt(void){
		//track dt between reads
                clock_gettime(CLOCK_REALTIME,&newT);
                (this->calc_dt) = UTILITY::timespec2float(UTILITY::time_diff(oldT, newT));
                clock_gettime(CLOCK_REALTIME,&oldT);
}
int Sonar::indexOfStartByte(const char arr[], int arr_length)
{
		//iterate through array and find which index, starting at zero is the first (82)
		int i;
		for(int i = 0; i < arr_length; i++)	if(arr[i] == 82) return i;
		return -1;
}
float Sonar::getDt(void)
{
return this->calc_dt;
}
int Sonar::returnLastDistance(void)
{
return this->last_distance;
}

void Sonar::printStats(void)
{
 
printf("Not Ready to Read Count:  %ld, Ready to Read Count: %ld \n\n",  (this->num_fds_0), (this->num_fds_1));
}
/*
int main()
{
	long positive = 0, one = 0, two = 0, three = 0, four = 0, five = 0, six = 0, seven = 0, eight = 0;
	long num_fds_n1 = 0, num_fds_0 = 0, num_fds_1 = 0, num_fds_p = 0;
	Sonar sonar("/dev/ttyUSB0");
	
	logger logger("file.log", 100, true);
	
	while(1)
	{

	SonarTest s = {0};
	int dist_mm = sonar.get_sonar_data(s);
 	 if(s.succ_read == -1) one++;
	 if(s.succ_read == -2) two++;
	 if(s.succ_read == -3) three++;
	 if(s.succ_read == -4) four++;
	 if(s.succ_read == -5) five++;
	 if(s.succ_read == -6) six++;
	 if(s.succ_read == -7) seven++;
	 if(s.succ_read == -8) eight++;
	 if(s.succ_read > 0) positive++;
	 if(s.num_fds == -1) num_fds_n1++;
	 if(s.num_fds == 0) num_fds_0++;
	 if(s.num_fds == 1) num_fds_1++;
	 if(s.num_fds > 1) num_fds_p++;

	sonar.printStats();
	Data_log d = {0};
	d.sonar_test = sonar.get_stats();
	printf("num_fds: \n -1: %ld, 0: %ld, 1: %ld, other pos: %ld \n", d.sonar_test.num_fds_n1, d.sonar_test.num_fds_0, d.sonar_test.num_fds_1, d.sonar_test.num_fds_p);
	logger.log(d);
	
	//printf("-1: %ld, -2: %ld, -3: %ld, -4: %ld, -5: %ld, -6: %ld, -7: %ld, -8: %ld,  positive: %ld\n", one, two, three, four, five, six, seven, eight, positive);
	printf("num_fds: \n -1: %ld, 0: %ld, 1: %ld, other pos: %ld \n\n", num_fds_n1, num_fds_0, num_fds_1, num_fds_p);
	printf("succ_read: %i, dt: %f, index: %i, distance: %i, foundFirstByte: %i, byte_read: %d \n", s.succ_read,1/sonar.getDt(), s.index, s.distance, s.foundFirstByte, s.lastByte);

//
//	int dist_mm = sonar.get_sonar_data();
//	printf("distance (mm): %i , %i, dt: %f positive: %ld \n", dist_mm, sonar.returnLastDistance(), 1/sonar.getDt(), positive);
//	if(dist_mm > 0) 
//		{
//			positive++;
//			//printf("distance (mm): %i , %i \n", dist_mm, sonar.returnLastDistance());
//		}

	}
	

}
*/
