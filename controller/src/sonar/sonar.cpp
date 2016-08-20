#include "sonar.h"
//g++ sonar.cpp -I ../../include

Sonar::Sonar(std::string PATH2SONAR)
{
	//(this->data_size) = DATASIZE; //including first and last checkbytes

	open_port(PATH2SONAR, 1);
}
int Sonar::open_port(std::string PATH2SONAR, int bytes_to_block)
{
	printf("Opening an USB port...   ");
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
		printf("Configuring comport failed\n");
		
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
                char sensor_byte[1] = {255}; 

                lseek(port, -(this->data_size), SEEK_END);
		
                //read in 1 byte of data from port to the address in memory &sensor_bytes2 result indicates success of reading
		//read is guarunteed not to block! but can still return 0 bytes!!!!
     		int result = read(port, &sensor_byte[0], 1);
		
		//printf("FirstByteFound %i, index %i, New byte is %d result of read: %i\n", foundFirstByte, index, sensor_byte[0], result);
		if(result == 0) 
		{
			printf("Failed read: read returns no (0) bytes! Might be disconnected\n\n");
			clean();
			return -1;
		}

		//finished, found all bytes
		if( (foundFirstByte == true) && (index == 4) )
		{
			//check to make sure last byte correct then return distance value
			//ensure to clean up array after
			if(sensor_byte[0] == 13) 
			{
				uint32_t range_char = 0;
				sscanf(&dist[0], "%d", &range_char);
				//printf("Range: %d \n\n", range_char);
				//print_raw_bytes(dist, 4);
				//uint32_t range_char_copy = range_char;
				clean();
				//printf("1\n");
				calcDt();
				//printf("frequency of succ read: %f \n", 1/calc_dt); 
				return range_char;
			}
			else
			{
				printf("Last byte wrong: byte is %d should be 13. index is %i \n", sensor_byte[0], index);
				tcflush(port, TCIFLUSH);
				clean();
				//printf("2\n"); 
				return -4;
			}		
		}
		
		//still collecting bytes, add to array, increment index 
		else if(foundFirstByte == true)
		{
			dist[index] = sensor_byte[0];
			index++;
			//printf("3\n"); 
			return -1; 
		}
		//havent found first or last byte, check each byte to see if this is first
		else if(sensor_byte[0] == 82)
		{
			foundFirstByte = true;
			index = 0;
			//printf("4\n"); 
			return -1;
		}

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

int main()
{
	Sonar sonar("/dev/ttyUSB0");
	while(1)
	{
	int dist_mm = sonar.get_sonar_data();
	if(dist_mm > 0) printf("distance (mm): %i \n", dist_mm);
	}
}

