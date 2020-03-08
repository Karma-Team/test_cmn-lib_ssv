/*
 * "test_serial_communication_servo.cpp"
 */


#include <iostream>
#include <SSV_ReadWrite.hpp>
#include <csv.hpp>
#include <unistd.h>
#include <time.h>



using namespace std;
using namespace SSV;



#define CSV_FILE_MAX_SERVO_CMD 100



int main()
{
	cout << "Test C++ library for serial communication" << endl;

	string 		l_inputMsg;
	uint32_t 	l_writeCounter 	= 0;
	uint32_t 	l_servoId	 	= 1;
	int 		l_ret;
	int 		l_quitRequested	= 0;

	CLx16a l_pilotageServo("/dev/ttyUSB0", 115200);
	l_ret = l_pilotageServo.initDeviceSerialPort();
	if(l_ret != -1)
	{
		//	While loop: write and read on the device serial port
			do
			{
				// Enter client cmd
					cout << "> Client cmd : ";
					getline(cin, l_inputMsg);

				// Treat client cmd
					if(strcmp(l_inputMsg.c_str(), "help") == 0)
					{
						cout << "	help : display help message\n";
						cout << "	quit : close device serial port communication\n";
						cout << "	read_csv\n";
						cout << "	read\n";
						cout << "	write\n";
					}

					else if(strcmp(l_inputMsg.c_str(), "quit") == 0)
					{
						cout << "> Quit requested\n";
						l_quitRequested = 1;
					}

					else if(strcmp(l_inputMsg.c_str(), "read_csv") == 0)
					{
						io::CSVReader<3> in("/home/ahu/tmp/servo.csv");
						in.read_header(io::ignore_extra_column, "servoId", "positionAngle", "timeoutUs");
						int 			l_indexServoCmd;
						int 			l_servoCmdCounter 							= 0;
						int 			l_servoIdTab[CSV_FILE_MAX_SERVO_CMD];
						int 			l_positionAngle[CSV_FILE_MAX_SERVO_CMD];
						int 			l_timeoutUs[CSV_FILE_MAX_SERVO_CMD];
						int 			l_testCounter = 0;
						signed short 	l_position;

						while(in.read_row(l_servoIdTab[l_servoCmdCounter], l_positionAngle[l_servoCmdCounter], l_timeoutUs[l_servoCmdCounter]))
						{
							cout << "cmdId = " << l_servoCmdCounter << " servoId = " << l_servoIdTab[l_servoCmdCounter] << " positionAngle = " << l_positionAngle[l_servoCmdCounter] << " timeoutUs = " << l_timeoutUs[l_servoCmdCounter] << "\n";
							l_servoCmdCounter++;
						}

						for(l_indexServoCmd = 0 ; l_indexServoCmd < l_servoCmdCounter; l_indexServoCmd++)
						{
							cout << "RequestedPositionAngle = " << l_positionAngle[l_indexServoCmd] << "\n";
							if(l_timeoutUs[l_indexServoCmd] == 0)
							{
								l_pilotageServo.writeDeviceSerialPort(l_servoIdTab[l_indexServoCmd], SSV_SERVO_MOVE_TIME_WRITE, l_positionAngle[l_indexServoCmd]);
								sleep(2);
								l_pilotageServo.readDeviceSerialPort(l_servoIdTab[l_indexServoCmd], SSV_SERVO_POS_READ, &l_position);
							}
							else
							{
								int l_elapsedTimeUs = 0;
								clock_t l_beginTimeUs = clock();
								clock_t l_diffTimeUs;
								do
								{
									do
									{
										l_pilotageServo.writeDeviceSerialPort(l_servoIdTab[l_indexServoCmd], SSV_SERVO_MOVE_TIME_WRITE, l_positionAngle[l_indexServoCmd]);
										sleep(1);
										l_pilotageServo.readDeviceSerialPort(l_servoIdTab[l_indexServoCmd], SSV_SERVO_POS_READ, &l_position);
									} while((l_position < ((signed short) (l_positionAngle[l_indexServoCmd] - SSV_ANGLE_DEG_TOL))) || (l_position > ((signed short) (l_positionAngle[l_indexServoCmd] + SSV_ANGLE_DEG_TOL))));

									l_diffTimeUs 	= clock() - l_beginTimeUs;
									l_elapsedTimeUs = l_diffTimeUs * 1000 / CLOCKS_PER_SEC;
								} while(l_elapsedTimeUs < l_timeoutUs[l_indexServoCmd]);
							}
							cout << "CurrentPositionAngle = " << l_position << "\n";
							sleep(1);
						}
					}

					else if(strcmp(l_inputMsg.c_str(), "read") == 0)
					{
						for(uint32_t l_indexServoId = 0 ; l_indexServoId < 3 ; l_indexServoId++)
						{
							cout << "SERVO ID : " << l_indexServoId+1 << "\n";

							// Read position
							signed short l_position;
							l_pilotageServo.readDeviceSerialPort(l_indexServoId+1, SSV_SERVO_POS_READ, &l_position);
							cout << "> Position : " << l_position << " ° \n";

							// Read voltage
							signed short l_voltage;
							l_pilotageServo.readDeviceSerialPort(l_indexServoId+1, SSV_SERVO_VIN_READ, &l_voltage);
							cout << "> Voltage : " << l_voltage << " mV \n";

							// Read temperature
							signed short l_temperature;
							l_pilotageServo.readDeviceSerialPort(l_indexServoId+1, SSV_SERVO_TEMP_READ, &l_temperature);
							cout << "> Temperature : " << l_temperature << " °C \n";
						}
					}

					else if(strcmp(l_inputMsg.c_str(), "write") == 0)
					{
						if((l_writeCounter%2) == 0)
						{
							l_pilotageServo.writeDeviceSerialPort(l_servoId, SSV_SERVO_MOVE_TIME_WRITE, 240);
						}
						else
						{
							l_pilotageServo.writeDeviceSerialPort(l_servoId, SSV_SERVO_MOVE_TIME_WRITE, 0);
						}
						l_writeCounter++;
					}

					else
					{
						cout << "> Unknown client cmd : enter 'help' to display available cmd\n";
					}
			} while(l_quitRequested == 0);
	}
	else
	{
		cerr << "Device serial port is not initialized! Quitting" << endl;
		return -1;
	}

    return 0;

	return 0;
}