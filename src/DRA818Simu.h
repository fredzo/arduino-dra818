#ifndef DRA818_SIMU_h
#define DRA818_SIMU_h

#include <DRA818.h>

#define RESPONSE_DELAY      300 // Simulate a 300ms response time from the DRA module

#define COMMAND_SET_GROUP   "AT+DMOSETGROUP="
#define COMMAND_SCAN        "S+"

#define RESPONSE_STACK_SIZE  16

char commandBuffer[64];
int commandBufferIndex = 0;
unsigned long commandTime;
char responseStack[RESPONSE_STACK_SIZE][64];
int responseStackWriteIndex = 0;
int responsetackReadIndex = 0;
char* responseBuffer = NULL;
int responseBufferIndex = 0;
int rssiValues[] = {0,32,64,96,128,160,192,224,255};
DRA818::Parameters parameters;
float busyFrequencies[] = {433.0,406.025,406.037,406.049};
#define BUSYS_FREQ_NUMBER 4

void dra818SimuWrite(char character)
{
    if(character == 0x0a)
    {   // Process command
        commandBuffer[commandBufferIndex] = 0; // String termination
        char* responseBuffer = responseStack[responseStackWriteIndex];
        String commandString = String(commandBuffer);
        commandTime = millis();
        if(commandString.startsWith("AT+DMOCONNECT"))
        {
            strcpy(responseBuffer,"+DMOCONNECT:0\r\n");
        }
        else if (commandString.startsWith(COMMAND_SCAN))
        {
            commandString = commandString.substring(strlen(COMMAND_SCAN));
            // Parse frequency value
            float frequency = atof(commandString.c_str());
            bool busy = false;
            for(int i = 0 ; i < BUSYS_FREQ_NUMBER ; i++)
            {
                if(frequency == busyFrequencies[i])
                {
                    busy = true;
                    break;
                }
            }
            sprintf(responseBuffer,"S=%d\r\n", busy ? 0 : 1);
        }
        else if (commandString.startsWith(COMMAND_SET_GROUP))
        {
            commandString = commandString.substring(strlen(COMMAND_SET_GROUP));
            parameters = DRA818::parseParameters(commandString);
            strcpy(responseBuffer,"+DMOSETGROUP:0\r\n");
        }
        else if (commandString.startsWith("AT+DMOSETVOLUME"))
        {
            strcpy(responseBuffer,"+DMOSETVOLUME:0\r\n");
        }
        else if (commandString.startsWith("RSSI?"))
        {
            int rssi = rssiValues[random(8)];
            sprintf(responseBuffer,"RSSI:%03d\r\n",rssi);
        }
        else if (commandString.startsWith("AT+SETFILTER"))
        {
            strcpy(responseBuffer,"+DMOSETFILTER:0\r\n");
        }
        else if (commandString.startsWith("AT+SETTAIL"))
        {
            strcpy(responseBuffer,"+DMOSETTAIL:0\r\n");
        }
        else if (commandString.startsWith("AT+DMOREADGROUP"))
        {
            sprintf(responseBuffer,"+DMOREADGROUP:%d,%3.4f,%3.4f,%03d,%d,%03d\r\n",parameters.bandwidth,parameters.freq_tx,parameters.freq_rx,parameters.ctcss_tx,parameters.squelch,parameters.ctcss_rx);
        }
        else if (commandString.startsWith("AT+VERSION"))
        {
            strcpy(responseBuffer,"+VERSION:SA818_V5.0\r\n");
        }
        else
        {
            strcpy(responseBuffer,"+ERROR:1\r\n");
        }
        commandBufferIndex = 0;
        responseStackWriteIndex++;
        if(responseStackWriteIndex >= RESPONSE_STACK_SIZE) responseStackWriteIndex = 0; // Circular stack
    }
    else
    {
        commandBuffer[commandBufferIndex] = character;
        commandBufferIndex++;
    }
}

void dra818SimuWrite(const char* buffer)
{
    int len = strlen(buffer);
    for(int i = 0; i < len ; i++)
    {
        dra818SimuWrite(buffer[i]);
    }
}

bool dra818SimuAvailable()
{   // No response before the delay
    if(millis()-commandTime<RESPONSE_DELAY) return false;
    // Noting to read
    if((responsetackReadIndex == responseStackWriteIndex) && (responseBuffer == NULL)) return false;
    return true;
 }

void dra818SimuTask()
{

}

char dra818SimuRead()
{   
    if(!dra818SimuAvailable()) return 0;
    if(responseBuffer == NULL)
    {   // Start a new response
        responseBuffer = responseStack[responsetackReadIndex];
        responsetackReadIndex++;
        if(responsetackReadIndex >= RESPONSE_STACK_SIZE) responsetackReadIndex = 0; // Circular stack
    }
    char result = responseBuffer[responseBufferIndex];
    responseBufferIndex++;
    if(result == 0x0a)
    {   // End of command
        responseBuffer = NULL;
        responseBufferIndex = 0;
    }
    return result;
}

#endif
