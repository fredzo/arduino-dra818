#ifndef DRA818_SIMU_h
#define DRA818_SIMU_h

#define RESPONSE_DELAY   300 // Simulate a 300ms response time from the DRA module

static bool async = false;
char commandBuffer[64];
int commandBufferIndex = 0;
unsigned long commandTime;
char responseBuffer[64];
int responseBufferIndex = 0;
bool responseAvailable = false;
int rssiValues[] = {0,32,64,96,128,160,192,224,255};


void dra818SimuSetAsync(bool asyncMode)
{
    async = asyncMode;
}

void dra818SimuWrite(char character)
{
    if(character == 0x0a)
    {   // Process command
        commandBuffer[commandBufferIndex] = 0; // String termination
        String commandString = String(commandBuffer);
        commandTime = millis();
        if(commandString.startsWith("AT+DMOCONNECT"))
        {
            strcpy(responseBuffer,"+DMOCONNECT:0\r\n");
        }
        else if (commandString.startsWith("S+"))
        {   // TODO change from 0 to 1 according to frequency
            strcpy(responseBuffer,"S=0\r\n");
        }
        else if (commandString.startsWith("AT+DMOSETGROUP"))
        {
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
            strcpy(responseBuffer,"+DMOREADGROUP:0,433.5000,433.5000,0123,3,0456\r\n");
        }
        else if (commandString.startsWith("AT+VERSION"))
        {
            strcpy(responseBuffer,"+VERSION:SA818_V5.0\r\n");
        }
        else
        {
            strcpy(responseBuffer,"+ERROR:1\r\n");
        }
        responseAvailable = true;
        responseBufferIndex = 0;
        commandBufferIndex = 0;
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
{
    if(!responseAvailable) return false;
    if(millis()-commandTime<RESPONSE_DELAY) return false;
    return true;
}

void dra818SimuTask()
{

}

char dra818SimuRead()
{   // Noting to read
    if(!responseAvailable) return 0;
    // No response before the delay
    if(millis()-commandTime<RESPONSE_DELAY) return 0;
    char result = responseBuffer[responseBufferIndex];
    responseBufferIndex++;
    if(result == 0x0a)
    {   // End of command
        responseAvailable = false;
    }
    return result;
}

#endif
