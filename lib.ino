#include <stdio.h>
#include <string.h>
#include <stdlib.h>

char buff[100];
int buffOfset = 0;
uint8_t sum = 0;
char sumVal [3];
bool sumStat = false;
char svIndex = 0;

void procGPGGA(char *, struct dataGPS *);
void procGPRMC(char *, struct dataGPS *);
int chksum(char *);
int toInt(char *, int);
float parseLoc(char *, char *);
void gpsENC(char );

/*
int main(void){
  char GPS[] = "$GPGGA,230611.016,3907.3813,N,12102.4635,W,0,04,5.7,507.9,M,,,,0000*11\r\n";
  char GPS1[] = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62";

  if(chksum(GPS))
    procGPGGA(GPS);

  printf("\n");
  if(chksum(GPS1))
    procGPRMC(GPS1);

  return 0;
}
*/

void gpsENC(char c){
    switch (c) {
      case '*':
        sumStat = true;
      case '\r':
        break;
      case '\n':
        break;
      case '$':
      sumVal[2] = '\0';
        buff[buffOfset] = '\0';
        buffOfset = 0;
        sumStat = false;
        svIndex = 0;

        if(sum == strtol(sumVal, 0, 16)){
          gpsStat = true;
          if(buff[3] == 'G' && buff[4] == 'A'){
            procGPGGA(buff, &dataGPS);
          }
          else if(buff[3] = 'M' && buff[4] == 'C'){
            procGPRMC(buff, &dataGPS);
          }
        }
        sum = 0;
      break;
      default:
        if(!sumStat){
          buff[buffOfset++] = c;
          sum ^= (uint8_t)c;
        }
        else
          sumVal[svIndex++] = c;
    }
}

float parseLoc(char * data, char * FLAG){
  float result;

  result = toInt(data, (FLAG - data) - 9) + atof(FLAG - 9) / 60;

  if (*FLAG == 'S' || *FLAG == 'W')
    return -result;

  return result;
}

int toInt(char * data, int len){
  int i = 0;
  char tmp [10];

  while(i < len){
    tmp[i] = data[i];
    i++;
  }
  tmp[i] = '\0';

  return atoi(tmp);
}

void procGPGGA(char * data, struct dataGPS * res){
  int index = 0;
  float tmpF;
  int tmpI;

  char c;
  char * ptr [20];

  // Proses GPS
  int i = 0;
  ptr[index++] = &data[0];
  while((c = data[i++]) != '\0'){
    if(c == ','){
      ptr[index++] = &data[i];
      *(data+i-1) = '\0';
    }
  }
  //TIME
  if(ptr[1][0]){
    res->hour = (uint8_t)toInt(ptr[1], 2);
    res->minute = (uint8_t)toInt(ptr[1]+2, 2);
    res->second = (uint8_t)toInt(ptr[1]+4, 2);
  }

  //Location
  if(ptr[2][0]){
    res->lati = parseLoc(ptr[2], ptr[3]);
    res->longi = parseLoc(ptr[4], ptr[5]);
  }

  //FIX QUALITY
  if(ptr[6][0])
    res->fixQ = atoi(ptr[6]);

  //SATELLITES
  if(ptr[7][0])
    res->sat = atoi(ptr[7]);

  //ALTITUDE
  if(ptr[9][0])
    res->alti = atof(ptr[9]);
}

void procGPRMC(char * data, struct dataGPS * res){
  int index = 0;

  char c;
  char * ptr [20];

  // Proses GPS
  int i = 0;
  ptr[index++] = &data[0];
  while((c = data[i++]) != '\0'){
    if(c == ','){
      ptr[index++] = &data[i];
      *(data+i-1) = '\0';
    }
  }

  //SPEED
  if(ptr[7][0])
    res->speed = atof(ptr[7]);

  //HEADING
  if(ptr[10][0])
    res->head = atof(ptr[10]);
}

int chksum(char * data){
  int index = 1; //skip '$'

  unsigned char sum = 0;
  char valid[3];
  char result[3];

  unsigned char c;
  while((c = data[index++]) != '*')
    sum ^= c;

  valid[0] = data[index++];
  valid[1] = data[index++];
  valid[2] = '\0';

  sprintf(result, "%x", sum);

  if(!strcmp(valid, result))
    return 1;

  return 0;
}
